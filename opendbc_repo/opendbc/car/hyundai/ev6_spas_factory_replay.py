"""Offline, receive-only replay of genuine 2022 EV6 factory parking CSV logs.

This module has no CAN/UDS driver or vehicle communication. Input CSV is an
existing capture (t_sec, can_id, bus, payload_hex). Only bus-0 observations
are treated as original source; bus 2/130 forwarding/echo is excluded.
A positive report is evidence of logged OEM behavior, NEVER external actuation.
"""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

from ev6_spas_control import EV6SpasControlPlan, SPAS_LENGTHS, check_stock_frame

SPAS2 = 0x16A
LAMP = 0x413
LEVER = 0x3C1
SOURCE_BUS = 0
MAX_LAMP_DELAY_S = .250


def _decode_capture(path):
  result = []
  with Path(path).open(newline='', encoding='utf-8-sig') as handle:
    reader = csv.DictReader(handle)
    required = {'t_sec', 'can_id', 'bus', 'payload_hex'}
    if not required.issubset(reader.fieldnames or []):
      raise ValueError('Missing CSV columns: ' + ', '.join(sorted(required - set(reader.fieldnames or []))))
    last_stamp = -math.inf
    for line, row in enumerate(reader, 2):
      try:
        stamp = float(row['t_sec'])
        addr = int(row['can_id'], 0)
        bus = int(row['bus'])
        raw = bytes.fromhex(row['payload_hex'])
      except (ValueError, TypeError, KeyError) as exc:
        raise ValueError(f'Malformed CSV row {line}') from exc
      if not math.isfinite(stamp) or stamp < last_stamp:
        raise ValueError(f'Nonmonotonic/nonfinite capture timestamp at row {line}')
      if bus < 0 or addr < 0 or addr > 0x1FFFFFFF:
        raise ValueError(f'Invalid bus/address at row {line}')
      last_stamp = stamp
      result.append((stamp, addr, bus, raw))
  if not result:
    raise ValueError('No capture events')
  return result


def analyze_capture(path, side):
  if side not in ('left', 'right'):
    raise ValueError('side must be left or right')
  frames = _decode_capture(path)
  first_time = frames[0][0]
  planner = EV6SpasControlPlan(bus=SOURCE_BUS)
  stock_totals = Counter()
  ctrl_states = Counter()
  active_states = Counter()
  ignored_bus = Counter()
  prev_stock = {}
  counter_gaps = []
  prev_spas2 = None
  first_command_at = None
  command_frame = None
  lamp_previous = None
  first_lamp_at = None
  accepted_passive_snapshots = 0
  crc_invalid = 0
  request_or_response_records = 0
  lamp_bit = 0x10 if side == 'left' else 0x40
  expected_cmd = 3 if side == 'left' else 4

  for stamp, addr, bus, raw in frames:
    if bus != SOURCE_BUS:
      ignored_bus[bus] += 1
      continue
    if addr in (0x7B1, 0x7B9):
      request_or_response_records += 1  # presence is NOT proof of provenance or ECU handoff
    if addr in SPAS_LENGTHS:
      stock_totals[addr] += 1
      if not check_stock_frame(addr, raw):
        crc_invalid += 1
        raise ValueError(f'Invalid stock CRC/length at {stamp:.6f}s CAN 0x{addr:X}')
      if addr in prev_stock:
        last_t, last_counter = prev_stock[addr]
        if raw[2] != (last_counter + 1) % 256:
          counter_gaps.append({'addr': f'0x{addr:X}', 'at_s': round(stamp, 6),
                               'gap_ms': round((stamp-last_t)*1000, 3),
                               'previous_counter': last_counter, 'next_counter': raw[2]})
      prev_stock[addr] = stamp, raw[2]
      planner.observe_stock(addr, SOURCE_BUS, raw, stamp - first_time)
      if addr == SPAS2:
        ctl = (raw[16] >> 5) & 7
        ctrl_states[ctl] += 1
        if ctl == expected_cmd:
          active_states[(raw[7], raw[18])] += 1
        if (prev_spas2 is not None and prev_spas2[0] == 0 and ctl == expected_cmd
            and first_command_at is None):
          first_command_at = stamp
          command_frame = {'byte7': raw[7], 'byte18': raw[18], 'blinker_control': ctl}
        prev_spas2 = ctl, stamp
        if planner.passive_spas2_snapshot(stamp - first_time) is not None:
          accepted_passive_snapshots += 1
    if addr == LAMP:
      if len(raw) != 8:
        raise ValueError(f'Invalid lamp feedback length at {stamp:.6f}s')
      lit = bool(raw[2] & lamp_bit)
      if lamp_previous is not None and not lamp_previous and lit and first_command_at is not None:
        if 0 <= stamp-first_command_at <= MAX_LAMP_DELAY_S and first_lamp_at is None:
          first_lamp_at = stamp
      lamp_previous = lit

  if not all(stock_totals[addr] for addr in SPAS_LENGTHS):
    raise ValueError('Both original bus-0 SPAS streams are required')

  top_states = [{'byte7': b7, 'byte18': b18, 'frames': count}
                for (b7, b18), count in active_states.most_common(12)]
  factory_indicator_observed = first_command_at is not None and first_lamp_at is not None
  return {
    'kind': 'EV6_FACTORY_CSV_PASSIVE_REPLAY_NO_TX',
    'side': side,
    'can_tx_by_this_tool': 0,
    'permits_vehicle_actuation': False,
    'ecu_origin_independently_authenticated': False,
    'external_ecu_handoff_tested': False,
    'bus': SOURCE_BUS,
    'ignored_other_bus_frames': dict(sorted(ignored_bus.items())),
    'stock_spas_frames': {f'0x{a:X}': stock_totals[a] for a in SPAS_LENGTHS},
    'invalid_spas_crc_frames': crc_invalid,
    'stock_counter_discontinuities': counter_gaps,
    'aprk_request_or_reply_frames_seen_on_bus0': request_or_response_records,
    'spas2_control_frame_counts': dict(sorted(ctrl_states.items())),
    'factory_direction_cmd': expected_cmd,
    'factory_cmd_onset_s': round(first_command_at, 6) if first_command_at is not None else None,
    'factory_cmd_onset_state': command_frame,
    'lamp_off_on_after_cmd_s': round(first_lamp_at, 6) if first_lamp_at is not None else None,
    'lamp_onset_delay_ms': round((first_lamp_at-first_command_at)*1000, 3) if factory_indicator_observed else None,
    'factory_indicator_observed_in_log': factory_indicator_observed,
    'active_factory_spas2_byte7_byte18_variants': len(active_states),
    'active_factory_spas2_byte7_byte18_top_pairs': top_states,
    'passive_planner_snapshots_accepted': accepted_passive_snapshots,
    'donor_zero_fill_matches_factory_command_onset': bool(command_frame and
                                                           command_frame['byte7'] == 0 and command_frame['byte18'] == 0),
    'warning': ('This replay neither issues CAN/UDS requests nor validates external control. '
                'It cannot authenticate capture origin, prove UDS handoff, or authorize real-car TX.'),
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--left', required=True, type=Path)
  parser.add_argument('--right', required=True, type=Path)
  parser.add_argument('--output', type=Path)
  opts = parser.parse_args()
  report = {'left': analyze_capture(opts.left, 'left'),
            'right': analyze_capture(opts.right, 'right')}
  raw = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
  if opts.output:
    opts.output.write_text(raw, encoding='utf-8')
  print(raw)


if __name__ == '__main__':
  main()
