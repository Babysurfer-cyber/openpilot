"""Read-only factory EV6 SPAS2 request vs physical-lamp feedback timing.

Never creates a CAN packet, opens CAN/panda/UDS, or sends anything. Analysis of
already-recorded bus-0 OEM frames cannot prove external controller acceptance.
The lamp edge timestamps are sampled observations, not physical actuation times.
"""

import argparse
from bisect import bisect_left
from collections import Counter
import json
from pathlib import Path
from statistics import median

from ev6_spas_control import check_stock_frame
from ev6_spas_factory_replay import _decode_capture


def analyze_lamp_cadence(path, side):
  if side not in ('left', 'right'):
    raise ValueError('side must be left or right')
  expected = 3 if side == 'left' else 4
  lamp_bit = 0x10 if side == 'left' else 0x40
  spas, lamp = [], []
  skipped_other_bus = 0
  for t, addr, bus, raw in _decode_capture(path):
    if bus != 0:
      skipped_other_bus += 1
      continue
    if addr == 0x16A:
      if not check_stock_frame(addr, raw):
        raise ValueError(f'Invalid CRC/length in bus-0 SPAS2 at {t}')
      spas.append((t, (raw[16] >> 5) & 7, (raw[7], raw[18])))
    elif addr == 0x413:
      if len(raw) != 8:
        raise ValueError(f'Invalid bus-0 0x413 lamp data length at {t}')
      lamp.append((t, bool(raw[2] & lamp_bit)))
  if len(spas) < 2 or len(lamp) < 2:
    raise ValueError('Need genuine bus-0 SPAS2 and 0x413 observations')
  onset = next((i for i in range(1, len(spas)) if spas[i-1][1] == 0 and spas[i][1] == expected), None)
  if onset is None:
    raise ValueError('No factory SPAS2 direction onset')
  ended = next((i for i in range(onset+1, len(spas)) if spas[i-1][1] == expected and spas[i][1] == 0), None)
  if ended is None:
    raise ValueError('No factory SPAS2 direction end')
  t0, t1 = spas[onset][0], spas[ended][0]
  active_lamps = [(t, is_on) for t, is_on in lamp if t0 <= t < t1]
  if len(active_lamps) < 2:
    raise ValueError('Missing sampled lamp evidence in factory command window')
  edges = []
  first_on_edge = None
  for (before, was_on), (t, is_on) in zip(lamp, lamp[1:]):
    if was_on != is_on and t0 <= t < t1:
      edge = (before, t, is_on)
      edges.append(edge)
      if first_on_edge is None and is_on and 0 <= t-t0 <= .250:
        first_on_edge = edge
  if first_on_edge is None:
    raise ValueError('No fresh lamp OFF-to-ON observation within 250ms of command')
  on_times = [t for _, t, s in edges if s]
  off_times = [t for _, t, s in edges if not s]
  on_periods_ms = [(b-a)*1000 for a, b in zip(on_times, on_times[1:])]
  off_periods_ms = [(b-a)*1000 for a, b in zip(off_times, off_times[1:])]
  sample_gaps_ms = [(b[0]-a[0])*1000 for a,b in zip(active_lamps,active_lamps[1:])]
  # Constant raw byte7/byte18 state plateaus while control remains 3/4.
  start = onset
  plateaus = []
  for i in range(onset+1, ended+1):
    if i == ended or spas[i][2] != spas[i-1][2]:
      start_t, end_t = spas[start][0], spas[i][0]
      lamp_edges = sum(start_t <= edge_t < end_t for _,edge_t,_ in edges)
      plateaus.append({'start_s': round(start_t, 6), 'duration_s': round(end_t-start_t, 4),
                       'byte7': spas[start][2][0], 'byte18': spas[start][2][1],
                       'lamp_edge_count': lamp_edges})
      start = i
  longest = max(plateaus, key=lambda item: item['duration_s'])
  changes = len(plateaus)
  first_before, first_after, _ = first_on_edge
  return {
    'kind': 'EV6_FACTORY_SPAS2_LAMP_CADENCE_PASSIVE_NO_TX',
    'side': side, 'bus': 0,
    'can_tx_by_this_tool': 0, 'permits_vehicle_actuation': False,
    'external_ecu_handoff_tested': False, 'physical_lamp_onset_measured_directly': False,
    'ignored_nonzero_bus_records': skipped_other_bus,
    'factory_direction_control': expected,
    'factory_direction_onset_s': round(t0, 6),
    'factory_direction_end_s': round(t1, 6),
    'factory_direction_duration_s': round(t1-t0, 4),
    'continuous_factory_direction_frames': ended-onset,
    'observed_lamp_samples_while_requested': len(active_lamps),
    'lamp_on_samples_while_requested': sum(x for _, x in active_lamps),
    'lamp_off_samples_while_requested': sum(not x for _, x in active_lamps),
    'lamp_observed_on_edges': len(on_times),
    'lamp_observed_off_edges': len(off_times),
    'median_observed_on_on_period_ms': round(median(on_periods_ms), 2) if on_periods_ms else None,
    'median_observed_off_off_period_ms': round(median(off_periods_ms), 2) if off_periods_ms else None,
    'max_lamp_sample_gap_during_request_ms': round(max(sample_gaps_ms), 2) if sample_gaps_ms else None,
    'first_off_sample_before_first_on_s': round(first_before, 6),
    'first_on_sample_after_command_s': round(first_after, 6),
    'first_off_sample_relative_to_command_ms': round((first_before-t0)*1000, 2),
    'first_on_sample_relative_to_command_ms': round((first_after-t0)*1000, 2),
    'observed_lamp_state_transition_bracket_ms': round((first_after-first_before)*1000, 2),
    'raw_byte7_byte18_plateau_count': changes,
    'longest_unchanged_raw_pair_plateau': longest,
    'warning': ('0x413 is sampled feedback. A first ON sample after command does not '
                'establish the true physical turn-on delay, especially if its prior OFF '
                'sample predates the SPAS2 command. Raw byte associations do not '
                'establish causality or authority for external CAN TX.'),
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--left', required=True, type=Path)
  parser.add_argument('--right', required=True, type=Path)
  parser.add_argument('--output', type=Path)
  args = parser.parse_args()
  result = {'left': analyze_lamp_cadence(args.left, 'left'),
            'right': analyze_lamp_cadence(args.right, 'right')}
  content = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
  if args.output:
    args.output.write_text(content, encoding='utf-8')
  print(content)


if __name__ == '__main__':
  main()
