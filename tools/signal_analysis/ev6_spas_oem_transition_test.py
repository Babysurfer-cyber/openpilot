#!/usr/bin/env python3
"""Offline read-only comparison of genuine Hyundai EV6 auto-parking turn signals.

Never opens a CAN send socket, never speaks UDS and never changes the car.
Requires locally saved rlogs and an openpilot Python environment for LogReader.

  python -m openpilot.tools.signal_analysis.ev6_spas_oem_transition_test \
      --left '/path/rlog (6).zst' --right '/path/rlog (7).zst' \
      --idle '/path/rlog (9).zst' --output ev6_spas_oem_report.json

The OEM SPAS frames show only what the factory ECU did, not what an
external device is allowed to transmit or whether it would be accepted.
"""

import argparse
import json
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

SPAS1 = 0x165
SPAS2 = 0x16A
LAMP = 0x413
LEVER = 0x3C1
WATCH = {SPAS1, SPAS2, LAMP, LEVER, 0x7B1, 0x7B9}
CRC_XOR = {24: 0x819D, 32: 0x9F5B}


@dataclass(frozen=True)
class Frame:
  ns: int
  data: bytes


def xmodem_crc(data):
  crc = 0
  for byte in data:
    crc ^= byte << 8
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
  return crc


def checksum_ok(address, data):
  if len(data) not in CRC_XOR:
    return False
  expected = xmodem_crc(data[2:] + address.to_bytes(2, 'little')) ^ CRC_XOR[len(data)]
  return expected == int.from_bytes(data[:2], 'little')


def control(data):
  return (data[16] >> 5) & 7 if len(data) == 32 else None


def lamp_active(data, side):
  if len(data) != 8:
    return False
  # Genuine user's 2022 EV6 auto-park recordings, independent of UI status.
  return bool(data[2] & (0x10 if side == 'left' else 0x40))


def stalk_crc8_ok(data):
  """Stock 0x3C1 CRC8 poly 0x1D, init=0, xorout=0x65 on bytes 1..7."""
  if len(data) != 8:
    return False
  crc = 0
  for byte in data[1:]:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1D if crc & 0x80 else 0)) & 0xFF
  return (crc ^ 0x65) == data[0]


def lever_neutral(data):
  """EV6 2022 recorded 0x3C1 left/right lever/tap bytes both neutral."""
  return len(data) == 8 and data[3] == 0 and data[4] == 0


def factory_lamp_without_stalk(frames, side):
  """OEM lamp-on observations supported by time-near, CRC-valid neutral stalk RX.

  Stock 0x3C1 is ~5 Hz. A missing lever stream cannot establish that a
  physical stalk was neutral during the lamp-on period. Require a genuine
  neutral sample within 150 ms of EVERY lamp-on observation.
  """
  from bisect import bisect_left
  levers = frames.get(LEVER, [])
  lamps = frames.get(LAMP, [])
  lamp_on = [f for f in lamps if lamp_active(f.data, side)]
  invalid = sum(not stalk_crc8_ok(f.data) for f in levers)
  active = sum(stalk_crc8_ok(f.data) and not lever_neutral(f.data) for f in levers)
  neutral = sorted(f.ns for f in levers if stalk_crc8_ok(f.data) and lever_neutral(f.data))
  nearest_ms = []
  for lamp in lamp_on:
    i = bisect_left(neutral, lamp.ns)
    dist = min((abs(lamp.ns - neutral[j]) / 1e6 for j in (i - 1, i)
                if 0 <= j < len(neutral)), default=float('inf'))
    nearest_ms.append(dist)
  uncovered = sum(d > 150 for d in nearest_ms)
  supported = bool(lamp_on and levers) and invalid == 0 and active == 0 and uncovered == 0
  return {
    'physical_lamp_on_frames': len(lamp_on),
    'stock_stalk_bus0_frames': len(levers),
    'stock_stalk_crc_invalid': invalid,
    'stock_stalk_active_frames': active,
    'stock_stalk_neutral_frames': len(neutral),
    'lamp_on_frames_without_neutral_stalk_within_150ms': uncovered,
    'max_lamp_on_to_neutral_stalk_distance_ms': round(max(nearest_ms), 2) if nearest_ms and uncovered == 0 else None,
    'factory_lamp_with_all_observed_stalk_frames_neutral': supported,
    'interpretation': ('OEM auto-park lamps illuminated while all observed stock stalk frames were valid/neutral, '
                       'with time-near neutral coverage for each lamp-on sample. Not external actuation proof.'
                       if supported else 'Insufficient or conflicting temporal stalk evidence; no inference'),
  }


def load_rlog(path):
  """Read only genuine bus-0 incoming frames, never sendcan or bus-128 echo."""
  from openpilot.tools.lib.logreader import LogReader
  seen = {key: [] for key in WATCH}
  for event in LogReader(str(path)):
    if event.which() != 'can':
      continue
    stamp = int(event.logMonoTime)
    for msg in event.can:
      if msg.src == 0 and msg.address in WATCH:
        seen[int(msg.address)].append(Frame(stamp, bytes(msg.dat)))
  for values in seen.values():
    values.sort(key=lambda f: f.ns)
  return seen


def transition_data(frames, lamps, side):
  nonzero = 3 if side == 'left' else 4
  for i in range(1, len(frames)):
    prev, curr = frames[i-1], frames[i]
    if control(prev.data) == 0 and control(curr.data) == nonzero:
      changes = [j for j, (a, b) in enumerate(zip(prev.data, curr.data)) if a != b]
      next_lamp = next((lamp for lamp in lamps if lamp.ns >= curr.ns and lamp_active(lamp.data, side)), None)
      return {
        'command_from': 0,
        'command_to': nonzero,
        'transition_ns': curr.ns,
        'period_before_ms': round((curr.ns - prev.ns) / 1e6, 2),
        'changed_byte_offsets_including_crc_counter': changes,
        'changed_offsets_without_crc_counter': [j for j in changes if j not in (0, 1, 2)],
        'first_physical_lamp_after_ms': round((next_lamp.ns - curr.ns) / 1e6, 2) if next_lamp else None,
        'before_stock_crc_ok': checksum_ok(SPAS2, prev.data),
        'after_stock_crc_ok': checksum_ok(SPAS2, curr.data),
        'warning': 'Observed OEM parking state only; do NOT transmit/replay these frames',
      }
  return None


def inspect_case(frames, side=None):
  spas1, spas2 = frames.get(SPAS1, []), frames.get(SPAS2, [])
  all_controls = Counter(control(f.data) for f in spas2)
  result = {
    'spas1_frames': len(spas1),
    'spas2_frames': len(spas2),
    'spas1_crc_invalid': sum(not checksum_ok(SPAS1, f.data) for f in spas1),
    'spas2_crc_invalid': sum(not checksum_ok(SPAS2, f.data) for f in spas2),
    'spas2_control_histogram': {str(k): v for k, v in sorted(all_controls.items())},
    'spas1_byte3_histogram': {f'0x{k:02x}': v for k, v in sorted(Counter(f.data[3] for f in spas1 if len(f.data) > 3).items())},
    'spas2_byte3_histogram': {f'0x{k:02x}': v for k, v in sorted(Counter(f.data[3] for f in spas2 if len(f.data) > 3).items())},
    'spas2_byte18_histogram': {f'0x{k:02x}': v for k, v in sorted(Counter(f.data[18] for f in spas2 if len(f.data) > 18).items())},
    'uds_7b1_rx_frames': len(frames.get(0x7B1, [])),
    'uds_7b9_rx_frames': len(frames.get(0x7B9, [])),
    'blinkers_lamp_frames': len(frames.get(LAMP, [])),
    'lever_frames': len(frames.get(LEVER, [])),
  }
  if side:
    result['factory_turn_on_transition'] = transition_data(spas2, frames.get(LAMP, []), side)
    result['factory_lamp_without_stalk'] = factory_lamp_without_stalk(frames, side)
  return result


def build_report(groups):
  result = {
    'test': 'EV6_SPAS2_OEM_OFFLINE_READ_ONLY',
    'can_tx_count': 0,
    'warning': 'No live CAN or UDS activity. OEM captures do not establish safe third-party ECU control.',
    'cases': {name: inspect_case(rows, name if name in ('left', 'right') else None) for name, rows in groups.items()},
  }
  for name, case in result['cases'].items():
    if case['spas1_crc_invalid'] or case['spas2_crc_invalid']:
      raise ValueError(f'{name}: stock SPAS CRC validation failed; do not rely on results')
    if not case['spas1_frames'] or not case['spas2_frames']:
      raise ValueError(f'{name}: missing incoming stock SPAS1/2 on bus 0')
    if name in ('left', 'right'):
      if case['factory_turn_on_transition'] is None:
        raise ValueError(f'{name}: no factory 0-to-{3 if name=="left" else 4} transition')
      # The same proof can legitimately be False in a different capture.
      # A malformed stalk recording, however, must never support it.
      proof = case['factory_lamp_without_stalk']
      if proof['stock_stalk_crc_invalid']:
        raise ValueError(f'{name}: invalid factory 0x3C1 checksum; evidence not trustworthy')
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--left', type=Path, required=True, help='factory left auto-park rlog')
  parser.add_argument('--right', type=Path, required=True, help='factory right auto-park rlog')
  parser.add_argument('--idle', type=Path, required=True, help='normal parked rlog')
  parser.add_argument('--output', type=Path, help='write JSON report to this local file')
  args = parser.parse_args()
  results = build_report({name: load_rlog(path) for name, path in [('left', args.left), ('right', args.right), ('idle', args.idle)]})
  report = json.dumps(results, indent=2, ensure_ascii=False)
  if args.output:
    args.output.write_text(report + '\n', encoding='utf-8')
    print(f'Wrote read-only comparison to {args.output}')
  print(report)


if __name__ == '__main__':
  main()
