#!/usr/bin/env python3
"""Read-only offline EV6 factory SPAS2 command/state/lamp timing comparison.

Input: CSV columns t_sec,can_id,bus,payload_hex exported from authentic rlog.
No CAN socket, no panda interface, no UDS and no message construction/TX.
Offsets below are *raw undocumented bytes*, not vehicle control commands.
"""
import argparse
import csv
import json
from pathlib import Path

SPAS2 = 0x16A
LAMP = 0x413
STATE_OFFSETS = (7, 18)


def crc16(data):
  crc = 0
  for byte in data:
    crc ^= byte << 8
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
  return crc


def check_spas2(data):
  return len(data) == 32 and int.from_bytes(data[:2], 'little') == (
    crc16(data[2:] + SPAS2.to_bytes(2, 'little')) ^ 0x9F5B)


def parse_capture(path):
  frames = {SPAS2: [], LAMP: []}
  with Path(path).open(newline='', encoding='utf-8') as f:
    columns = csv.DictReader(f)
    required = {'t_sec', 'can_id', 'bus', 'payload_hex'}
    if not required.issubset(columns.fieldnames or []):
      raise ValueError('Required CSV columns: ' + ','.join(sorted(required)))
    for row in columns:
      if int(row['bus']) != 0:
        continue  # never mistake forwarded / returned frames for original bus-0 RX
      addr = int(row['can_id'], 0)
      if addr not in frames:
        continue
      t = float(row['t_sec'])
      dat = bytes.fromhex(row['payload_hex'])
      if addr == SPAS2 and not check_spas2(dat):
        raise ValueError(f'CRC/length invalid for SPAS2 at {t:.5f}s in {path}')
      if addr == LAMP and len(dat) != 8:
        raise ValueError(f'lamp feedback wrong length in {path}')
      frames[addr].append((t, dat))
  for seq in frames.values():
    seq.sort(key=lambda r: r[0])
  return frames


def control(d):
  return (d[16] >> 5) & 7


def lamp_on(d, side):
  return bool(d[2] & (0x10 if side == 'left' else 0x40))


def analyze(frames, side):
  if side not in ('left', 'right'):
    raise ValueError('side must be left/right')
  seq, lamps = frames[SPAS2], frames[LAMP]
  expected = 3 if side == 'left' else 4
  if len(seq) < 2 or len(lamps) < 2:
    raise ValueError('Insufficient stock bus-0 SPAS2 or lamp feedback')
  onset_index = next((i for i in range(1, len(seq)) if control(seq[i-1][1]) == 0 and control(seq[i][1]) == expected), None)
  if onset_index is None:
    raise ValueError('No factory 0-to-direction SPAS2 transition')
  onset_t, current = seq[onset_index]
  prev_t, previous = seq[onset_index-1]
  changed = [j for j in range(3, 32) if previous[j] != current[j]]
  states = {}
  for offset in STATE_OFFSETS:
    prior = None
    # Last transition to the state seen at ON, within 500 ms, including ON.
    for i in range(onset_index, 0, -1):
      if seq[i-1][1][offset] != seq[i][1][offset]:
        t, dat = seq[i]
        if 0 <= onset_t - t <= .5:
          prior = {
            'transition_time_s': round(t, 5),
            'lead_before_direction_ms': round((onset_t - t) * 1000, 2),
            'from_hex': f'0x{seq[i-1][1][offset]:02x}',
            'to_hex': f'0x{dat[offset]:02x}',
          }
        break
    states[f'byte_{offset}'] = {'value_at_direction_hex': f'0x{current[offset]:02x}', 'last_transition': prior}
  # Require genuine lamp OFF -> ON edge, not first already-ON sample.
  lamp_transition = next(((t, oldt) for (oldt, old), (t, new) in zip(lamps, lamps[1:])
                          if not lamp_on(old, side) and lamp_on(new, side)
                          and 0 <= t-onset_t <= .25 and 0 <= t-oldt <= .5), None)
  if lamp_transition is None:
    raise ValueError('No fresh physical OFF-to-ON edge within 250ms of SPAS2 command')
  off_idx = next((i for i in range(onset_index+1, len(seq))
                  if control(seq[i-1][1]) == expected and control(seq[i][1]) == 0), None)
  return {
    'side': side, 'onset_time_s': round(onset_t, 5),
    'previous_frame_gap_ms': round((onset_t-prev_t)*1000, 2),
    'direction_command_from': 0, 'direction_command_to': expected,
    'raw_changed_offsets_excluding_crc_counter': changed,
    'raw_states': states,
    'first_physical_lamp_off_to_on_s': round(lamp_transition[0], 5),
    'first_lamp_on_delay_ms': round((lamp_transition[0] - onset_t)*1000, 2),
    'direction_ended_time_s': round(seq[off_idx][0], 5) if off_idx is not None else None,
    'spas2_stock_frames_bus0': len(seq),
    'warning': ('Single genuine OEM auto-parking observation. Raw state-byte meaning unknown; '
                'timing association does NOT prove required ordering or authorize external SPAS2 transmission.'),
  }


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--left', required=True, type=Path)
  p.add_argument('--right', required=True, type=Path)
  p.add_argument('--output', type=Path)
  a = p.parse_args()
  result = {'kind': 'EV6_FACTORY_SPAS2_ONSET_SEQUENCE_PASSIVE_ONLY',
            'vehicle_can_tx_by_this_tool': 0,
            'cases': {'left': analyze(parse_capture(a.left), 'left'),
                      'right': analyze(parse_capture(a.right), 'right')}}
  result['comparative_note'] = ('byte7/18 transitions are simultaneous with direction on left, '
                                'precede direction on right; no fixed lead time can be inferred')
  out = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
  if a.output:
    a.output.write_text(out, encoding='utf-8')
  print(out)


if __name__ == '__main__':
  main()
