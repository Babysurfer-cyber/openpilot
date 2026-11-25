"""Read-only EV6 SPAS2 state timeline from previously captured CSV files.

Uses *only* bus-0 stock RX observations. Never constructs messages for CAN TX,
never opens CAN/panda/UDS, and cannot prove ECU origin or safe external control.
Raw state bytes 7 and 18 are undocumented: co-occurrence is not causation.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

from ev6_spas_control import check_stock_frame
from ev6_spas_factory_replay import _decode_capture, SPAS2, SOURCE_BUS


def analyze_state_timeline(path, side):
  if side not in ('left', 'right'):
    raise ValueError('side must be left or right')
  expected = 3 if side == 'left' else 4
  stock = []
  for stamp, addr, bus, raw in _decode_capture(path):
    if bus != SOURCE_BUS or addr != SPAS2:
      continue
    if not check_stock_frame(addr, raw):
      raise ValueError(f'Invalid stock SPAS2 CRC/length at {stamp:.6f}s')
    stock.append((stamp, (raw[16] >> 5) & 7, (raw[7], raw[18])))
  if len(stock) < 2:
    raise ValueError('Not enough bus-0 SPAS2 factory observations')
  onset_idx = next((i for i in range(1, len(stock))
                    if stock[i-1][1] == 0 and stock[i][1] == expected), None)
  if onset_idx is None:
    raise ValueError('No factory 0-to-direction SPAS2 transition')
  ended_idx = next((i for i in range(onset_idx+1, len(stock))
                    if stock[i-1][1] == expected and stock[i][1] == 0), None)
  if ended_idx is None:
    raise ValueError('No factory direction-to-zero transition')
  onset_time = stock[onset_idx][0]
  ended_time = stock[ended_idx][0]
  # Track the nearest transition into the *actual onset* byte7/byte18 state,
  # not a hardcoded 0x01/0x40 request, and never treat a state as a TX recipe.
  onset_state = stock[onset_idx][2]
  state_entry_idx = next((i for i in range(onset_idx, 0, -1)
                          if stock[i][2] != stock[i-1][2]), None)
  # The state must have entered its onset value within 250 ms before onset.
  # If the latest change introduced a different pair, it is not evidence.
  state_entry_time = None
  if (state_entry_idx is not None and stock[state_entry_idx][2] == onset_state and
      0 <= onset_time - stock[state_entry_idx][0] <= .250):
    state_entry_time = stock[state_entry_idx][0]
  pairs = Counter(entry[2] for entry in stock[onset_idx:ended_idx])
  transitions = []
  for i in range(onset_idx, ended_idx):
    if i > onset_idx and stock[i][2] == stock[i-1][2]:
      continue
    at, ctrl, state = stock[i]
    transitions.append({'ms_after_command': round((at-onset_time)*1000, 2),
                        'byte7': state[0], 'byte18': state[1]})
  transition_sequence = [[v['byte7'], v['byte18']] for v in transitions]
  return {
    'kind': 'EV6_FACTORY_SPAS2_STATE_SEQUENCE_PASSIVE_ONLY',
    'side': side, 'vehicle_can_tx': 0, 'external_handoff_tested': False,
    'ecu_origin_independently_authenticated': False,
    'signal_causality_established': False,
    'bus': SOURCE_BUS, 'stock_spas2_frames': len(stock),
    'factory_direction_value': expected,
    'onset_time_s': round(onset_time, 6),
    'onset_previous_pair': list(stock[onset_idx-1][2]),
    'onset_pair': list(onset_state),
    'last_pair_transition_to_onset_time_s': round(state_entry_time, 6) if state_entry_time is not None else None,
    'onset_pair_state_lead_ms': round((onset_time-state_entry_time)*1000, 2) if state_entry_time is not None else None,
    'direction_end_time_s': round(ended_time, 6),
    'direction_duration_s': round(ended_time-onset_time, 4),
    'direction_frames': ended_idx-onset_idx,
    'unique_state_pairs': len(pairs),
    'state_pair_counts': [{'byte7': b7, 'byte18': b18, 'frames': count}
                          for (b7, b18),count in pairs.most_common()],
    'pair_transition_count': len(transitions),
    'pair_transition_sequence': transition_sequence,
    'pair_transitions': transitions,
    'warning': ('Observed OEM byte states can be descriptive or correlated. '
                'The record does not identify their causal meaning or authorize any transmission.'),
  }


def compare_left_right(left, right):
  return {
    'same_pair_transition_sequence': left['pair_transition_sequence'] == right['pair_transition_sequence'],
    'left_count': left['pair_transition_count'],
    'right_count': right['pair_transition_count'],
    'same_number_of_pairs': left['unique_state_pairs'] == right['unique_state_pairs'],
    'identical_timing': left['pair_transitions'] == right['pair_transitions'],
    'causality_established': False,
    'permits_vehicle_actuation': False,
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--left', required=True, type=Path)
  parser.add_argument('--right', required=True, type=Path)
  parser.add_argument('--output', type=Path)
  args = parser.parse_args()
  left = analyze_state_timeline(args.left, 'left')
  right = analyze_state_timeline(args.right, 'right')
  report = {'left': left, 'right': right, 'comparison': compare_left_right(left, right)}
  result = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
  if args.output:
    args.output.write_text(result, encoding='utf-8')
  print(result)


if __name__ == '__main__':
  main()
