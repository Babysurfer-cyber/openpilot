"""Offline-only state-sequence checks with synthetic factory-like traces."""
import csv
from pathlib import Path
import tempfile
import unittest

from ev6_spas_state_timeline import analyze_state_timeline, compare_left_right
from test_ev6_spas_factory_replay import frame


STATE_SEQUENCE = ((1,64), (1,70), (13,70), (26,70), (13,70),
                  (13,64), (13,69), (14,69), (14,64), (14,69))


def synthetic(side, *, early_state=False, wrong_bus=False, truncated=False):
  expected = 3 if side == 'left' else 4
  states = {12:(1,64), 15:(1,70), 16:(13,70), 17:(26,70),
            18:(13,70), 19:(13,64), 20:(13,69), 21:(14,69),
            22:(14,64), 23:(14,69)}
  if early_state:
    states = {(i+1 if i>=15 else i): value for i,value in states.items()}
    states[11] = states.pop(12)
  bus = 130 if wrong_bus else 0
  last = (0x82, 0)
  rows = []
  for i in range(27 if not truncated else 23):
    if i in states:
      last = states[i]
    control = expected if 12 <= i < 25 else 0
    rows.append((round(1+i*.02, 4), '0x16a', bus,
                 frame(0x16A, i, control, byte7=last[0], byte18=last[1])))
  return rows


class TestTimeline(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.path = Path(self.temp.name)/'fake.csv'

  def write(self, rows):
    with self.path.open('w', newline='') as f:
      csv.writer(f).writerows([('t_sec','can_id','bus','payload_hex'),*rows])
    return self.path

  def test_left_normal_onset_state_changes_with_direction(self):
    r = analyze_state_timeline(self.write(synthetic('left')), 'left')
    self.assertEqual(r['onset_pair_state_lead_ms'], 0)
    self.assertEqual(r['onset_previous_pair'], [0x82,0])
    self.assertEqual(r['onset_pair'], [1,0x40])
    self.assertEqual(r['pair_transition_count'], 10)
    self.assertEqual(r['unique_state_pairs'], 8)
    self.assertEqual(r['pair_transition_sequence'], [list(p) for p in STATE_SEQUENCE])
    self.assertEqual(r['vehicle_can_tx'], 0)
    self.assertFalse(r['signal_causality_established'])

  def test_right_state_leads_command_one_stock_frame(self):
    r = analyze_state_timeline(self.write(synthetic('right', early_state=True)), 'right')
    self.assertEqual(r['onset_pair_state_lead_ms'], 20)
    self.assertEqual(r['onset_previous_pair'], [1,64])
    self.assertEqual(r['onset_pair'], [1,64])
    self.assertEqual(r['pair_transition_count'], 10)

  def test_compare_order_equal_but_timing_different(self):
    left = analyze_state_timeline(self.write(synthetic('left')), 'left')
    right = analyze_state_timeline(self.write(synthetic('right', early_state=True)), 'right')
    result = compare_left_right(left, right)
    self.assertTrue(result['same_pair_transition_sequence'])
    self.assertFalse(result['identical_timing'])
    self.assertFalse(result['permits_vehicle_actuation'])

  def test_wrong_bus_is_not_factory_evidence(self):
    with self.assertRaisesRegex(ValueError, 'Not enough bus-0'):
      analyze_state_timeline(self.write(synthetic('left', wrong_bus=True)), 'left')

  def test_missing_direction_end_is_rejected(self):
    with self.assertRaisesRegex(ValueError, 'No factory direction-to-zero'):
      analyze_state_timeline(self.write(synthetic('left', truncated=True)), 'left')

  def test_corrupted_crc_is_rejected(self):
    rows = synthetic('left')
    entry = list(rows[15]); data = bytearray.fromhex(entry[3]); data[18] ^= 2
    entry[3] = data.hex();rows[15] = tuple(entry)
    with self.assertRaisesRegex(ValueError, 'Invalid stock SPAS2 CRC'):
      analyze_state_timeline(self.write(rows), 'left')

  def test_no_direction_onset_is_rejected(self):
    rows = synthetic('left')
    with self.assertRaisesRegex(ValueError, 'No factory 0-to-direction'):
      analyze_state_timeline(self.write(rows), 'right')

  def test_nonmonotonic_capture_is_rejected(self):
    rows = synthetic('left');rows[4],rows[8] = rows[8],rows[4]
    with self.assertRaisesRegex(ValueError, 'Nonmonotonic'):
      analyze_state_timeline(self.write(rows), 'left')


if __name__ == '__main__':
  unittest.main()
