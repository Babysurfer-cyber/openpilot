"""Synthetic-only negative and positive replay checks; no vehicle communication."""
import csv
from pathlib import Path
import tempfile
import unittest

from ev6_spas_lamp_cadence import analyze_lamp_cadence
from test_ev6_spas_factory_replay import frame


def scenario(side='left', *, foreign_bus=False, has_onset=True, has_end=True,
             with_lamp=True, late_off=False):
  direction = 3 if side == 'left' else 4
  lamp_bit = 0x10 if side == 'left' else 0x40
  bus = 130 if foreign_bus else 0
  rows = []
  for i in range(140):
    t = round(1.0 + .02*i, 5)
    is_active = 12 <= i < 125 if has_end else i >= 12
    control = direction if (is_active and has_onset) else 0
    raw_pair = (1, 0x40) if i < 70 else (14, 0x40)
    rows.append((t, '0x16a', bus, frame(0x16A, i, control,
                                     byte7=raw_pair[0], byte18=raw_pair[1])))
  if with_lamp:
    for i in range(32):
      t = round(1.0 + .1*i, 5)
      active = t >= 1.3 and int(round((t-1.3)*10)) % 6 < 3
      rows.append((t, '0x413', bus, frame(0x413, lamp=lamp_bit if active else 0)))
    if late_off:
      rows.append((1.26, '0x413', bus, frame(0x413, lamp=0)))
  return sorted(rows, key=lambda x: x[0])


class TestLampCadence(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.path = Path(self.temp.name) / 'capture.csv'

  def write(self, rows):
    with self.path.open('w', newline='', encoding='utf-8') as f:
      writer = csv.writer(f)
      writer.writerow(('t_sec', 'can_id', 'bus', 'payload_hex'))
      writer.writerows(rows)
    return self.path

  def test_left_hold_command_still_has_multiple_lamp_edges(self):
    r = analyze_lamp_cadence(self.write(scenario('left')), 'left')
    self.assertEqual(r['factory_direction_control'], 3)
    self.assertGreater(r['lamp_observed_on_edges'], 2)
    self.assertGreater(r['lamp_observed_off_edges'], 2)
    self.assertGreater(r['longest_unchanged_raw_pair_plateau']['lamp_edge_count'], 1)
    self.assertFalse(r['physical_lamp_onset_measured_directly'])
    self.assertFalse(r['permits_vehicle_actuation'])
    self.assertEqual(r['can_tx_by_this_tool'], 0)

  def test_right_hold_command_works_as_read_only_correlation(self):
    r = analyze_lamp_cadence(self.write(scenario('right')), 'right')
    self.assertEqual(r['factory_direction_control'], 4)
    self.assertGreater(r['observed_lamp_samples_while_requested'], 10)

  def test_onset_bracket_can_begin_before_command(self):
    r = analyze_lamp_cadence(self.write(scenario()), 'left')
    self.assertEqual(r['first_off_sample_relative_to_command_ms'], -40.0)
    self.assertEqual(r['first_on_sample_relative_to_command_ms'], 60.0)
    self.assertEqual(r['observed_lamp_state_transition_bracket_ms'], 100.0)

  def test_onset_bracket_can_be_strictly_after_command(self):
    r = analyze_lamp_cadence(self.write(scenario(late_off=True)), 'left')
    self.assertEqual(r['first_off_sample_relative_to_command_ms'], 20.0)
    self.assertEqual(r['first_on_sample_relative_to_command_ms'], 60.0)

  def test_other_bus_cannot_replace_factory_lamp_evidence(self):
    with self.assertRaisesRegex(ValueError, 'Need genuine bus-0'):
      analyze_lamp_cadence(self.write(scenario(foreign_bus=True)), 'left')

  def test_missing_lamp_feedback_is_rejected(self):
    with self.assertRaisesRegex(ValueError, 'Need genuine bus-0'):
      analyze_lamp_cadence(self.write(scenario(with_lamp=False)), 'left')

  def test_missing_factory_direction_onset_is_rejected(self):
    with self.assertRaisesRegex(ValueError, 'No factory SPAS2 direction onset'):
      analyze_lamp_cadence(self.write(scenario(has_onset=False)), 'left')

  def test_missing_factory_direction_end_is_rejected(self):
    with self.assertRaisesRegex(ValueError, 'No factory SPAS2 direction end'):
      analyze_lamp_cadence(self.write(scenario(has_end=False)), 'left')

  def test_wrong_side_wont_report_factory_direction_onset(self):
    with self.assertRaisesRegex(ValueError, 'No factory SPAS2 direction onset'):
      analyze_lamp_cadence(self.write(scenario('left')), 'right')

  def test_invalid_spas2_crc_is_rejected(self):
    rows = scenario()
    ix = next(i for i,r in enumerate(rows) if r[1] == '0x16a')
    data = bytearray.fromhex(rows[ix][3]);data[18] ^= 0x40
    row=list(rows[ix]);row[3]=data.hex();rows[ix]=tuple(row)
    with self.assertRaisesRegex(ValueError, 'Invalid CRC/length'):
      analyze_lamp_cadence(self.write(rows), 'left')

  def test_invalid_lamp_length_is_rejected(self):
    rows = scenario()
    ix = next(i for i,r in enumerate(rows) if r[1] == '0x413')
    row=list(rows[ix]);row[3]='0000';rows[ix]=tuple(row)
    with self.assertRaisesRegex(ValueError, 'Invalid bus-0 0x413'):
      analyze_lamp_cadence(self.write(rows), 'left')

  def test_nonmonotonic_csv_is_rejected(self):
    rows = scenario();rows[4],rows[18]=rows[18],rows[4]
    with self.assertRaisesRegex(ValueError, 'Nonmonotonic'):
      analyze_lamp_cadence(self.write(rows), 'left')


if __name__ == '__main__':
  unittest.main()
