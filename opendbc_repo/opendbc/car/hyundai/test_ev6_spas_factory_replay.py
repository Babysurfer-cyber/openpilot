"""Offline synthetic regression tests for the read-only factory CSV replay tool."""
import csv
from pathlib import Path
import tempfile
import unittest

from ev6_spas_factory_replay import analyze_capture, lever_checksum
from ev6_spas_control import crc16_xmodem


def frame(addr, counter=0, control=0, *, byte7=0, byte18=0, lamp=0):
  n = 24 if addr == 0x165 else (32 if addr == 0x16A else 8)
  b = bytearray(n)
  if addr == 0x413:
    b[2] = lamp
  else:
    b[2] = counter
    if addr == 0x16A:
      b[16] = control << 5
      b[7], b[18] = byte7, byte18
    mask = {24: 0x819D, 32: 0x9F5B}[n]
    b[:2] = (crc16_xmodem(b[2:] + addr.to_bytes(2, 'little')) ^ mask).to_bytes(2, 'little')
  return b.hex()


def scenario(side, foreign=False):
  expected = 3 if side == 'left' else 4
  lamp_bit = 0x10 if side == 'left' else 0x40
  rows = []
  for i in range(16):
    t = 1.00+i*.02
    ctl = expected if i >= 12 else 0
    rows.append((round(t, 4), '0x165', 0, frame(0x165, i)))
    rows.append((round(t, 4), '0x16a', 0, frame(0x16A, i, ctl, byte7=1 if ctl else 0x82,
                                                  byte18=0x40 if ctl else 0)))
    if foreign:
      rows.append((round(t, 4), '0x16a', 130, frame(0x16A, i, 7, byte7=0xFF)))
    if i in (10, 11, 12, 13):
      rows.append((round(t+.01, 4), '0x413', 0, frame(0x413, lamp=lamp_bit if i>=12 else 0)))
  return sorted(rows, key=lambda row: row[0])


def sample_lever(*, active=False, corrupt=False):
  raw = bytearray.fromhex('49e0300000000000')
  if active:
    raw[3] = 1  # synthetic nonzero stalk direction byte
    raw[0] = lever_checksum(raw)
  if corrupt:
    raw[0] ^= 1
  return raw.hex()


def with_lever(rows, before=True, after=True, active=False, corrupt=False, bus=0):
  if before:
    rows.append((1.17, '0x3c1', bus, sample_lever(active=active, corrupt=corrupt)))
  if after:
    rows.append((1.29, '0x3c1', bus, sample_lever(active=active, corrupt=corrupt)))
  return sorted(rows, key=lambda item:item[0])


class TestFactoryReplay(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.path = Path(self.tmp.name)/'capture.csv'

  def save(self, rows, header=('t_sec', 'can_id', 'bus', 'payload_hex')):
    with self.path.open('w', newline='', encoding='utf-8') as f:
      writer = csv.writer(f)
      writer.writerow(header)
      writer.writerows(rows)
    return self.path

  def test_left_realistic_factory_transition(self):
    r = analyze_capture(self.save(scenario('left')), 'left')
    self.assertTrue(r['factory_indicator_observed_in_log'])
    self.assertEqual(r['factory_direction_cmd'], 3)
    self.assertEqual(r['lamp_onset_delay_ms'], 10)
    self.assertEqual(r['stock_spas_frames'], {'0x165': 16, '0x16A': 16})
    self.assertEqual(r['can_tx_by_this_tool'], 0)
    self.assertFalse(r['donor_zero_fill_matches_factory_command_onset'])
    self.assertFalse(r['permits_vehicle_actuation'])

  def test_right_factory_transition(self):
    r = analyze_capture(self.save(scenario('right')), 'right')
    self.assertEqual(r['factory_direction_cmd'], 4)
    self.assertEqual(r['lamp_onset_delay_ms'], 10)
    self.assertTrue(r['factory_indicator_observed_in_log'])

  def test_bus130_not_oem_evidence(self):
    r = analyze_capture(self.save(scenario('left', foreign=True)), 'left')
    self.assertEqual(r['ignored_other_bus_frames'][130], 16)
    self.assertEqual(r['stock_spas_frames']['0x16A'], 16)
    self.assertEqual(r['invalid_spas_crc_frames'], 0)

  def test_bad_crc_fails_hard(self):
    rows = scenario('left')
    i = next(i for i,x in enumerate(rows) if x[1] == '0x16a')
    one = list(rows[i]); bad = bytearray.fromhex(one[3]); bad[16] ^= 0x20
    one[3] = bad.hex(); rows[i] = tuple(one)
    with self.assertRaisesRegex(ValueError, 'Invalid stock CRC'):
      analyze_capture(self.save(rows), 'left')

  def test_missing_required_csv_field(self):
    with self.assertRaisesRegex(ValueError, 'Missing CSV columns'):
      analyze_capture(self.save([], header=('t_sec', 'bus')), 'left')

  def test_unordered_timestamp_rejected(self):
    rows = scenario('left')
    rows[8],rows[14] = rows[14],rows[8]
    with self.assertRaisesRegex(ValueError, 'Nonmonotonic'):
      analyze_capture(self.save(rows), 'left')

  def test_no_lamp_feedback_no_factory_actuation_claim(self):
    rows = [row for row in scenario('left') if row[1] != '0x413']
    r = analyze_capture(self.save(rows), 'left')
    self.assertFalse(r['factory_indicator_observed_in_log'])
    self.assertIsNone(r['lamp_onset_delay_ms'])

  def test_wrong_bus_alone_is_not_stock(self):
    rows = [(t, a, 130, data) for t, a, _bus, data in scenario('left')]
    with self.assertRaisesRegex(ValueError, 'Both original bus-0 SPAS'):
      analyze_capture(self.save(rows), 'left')

  def test_wrong_side_no_false_match(self):
    r = analyze_capture(self.save(scenario('left')), 'right')
    self.assertFalse(r['factory_indicator_observed_in_log'])

  def test_bad_lamp_length_rejected(self):
    rows = scenario('left')
    i = next(i for i,x in enumerate(rows) if x[1] == '0x413')
    bad=list(rows[i]);bad[-1]='0102';rows[i]=tuple(bad)
    with self.assertRaisesRegex(ValueError, 'Invalid lamp feedback length'):
      analyze_capture(self.save(rows), 'left')


  def test_factory_lamp_observed_without_stalk_does_not_imply_neutral_lever(self):
    r = analyze_capture(self.save(scenario('left')), 'left')
    self.assertTrue(r['factory_indicator_observed_in_log'])
    self.assertFalse(r['factory_event_neutral_lever_corroborated'])
    self.assertEqual(r['lever_onset_context']['window_samples'], 0)

  def test_valid_neutral_stalk_before_and_after_corrobates_recorded_event(self):
    r = analyze_capture(self.save(with_lever(scenario('left'))), 'left')
    self.assertTrue(r['factory_event_neutral_lever_corroborated'])
    self.assertEqual(r['lever_original_bus0_frames'], 2)
    self.assertEqual(r['lever_non_neutral_observed_frames'], 0)
    self.assertTrue(r['lever_onset_context']['does_not_exclude_inter_sample_driver_action'])
    self.assertEqual(r['lever_onset_context']['before_onset_ms'], -70.)
    self.assertEqual(r['lever_onset_context']['after_onset_ms'], 50.)

  def test_driver_lever_active_during_onset_prevents_neutral_corrob(self):
    r = analyze_capture(self.save(with_lever(scenario('left'), active=True)), 'left')
    self.assertTrue(r['factory_indicator_observed_in_log'])
    self.assertEqual(r['lever_non_neutral_observed_frames'], 2)
    self.assertFalse(r['factory_event_neutral_lever_corroborated'])

  def test_invalid_lever_crc_is_rejected(self):
    with self.assertRaisesRegex(ValueError, 'Invalid OEM 0x3C1 lever CRC'):
      analyze_capture(self.save(with_lever(scenario('left'), corrupt=True)), 'left')

  def test_wrong_bus_neutral_lever_is_not_independent_evidence(self):
    r = analyze_capture(self.save(with_lever(scenario('left'), bus=130)), 'left')
    self.assertEqual(r['lever_original_bus0_frames'], 0)
    self.assertFalse(r['factory_event_neutral_lever_corroborated'])

  def test_missing_pre_onset_lever_prevents_neutral_corroboration(self):
    r = analyze_capture(self.save(with_lever(scenario('left'), before=False)), 'left')
    self.assertFalse(r['factory_event_neutral_lever_corroborated'])
    self.assertIsNone(r['lever_onset_context']['before_onset_ms'])

  def test_missing_post_onset_lever_prevents_neutral_corroboration(self):
    r = analyze_capture(self.save(with_lever(scenario('left'), after=False)), 'left')
    self.assertFalse(r['factory_event_neutral_lever_corroborated'])
    self.assertIsNone(r['lever_onset_context']['after_onset_ms'])

  def test_diagnostic_messages_prevent_pure_factory_corroboration(self):
    rows = with_lever(scenario('left'))
    rows.append((1.25, '0x7b1', 0, '0210030000000000'))
    rows.sort(key=lambda item:item[0])
    r = analyze_capture(self.save(rows), 'left')
    self.assertTrue(r['factory_indicator_observed_in_log'])
    self.assertFalse(r['factory_event_neutral_lever_corroborated'])
    self.assertEqual(r['aprk_request_or_reply_frames_seen_on_bus0'], 1)


if __name__ == '__main__':
  unittest.main()
