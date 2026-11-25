"""Proof that saved-CAN-log evidence can never be a vehicle TX authorization."""
import csv
from pathlib import Path
import tempfile
import unittest

from ev6_spas_evidence_gate import inspect_saved_capture
from test_ev6_spas_factory_replay import frame, sample_lever


class TestEvidenceGate(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.file = Path(self.temp.name) / 'capture.csv'

  def capture(self, rows):
    with self.file.open('w', newline='', encoding='utf-8') as out:
      writer = csv.writer(out)
      writer.writerow(('t_sec', 'can_id', 'bus', 'payload_hex'))
      writer.writerows(sorted(rows, key=lambda x:x[0]))
    return self.file

  def baseline(self):
    return [(1.0, '0x165', 0, frame(0x165, 1)),
            (1.0, '0x16a', 0, frame(0x16A, 1)),
            (1.0, '0x3c1', 0, sample_lever()),
            (1.0, '0x413', 0, frame(0x413))]

  def test_factory_frames_cannot_authorize_external_transmit(self):
    r = inspect_saved_capture(self.capture(self.baseline()))
    self.assertEqual(r['verdict'], 'NO_GO_FOR_VEHICLE_CAN_TRANSMIT')
    self.assertEqual(r['transmitted_by_tool'], 0)
    self.assertEqual(r['park_guard_missing_bus0_identifiers'], ['0x35','0x175','0xA0'])
    self.assertEqual(r['neutral_0x3c1_samples'], 1)

  def test_mirrored_other_bus_does_not_fake_park_inputs(self):
    rows = self.baseline() + [(1.1, '0x35', 2, '00'*32),
                              (1.1, '0x175', 2, '00'*24),
                              (1.1, '0xa0', 2, '00'*24)]
    r = inspect_saved_capture(self.capture(rows))
    self.assertEqual(len(r['park_guard_missing_bus0_identifiers']), 3)
    self.assertEqual(r['ignored_nonzero_bus_records'], 3)

  def test_full_id_presence_even_with_aprk_is_not_hardware_proof(self):
    rows = self.baseline() + [(1.1, '0x35', 0, '00'*32),
                              (1.1, '0x175', 0, '00'*24),
                              (1.1, '0xa0', 0, '00'*24),
                              (1.1, '0x7b1', 0, '0210030000000000'),
                              (1.2, '0x7b9', 0, '0250030000000000')]
    r = inspect_saved_capture(self.capture(rows))
    self.assertEqual(r['park_guard_missing_bus0_identifiers'], [])
    self.assertEqual(r['verdict'], 'NO_GO_FOR_VEHICLE_CAN_TRANSMIT')
    self.assertFalse(r['evidence_is_authenticated'])
    self.assertIn('INDEPENDENT_ISOLATED_ECU_HANDOFF_NOT_VERIFIED',r['blockers'])

  def test_bad_spas_crc_is_reported_as_blocker(self):
    rows = self.baseline()
    broken=bytearray.fromhex(rows[1][3]);broken[18]^=0x10
    rows[1]=(1.0,'0x16a',0,broken.hex())
    r=inspect_saved_capture(self.capture(rows))
    self.assertEqual(r['invalid_spas_crc'],1)
    self.assertIn('CAPTURE_CRC_OR_LENGTH_ERRORS',r['blockers'])

  def test_invalid_lever_crc_is_reported_not_neutral(self):
    rows=self.baseline()
    bad=bytearray.fromhex(rows[2][3]);bad[0]^=1
    rows[2]=(1.0,'0x3c1',0,bad.hex())
    r=inspect_saved_capture(self.capture(rows))
    self.assertEqual(r['neutral_0x3c1_samples'],0)
    self.assertEqual(r['invalid_stalk_crc_or_length'],1)

  def test_active_driver_stalk_is_not_neutral(self):
    rows=self.baseline()
    rows[2]=(1.0,'0x3c1',0,sample_lever(active=True))
    r=inspect_saved_capture(self.capture(rows))
    self.assertEqual(r['nonneutral_0x3c1_samples'],1)
    self.assertEqual(r['neutral_0x3c1_samples'],0)

  def test_missing_spas_blocks_factory_stream_evidence(self):
    rows=[x for x in self.baseline() if x[1]!='0x165']
    r=inspect_saved_capture(self.capture(rows))
    self.assertIn('NO_COMPLETE_BUS0_FACTORY_SPAS1_SPAS2_EVIDENCE',r['blockers'])

  def test_bad_lamp_length_fails_evidence(self):
    rows=self.baseline()
    rows[3]=(1.0,'0x413',0,'0000')
    r=inspect_saved_capture(self.capture(rows))
    self.assertEqual(r['invalid_lamp_length'],1)
    self.assertIn('CAPTURE_CRC_OR_LENGTH_ERRORS',r['blockers'])

  def test_empty_capture_rejected(self):
    with self.assertRaisesRegex(ValueError,'No capture events'):
      inspect_saved_capture(self.capture([]))


if __name__ == '__main__':
  unittest.main()
