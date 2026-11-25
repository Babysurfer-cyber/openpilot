"""No-CAN tests for saved-rlog signal filtering and privacy-by-exclusion."""
import csv
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from ev6_spas_rlog_extract import export_saved_rlog, extract_receive_rows


def ev(kind, ns, msgs=()):
  return SimpleNamespace(which=lambda: kind, logMonoTime=ns, can=msgs)


def msg(addr=0x165, bus=0, dat=b'\x01\x02'):
  return SimpleNamespace(address=addr, src=bus, dat=dat)


class TestReceiveOnlyExtractor(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.path = Path(self.tmp.name) / 'filtered.csv'

  def test_keeps_bus0_and_bus1_relevant_can(self):
    x = [ev('can', 3_000_000_000, [msg(0x165, 0), msg(0x35, 1)]),
         ev('can', 3_020_000_000, [msg(0x3c1, 0, b'abcdefgh')])]
    rows=list(extract_receive_rows(x))
    self.assertEqual(rows,[('0.000000000','0x165',0,'0102'),
                           ('0.000000000','0x35',1,'0102'),
                           ('0.020000000','0x3C1',0,'6162636465666768')])

  def test_ignores_sendcan_gps_camera_and_unwanted_ids(self):
    x = [ev('sendcan', 1, [msg(0x7B1)]),
         ev('gpsLocationExternal', 2, [msg(0x35)]),
         ev('roadCameraState', 3, [msg(0x165)]),
         ev('can', 4, [msg(0x456),msg(0x165)])]
    self.assertEqual(list(extract_receive_rows(x)),[('0.000000000','0x165',0,'0102')])

  def test_panda_pseudobus_and_untrusted_other_bus_ignored(self):
    rows=list(extract_receive_rows([ev('can',10,[msg(0x165,130),msg(0x165,7),msg(0x165,2)])]))
    self.assertEqual(rows,[('0.000000000','0x165',2,'0102')])

  def test_selected_ids_all_included_without_assuming_source_bus(self):
    ids=[0x35,0x175,0xa0,0x3c1,0x165,0x16a,0x413,0x7b1,0x7b9]
    rows=list(extract_receive_rows([ev('can',1,[msg(addr,0) for addr in ids])]))
    self.assertEqual([int(row[1],16) for row in rows],ids)

  def test_backward_can_timestamps_fail(self):
    with self.assertRaisesRegex(ValueError,'Nonmonotonic'):
      list(extract_receive_rows([ev('can',2,[msg()]),ev('can',1,[msg()])]))

  def test_empty_payload_rejected(self):
    with self.assertRaisesRegex(ValueError,'Empty CAN payload'):
      list(extract_receive_rows([ev('can',1,[msg(dat=b'')])]))

  def test_saved_file_contains_only_four_csv_columns(self):
    inputs=[ev('can',3_000_000,[msg(0x165,0),msg(0xa0,1)]),
            ev('sendcan',4_000_000,[msg(0x7b1,0)])]
    self.assertEqual(export_saved_rlog(inputs,self.path),2)
    with self.path.open(newline='') as f:
      data=list(csv.DictReader(f))
    self.assertEqual(set(data[0]),{'t_sec','can_id','bus','payload_hex'})
    self.assertEqual(len(data),2)
    self.assertEqual({row['can_id'] for row in data},{'0x165','0xA0'})

  def test_no_matching_receive_frames_fails_and_removes_empty_file(self):
    with self.assertRaisesRegex(ValueError,'No selected receive-side'):
      export_saved_rlog([ev('sendcan',1,[msg()])],self.path)
    self.assertFalse(self.path.exists())


if __name__ == '__main__':
  unittest.main()
