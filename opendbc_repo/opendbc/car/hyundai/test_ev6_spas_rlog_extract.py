"""No-CAN tests for saved-rlog signal filtering and privacy-by-exclusion."""
import csv
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from ev6_spas_rlog_extract import export_saved_rlog, extract_receive_rows, write_bus_summary


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


  def test_failed_midstream_export_leaves_no_partial_csv(self):
    frames=[ev('can', 2, [msg()]), ev('can', 1, [msg()])]
    with self.assertRaisesRegex(ValueError, 'Nonmonotonic'):
      export_saved_rlog(frames, self.path)
    self.assertFalse(self.path.exists())
    self.assertEqual(list(self.path.parent.glob('.ev6_can_*.tmp')), [])

  def test_failed_export_does_not_replace_existing_csv(self):
    self.path.write_text('user original data', encoding='utf-8')
    with self.assertRaisesRegex(ValueError, 'Empty CAN payload'):
      export_saved_rlog([ev('can', 1, [msg(dat=b'')])], self.path)
    self.assertEqual(self.path.read_text(encoding='utf-8'), 'user original data')
    self.assertEqual(list(self.path.parent.glob('.ev6_can_*.tmp')), [])

  def test_counts_only_summary_has_no_can_payload_or_timestamp(self):
    rows=[ev('can', 100, [msg(0x35, 1, b'secretA'),
                           msg(0x175, 1, b'secretB'),
                           msg(0xA0, 1, b'secretC'),
                           msg(0x16A, 0, b'private')])]
    export_saved_rlog(rows, self.path)
    output=self.path.with_suffix('.summary.json')
    summary=write_bus_summary(self.path, output)
    content=output.read_text(encoding='utf-8')
    self.assertEqual(summary['buses_with_all_three_guard_ids_observed'], [1])
    self.assertFalse(summary['permits_vehicle_actuation'])
    self.assertFalse(summary['panda_guard_safety_validated'])
    for secret in ('secretA', 'secretB', 'secretC', 'private', '0.000000000', 'payload_hex'):
      self.assertNotIn(secret, content)
    self.assertEqual(json.loads(content)['counts_by_bus_and_can_id']['1']['0x35'], 1)

  def test_counts_summary_from_only_one_guard_bus_is_not_permission(self):
    export_saved_rlog([ev('can',1,[msg(0x35, 1, b'1234')])], self.path)
    output=self.path.with_suffix('.summary.json')
    summary=write_bus_summary(self.path, output)
    self.assertEqual(summary['buses_with_all_three_guard_ids_observed'], [])
    self.assertFalse(summary['independent_ecu_handoff_tested'])


if __name__ == '__main__':
  unittest.main()
