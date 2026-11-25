"""Synthetic, read-only bus/source audit regressions; no vehicle CAN TX."""
import csv
import tempfile
import unittest
from pathlib import Path

from ev6_spas_bus_route_audit import summarize
from ev6_spas_onset_sequence import SPAS2, crc16


def spas2(control):
  data = bytearray(32)
  data[16] = (control & 7) << 5
  check = crc16(data[2:] + SPAS2.to_bytes(2, "little")) ^ 0x9f5b
  data[:2] = check.to_bytes(2, "little")
  return data.hex()


class TestEV6BusRouteAudit(unittest.TestCase):
  def summarize_rows(self, rows):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "synthetic.csv"
      with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=("t_sec", "can_id", "bus", "payload_hex"))
        writer.writeheader()
        for t, bus, payload in rows:
          writer.writerow({"t_sec": t, "can_id": "0x16a", "bus": bus, "payload_hex": payload})
      return summarize(path)["addresses"]["0x16a"]

  def test_valid_crc_counts_and_distinguishes_returned_source(self):
    report = self.summarize_rows([
      (0.0, 0, spas2(0)), (1.2, 0, spas2(3)),
      (1.2, 2, spas2(3)), (1.2, 130, spas2(3)),
    ])
    self.assertTrue(report["spas2_direction_inference_valid_bus0"])
    self.assertEqual(report["spas2_control_histogram_bus0"], {"0": 1, "3": 1})
    self.assertEqual(report["first_direction_command_s"], 1.2)
    self.assertEqual(report["direct_bus2_rx_during_direction_command_count"], 1)
    self.assertEqual(report["src130_forwarded_or_returned"], 1)
    self.assertEqual(report["invalid_spas2_crc_or_length_by_source"], {"0": 0, "2": 0, "130": 0})

  def test_crc_corruption_on_bus0_suppresses_direction_inference(self):
    tampered = bytearray.fromhex(spas2(0))
    tampered[16] = 3 << 5  # Direction now looks active but original CRC is wrong.
    report = self.summarize_rows([(0, 0, spas2(0)), (1, 0, tampered.hex())])
    self.assertFalse(report["spas2_direction_inference_valid_bus0"])
    self.assertEqual(report["invalid_spas2_crc_or_length_by_source"]["0"], 1)
    self.assertEqual(report["spas2_control_histogram_bus0"], {})
    self.assertIsNone(report["first_direction_command_s"])

  def test_invalid_bus2_does_not_prove_zero_direction_commands(self):
    corrupted = bytearray.fromhex(spas2(4))
    corrupted[0] ^= 0xFF
    report = self.summarize_rows([(0, 0, spas2(4)), (0, 2, corrupted.hex())])
    self.assertEqual(report["first_direction_command_s"], 0)
    self.assertEqual(report["invalid_spas2_crc_or_length_by_source"]["2"], 1)
    self.assertIsNone(report["direct_bus2_rx_during_direction_command_count"])

  def test_short_nonhex_and_invalid_crc_fail_closed_without_crashing(self):
    report = self.summarize_rows([
      (0, 0, "1234"), (0.1, 0, "gg"), (0.2, 0, "00" * 32),
    ])
    self.assertEqual(report["bus0_rx"], 3)
    self.assertEqual(report["invalid_spas2_crc_or_length_by_source"]["0"], 3)
    self.assertFalse(report["spas2_direction_inference_valid_bus0"])
    self.assertIsNone(report["first_direction_command_s"])

  def test_empty_original_bus_is_not_evidence_of_stock_direction(self):
    report = self.summarize_rows([(0, 130, spas2(3))])
    self.assertEqual(report["bus0_rx"], 0)
    self.assertEqual(report["src130_forwarded_or_returned"], 1)
    self.assertFalse(report["spas2_direction_inference_valid_bus0"])
    self.assertIsNone(report["first_direction_command_s"])
    self.assertEqual(report["direct_bus2_rx_during_direction_command_count"], 0)


if __name__ == "__main__":
  unittest.main()
