#!/usr/bin/env python3
"""No-CAN tests of the genuine EV6 factory lamp-vs-stalk evidence classifier."""
import unittest
from ev6_spas_oem_transition_test import (
  Frame, LEVER, LAMP, stalk_crc8_ok, lever_neutral,
  factory_lamp_without_stalk, transition_data, SPAS2, xmodem_crc,
)

# Exact stock frames in the user's EV6 rlog(6)/(7) CSV extracts.
STALK_NEUTRAL = bytes.fromhex("49e0300000000000")
LAMP_LEFT_ON = bytes.fromhex("6a21104100000000")
LAMP_RIGHT_ON = bytes.fromhex("02b4404100000000")


class TestFactoryLampWithoutStalk(unittest.TestCase):
  def rows(self, lever=STALK_NEUTRAL, lamp=LAMP_LEFT_ON):
    return {LEVER: [Frame(0, lever)], LAMP: [Frame(50000000, lamp)]}

  def test_factory_left_lamp_with_stalk_neutral(self):
    result = factory_lamp_without_stalk(self.rows(), "left")
    self.assertTrue(result['factory_lamp_with_all_observed_stalk_frames_neutral'])
    self.assertEqual(result['stock_stalk_active_frames'], 0)
    self.assertEqual(result['physical_lamp_on_frames'], 1)

  def test_factory_right_lamp_with_stalk_neutral(self):
    result = factory_lamp_without_stalk(self.rows(lamp=LAMP_RIGHT_ON), "right")
    self.assertTrue(result['factory_lamp_with_all_observed_stalk_frames_neutral'])

  def test_stock_crc_real_recorded_frame(self):
    self.assertTrue(stalk_crc8_ok(STALK_NEUTRAL))
    self.assertTrue(lever_neutral(STALK_NEUTRAL))

  def test_corrupted_stock_frame_not_proof(self):
    bad = bytearray(STALK_NEUTRAL)
    bad[0] ^= 0xFF
    result = factory_lamp_without_stalk(self.rows(lever=bytes(bad)), "left")
    self.assertFalse(result['factory_lamp_with_all_observed_stalk_frames_neutral'])
    self.assertEqual(result['stock_stalk_crc_invalid'], 1)

  def test_no_stalk_data_not_proof(self):
    result = factory_lamp_without_stalk({LEVER: [], LAMP: self.rows()[LAMP]}, "left")
    self.assertFalse(result['factory_lamp_with_all_observed_stalk_frames_neutral'])

  def test_no_lamp_on_not_proof(self):
    result = factory_lamp_without_stalk(self.rows(lamp=bytes(8)), "left")
    self.assertFalse(result['factory_lamp_with_all_observed_stalk_frames_neutral'])

  def test_wrong_side_lamp_not_proof(self):
    result = factory_lamp_without_stalk(self.rows(lamp=LAMP_RIGHT_ON), "left")
    self.assertFalse(result['factory_lamp_with_all_observed_stalk_frames_neutral'])

  def test_active_stalk_not_proof_even_with_valid_crc(self):
    # Valid captured left lever-tap state from user's authentic rlog(3).
    active = bytes.fromhex("3530304010000000")
    self.assertTrue(stalk_crc8_ok(active))
    result = factory_lamp_without_stalk(self.rows(lever=active), "left")
    self.assertFalse(result['factory_lamp_with_all_observed_stalk_frames_neutral'])
    self.assertEqual(result['stock_stalk_active_frames'], 1)


  def test_missing_stalk_coverage_in_lamp_window(self):
    rows = {LEVER: [Frame(0, STALK_NEUTRAL)], LAMP: [Frame(1_500_000_000, LAMP_LEFT_ON)]}
    proof = factory_lamp_without_stalk(rows, 'left')
    self.assertFalse(proof['factory_lamp_with_all_observed_stalk_frames_neutral'])
    self.assertEqual(proof['lamp_on_frames_without_neutral_stalk_within_150ms'], 1)

  def test_exact_150ms_boundary_allowed(self):
    rows = {LEVER: [Frame(0, STALK_NEUTRAL)], LAMP: [Frame(150_000_000, LAMP_LEFT_ON)]}
    proof = factory_lamp_without_stalk(rows, 'left')
    self.assertTrue(proof['factory_lamp_with_all_observed_stalk_frames_neutral'])

  def test_one_ns_beyond_boundary_blocked(self):
    rows = {LEVER: [Frame(0, STALK_NEUTRAL)], LAMP: [Frame(150_000_001, LAMP_LEFT_ON)]}
    proof = factory_lamp_without_stalk(rows, 'left')
    self.assertFalse(proof['factory_lamp_with_all_observed_stalk_frames_neutral'])

  def test_mixed_lamp_on_coverage_blocks_all(self):
    rows = {LEVER: [Frame(0, STALK_NEUTRAL)],
            LAMP: [Frame(50_000_000, LAMP_LEFT_ON), Frame(1_200_000_000, LAMP_LEFT_ON)]}
    proof = factory_lamp_without_stalk(rows, 'left')
    self.assertFalse(proof['factory_lamp_with_all_observed_stalk_frames_neutral'])
    self.assertEqual(proof['lamp_on_frames_without_neutral_stalk_within_150ms'], 1)

  def test_corrupt_stalk_frame_near_lamp_blocks(self):
    bad = bytearray(STALK_NEUTRAL)
    bad[0] ^= 0xFF
    rows = {LEVER: [Frame(0, STALK_NEUTRAL), Frame(40_000_000, bytes(bad))],
            LAMP: [Frame(50_000_000, LAMP_LEFT_ON)]}
    proof = factory_lamp_without_stalk(rows, 'left')
    self.assertFalse(proof['factory_lamp_with_all_observed_stalk_frames_neutral'])
    self.assertEqual(proof['stock_stalk_crc_invalid'], 1)


class TestFactoryLampOffToOnEvidence(unittest.TestCase):
  def command_frames(self):
    def body(value):
      dat = bytearray(32)
      dat[16] = value << 5
      dat[:2] = (xmodem_crc(dat[2:] + SPAS2.to_bytes(2, 'little')) ^ 0x9f5b).to_bytes(2, 'little')
      return bytes(dat)
    return [Frame(0, body(0)), Frame(20_000_000, body(3))]

  def lamps(self, off_at=5_000_000, on_at=40_000_000, prior_off=True):
    on = bytearray(8)
    on[2] = 0x10
    rows = [Frame(on_at, bytes(on))]
    if prior_off:
      rows.insert(0, Frame(off_at, bytes(8)))
    return rows

  def test_fresh_off_to_on_edge(self):
    proof = transition_data(self.command_frames(), self.lamps(), 'left')
    self.assertTrue(proof['lamp_feedback_off_to_on_verified_within_250ms'])
    self.assertEqual(proof['first_physical_lamp_after_ms'], 20.)

  def test_already_on_is_not_a_fresh_edge(self):
    rows = self.lamps()
    rows[0] = rows[1]
    proof = transition_data(self.command_frames(), rows, 'left')
    self.assertFalse(proof['lamp_feedback_off_to_on_verified_within_250ms'])

  def test_very_late_edge_is_not_correlated(self):
    proof = transition_data(self.command_frames(), self.lamps(on_at=320_000_000), 'left')
    self.assertFalse(proof['lamp_feedback_off_to_on_verified_within_250ms'])

  def test_no_preceding_off_sample_is_not_an_edge(self):
    proof = transition_data(self.command_frames(), self.lamps(prior_off=False), 'left')
    self.assertFalse(proof['lamp_feedback_off_to_on_verified_within_250ms'])



if __name__ == '__main__':
  unittest.main()
