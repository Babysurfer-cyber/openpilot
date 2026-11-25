"""Synthetic read-only EV6 factory onset classifier tests; no vehicle CAN."""
import unittest
from ev6_spas_onset_sequence import SPAS2, LAMP, crc16, analyze, check_spas2


def sample(t, control=0, b7=0x82, b18=0):
  d = bytearray(32)
  d[7] = b7
  d[16] = control << 5
  d[18] = b18
  d[:2] = (crc16(d[2:] + SPAS2.to_bytes(2, 'little')) ^ 0x9f5b).to_bytes(2, 'little')
  return (t, bytes(d))


def lamp(t, on=False, side='left'):
  d = bytearray(8)
  d[2] = (0x10 if side == 'left' else 0x40) if on else 0
  return (t, bytes(d))


class TestFactoryOnset(unittest.TestCase):
  def left(self):
    return {SPAS2: [sample(0.), sample(.02), sample(.04, 3, 1, 64), sample(.06, 3, 1, 64)],
            LAMP: [lamp(.03), lamp(.05, True)]}

  def right(self):
    return {SPAS2: [sample(0.), sample(.02, 0, 1, 64), sample(.04, 4, 1, 64)],
            LAMP: [lamp(.03, side='right'), lamp(.05, True, side='right')]}

  def test_left_state_changes_same_frame(self):
    r = analyze(self.left(), 'left')
    self.assertEqual(r['raw_changed_offsets_excluding_crc_counter'], [7, 16, 18])
    self.assertEqual(r['raw_states']['byte_7']['last_transition']['lead_before_direction_ms'], 0)
    self.assertEqual(r['first_lamp_on_delay_ms'], 10)

  def test_right_state_leads_direction(self):
    r = analyze(self.right(), 'right')
    self.assertEqual(r['raw_changed_offsets_excluding_crc_counter'], [16])
    self.assertEqual(r['raw_states']['byte_7']['last_transition']['lead_before_direction_ms'], 20)
    self.assertEqual(r['raw_states']['byte_18']['last_transition']['lead_before_direction_ms'], 20)

  def test_missing_lamp_edge_is_rejected(self):
    r = self.left()
    r[LAMP] = [lamp(.03, True), lamp(.05, True)]
    with self.assertRaises(ValueError):
      analyze(r, 'left')

  def test_delayed_lamp_is_rejected(self):
    r = self.left()
    r[LAMP] = [lamp(.03), lamp(.41, True)]
    with self.assertRaises(ValueError):
      analyze(r, 'left')

  def test_missing_state_change_is_reported_not_inferred(self):
    r = self.left()
    r[SPAS2] = [sample(0., b7=1, b18=64), sample(.02, b7=1, b18=64), sample(.04, 3, 1, 64)]
    proof = analyze(r, 'left')
    self.assertIsNone(proof['raw_states']['byte_7']['last_transition'])

  def test_other_side_direction_is_not_accepted(self):
    with self.assertRaises(ValueError):
      analyze(self.left(), 'right')

  def test_crc_valid_for_synthetic_frame(self):
    self.assertTrue(check_spas2(sample(0)[1]))
    d = bytearray(sample(0)[1])
    d[16] ^= 0x20
    self.assertFalse(check_spas2(d))


if __name__ == '__main__':
  unittest.main()
