import unittest

from selfdrive.carrot.atc_one_shot_skip import AtcOneShotSkip, fresh_atc_skip_request, should_request_atc_skip


class TestAtcOneShotSkip(unittest.TestCase):
  def setUp(self):
    self.skip = AtcOneShotSkip()
    self.key = ("carrotNavi", "session-1", 10, 13)
    self.base = dict(enabled=True, cruise_active=True, turn_type=2,
                     distance_m=180, guide_distance_m=200, atc_type="atc right",
                     deceleration_started=False)
    self.assertFalse(self.skip.observe(self.key, 2, 180))

  def test_accepted_once_and_persists_while_approaching(self):
    self.assertTrue(self.skip.request(**self.base))
    self.assertFalse(self.skip.request(**self.base))
    for d in (170, 120, 30, 0, -20):
      self.assertTrue(self.skip.observe(self.key, 2, d))
    self.assertFalse(self.skip.request(**self.base))
    self.assertFalse(self.skip.observe(self.key, 2, -51))
    self.assertFalse(self.skip.skipped)

  def test_next_maneuver_new_sequence_resets(self):
    self.assertTrue(self.skip.request(**self.base))
    self.assertFalse(self.skip.observe(("carrotNavi", "session-1", 11, 13), 2, 290))
    self.assertTrue(self.skip.request(**self.base))

  def test_new_session_resets(self):
    self.assertTrue(self.skip.request(**self.base))
    self.assertFalse(self.skip.observe(("carrotNavi", "session-2", 10, 13), 2, 180))

  def test_legacy_same_text_next_turn_resets_on_distance_jump(self):
    legacy = ("legacy", 2, 13, "우회전", "", "")
    self.skip.observe(legacy, 2, 180)
    self.assertTrue(self.skip.request(**self.base))
    self.assertTrue(self.skip.observe(legacy, 2, 30))
    self.assertFalse(self.skip.observe(legacy, 2, 190))
    self.assertTrue(self.skip.request(**self.base))

  def test_distance_noise_does_not_reset(self):
    self.assertTrue(self.skip.request(**self.base))
    self.assertTrue(self.skip.observe(self.key, 2, 150))
    self.assertTrue(self.skip.observe(self.key, 2, 200))  # +50m, no new maneuver

  def test_skip_requires_active_guide_not_actual_decel(self):
    for change in (
      {"enabled": False}, {"cruise_active": False},
      {"turn_type": 5}, {"distance_m": -1}, {"distance_m": 0},
      {"distance_m": 201}, {"guide_distance_m": 0},
      {"atc_type": "atc right prepare"}, {"atc_type": "atc right canceled"},
      {"atc_type": "none"}, {"deceleration_started": True},
    ):
      case = dict(self.base, **change)
      self.assertFalse(self.skip.request(**case), msg=str(change))
    self.assertTrue(self.skip.request(**dict(self.base, distance_m=200)))

  def test_transient_navigation_gap_keeps_same_maneuver_skipped(self):
    self.assertTrue(self.skip.request(**self.base))
    self.assertFalse(self.skip.observe(None, -1, 0))
    self.assertTrue(self.skip.skipped)
    self.assertTrue(self.skip.observe(self.key, 2, 170))
    self.assertFalse(self.skip.observe(("carrotNavi", "new-session", 1, 13), 2, 280))
    self.assertFalse(self.skip.skipped)

  def test_explicit_reset(self):
    self.assertTrue(self.skip.request(**self.base))
    self.skip.reset()
    self.assertFalse(self.skip.skipped)
    self.assertIsNone(self.skip.guidance_key)

  def test_no_nan_inf(self):
    for value in (float("nan"), float("inf"), -float("inf")):
      self.assertFalse(self.skip.request(**dict(self.base, distance_m=value)))
      self.assertFalse(self.skip.request(**dict(self.base, guide_distance_m=value)))

  def test_only_short_left_release_in_mode_4_requests_skip(self):
    base = dict(paddle_mode=4, button_type=1, left_paddle_type=1,
                long_pressed=False, cruise_enabled=True, cruise_available=True)
    self.assertTrue(should_request_atc_skip(**base))
    for override in (
      {"paddle_mode": 0}, {"paddle_mode": 1}, {"paddle_mode": 2},
      {"paddle_mode": 3}, {"button_type": 2}, {"button_type": 0},
      {"long_pressed": True}, {"cruise_enabled": False}, {"cruise_available": False},
    ):
      with self.subTest(override=override):
        self.assertFalse(should_request_atc_skip(**dict(base, **override)))

  def test_request_timestamp_freshness_deduplication(self):
    now = 9_000_000_000
    self.assertTrue(fresh_atc_skip_request(str(now - 10_000_000), None, now))
    self.assertFalse(fresh_atc_skip_request(str(now - 10_000_000), str(now - 10_000_000), now))
    self.assertTrue(fresh_atc_skip_request(str(now - 1_000_000_000), None, now))
    self.assertFalse(fresh_atc_skip_request(str(now - 1_000_000_001), None, now))
    self.assertFalse(fresh_atc_skip_request(str(now + 1), None, now))
    self.assertFalse(fresh_atc_skip_request("not-an-int", None, now))
    self.assertFalse(fresh_atc_skip_request(None, None, now))


if __name__ == "__main__":
  unittest.main()
