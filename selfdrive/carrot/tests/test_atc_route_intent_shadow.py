"""Read-only observer tests: no ATC target or vehicle control is changed."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atc_route_intent_shadow import AtcRouteIntentShadow, mismatch_evidence, REQUIRED_STABLE_FRAMES


def fixture(turn=2, model_turn=False, lane_prob=.99, lever=0, lamp=False, route=True, preview=150.0):
  def loc(north, east):
    return (37.0 + north / 111195, 127.0 + east / (111195 * math.cos(math.radians(37))))
  points = [loc(y, 0) for y in range(0, 41, 5)] + [loc(40, x * (-1 if turn == 1 else 1)) for x in range(5, 91, 5)]
  if not route:
    points = points[:3]
  xs = [i * preview / 32 for i in range(33)]
  ys = [0. if not model_turn else (0 if x < 40 else (-x + 40) / 3) for x in xs]
  lines = [NS(x=xs, y=[offset for _ in xs]) for offset in (5.4, 1.8, -1.8, -5.4)]
  navi = NS(session_id="sample", current=NS(present=True, sequence=3),
            route=NS(present=True, polyline=points),
            vehicle=NS(present=True, latitude=37., longitude=127., heading_deg=0.),
            guidance_active=True, off_route=False)
  car = NS(blinkerLever=lever, leftBlinker=lamp, rightBlinker=False)
  model = NS(meta=NS(laneChangeState="off"), position=NS(x=xs, y=ys),
             laneLines=lines, laneLineProbs=[.99, lane_prob, lane_prob, .99])
  return navi, car, model, turn, 40.0


class TestAtcRouteIntentShadow(unittest.TestCase):
  def test_candidate_is_shadow_only_after_twenty_frames(self):
    navi, car, model, turn, d = fixture()
    self.assertEqual(mismatch_evidence(navi, car, model, turn, d), "strong_route_model_disagreement")
    observer = AtcRouteIntentShadow()
    for _ in range(REQUIRED_STABLE_FRAMES - 1):
      self.assertEqual(observer.observe(navi, car, model, turn, d).state, "keep_atc")
    self.assertEqual(observer.observe(navi, car, model, turn, d).state, "candidate_only")

  def test_left_turn_geometry(self):
    self.assertEqual(mismatch_evidence(*fixture(turn=1)), "strong_route_model_disagreement")

  def test_one_touch_lane_change_vetoes(self):
    self.assertEqual(mismatch_evidence(*fixture(lever=1)), "stalk_or_lamp_active")

  def test_full_stalk_latch_vetoes(self):
    self.assertEqual(mismatch_evidence(*fixture(lever=2)), "stalk_or_lamp_active")

  def test_lamp_after_stalk_vetoes(self):
    self.assertEqual(mismatch_evidence(*fixture(lamp=True)), "stalk_or_lamp_active")

  def test_lane_confidence_low_vetoes(self):
    self.assertEqual(mismatch_evidence(*fixture(lane_prob=.2)), "lane_not_confident")

  def test_model_preview_short_vetoes(self):
    self.assertEqual(mismatch_evidence(*fixture(preview=35)), "model_not_far_enough")

  def test_route_truncated_vetoes(self):
    self.assertNotEqual(mismatch_evidence(*fixture(route=False)), "strong_route_model_disagreement")

  def test_active_lane_change_vetoes(self):
    navi, car, model, turn, d = fixture()
    model.meta.laneChangeState = "preLaneChange"
    self.assertEqual(mismatch_evidence(navi, car, model, turn, d), "lane_change_active")

  def test_missing_and_offroute_vetoes(self):
    navi, car, model, turn, d = fixture()
    self.assertEqual(mismatch_evidence(None, car, model, turn, d), "unavailable")
    navi.off_route = True
    self.assertEqual(mismatch_evidence(navi, car, model, turn, d), "unavailable")

  def test_signal_clears_candidate_latch(self):
    navi, car, model, turn, d = fixture()
    observer = AtcRouteIntentShadow()
    for _ in range(REQUIRED_STABLE_FRAMES):
      result = observer.observe(navi, car, model, turn, d)
    self.assertEqual(result.state, "candidate_only")
    car.blinkerLever = 2
    self.assertEqual(observer.observe(navi, car, model, turn, d).state, "keep_atc")
    car.blinkerLever = 0
    self.assertEqual(observer.observe(navi, car, model, turn, d).consecutive_frames, 1)

  def test_new_guidance_resets_candidate_latch(self):
    navi, car, model, turn, d = fixture()
    observer = AtcRouteIntentShadow()
    for _ in range(REQUIRED_STABLE_FRAMES):
      observer.observe(navi, car, model, turn, d)
    navi.current.sequence += 1
    self.assertEqual(observer.observe(navi, car, model, turn, d).consecutive_frames, 1)


if __name__ == "__main__":
  unittest.main()
