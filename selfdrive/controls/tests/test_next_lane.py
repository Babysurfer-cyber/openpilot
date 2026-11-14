"""Input and crossing observations; no simulated signal constitutes steering permission."""
from types import SimpleNamespace as NS

import pytest

from cereal import car, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_lib.constants import BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT
from openpilot.selfdrive.controls.lib.desire_lib.next_lane import LaneCrossing, StalkPressTracker


def state(left=0, right=0, valid=True, lever=0):
  return NS(leftBlinkerStalkCount=left, rightBlinkerStalkCount=right, canValid=valid, blinkerLever=lever)


def model(left=-1.8, right=1.8):
  return NS(laneLines=[NS(y=[-5.4]), NS(y=[left]), NS(y=[right]), NS(y=[5.4])])


def test_first_sample_only_establishes_a_baseline():
  tracker = StalkPressTracker()
  assert tracker.update(state(104, 22)) == BLINKER_NONE
  assert tracker.update(state(104, 22)) == BLINKER_NONE
  assert tracker.update(state(105, 22)) == BLINKER_LEFT
  assert tracker.update(state(105, 23)) == BLINKER_RIGHT


@pytest.mark.parametrize("left_side,direction", [(True, BLINKER_LEFT), (False, BLINKER_RIGHT)])
def test_uint8_wrap_is_one_fresh_press(left_side, direction):
  tracker = StalkPressTracker()
  tracker.update(state(255 if left_side else 14, 14 if left_side else 255))
  assert tracker.update(state(0 if left_side else 14, 14 if left_side else 0)) == direction
  assert tracker.update(state(0 if left_side else 14, 14 if left_side else 0)) == BLINKER_NONE


@pytest.mark.parametrize("before,after", [((30, 40), (0, 40)), ((3, 1), (5, 1)),
                                           ((3, 1), (4, 2)), ((3, 1), (4, 0)), ((0, 0), (255, 0))])
def test_reset_jump_or_ambiguous_both_sides_is_not_a_press(before, after):
  tracker = StalkPressTracker()
  tracker.update(state(*before))
  assert tracker.update(state(*after)) == BLINKER_NONE
  assert tracker.update(state((after[0] + 1) % 256, after[1])) == BLINKER_LEFT


def test_invalid_can_consumes_the_baseline_without_queuing_the_old_event():
  tracker = StalkPressTracker()
  tracker.update(state(4, 9))
  assert tracker.update(state(5, 9, valid=False)) == BLINKER_NONE
  assert tracker.update(state(5, 9)) == BLINKER_NONE
  assert tracker.update(state(6, 9)) == BLINKER_LEFT


@pytest.mark.parametrize("value", [-1, 256, 1.5, "1", None, True, float("nan")])
def test_invalid_counter_reestablishes_a_baseline(value):
  tracker = StalkPressTracker()
  tracker.update(state(4, 9))
  assert tracker.update(state(value, 9)) == BLINKER_NONE
  assert tracker.update(state(5, 9)) == BLINKER_NONE
  assert tracker.update(state(6, 9)) == BLINKER_LEFT


def test_missing_counter_attributes_do_not_synthesize_a_press():
  tracker = StalkPressTracker()
  assert tracker.update(NS()) == BLINKER_NONE
  assert tracker.update(NS(leftBlinkerStalkCount=4)) == BLINKER_NONE
  assert tracker.update(state(5, 9)) == BLINKER_NONE


def test_stalk_depth_is_not_used_as_an_automatic_permission():
  tracker = StalkPressTracker()
  assert tracker.update(state(0, 0, lever=2)) == BLINKER_NONE
  assert tracker.update(state(0, 0, lever=1)) == BLINKER_NONE
  # This is a raw physical press; the caller must still classify tap/latch.
  assert tracker.update(state(1, 0, lever=2)) == BLINKER_LEFT


def test_tracker_reset_does_not_replay_a_count_already_seen():
  tracker = StalkPressTracker()
  tracker.update(state(4, 9))
  tracker.reset()
  assert tracker.update(state(5, 9)) == BLINKER_NONE
  assert tracker.update(state(6, 9)) == BLINKER_LEFT


@pytest.mark.parametrize("left_side", [True, False])
def test_relabel_crossing_and_elapsed_time(left_side):
  crossing = LaneCrossing()
  sequence = [(-1.8, 1.8), (-0.28, 3.32), (-1.36, 2.24), (-3.46, 0.14)]
  if not left_side:
    sequence = [(-r, -l) for l, r in sequence]
  assert [crossing.update(model(l, r), left_side) for l, r in sequence] == [-1., -1., -1., 0.]
  assert crossing.update(model(), left_side) == pytest.approx(DT_MDL)
  assert crossing.update(model(), left_side) == pytest.approx(2 * DT_MDL)


def test_relabel_uses_closest_approach_across_several_frames():
  crossing = LaneCrossing()
  for target in [-0.3, -0.9, -1.5]:
    assert crossing.update(model(target, target + 3.6), True) == -1.
  # The previous-frame difference is only 0.6m, but the closest-to-current
  # difference is 1.8m, matching the source's multi-frame relabel behavior.
  assert crossing.update(model(-2.1, 1.5), True) == 0.


def test_large_jump_without_near_approach_is_not_a_crossing():
  crossing = LaneCrossing()
  assert crossing.update(model(-1.1, 2.5), True) == -1.
  assert crossing.update(model(-3.4, 0.2), True) == -1.


def test_near_and_jump_thresholds_remain_strict():
  crossing = LaneCrossing()
  crossing.update(model(-1., 2.6), True)
  assert crossing.update(model(-3., .6), True) == -1.
  crossing.reset()
  crossing.update(model(-.25, 3.35), True)
  assert crossing.update(model(-1.75, 1.85), True) == -1.
  assert crossing.update(model(-1.76, 1.84), True) == 0.


@pytest.mark.parametrize("bad", [
  None, NS(), NS(laneLines=[]), NS(laneLines=[NS(y=[0.])]),
  NS(laneLines=[NS(y=[]), NS(y=[]), NS(y=[3.])]),
  NS(laneLines=[NS(y=[]), NS(y=None), NS(y=[3.])]),
  NS(laneLines=[NS(y=[]), NS(y=[[0.]]), NS(y=[3.])]),
  model(float("nan"), 3.4), model(-.2, float("inf")), model("-.2", 3.4), model(False, 3.4),
  model(2., -2.), model(-.2, .3), model(-.2, 7.), model(-9., -5.4),
])
def test_bad_geometry_cannot_confirm_or_bridge_an_unconfirmed_crossing(bad):
  crossing = LaneCrossing()
  assert crossing.update(model(-.2, 3.4), True) == -1.
  assert crossing.update(bad, True) == -1.
  assert crossing.closest is None
  assert crossing.update(model(-3.4, .2), True) == -1.


def test_confirmed_crossing_stays_latched_until_explicit_reset():
  crossing = LaneCrossing()
  crossing.update(model(-.2, 3.4), True)
  assert crossing.update(model(-3.4, .2), True) == 0.
  assert crossing.update(None, True) == pytest.approx(DT_MDL)
  crossing.reset()
  assert crossing.since == -1. and crossing.closest is None
  assert crossing.update(model(-3.4, .2), True) == -1.


def test_real_cereal_types_match_the_observer_contract():
  tracker = StalkPressTracker()
  cs = car.CarState.new_message(canValid=True)
  assert tracker.update(cs) == BLINKER_NONE
  cs.rightBlinkerStalkCount = 1
  assert tracker.update(cs) == BLINKER_RIGHT
  md = log.ModelDataV2.new_message()
  md.init("laneLines", 4)
  for line in md.laneLines:
    line.y = [0.] * 33
  crossing = LaneCrossing()
  md.laneLines[1].y = [-.2] * 33
  md.laneLines[2].y = [3.4] * 33
  assert crossing.update(md, True) == -1.
  md.laneLines[1].y = [-3.4] * 33
  md.laneLines[2].y = [.2] * 33
  assert crossing.update(md, True) == 0.
