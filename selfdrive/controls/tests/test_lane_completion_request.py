"""Keep a distinct released stalk tap across the first maneuver's completion."""
import pytest

from cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib import desire_helper as desire
from openpilot.selfdrive.controls.tests.test_desire_continuation import MemoryParams, Rig, LCS, LCD


@pytest.fixture(autouse=True)
def no_device_params(monkeypatch):
  monkeypatch.setattr(desire, "Params", MemoryParams)


class CompletionRig(Rig):
  def __init__(self, side):
    self.starts = 0
    super().__init__(side)

  def step(self, frames=1):
    for _ in range(frames):
      before = self.h.lane_change_state
      result = super().step()
      if before != LCS.laneChangeStarting and result == LCS.laneChangeStarting:
        self.starts += 1
    return result


def final_finishing_frame(rig, crossed=False):
  rig.start()
  if crossed:
    rig.cross()
  rig.finishing()
  while rig.h.lane_change_ll_prob < .949:
    assert rig.step() == LCS.laneChangeFinishing
  assert rig.h.lane_change_ll_prob == pytest.approx(.95)


def finish_current(rig):
  rig.probability = 0.
  for _ in range(100):
    state = rig.step()
    if state in (LCS.off, LCS.preLaneChange):
      return state
  raise AssertionError("maneuver did not finish")


@pytest.mark.parametrize("side", ["left", "right"])
def test_released_tap_on_completion_survives_without_crossing_history(side):
  rig = CompletionRig(side)
  final_finishing_frame(rig)
  assert rig.h.crossing.since < 0.
  assert rig.press() == LCS.preLaneChange
  assert rig.h.queued_lane_change == rig.number
  assert rig.h.blinker_hold_direction == rig.direction
  assert rig.h.desire == log.Desire.none
  assert rig.step() == LCS.laneChangeStarting
  assert rig.starts == 2
  assert rig.h.queued_lane_change == 0
  rig.probability = .95
  assert rig.step() == LCS.laneChangeStarting
  assert rig.h.blinker_hold_direction == rig.direction
  assert finish_current(rig) == LCS.off
  assert rig.step(60) == LCS.off
  assert rig.starts == 2


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("crossed", [False, True])
def test_completion_tap_waits_for_its_lamp_without_losing_the_request(side, crossed):
  rig = CompletionRig(side)
  final_finishing_frame(rig, crossed)
  rig.lamps("none")
  counter = side + "BlinkerStalkCount"
  setattr(rig.cs, counter, (getattr(rig.cs, counter) + 1) % 256)
  assert rig.step() == LCS.preLaneChange
  assert rig.h.queued_lane_change == rig.number
  assert rig.h.blinker_hold_direction == rig.direction
  assert rig.step() == LCS.preLaneChange
  assert rig.starts == 1
  rig.lamps(side)
  assert rig.step() == LCS.laneChangeStarting
  assert rig.starts == 2


@pytest.mark.parametrize("side", ["left", "right"])
def test_held_tap_straddling_completion_still_starts_after_release(side):
  rig = CompletionRig(side)
  final_finishing_frame(rig)
  assert rig.press(lever=1) == LCS.off
  assert rig.h.queued_lane_change == 0
  rig.cs.blinkerLever = 0
  assert rig.step() == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting
  assert rig.starts == 2


@pytest.mark.parametrize("side", ["left", "right"])
def test_single_tap_completion_and_residual_lamps_cannot_start_another_lane(side):
  rig = CompletionRig(side)
  rig.start()
  assert finish_current(rig) == LCS.off
  assert rig.step(60) == LCS.off
  rig.lamps("none")
  rig.step(10)
  rig.lamps(side)
  assert rig.step(60) == LCS.off
  assert rig.starts == 1
  assert rig.h.queued_lane_change == 0


@pytest.mark.parametrize("side", ["left", "right"])
def test_pending_completion_rechecks_bsd_then_starts_once_when_it_clears(side):
  rig = CompletionRig(side)
  final_finishing_frame(rig)
  rig.block_bsd()
  assert rig.press() == LCS.preLaneChange
  assert rig.step(5) == LCS.preLaneChange
  assert rig.starts == 1
  rig.block_bsd(False)
  for _ in range(30):
    if rig.step() == LCS.laneChangeStarting:
      break
  assert rig.h.lane_change_state == LCS.laneChangeStarting
  assert rig.starts == 2
  assert finish_current(rig) == LCS.off
  assert rig.step(30) == LCS.off


@pytest.mark.parametrize("side", ["left", "right"])
def test_early_reservation_keeps_original_age_across_completion_and_expires(side):
  rig = CompletionRig(side)
  rig.start()
  rig.block_bsd()
  assert rig.press() == LCS.laneChangeStarting
  assert rig.h.queued_lane_change == rig.number
  assert finish_current(rig) == LCS.preLaneChange
  age = rig.h.queued_wait
  assert 1.0 < age < desire.QUEUED_LANE_WAIT_S
  assert rig.step() == LCS.preLaneChange
  assert rig.h.queued_wait == pytest.approx(age + DT_MDL)
  while rig.h.queued_lane_change:
    rig.step()
  assert rig.h.queued_wait <= desire.QUEUED_LANE_WAIT_S + DT_MDL + 1e-6
  rig.block_bsd(False)
  rig.cs.steeringPressed = True
  rig.cs.steeringTorque = 1. if side == "left" else -1.
  rig.step(40)
  assert rig.starts == 1
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("fault", ["hazards", "opposite", "latch", "lateral", "can"])
def test_completion_reservation_cannot_replay_after_cancellation(side, fault):
  rig = CompletionRig(side)
  final_finishing_frame(rig)
  rig.block_bsd()
  assert rig.press() == LCS.preLaneChange
  if fault == "hazards":
    rig.lamps("both")
  elif fault == "opposite":
    rig.press("right" if side == "left" else "left", lever=1)
  elif fault == "latch":
    rig.cs.blinkerLever = 2
  elif fault == "lateral":
    rig.active = False
  elif fault == "can":
    rig.cs.canValid = False
  rig.step()
  assert rig.h.queued_lane_change == 0
  rig.lamps(side)
  rig.cs.blinkerLever = 0
  rig.active = rig.cs.canValid = True
  rig.block_bsd(False)
  rig.step(40)
  assert rig.starts == 1


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("gate", ["solid", "no_next_lane", "torque"])
def test_completion_reservation_does_not_bypass_current_start_gates(side, gate):
  rig = CompletionRig(side)
  final_finishing_frame(rig)
  rig.block_bsd()
  assert rig.press() == LCS.preLaneChange
  if gate == "solid":
    rig.h.laneLineCheck = 2
    setattr(rig.cs, side + "LaneLine", 21)
  elif gate == "no_next_lane":
    rig.geometry(adjacent=False)
  elif gate == "torque":
    rig.h.laneChangeNeedTorque = 1
  rig.step(22)
  rig.block_bsd(False)
  assert rig.step(25) == LCS.preLaneChange
  assert rig.starts == 1
