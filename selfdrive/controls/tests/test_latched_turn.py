"""Full-depth lever input denotes a turn, never an automatic lane change."""
import pytest

from cereal import log
from openpilot.selfdrive.controls.lib import desire_helper as desire
from opendbc.car.hyundai import hyundaicanfd
from selfdrive.controls.tests.test_desire_continuation import Rig, MemoryParams, LCS, LCD
from selfdrive.controls.tests.test_continuation_transport import wire_packet


@pytest.fixture(autouse=True)
def no_device_params(monkeypatch):
  monkeypatch.setattr(desire, "Params", MemoryParams)
  monkeypatch.setattr(hyundaicanfd, "Params", MemoryParams)


def eligible_turn(rig):
  rig.cs.vEgo = 15. / 3.6
  probabilities = [0.] * 8
  probabilities[log.Desire.turnLeft if rig.side == "left" else log.Desire.turnRight] = .8
  rig.md.meta.desireState = probabilities


def assert_no_lane_change(rig):
  assert rig.h.lane_change_state == LCS.off
  assert rig.h.desire not in (log.Desire.laneChangeLeft, log.Desire.laneChangeRight)
  assert rig.h.queued_lane_change == 0
  assert rig.h.handoff_blinker_hold == 0
  assert rig.h.blinker_hold_direction == LCD.none


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("speed", [15., 40., 90.])
@pytest.mark.parametrize("navigation", ["", "atc left", "fork right", "turn right"])
def test_full_latch_always_classifies_turn_and_cannot_start_lane_change(side, speed, navigation):
  rig = Rig(side)
  rig.cs.vEgo = speed / 3.6
  rig.cm.atcType = navigation
  rig.step()  # Also cover a navigation request whose type has already stabilized.
  rig.press(lever=2)
  for _ in range(15):
    assert_no_lane_change(rig)
    assert rig.h.maneuver_type == "turn"
    rig.step()


@pytest.mark.parametrize("side", ["left", "right"])
def test_high_speed_manual_latch_does_not_introduce_turn_assistance(side):
  rig = Rig(side)
  rig.cs.vEgo = 90. / 3.6
  rig.press(lever=2)
  assert_no_lane_change(rig)
  assert rig.h.maneuver_type == "turn"
  assert rig.h.desire == log.Desire.none


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("need_torque", [-1, 0, 1])
def test_existing_eligible_turn_output_is_independent_of_lane_change_torque_setting(side, need_torque):
  rig = Rig(side)
  eligible_turn(rig)
  rig.h.laneChangeNeedTorque = need_torque
  rig.press(lever=2)
  assert_no_lane_change(rig)
  assert rig.h.desire == (log.Desire.turnLeft if side == "left" else log.Desire.turnRight)


@pytest.mark.parametrize("stage", ["pre", "starting", "finishing", "queued_pre", "handoff_starting"])
def test_full_latch_clears_prepared_active_and_queued_lane_changes(stage):
  rig = Rig()
  if stage == "pre":
    assert rig.tap() == LCS.preLaneChange
  elif stage == "starting":
    rig.start()
  elif stage == "finishing":
    rig.start()
    rig.finishing()
  else:
    rig.queue()
    if stage == "queued_pre":
      rig.block_bsd()
    assert rig.finish() == LCS.preLaneChange
    if stage == "handoff_starting":
      assert rig.step() == LCS.laneChangeStarting
      assert rig.h.handoff_blinker_hold
  eligible_turn(rig)
  rig.press(lever=2)
  assert_no_lane_change(rig)
  assert rig.h.lane_change_ll_prob == 1.
  assert rig.h.lane_change_timer == 0.
  assert rig.h.desire == log.Desire.turnLeft


@pytest.mark.parametrize("navigation", ["", "atc left"])
def test_held_one_touch_detent_does_not_start_before_latch_depth_is_known(navigation):
  rig = Rig()
  rig.cm.atcType = navigation
  rig.step()
  rig.press(lever=1)
  for _ in range(16):  # A slow movement through detent 1 exceeds the old 0.3s timer.
    assert rig.h.lane_change_state not in (LCS.laneChangeStarting, LCS.laneChangeFinishing)
    assert rig.h.desire not in (log.Desire.laneChangeLeft, log.Desire.laneChangeRight)
    rig.step()
  rig.cs.blinkerLever = 2
  rig.step()
  assert_no_lane_change(rig)
  assert rig.h.maneuver_type == "turn"


def test_second_press_held_in_detent_one_cannot_start_prepared_followup():
  rig = Rig()
  rig.start()
  assert rig.finish() == LCS.preLaneChange
  rig.press(lever=1)
  for _ in range(15):
    assert rig.h.lane_change_state != LCS.laneChangeStarting
    assert not rig.h.queued_lane_change
    rig.step()
  rig.cs.blinkerLever = 2
  rig.step()
  assert_no_lane_change(rig)


@pytest.mark.parametrize("side", ["left", "right"])
def test_released_genuine_one_touch_still_starts_lane_change(side):
  rig = Rig(side)
  rig.press(lever=1)
  rig.step(15)
  assert rig.h.lane_change_state == LCS.off
  rig.cs.blinkerLever = 0
  assert rig.step() == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting


def test_releasing_a_latched_session_cannot_replay_a_tap_from_remaining_lamp():
  rig = Rig()
  rig.press(lever=2)
  rig.step(5)
  rig.cs.blinkerLever = 0
  for _ in range(30):
    assert rig.step() == LCS.off
    assert rig.h.desire == log.Desire.none
  assert rig.tap() == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting


def test_latched_turn_preserves_known_physical_side_through_a_dark_lamp_sample():
  rig = Rig()
  eligible_turn(rig)
  rig.press(lever=2)
  assert rig.h.desire == log.Desire.turnLeft
  rig.lamps("none")
  for _ in range(4):
    rig.step()
    assert_no_lane_change(rig)
    assert rig.h.desire == log.Desire.turnLeft


@pytest.mark.parametrize("side", ["left", "right"])
def test_fresh_latched_stalk_direction_wins_over_a_stale_opposite_lamp(side):
  rig = Rig(side)
  eligible_turn(rig)
  rig.lamps("right" if side == "left" else "left")
  rig.cs.blinkerLever = 2
  key = side + "BlinkerStalkCount"
  setattr(rig.cs, key, (getattr(rig.cs, key) + 1) % 256)
  rig.step()
  assert_no_lane_change(rig)
  assert rig.h.desire == (log.Desire.turnLeft if side == "left" else log.Desire.turnRight)


@pytest.mark.parametrize("lamps", ["none", "both"])
def test_unknown_latched_side_never_uses_navigation_as_its_direction(lamps):
  rig = Rig()
  rig.cm.atcType = "turn right"
  rig.cs.blinkerLever = 2
  rig.lamps(lamps)
  rig.step(5)
  assert_no_lane_change(rig)
  assert rig.h.desire == log.Desire.none
  assert rig.h.lane_change_direction == LCD.none


@pytest.mark.parametrize("gate", ["lateral", "can", "turn_disabled", "turn_timer"])
def test_turn_suppression_never_falls_back_to_lane_change(gate):
  rig = Rig()
  eligible_turn(rig)
  rig.cm.atcType = "atc left"
  rig.step()
  if gate == "lateral":
    rig.active = False
  elif gate == "can":
    rig.cs.canValid = False
  elif gate == "turn_disabled":
    rig.h.enable_turn_desires = False
  else:
    rig.h.turn_disable_count = 100
  rig.press(lever=2)
  for _ in range(10):
    assert_no_lane_change(rig)
    assert rig.h.desire == log.Desire.none
    rig.step()


@pytest.mark.parametrize("side", ["left", "right"])
def test_latched_input_withdraws_queued_and_lingering_model_probability_holds_on_wire(side):
  rig = Rig(side)
  rig.queue()
  confidence = [0.] * 8
  confidence[log.Desire.laneChangeLeft if side == "left" else log.Desire.laneChangeRight] = .95
  rig.md.meta.desireState = confidence
  assert wire_packet(rig)[1] == ((1, 0) if side == "left" else (0, 1))
  # The physical car-state update can precede the model's next 20Hz evaluation.
  rig.cs.blinkerLever = 2
  assert wire_packet(rig)[1] == (0, 0)
  rig.step()
  assert_no_lane_change(rig)
  # Native model lane-change probability can linger after the FSM withdrew it.
  assert wire_packet(rig)[1] == (0, 0)
  rig.cs.blinkerLever = 0
  rig.step()
  assert_no_lane_change(rig)
  # Releasing a full-depth gesture cannot revive the old model's hold either.
  assert wire_packet(rig)[1] == (0, 0)


@pytest.mark.parametrize("navigation", ["atc left", "fork left"])
def test_navigation_request_seen_during_latch_does_not_fire_on_release(navigation):
  rig = Rig()
  rig.press(lever=2)
  rig.cm.atcType = navigation
  rig.step()  # The type changes on the final fully latched frame.
  rig.cs.blinkerLever = 0
  for _ in range(20):
    rig.step()
    assert_no_lane_change(rig)
