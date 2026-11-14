"""Driver continuation requests through the target FSM and current safety gates."""
from pathlib import Path
import subprocess
from types import ModuleType

import pytest

from cereal import car, log, custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib import desire_helper as desire


LCS, LCD = log.LaneChangeState, log.LaneChangeDirection


class MemoryParams:
  def get_int(self, key):
    return 0

  def get_float(self, key):
    return 0.


class Rig:
  CROSSING = [(-1.3, 1.9), (-.9, 2.2), (-.5, 2.7), (-.34, 2.84), (-.41, 2.83), (-2.5, .47), (-2.94, .16)]

  def __init__(self, side="left", module=desire):
    self.side = side
    self.direction = LCD.left if side == "left" else LCD.right
    self.number = 1 if side == "left" else 2
    self.h = module.DesireHelper()
    self.h._update_params_periodic = lambda: None
    self.cs = car.CarState.new_message(canValid=True, vEgo=25., aEgo=0.)
    self.md = log.ModelDataV2.new_message()
    self.md.init("laneLines", 4)
    self.md.init("roadEdges", 2)
    self.md.laneLineProbs = [.95] * 4
    self.md.orientationRate.z = [0.] * 33
    self.md.meta.desireState = [0.] * 8
    self.cm = custom.CarrotMan.new_message()
    self.radar = log.RadarState.new_message()
    self.probability = .5
    self.active = True
    self.geometry()
    self.step(30)

  @property
  def selected(self):
    return self.h.left if self.side == "left" else self.h.right

  def geometry(self, left=-1.8, right=1.8, adjacent=True):
    outer_left, outer_right = left - 3.6, right + 3.6
    edges = [-12., 12.]
    if not adjacent:
      if self.side == "left":
        outer_left, edges[0] = left, left - .2
      else:
        outer_right, edges[1] = right, right + .2
    for line, y in zip(self.md.laneLines, (outer_left, left, right, outer_right), strict=True):
      line.y = [y] * 33
    for edge, y in zip(self.md.roadEdges, edges, strict=True):
      edge.y = [y] * 33

  def lamps(self, side=None):
    side = self.side if side is None else side
    self.cs.leftBlinker = side in ("left", "both")
    self.cs.rightBlinker = side in ("right", "both")

  def step(self, frames=1):
    for _ in range(frames):
      self.h.update(self.cs, self.md, self.active, self.probability, self.cm, self.radar)
    return self.h.lane_change_state

  def press(self, side=None, lever=0):
    side = self.side if side is None else side
    self.lamps(side)
    key = side + "BlinkerStalkCount"
    setattr(self.cs, key, (getattr(self.cs, key) + 1) % 256)
    self.cs.blinkerLever = lever
    return self.step()

  def tap(self, side=None):
    self.press(side, lever=1)
    self.cs.blinkerLever = 0
    return self.step()

  def start(self):
    assert self.tap() == LCS.preLaneChange
    assert self.step() == LCS.laneChangeStarting
    assert self.h.lane_change_direction == self.direction

  def cross(self):
    for left, right in self.CROSSING:
      if self.side == "right":
        left, right = -right, -left
      self.geometry(left, right)
      assert self.step() == LCS.laneChangeStarting
    assert self.h.crossing.since >= 0.

  def queue(self):
    self.start()
    self.cross()
    assert self.press() == LCS.laneChangeStarting
    assert self.h.queued_lane_change == self.number
    assert self.h.blinker_hold_direction == self.direction

  def finishing(self):
    self.probability = 0.
    for _ in range(20):
      if self.step() == LCS.laneChangeFinishing:
        return
    raise AssertionError("did not finish starting phase")

  def finish(self):
    self.finishing()
    for _ in range(30):
      if self.step() != LCS.laneChangeFinishing:
        return self.h.lane_change_state
    raise AssertionError("did not finish lane change")

  def block_bsd(self, value=True):
    setattr(self.cs, self.side + "Blindspot", value)


@pytest.fixture(autouse=True)
def no_device_params(monkeypatch):
  monkeypatch.setattr(desire, "Params", MemoryParams)


@pytest.mark.parametrize("side", ["left", "right"])
def test_second_tap_queues_one_change_after_crossing_and_waits_for_first_completion(side):
  rig = Rig(side)
  rig.queue()
  rig.step(5)
  assert rig.h.lane_change_state == LCS.laneChangeStarting
  assert rig.h.lane_change_direction == rig.direction
  assert rig.finish() == LCS.preLaneChange
  assert not rig.h.next_lane_change
  assert rig.step() == LCS.laneChangeStarting
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == rig.direction
  # Queue consumption does not extinguish the signal while the next model
  # probability ramps. It releases after that maneuver's own crossing.
  rig.geometry()
  rig.step()
  assert rig.h.blinker_hold_direction == rig.direction
  rig.cross()
  assert rig.h.blinker_hold_direction == LCD.none
  # Continuing to signal is not a third explicit request.
  assert rig.finish() == LCS.preLaneChange
  assert rig.h.next_lane_change
  assert rig.step(30) == LCS.preLaneChange


@pytest.mark.parametrize("side", ["left", "right"])
def test_early_extra_tap_does_not_request_another_lane(side):
  rig = Rig(side)
  rig.start()
  rig.press()
  assert rig.h.queued_lane_change == 0
  assert rig.finish() == LCS.preLaneChange
  assert rig.h.next_lane_change
  assert rig.step(30) == LCS.preLaneChange


def test_finishing_guard_can_accept_a_request_when_crossing_geometry_is_missing():
  rig = Rig()
  rig.start()
  rig.md.laneLines[1].y = []
  rig.md.laneLines[2].y = []
  # Keep target safety geometry from valid inputs while independently testing
  # the crossing observer's absent-geometry fallback.
  rig.h._process_sides = lambda *args: None
  rig.finishing()
  rig.step(round((desire.STALK_QUEUE_GUARD_S + .1) / DT_MDL))
  assert rig.h.crossing.since < 0.
  rig.press()
  assert rig.h.queued_lane_change == 1


@pytest.mark.parametrize("side", ["left", "right"])
def test_held_lamp_alone_never_repeats_automatic_changes(side):
  rig = Rig(side)
  rig.start()
  assert rig.finish() == LCS.preLaneChange
  assert rig.h.next_lane_change
  assert rig.step(80) == LCS.preLaneChange
  assert rig.h.queued_lane_change == 0
  rig.press()  # The new physical event works without a lamp-off edge.
  assert not rig.h.next_lane_change
  assert rig.step() == LCS.laneChangeStarting


def test_released_quick_tap_between_model_frames_still_queues():
  rig = Rig()
  rig.start()
  rig.cross()
  rig.press(lever=0)
  assert rig.h.queued_lane_change == 1


def test_lever_that_passes_through_tap_detent_then_latches_does_not_queue():
  rig = Rig()
  rig.start()
  rig.cross()
  rig.press(lever=1)
  rig.step(2)
  assert rig.h.queued_lane_change == 0
  rig.cs.blinkerLever = 2
  rig.step(8)
  assert rig.h.queued_lane_change == 0
  assert not rig.h.one_touch_triggered


@pytest.mark.parametrize("flash_after_expiry", [False, True])
def test_expired_queue_does_not_revive_when_bsd_clears_or_lamp_flashes(flash_after_expiry):
  rig = Rig()
  rig.queue()
  rig.block_bsd()
  assert rig.finish() == LCS.preLaneChange
  rig.step(round((desire.QUEUED_LANE_WAIT_S + .2) / DT_MDL))
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none
  assert rig.h.followup_requires_press
  rig.block_bsd(False)
  rig.step(30)
  assert rig.h.lane_change_state == LCS.preLaneChange
  if flash_after_expiry:
    rig.lamps("none")
    rig.step(8)
    rig.lamps()
    rig.step(30)
    assert rig.h.lane_change_state == LCS.off
  rig.press()
  if rig.h.lane_change_state == LCS.preLaneChange:
    rig.step()
  assert rig.h.lane_change_state == LCS.laneChangeStarting


@pytest.mark.parametrize("side", ["left", "right"])
def test_queued_change_rechecks_fresh_bsd_and_its_one_second_hold(side):
  rig = Rig(side)
  rig.queue()
  rig.block_bsd()
  assert rig.finish() == LCS.preLaneChange
  assert rig.step(5) == LCS.preLaneChange
  rig.block_bsd(False)
  assert rig.step(12) == LCS.preLaneChange
  assert rig.selected.bsd_hold_counter > 0
  for _ in range(20):
    if rig.step() == LCS.laneChangeStarting:
      break
  assert rig.h.lane_change_state == LCS.laneChangeStarting


def test_queued_change_rechecks_current_side_radar_object():
  rig = Rig()
  rig.queue()
  rig.radar.leadLeft.status = True
  rig.radar.leadLeft.dRel = 5.
  rig.radar.leadLeft.vLead = 0.
  assert rig.finish() == LCS.preLaneChange
  assert rig.step(10) == LCS.preLaneChange
  assert rig.selected.side_object_detected
  rig.radar.leadLeft.status = False
  for _ in range(20):
    if rig.step() == LCS.laneChangeStarting:
      break
  assert rig.h.lane_change_state == LCS.laneChangeStarting


def test_queued_change_does_not_automatically_cross_a_solid_line():
  rig = Rig()
  rig.queue()
  rig.h.laneLineCheck = 2
  rig.cs.leftLaneLine = 21
  assert rig.finish() == LCS.preLaneChange
  assert rig.step(15) == LCS.preLaneChange
  rig.cs.leftLaneLine = 20
  assert rig.step() == LCS.laneChangeStarting


def test_a_second_request_does_not_create_a_lane_past_the_road_edge():
  rig = Rig()
  rig.queue()
  rig.geometry(-2.94, .16, adjacent=False)
  assert rig.finish() == LCS.preLaneChange
  assert rig.step(35) == LCS.preLaneChange
  assert not rig.selected.lane_available and not rig.selected.edge_available


def test_global_torque_requirement_still_applies_to_a_queued_request():
  rig = Rig()
  rig.queue()
  rig.h.laneChangeNeedTorque = 1
  assert rig.finish() == LCS.preLaneChange
  assert rig.step(20) == LCS.preLaneChange
  rig.cs.steeringPressed, rig.cs.steeringTorque = True, 1.
  assert rig.step() == LCS.laneChangeStarting


def test_disabling_driver_lane_changes_discards_the_queue():
  rig = Rig()
  rig.queue()
  rig.h.laneChangeNeedTorque = -1
  rig.step()
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none


@pytest.mark.parametrize("loss", ["lateral", "can", "opposite_torque", "opposite_press", "hazards"])
def test_interruption_clears_the_pending_request_and_never_reverses_mid_change(loss):
  rig = Rig()
  rig.queue()
  if loss == "lateral":
    rig.active = False
  elif loss == "can":
    rig.cs.canValid = False
  elif loss == "opposite_torque":
    rig.cs.steeringPressed, rig.cs.steeringTorque = True, -1.
  elif loss == "opposite_press":
    rig.press("right")
  else:
    rig.lamps("both")
  rig.step()
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none
  assert rig.h.lane_change_direction in (LCD.none, LCD.left)


def test_opposite_torque_cancel_cannot_rearm_from_the_old_lamp_flash():
  rig = Rig()
  rig.queue()
  rig.cs.steeringPressed, rig.cs.steeringTorque = True, -1.
  assert rig.step() == LCS.off
  rig.cs.steeringPressed, rig.cs.steeringTorque = False, 0.
  rig.lamps("none")
  rig.step(5)
  rig.lamps()
  assert rig.step(35) == LCS.off


@pytest.mark.parametrize("side", ["left", "right"])
def test_pending_request_waits_for_physical_signal_before_starting_next_change(side):
  rig = Rig(side)
  rig.queue()
  rig.lamps("none")
  assert rig.finish() == LCS.preLaneChange
  assert rig.h.blinker_hold_direction == rig.direction
  assert rig.step(25) == LCS.preLaneChange
  assert rig.h.queued_lane_change == rig.number
  rig.lamps()
  assert rig.step() == LCS.laneChangeStarting
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == rig.direction


def test_no_physical_signal_expires_without_starting_when_signal_later_returns():
  rig = Rig()
  rig.queue()
  rig.lamps("none")
  assert rig.finish() == LCS.preLaneChange
  rig.step(round((desire.QUEUED_LANE_WAIT_S + .2) / DT_MDL))
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none
  rig.lamps()
  assert rig.step(35) == LCS.off


@pytest.mark.parametrize("event", ["duplicate", "reset", "jump", "both"])
def test_non_press_counter_updates_cannot_queue_another_change(event):
  rig = Rig()
  rig.start()
  rig.cross()
  if event == "reset":
    rig.cs.leftBlinkerStalkCount = 0
  elif event == "jump":
    rig.cs.leftBlinkerStalkCount += 2
  elif event == "both":
    rig.cs.leftBlinkerStalkCount += 1
    rig.cs.rightBlinkerStalkCount += 1
  rig.step(5)
  assert rig.h.queued_lane_change == 0
  assert rig.finish() in (LCS.off, LCS.preLaneChange)
  assert rig.step(20) in (LCS.off, LCS.preLaneChange)
  rig.lamps("none")
  rig.step(3)
  rig.lamps()
  assert rig.step(20) == LCS.off


@pytest.mark.parametrize("loss", ["can", "reset", "both"])
def test_unconfirmed_tap_cannot_be_replayed_after_invalid_input(loss):
  rig = Rig()
  rig.start()
  rig.cross()
  rig.press(lever=1)
  assert rig.h.queued_lane_change == 0
  if loss == "can":
    rig.cs.canValid = False
  elif loss == "reset":
    rig.cs.leftBlinkerStalkCount = 0
  else:
    rig.cs.leftBlinkerStalkCount += 1
    rig.cs.rightBlinkerStalkCount += 1
  rig.step()
  rig.cs.canValid = True
  rig.cs.blinkerLever = 0
  rig.step(8)
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none


@pytest.mark.parametrize("side", ["left", "right"])
def test_press_before_crossing_is_not_promoted_when_released_after_crossing(side):
  rig = Rig(side)
  rig.start()
  near = (-.34, 2.84) if side == "left" else (-2.84, .34)
  crossed = (-2.5, .47) if side == "left" else (-.47, 2.5)
  rig.geometry(*near)
  rig.step()
  rig.press(lever=1)
  assert rig.h.crossing.since < 0.
  rig.geometry(*crossed)
  rig.step(2)
  assert rig.h.crossing.since >= 0.
  rig.cs.blinkerLever = 0
  rig.step()
  assert rig.h.queued_lane_change == 0
  assert rig.finish() == LCS.preLaneChange
  assert rig.h.next_lane_change
  assert rig.step(20) == LCS.preLaneChange


def test_press_before_finishing_guard_is_not_promoted_by_later_release():
  rig = Rig()
  rig.start()
  rig.finishing()
  rig.step(2)
  assert rig.h.finishing_timer < desire.STALK_QUEUE_GUARD_S
  rig.press(lever=1)
  rig.step(3)
  rig.cs.blinkerLever = 0
  rig.step()
  assert rig.h.finishing_timer >= desire.STALK_QUEUE_GUARD_S
  assert rig.h.queued_lane_change == 0


@pytest.mark.parametrize("loss", ["lateral", "can", "opposite_torque", "opposite_press", "hazards", "latched"])
def test_handoff_signal_is_cleared_when_the_new_maneuver_is_interrupted(loss):
  rig = Rig()
  rig.queue()
  assert rig.finish() == LCS.preLaneChange
  rig.geometry()
  assert rig.step() == LCS.laneChangeStarting
  assert rig.h.blinker_hold_direction == LCD.left
  if loss == "lateral":
    rig.active = False
  elif loss == "can":
    rig.cs.canValid = False
  elif loss == "opposite_torque":
    rig.cs.steeringPressed, rig.cs.steeringTorque = True, -1.
  elif loss == "opposite_press":
    rig.press("right")
  elif loss == "hazards":
    rig.lamps("both")
  else:
    rig.cs.blinkerLever = 2
  rig.step()
  assert rig.h.blinker_hold_direction == LCD.none


def test_handoff_signal_does_not_survive_the_maximum_lane_change_time():
  rig = Rig()
  rig.queue()
  assert rig.finish() == LCS.preLaneChange
  rig.geometry()
  assert rig.step() == LCS.laneChangeStarting
  rig.probability = .5
  assert rig.h.blinker_hold_direction == LCD.left
  for _ in range(round(11. / DT_MDL)):
    rig.step()
  assert rig.h.lane_change_state == LCS.off
  assert rig.h.blinker_hold_direction == LCD.none
  rig.lamps("none")
  rig.step(2)
  rig.lamps()
  assert rig.step(20) == LCS.off


@pytest.mark.parametrize("atc_type", ["", "atc left", "fork left", "turn left"])
def test_navigation_only_state_machine_matches_the_pre_port_target(atc_type):
  root = Path(__file__).resolve().parents[3]
  baseline_result = subprocess.run(
    ["git", "show", "30ecfc140b7a0689c9ed46bce5a9420d0ead3bdc:selfdrive/controls/lib/desire_helper.py"],
    cwd=root, capture_output=True, text=True,
  )
  if baseline_result.returncode:
    pytest.skip("Pre-port baseline is unavailable in a shallow checkout or source archive")
  source = baseline_result.stdout
  baseline = ModuleType("baseline_desire_helper")
  exec(compile(source, "pre_continuation_desire_helper.py", "exec"), baseline.__dict__)
  baseline.Params = MemoryParams
  old, new = Rig(module=baseline), Rig()
  for i in range(90):
    for rig in (old, new):
      rig.cm.atcType = atc_type
      if atc_type == "" and i == 0:
        rig.cm.carrotCmdIndex, rig.cm.carrotCmd, rig.cm.carrotArg = 1, "LANECHANGE", "LEFT"
      rig.cs.steeringPressed, rig.cs.steeringTorque = (i == 8), (1. if i == 8 else 0.)
      rig.probability = .5 if i < 14 else 0.
      rig.step()
    keys = ("lane_change_state", "lane_change_direction", "desire", "lane_change_ll_prob",
            "lane_change_timer", "lane_change_delay", "next_lane_change", "auto_lane_change_enable", "turn_direction")
    assert {key: getattr(new.h, key) for key in keys} == {key: getattr(old.h, key) for key in keys}
