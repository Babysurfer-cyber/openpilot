"""The rolled-back lane-change path consumes one released stalk tap once."""
import pytest

from cereal import car, custom, log
from openpilot.selfdrive.controls.lib import desire_helper as desire


LCS = log.LaneChangeState


class MemoryParams:
  def get_int(self, _key):
    return 0

  def get_float(self, _key):
    return 0.


@pytest.fixture(autouse=True)
def no_device_settings(monkeypatch):
  monkeypatch.setattr(desire, "Params", MemoryParams)


class Rig:
  def __init__(self, side):
    self.side = side
    self.h = desire.DesireHelper()
    self.h._update_params_periodic = lambda: None
    self.cs = car.CarState.new_message(canValid=True, vEgo=25., aEgo=0.)
    self.md = log.ModelDataV2.new_message()
    self.md.init("laneLines", 4)
    self.md.init("roadEdges", 2)
    self.md.laneLineProbs = [.95] * 4
    self.md.orientationRate.z = [0.] * 33
    self.md.meta.desireState = [0.] * 8
    for line, position in zip(self.md.laneLines, (-5.4, -1.8, 1.8, 5.4), strict=True):
      line.y = [position] * 33
    for edge, position in zip(self.md.roadEdges, (-12., 12.), strict=True):
      edge.y = [position] * 33
    self.cm = custom.CarrotMan.new_message()
    self.radar = log.RadarState.new_message()
    self.active = True
    self.probability = .5
    self.starts = 0
    self.step(30)

  def lamps(self, enabled=True):
    self.cs.leftBlinker = enabled and self.side == "left"
    self.cs.rightBlinker = enabled and self.side == "right"

  def step(self, frames=1):
    for _ in range(frames):
      previous = self.h.lane_change_state
      self.h.update(self.cs, self.md, self.active, self.probability, self.cm, self.radar)
      if previous != LCS.laneChangeStarting and self.h.lane_change_state == LCS.laneChangeStarting:
        self.starts += 1
    return self.h.lane_change_state

  def press(self, level=1):
    counter = self.side + "BlinkerStalkCount"
    setattr(self.cs, counter, (getattr(self.cs, counter) + 1) % 256)
    self.cs.blinkerLever = level
    self.lamps()
    return self.step()

  def release(self):
    self.cs.blinkerLever = 0
    return self.step()

  def tap(self):
    self.press()
    return self.release()

  def start(self):
    assert self.tap() == LCS.preLaneChange
    assert self.step() == LCS.laneChangeStarting
    assert self.starts == 1

  def last_finishing_frame(self):
    self.probability = 0.
    for _ in range(60):
      if self.step() == LCS.laneChangeFinishing:
        break
    assert self.h.lane_change_state == LCS.laneChangeFinishing
    while self.h.lane_change_ll_prob < .949:
      assert self.step() == LCS.laneChangeFinishing

  def finish(self):
    self.last_finishing_frame()
    assert self.step() == LCS.off


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("dark_at_completion", [False, True])
def test_consumed_tap_cannot_restart_from_lamp_cycles(side, dark_at_completion):
  rig = Rig(side)
  rig.start()
  rig.last_finishing_frame()
  if dark_at_completion:
    rig.lamps(False)
  assert rig.step() == LCS.off
  assert not rig.h.one_touch_triggered
  rig.lamps()
  assert rig.step(20) == LCS.off
  for _ in range(2):
    rig.lamps(False)
    assert rig.step(10) == LCS.off
    rig.lamps()
    assert rig.step(20) == LCS.off
  assert rig.starts == 1


@pytest.mark.parametrize("side", ["left", "right"])
def test_a_new_released_tap_after_completion_starts_one_new_maneuver(side):
  rig = Rig(side)
  rig.start()
  rig.finish()
  rig.step(3)
  assert rig.tap() == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting
  assert rig.starts == 2
  rig.finish()
  assert rig.step(40) == LCS.off
  assert rig.starts == 2


@pytest.mark.parametrize("side", ["left", "right"])
def test_partial_detent_waits_for_release(side):
  rig = Rig(side)
  rig.press()
  assert rig.step(20) == LCS.off
  assert rig.starts == 0
  assert rig.release() == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("during_maneuver", [False, True])
def test_full_latch_does_not_acquire_or_continue_high_speed_lane_change(side, during_maneuver):
  rig = Rig(side)
  if during_maneuver:
    rig.start()
  rig.press(level=2)
  assert rig.step(10) == LCS.off
  assert not rig.h.one_touch_triggered
  assert rig.h.lane_change_ll_prob == pytest.approx(1.)
  rig.release()
  assert rig.step(20) == LCS.off
  assert rig.starts == int(during_maneuver)


@pytest.mark.parametrize("side", ["left", "right"])
def test_full_latch_preserves_eligible_low_speed_turn(side):
  rig = Rig(side)
  rig.cs.vEgo = 5.
  probability = [0.] * 8
  probability[1 if side == "left" else 2] = .95
  rig.md.meta.desireState = probability
  rig.press(level=2)
  rig.step(5)
  assert rig.h.lane_change_state == LCS.off
  assert rig.h.desire == (log.Desire.turnLeft if side == "left" else log.Desire.turnRight)
  assert rig.starts == 0


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("fault", ["can", "lateral"])
@pytest.mark.parametrize("phase", ["pressed", "started"])
def test_fault_consumes_existing_gesture_instead_of_replaying_after_recovery(side, fault, phase):
  rig = Rig(side)
  if phase == "started":
    rig.start()
  else:
    rig.press()
  if fault == "can":
    rig.cs.canValid = False
  else:
    rig.active = False
  assert rig.step() == LCS.off
  rig.cs.canValid = rig.active = True
  rig.cs.blinkerLever = 0
  assert rig.step(20) == LCS.off
  rig.lamps(False)
  rig.step(10)
  rig.lamps()
  assert rig.step(10) == LCS.off
  assert rig.starts == int(phase == "started")
  assert rig.tap() == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting


@pytest.mark.parametrize("side", ["left", "right"])
def test_navigation_lane_change_command_keeps_its_existing_path(side):
  rig = Rig(side)
  rig.cm.carrotCmdIndex = 1
  rig.cm.carrotCmd = "LANECHANGE"
  rig.cm.carrotArg = side.upper()
  assert rig.step() == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting
  assert rig.starts == 1


@pytest.mark.parametrize("side", ["left", "right"])
def test_navigation_turn_keeps_its_existing_path(side):
  rig = Rig(side)
  rig.cm.atcType = "turn " + side
  rig.step(3)
  assert rig.h.lane_change_state == LCS.off
  assert rig.h.desire == (log.Desire.turnLeft if side == "left" else log.Desire.turnRight)


@pytest.mark.parametrize("side", ["left", "right"])
def test_bsd_and_its_existing_hold_block_a_released_tap(side):
  rig = Rig(side)
  rig.h.laneChangeBsd = 1
  setattr(rig.cs, side + "Blindspot", True)
  assert rig.tap() == LCS.preLaneChange
  assert rig.step(5) == LCS.preLaneChange
  setattr(rig.cs, side + "Blindspot", False)
  assert rig.step(10) == LCS.preLaneChange
  for _ in range(25):
    if rig.step() == LCS.laneChangeStarting:
      break
  assert rig.h.lane_change_state == LCS.laneChangeStarting
  assert rig.starts == 1
