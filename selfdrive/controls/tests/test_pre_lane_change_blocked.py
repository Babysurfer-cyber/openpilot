"""Released one-touch requests expose their current pre-start block reason."""
import pytest

from cereal import car, custom, log
from opendbc.car.hyundai.blocked_lane_change_blinker import BlockedLaneChangeBlinker
from openpilot.selfdrive.controls.lib import desire_helper

LCS = log.LaneChangeState


class MemoryParams:
  def get_int(self, _key):
    return 0

  def get_float(self, _key):
    return 0.


class Rig:
  def __init__(self, side, monkeypatch):
    monkeypatch.setattr(desire_helper, 'Params', MemoryParams)
    self.side = side
    self.helper = desire_helper.DesireHelper()
    self.helper.laneLineCheck = 1
    self.state = car.CarState.new_message(canValid=True, vEgo=25., vEgoCluster=25., gearShifter='drive')
    self.model = log.ModelDataV2.new_message()
    self.model.init('laneLines', 4)
    self.model.init('roadEdges', 2)
    self.model.laneLineProbs = [.95] * 4
    self.model.orientationRate.z = [0.] * 33
    self.model.meta.desireState = [0.] * 8
    self.shift(0.)
    self.carrot = custom.CarrotMan.new_message()
    self.radar = log.RadarState.new_message()
    self.probability = .5
    self.active = True
    self.clock_ns = 1_000_000_000
    self.step(30)

  def shift(self, value):
    for line, original in zip(self.model.laneLines, (-5.4, -1.8, 1.8, 5.4), strict=True):
      line.y = [original + value] * 33
    for edge, original in zip(self.model.roadEdges, (-12., 12.), strict=True):
      edge.y = [original + value] * 33

  def step(self, frames=1):
    for _ in range(frames):
      self.clock_ns += 50_000_000
      self.model.timestampEof = self.clock_ns
      self.helper.update(self.state, self.model, self.active, self.probability, self.carrot, self.radar)
    return self.helper.lane_change_state

  def press(self, level=1):
    count = self.side + 'BlinkerStalkCount'
    setattr(self.state, count, (getattr(self.state, count) + 1) % 256)
    self.state.leftBlinker = self.side == 'left'
    self.state.rightBlinker = self.side == 'right'
    self.state.blinkerLever = level
    return self.step()

  def release(self):
    self.state.blinkerLever = 0
    return self.step()

  def tap(self):
    self.press()
    return self.release()

  def blindspot(self, value=True, *, opposite=False):
    side = ('right' if self.side == 'left' else 'left') if opposite else self.side
    setattr(self.state, side + 'Blindspot', value)

  def solid(self, value=21):
    setattr(self.state, self.side + 'LaneLine', value)

  def torque(self):
    self.state.steeringPressed = True
    self.state.steeringTorque = 1. if self.side == 'left' else -1.

  def object(self):
    lead = self.radar.leadLeft if self.side == 'left' else self.radar.leadRight
    lead.status, lead.dRel, lead.vLead = True, 5., 0.


@pytest.fixture(params=('left', 'right'))
def rig(request, monkeypatch):
  return Rig(request.param, monkeypatch)


def test_selected_bsd_reports_one(rig):
  rig.blindspot()
  assert rig.tap() == LCS.preLaneChange
  assert rig.helper.lane_change_blocked == 1
  assert rig.step() == LCS.preLaneChange
  assert rig.helper.lane_change_blocked == 1


def test_solid_line_reports_two(rig):
  rig.solid()
  assert rig.tap() == LCS.preLaneChange
  assert rig.helper.lane_change_blocked == 2


def test_bsd_has_priority_over_simultaneous_solid_line(rig):
  rig.blindspot()
  rig.solid()
  rig.tap()
  assert rig.helper.lane_change_blocked == 1


def test_available_request_and_opposite_bsd_have_no_block_reason(rig):
  rig.blindspot(opposite=True)
  assert rig.tap() == LCS.preLaneChange
  assert rig.helper.lane_change_blocked == 0
  assert rig.step() == LCS.laneChangeStarting
  assert rig.helper.lane_change_blocked == 0


def test_ignored_bsd_does_not_request_extra_flashes(rig):
  rig.helper.laneChangeBsd = -1
  rig.blindspot()
  rig.tap()
  assert rig.helper.lane_change_blocked == 0
  assert rig.step() == LCS.laneChangeStarting


@pytest.mark.parametrize(('setting', 'expected'), ((0, 0), (1, 1)))
def test_directional_torque_respects_bsd_override_setting(rig, setting, expected):
  rig.helper.laneChangeBsd = setting
  rig.blindspot()
  rig.torque()
  rig.tap()
  assert rig.helper.lane_change_blocked == expected
  assert rig.step() == (LCS.preLaneChange if setting == 1 else LCS.laneChangeStarting)
  assert rig.helper.lane_change_blocked == expected


def test_held_first_detent_only_reports_after_release(rig):
  rig.blindspot()
  rig.press()
  assert rig.step(8) == LCS.preLaneChange
  assert rig.helper.lane_change_blocked == 0
  assert rig.release() == LCS.preLaneChange
  assert rig.helper.lane_change_blocked == 1


def test_full_latch_does_not_report_one_touch_block(rig):
  rig.blindspot()
  rig.solid()
  rig.press(level=2)
  rig.step(8)
  assert rig.helper.lane_change_blocked == 0


def test_atc_only_pending_request_does_not_request_extra_flashes(rig):
  rig.blindspot()
  rig.carrot.carrotCmd = 'LANECHANGE'
  rig.carrot.carrotArg = rig.side.upper()
  rig.carrot.carrotCmdIndex = 1
  assert rig.step() == LCS.preLaneChange
  assert rig.step() == LCS.preLaneChange
  assert rig.helper.lane_change_blocked == 0


@pytest.mark.parametrize('wait', ('delay', 'torque'))
def test_other_start_requirements_do_not_misreport_bsd(rig, wait):
  rig.blindspot()
  if wait == 'delay':
    rig.helper.laneChangeDelay = 1.
  else:
    rig.helper.laneChangeNeedTorque = 1
  rig.tap()
  assert rig.helper.lane_change_blocked == 0


@pytest.mark.parametrize(('bsd', 'expected'), ((False, 0), (True, 1)))
def test_side_object_does_not_hide_bsd_but_alone_has_no_reason(rig, bsd, expected):
  rig.object()
  rig.blindspot(bsd)
  rig.tap()
  assert rig.helper.lane_change_blocked == expected


def test_lateral_deactivation_clears_previous_block(rig):
  rig.blindspot()
  rig.tap()
  assert rig.helper.lane_change_blocked == 1
  rig.active = False
  assert rig.step() == LCS.off
  assert rig.helper.lane_change_blocked == 0


def test_starting_and_finishing_never_report_pre_start_blocks(rig):
  rig.tap()
  assert rig.step() == LCS.laneChangeStarting
  rig.solid()
  rig.probability = 0.
  assert rig.step() == LCS.laneChangeStarting
  assert rig.helper.lane_change_blocked == 0
  for _ in range(15):
    if rig.step() == LCS.laneChangeFinishing:
      break
  assert rig.helper.lane_change_state == LCS.laneChangeFinishing
  rig.blindspot()
  rig.step()
  assert rig.helper.lane_change_blocked == 0


@pytest.mark.parametrize('block', ('bsd', 'solid'))
def test_extended_lamps_allow_start_without_another_lever_input(rig, block):
  rig.blindspot(block == 'bsd')
  if block == 'solid':
    rig.solid()
  rig.tap()
  counts = (rig.state.leftBlinkerStalkCount, rig.state.rightBlinkerStalkCount)
  assert rig.helper.lane_change_blocked == (1 if block == 'bsd' else 2)
  if block == 'bsd':
    rig.blindspot(False)
    for _ in range(22):
      if rig.step() == LCS.laneChangeStarting:
        break
  else:
    rig.solid(0)
    rig.step()
  assert rig.helper.lane_change_state == LCS.laneChangeStarting
  assert rig.helper.lane_change_blocked == 0
  assert (rig.state.leftBlinkerStalkCount, rig.state.rightBlinkerStalkCount) == counts


def test_manual_crossing_consumes_pending_request_without_restart(rig):
  rig.solid()
  rig.tap()
  assert rig.helper.lane_change_blocked == 2
  sign = 1. if rig.side == 'left' else -1.
  for shift in (.3, .6, .9, 1.2, 1.5, 1.8, 2.1, 2.1):
    rig.shift(sign * shift)
    rig.step()
  assert rig.helper.one_touch_completed
  assert rig.helper.lane_change_state == LCS.off
  assert rig.helper.lane_change_blocked == 0
  rig.solid(0)
  assert rig.step(4) == LCS.off
  assert rig.helper.lane_change_blocked == 0


@pytest.mark.parametrize(('line_check', 'raw', 'expected'), ((0, 11, 0), (0, 21, 2), (1, 11, 2), (1, 25, 0), (2, 21, 2)))
def test_solid_reason_uses_the_same_configured_line_rule_as_the_start_gate(rig, line_check, raw, expected):
  rig.helper.laneLineCheck = line_check
  rig.solid(raw)
  rig.tap()
  assert rig.helper.lane_change_blocked == expected
  if expected == 0:
    assert rig.step() == LCS.laneChangeStarting


def test_directional_torque_overrides_the_existing_solid_start_gate(rig):
  rig.helper.laneLineCheck = 2
  rig.solid()
  rig.torque()
  rig.tap()
  assert rig.helper.lane_change_blocked == 0
  assert rig.step() == LCS.laneChangeStarting


def test_missing_lane_geometry_does_not_misreport_bsd(rig):
  selected = 1 if rig.side == 'left' else 2
  outer = 0 if rig.side == 'left' else 3
  edge = 0 if rig.side == 'left' else 1
  rig.model.laneLines[outer].y = list(rig.model.laneLines[selected].y)
  rig.model.roadEdges[edge].y = list(rig.model.laneLines[selected].y)
  rig.step(30)
  rig.blindspot()
  rig.tap()
  assert rig.helper.lane_change_blocked == 0


def test_completed_automatic_change_cannot_restart_from_remaining_lamps(rig):
  rig.tap()
  assert rig.step() == LCS.laneChangeStarting
  rig.probability = 0.
  for _ in range(60):
    rig.step()
    if rig.helper.one_touch_completed:
      break
  assert rig.helper.one_touch_completed
  assert rig.helper.lane_change_state == LCS.off
  rig.blindspot()
  rig.solid()
  assert rig.step(4) == LCS.off
  assert rig.helper.lane_change_blocked == 0


@pytest.mark.parametrize('block', ('bsd', 'solid'))
def test_real_dh_reason_drives_extension_until_block_clears_during_extra_flashes(rig, block):
  extension = BlockedLaneChangeBlinker()
  direction = 1 if rig.side == 'left' else 2

  def publish(raw_lamp):
    # These are the real modeld metadata inputs to the CAN-side helper.
    rig.model.meta.laneChangeState = rig.helper.lane_change_state
    rig.model.meta.laneChangeDirection = rig.helper.lane_change_direction
    rig.model.meta.laneChangeBlocked = rig.helper.lane_change_blocked
    return extension.update(
      rig.clock_ns * 1e-9, rig.state, model_meta=rig.model.meta, model_valid=True,
      left_lamp=raw_lamp and direction == 1, right_lamp=raw_lamp and direction == 2,
      lamp_valid=True, lateral_active=rig.active,
    )

  def phase(on):
    result = 0
    for _ in range(7):  # 0.35 s ON/OFF phases, matching the observed 0.7 s period.
      rig.step()
      result = publish(on)
    return result

  assert publish(False) == 0  # Baseline stalk counts, not a new driver request.
  if block == 'bsd':
    rig.blindspot()
  else:
    rig.solid()
  rig.press()
  assert publish(True) == 0
  rig.release()
  assert publish(True) == 0
  original_counts = (rig.state.leftBlinkerStalkCount, rig.state.rightBlinkerStalkCount)
  for _ in range(3):
    phase(True)
    assert phase(False) == direction
    assert rig.helper.lane_change_state == LCS.preLaneChange

  if block == 'bsd':
    rig.blindspot(False)
  else:
    rig.solid(0)
  # No additional lever input: the existing model request remains available
  # during the two added flashes, including the ordinary BSD hold interval.
  for _ in range(2):
    phase(True)
    phase(False)
  assert (rig.state.leftBlinkerStalkCount, rig.state.rightBlinkerStalkCount) == original_counts
  assert rig.helper.lane_change_state == LCS.laneChangeStarting
  assert rig.helper.lane_change_blocked == 0
  assert publish(False) == 0  # The existing lane-change hold now owns the lamp.

  rig.probability = 0.
  for _ in range(40):
    rig.step()
    assert publish(False) == 0
    if rig.helper.one_touch_completed:
      break
  assert rig.helper.one_touch_completed
  assert rig.step(4) == LCS.off
  assert publish(False) == 0


@pytest.mark.parametrize('reason', (0, 1, 2))
def test_model_v2_block_reason_survives_wire_serialization(reason):
  event = log.Event.new_message(valid=True)
  event.init('modelV2')
  event.modelV2.meta.laneChangeBlocked = reason
  with log.Event.from_bytes(event.to_bytes()) as decoded:
    assert decoded.modelV2.meta.laneChangeBlocked == reason


def test_old_model_message_defaults_to_no_block_reason():
  event = log.Event.new_message(valid=True)
  event.init('modelV2')
  with log.Event.from_bytes(event.to_bytes()) as decoded:
    assert decoded.modelV2.meta.laneChangeBlocked == 0
