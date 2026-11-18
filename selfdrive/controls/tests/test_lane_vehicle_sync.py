"""Actual lane-change metadata and display CAN must agree without extending HOLD."""
from types import SimpleNamespace as NS

import pytest

from cereal import car, log

if "laneChangeBlinkerHold" not in log.ModelDataV2.MetaData.schema.fields:
  pytest.skip(
    "Queued lane-change feature was reverted; current behavior is covered by "
    "test_rollback_one_touch and test_rollback_lateral_metadata.",
    allow_module_level=True,
  )

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.selfdrive.controls.lib import desire_helper as desire
from openpilot.selfdrive.controls.tests.test_desire_continuation import MemoryParams, Rig, LCS, LCD


@pytest.fixture(autouse=True)
def isolated_settings(monkeypatch):
  monkeypatch.setattr(desire, "Params", MemoryParams)
  monkeypatch.setattr(hyundaicanfd, "Params", MemoryParams)
  for name, value in (("_lane_line_check", 0), ("_blinker_timer", 0),
                      ("_lever_max_level", 0), ("_lane_changed_this_session", False)):
    monkeypatch.setattr(hyundaicanfd.create_ccnc_messages, name, value, raising=False)
  monkeypatch.setattr(hyundaicanfd._make_ccnc_values, "_filt_curve", 0., raising=False)


def publish(rig):
  """Publish the same helper fields modeld puts in modelV2."""
  rig.md.meta.desire = rig.h.desire
  rig.md.meta.laneChangeState = rig.h.lane_change_state
  rig.md.meta.laneChangeDirection = rig.h.lane_change_direction
  rig.md.meta.laneChangeBlinkerHold = rig.h.blinker_hold_direction
  rig.md.meta.laneChangeProb = rig.h.lane_change_ll_prob
  probabilities = [0.] * 8
  probabilities[3 if rig.side == "left" else 4] = rig.probability
  rig.md.meta.desireState = probabilities
  return rig.md


def wire(md, *, out=None, active=True, valid=True, lever=0, stock_lane=0):
  if out is None:
    out = car.CarState.new_message(canValid=True, vEgo=25.)
  out.gearShifter = "drive"
  out.blinkerLever = lever
  packer = CANPacker("hyundai_canfd_generated")
  stock = {name: 0 for name in packer.dbc.name_to_msg["ADRV_0x1ea"].sigs}
  stock.update(COUNTER=71, LANE_CHANGING=stock_lane, LEFT_BLINK_HOLD=1, RIGHT_BLINK_HOLD=1)
  payload = md.to_bytes()
  md.clear_write_flag()
  with log.ModelDataV2.from_bytes(payload) as received:
    cs = NS(MD=received, MD_valid=valid, out=out, scc_control=None, lfahda_cluster=None,
            adrv_0x161=None, adrv_0x200=None, adrv_0x1ea=stock, ccnc_0x162=None)
    cp = NS(flags=HyundaiFlags.CAMERA_SCC.value, openpilotLongitudinalControl=True)
    control = car.CarControl.new_message(latActive=active)
    packets = hyundaicanfd.create_ccnc_messages(
      cp, packer, NS(ECAN=0, CAM=2), 5, control, cs, control.hudControl,
      0., 0, 0, False, 0, False, 0,
    )
  assert len(packets) == 1 and packets[0][0] == 0x1EA and packets[0][2] == 0
  parser = CANParser("hyundai_canfd_generated", [("ADRV_0x1ea", 0)], 0)
  parser.update([(1_000_000_000, packets)])
  return parser.vl["ADRV_0x1ea"]


def hold(fields):
  return int(fields["LEFT_BLINK_HOLD"]), int(fields["RIGHT_BLINK_HOLD"])


def expected_hold(side):
  return (1, 0) if side == "left" else (0, 1)


@pytest.mark.parametrize("side", ["left", "right"])
def test_real_fsm_crossing_releases_hold_while_active_hud_stays_synchronized(side):
  rig = Rig(side)
  rig.probability = .95
  rig.start()
  fields = wire(publish(rig), out=rig.cs)
  assert fields["LANE_CHANGING"] == (3 if side == "left" else 4)
  assert hold(fields) == expected_hold(side)
  rig.cross()
  assert rig.h.lane_change_state == LCS.laneChangeStarting
  assert rig.h.blinker_hold_direction == LCD.none
  fields = wire(publish(rig), out=rig.cs)
  assert fields["LANE_CHANGING"] == (3 if side == "left" else 4)
  assert hold(fields) == (0, 0)
  rig.step(3)  # High residual probability cannot override the release.
  assert hold(wire(publish(rig), out=rig.cs)) == (0, 0)
  assert rig.finish() == LCS.off
  fields = wire(publish(rig), out=rig.cs, stock_lane=3 if side == "left" else 4)
  assert fields["LANE_CHANGING"] == 0
  assert hold(fields) == (0, 0)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("probability,enabled", [(.8999, False), (.9, True), (.9001, True)])
def test_ordinary_hold_uses_model_probability_including_serialized_point_nine(side, probability, enabled):
  rig = Rig(side)
  # Exercise the Float32 value received from the actual model schema.
  sample = log.ModelDataV2.new_message()
  sample.meta.desireState = [probability]
  with log.ModelDataV2.from_bytes(sample.to_bytes()) as received:
    rig.probability = received.meta.desireState[0]
  rig.start()
  fields = wire(publish(rig), out=rig.cs)
  assert hold(fields) == (expected_hold(side) if enabled else (0, 0))
  assert fields["LANE_CHANGING"] == (3 if side == "left" else 4)


@pytest.mark.parametrize("side", ["left", "right"])
def test_no_crossing_probability_drop_releases_before_finishing_tail(side):
  rig = Rig(side)
  rig.probability = .95
  rig.start()
  assert hold(wire(publish(rig), out=rig.cs)) == expected_hold(side)
  rig.probability = .2
  rig.step()
  assert rig.h.crossing.since < 0.
  assert rig.h.lane_change_state == LCS.laneChangeStarting
  fields = wire(publish(rig), out=rig.cs)
  assert hold(fields) == (0, 0)
  assert fields["LANE_CHANGING"] == (3 if side == "left" else 4)


@pytest.mark.parametrize("side", ["left", "right"])
def test_queued_bootstrap_is_preserved_but_does_not_extend_into_finishing(side):
  rig = Rig(side)
  rig.queue()
  fields = wire(publish(rig), out=rig.cs)
  assert fields["LANE_CHANGING"] == 0
  assert hold(fields) == expected_hold(side)
  rig.probability = 0.
  rig.geometry()
  assert rig.step() == LCS.laneChangeStarting
  assert hold(wire(publish(rig), out=rig.cs)) == expected_hold(side)
  for _ in range(20):
    if rig.step() == LCS.laneChangeFinishing:
      break
  assert rig.h.lane_change_state == LCS.laneChangeFinishing
  assert rig.h.handoff_blinker_hold == 0
  assert hold(wire(publish(rig), out=rig.cs)) == (0, 0)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("state", ["off", "preLaneChange", "laneChangeStarting", "laneChangeFinishing"])
def test_hud_uses_current_state_and_direction_and_always_retracts_stale_stock(side, state):
  md = log.ModelDataV2.new_message()
  md.meta.laneChangeState = state
  md.meta.laneChangeDirection = side
  md.meta.laneChangeBlinkerHold = "none"
  md.meta.desire = "none" if state.startswith("laneChange") else "laneChangeLeft"
  md.meta.desireState = [0., 0., 0., .95, 0.]
  fields = wire(md, stock_lane=4 if side == "left" else 3)
  assert fields["LANE_CHANGING"] == ((3 if side == "left" else 4) if state.startswith("laneChange") else 0)
  assert hold(fields) == (0, 0)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("state", ["off", "preLaneChange", "laneChangeStarting"])
def test_session_warning_follows_fsm_instead_of_residual_model_probability(side, state):
  active_change = state == "laneChangeStarting"
  md = log.ModelDataV2.new_message()
  md.meta.laneChangeState = state
  md.meta.laneChangeDirection = side
  md.meta.desire = "none"
  probabilities = [0.] * 8
  probabilities[3 if side == "left" else 4] = 0. if active_change else .95
  md.meta.desireState = probabilities
  out = car.CarState.new_message(canValid=True, vEgo=25.)
  setattr(out, side + "Blinker", True)
  setattr(out, side + "Blindspot", True)
  for _ in range(6):
    fields = wire(md, out=out, lever=1)
  assert hyundaicanfd.create_ccnc_messages._lane_changed_this_session == active_change
  assert fields["LANE_CHANGING"] == ((3 if side == "left" else 4) if active_change else 0)
  assert fields["AUTOLANECHANGE_MSG"] == (0 if active_change else 1)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("guard", ["lateral", "model", "latch"])
def test_invalid_inactive_or_manually_latched_state_clears_hud_and_hold(side, guard):
  md = log.ModelDataV2.new_message()
  md.meta.laneChangeState = "laneChangeStarting"
  md.meta.laneChangeDirection = md.meta.laneChangeBlinkerHold = side
  md.meta.desire = "laneChangeLeft" if side == "left" else "laneChangeRight"
  md.meta.desireState = [0., 0., 0., .95, .95]
  fields = wire(md, active=guard != "lateral", valid=guard != "model", lever=2 if guard == "latch" else 0, stock_lane=3)
  assert fields["LANE_CHANGING"] == 0
  assert hold(fields) == (0, 0)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("interruption", ["hazards", "opposite", "counter"])
def test_cancelled_active_hold_does_not_reappear_when_lamps_clear(side, interruption):
  rig = Rig(side)
  rig.probability = .95
  rig.start()
  if interruption == "hazards":
    rig.lamps("both")
    rig.step()
  elif interruption == "opposite":
    rig.press("right" if side == "left" else "left", lever=1)
  else:
    rig.cs.leftBlinkerStalkCount += 1
    rig.cs.rightBlinkerStalkCount += 1
    rig.step()
  assert hold(wire(publish(rig), out=rig.cs)) == (0, 0)
  rig.lamps("none")
  rig.cs.blinkerLever = 0
  rig.step(3)
  assert hold(wire(publish(rig), out=rig.cs)) == (0, 0)
