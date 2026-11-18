"""Current rolled-back schema and probability-based Hyundai lane display."""
from types import SimpleNamespace as NS

import pytest

from cereal import log
from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.tests.test_cutin_hda_display import controller_prefix, radar_state
from opendbc.car.hyundai.values import HyundaiFlags


@pytest.fixture(autouse=True)
def isolated_settings(monkeypatch):
  class MemoryParams:
    def get_int(self, _key):
      return 0

    def get(self, _key):
      return b"0"

  monkeypatch.setattr(hyundaicanfd, "Params", MemoryParams)
  for name, value in (("_lane_line_check", 0), ("_blinker_timer", 0),
                      ("_lever_max_level", 0), ("_lane_changed_this_session", False)):
    monkeypatch.setattr(hyundaicanfd.create_ccnc_messages, name, value, raising=False)
  monkeypatch.setattr(hyundaicanfd._make_ccnc_values, "_filt_curve", 0., raising=False)


def emit(side, *, phase="laneChangeStarting", probability=.95, active=True,
         valid=True, can_valid=True, lever=0, missing_model=False, turn=False):
  md = log.ModelDataV2.new_message()
  assert not hasattr(md.meta, "laneChangeBlinkerHold")
  md.meta.laneChangeState = phase
  md.meta.laneChangeDirection = side
  changing = phase in ("laneChangeStarting", "laneChangeFinishing")
  md.meta.desire = (("turnLeft" if side == "left" else "turnRight") if turn else
                    ("laneChangeLeft" if side == "left" else "laneChangeRight") if changing else "none")
  md.meta.desireState = [0.] * 8
  md.meta.desireState[3 if side == "left" else 4] = probability
  out = structs.CarState(canValid=can_valid, vEgo=25., gearShifter="drive", blinkerLever=lever)
  packer = CANPacker("hyundai_canfd_generated")
  stock = {name: 0 for name in packer.dbc.name_to_msg["ADRV_0x1ea"].sigs}
  stock.update(COUNTER=71, LANE_CHANGING=4 if side == "left" else 3,
               LEFT_BLINK_HOLD=1, RIGHT_BLINK_HOLD=1)
  hud = {name: 0 for name in packer.dbc.name_to_msg["ADRV_0x161"].sigs} if turn else None
  with log.ModelDataV2.from_bytes(md.to_bytes()) as received:
    cs = NS(MD=None if missing_model else received, MD_valid=valid, out=out,
            scc_control=None, lfahda_cluster=None, adrv_0x161=hud, adrv_0x200=None,
            adrv_0x1ea=stock, ccnc_0x162=None, is_metric=True, paddle_button_prev=0,
            softHoldActive=0)
    cp = NS(flags=HyundaiFlags.CAMERA_SCC.value, openpilotLongitudinalControl=False)
    cc = structs.CarControl(latActive=active)
    messages = hyundaicanfd.create_ccnc_messages(
      cp, packer, NS(ECAN=0, CAM=2), 5, cc, cs, cc.hudControl,
      0., 0, 0, False, 0, False, 0,
    )
  names = [("ADRV_0x1ea", 0)]
  if turn:
    names.append(("ADRV_0x161", 0))
  parser = CANParser("hyundai_canfd_generated", names, 0)
  parser.update([(1_000_000_000, messages)])
  return parser.vl


def hold(values):
  return int(values["LEFT_BLINK_HOLD"]), int(values["RIGHT_BLINK_HOLD"])


@pytest.mark.parametrize("side", ("left", "right"))
@pytest.mark.parametrize("phase", ("off", "preLaneChange", "laneChangeStarting", "laneChangeFinishing"))
def test_current_schema_lane_messages_work_without_deleted_hold_field(side, phase):
  active = phase in ("laneChangeStarting", "laneChangeFinishing")
  values = emit(side, phase=phase, probability=.95 if active else 0.)["ADRV_0x1ea"]
  assert values["LANE_CHANGING"] == ((3 if side == "left" else 4) if active else 0)
  assert hold(values) == (((1, 0) if side == "left" else (0, 1)) if active else (0, 0))


@pytest.mark.parametrize("side", ("left", "right"))
@pytest.mark.parametrize("probability,enabled", ((.8999, False), (.9, False), (.9001, True)))
def test_rolled_back_probability_threshold_is_unchanged(side, probability, enabled):
  values = emit(side, probability=probability)["ADRV_0x1ea"]
  assert hold(values) == (((1, 0) if side == "left" else (0, 1)) if enabled else (0, 0))


@pytest.mark.parametrize("side", ("left", "right"))
@pytest.mark.parametrize("guard", ("lateral", "model", "can", "latch", "missing"))
def test_invalid_or_overridden_lane_output_releases_copied_hold_and_display(side, guard):
  values = emit(side, active=guard != "lateral", valid=guard != "model",
                can_valid=guard != "can", lever=2 if guard == "latch" else 0,
                missing_model=guard == "missing")["ADRV_0x1ea"]
  assert values["LANE_CHANGING"] == 0
  assert hold(values) == (0, 0)


@pytest.mark.parametrize("side", ("left", "right"))
def test_latched_turn_keeps_existing_turn_side_while_releasing_lane_hold(side):
  values = emit(side, phase="off", lever=2, turn=True)
  assert hold(values["ADRV_0x1ea"]) == (0, 0)
  assert values["ADRV_0x161"]["LANE_LEFT"] == (1 if side == "left" else 0)
  assert values["ADRV_0x161"]["LANE_RIGHT"] == (1 if side == "right" else 0)


@pytest.mark.parametrize("valid,alive", ((False, False), (False, True), (True, False), (True, True)))
def test_controller_forwards_current_model_validity(monkeypatch, valid, alive):
  controller, cs, _clock, update = controller_prefix(monkeypatch, radar_state())
  controller.sm.valid["modelV2"] = valid
  controller.sm.alive["modelV2"] = alive
  update()
  assert cs.MD is controller.sm.model
  assert cs.MD_valid == (valid and alive)
