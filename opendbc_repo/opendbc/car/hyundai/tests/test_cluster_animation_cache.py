"""Model receipt -> bounded EV6 cluster animation, with independent blink hold."""
from types import SimpleNamespace as NS

import pytest

from cereal import log
from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller, hyundaicanfd
from opendbc.car.hyundai.tests.test_cutin_hda_display import PrefixComplete, RadarSubMaster, radar_state
from opendbc.car.hyundai.values import HyundaiFlags


_UNCHANGED = object()
_DESIRES = {0: "none", 1: "turnLeft", 2: "turnRight", 3: "laneChangeLeft", 4: "laneChangeRight"}


def model(desire, probability=.95):
  result = log.ModelDataV2.new_message()
  result.meta.desire = _DESIRES[desire]
  result.meta.laneChangeState = "laneChangeStarting" if desire in (3, 4) else "off"
  result.meta.laneChangeDirection = "left" if desire in (1, 3) else "right" if desire in (2, 4) else "none"
  result.meta.desireState = [0.] * 8
  if desire in (3, 4):
    result.meta.desireState[desire] = probability
  # Pass the same Cap'n Proto reader that a real SubMaster exposes.
  return result.as_reader()


class BeforeSteering:
  def __init__(self, control):
    self.control = control

  def __getattr__(self, name):
    return getattr(self.control, name)

  @property
  def actuators(self):
    raise PrefixComplete


class ClusterDrive:
  """Exercise controller receipt handling and decode both transmitted messages."""
  def __init__(self, clock):
    self.clock = clock
    self.controller = carcontroller.CarController.__new__(carcontroller.CarController)
    self.controller.frame = 1  # Stop before steering and avoid unrelated parameter polls.
    self.controller.corner_radar_cutin = False
    self.controller.sm = RadarSubMaster(radar_state())
    self.control = structs.CarControl(latActive=True)
    self.packer = CANPacker("hyundai_canfd_generated")
    stock = {}
    for name in ("ADRV_0x161", "ADRV_0x1ea"):
      stock[name] = {signal: 0 for signal in self.packer.dbc.name_to_msg[name].sigs}
      stock[name]["COUNTER"] = 71
    stock["ADRV_0x1ea"].update(LANE_CHANGING=4, LEFT_BLINK_HOLD=1, RIGHT_BLINK_HOLD=1)
    self.cs = NS(out=structs.CarState(canValid=True, vEgo=25., gearShifter="drive"),
                 scc_control=None, lfahda_cluster=None, adrv_0x161=stock["ADRV_0x161"],
                 adrv_0x200=None, adrv_0x1ea=stock["ADRV_0x1ea"], ccnc_0x162=None,
                 is_metric=True, paddle_button_prev=0, softHoldActive=0, cruise_buttons_msg=None)
    self.cp = NS(flags=HyundaiFlags.CAMERA_SCC.value, openpilotLongitudinalControl=False)
    self.frame = 5

  def step(self, desire=_UNCHANGED, *, now, receipt=None, updated=None, frame=None,
           valid=True, alive=True, active=True, can_valid=True, lever=0,
           probability=.95, left_blinker=False, right_blinker=False,
           left_bsd=False, right_bsd=False):
    sm = self.controller.sm
    fresh = desire is not _UNCHANGED
    if fresh:
      sm.model = model(desire, probability)
      sm.recv_time["modelV2"] = now if receipt is None else receipt
    elif receipt is not None:
      sm.recv_time["modelV2"] = receipt
    sm.updated["modelV2"] = fresh if updated is None else updated
    sm.valid["modelV2"], sm.alive["modelV2"] = valid, alive
    self.clock.now = now
    self.control.latActive = active
    self.cs.out.canValid, self.cs.out.blinkerLever = can_valid, lever
    self.cs.out.leftBlinker, self.cs.out.rightBlinker = left_blinker, right_blinker
    self.cs.out.leftBlindspot, self.cs.out.rightBlindspot = left_bsd, right_bsd
    with pytest.raises(PrefixComplete):
      self.controller.update(BeforeSteering(self.control), self.cs, int(now * 1e9))
    if frame is not None:
      self.frame = frame
    messages = hyundaicanfd.create_ccnc_messages(
      self.cp, self.packer, NS(ECAN=0, CAM=2), self.frame,
      self.control, self.cs, self.control.hudControl, 0., 0, 0, False, 0, False, 0,
    )
    packet_frame = self.frame % 5 == 0
    self.frame += 5
    if not packet_frame:
      assert messages == []
      return None
    parser = CANParser("hyundai_canfd_generated", [("ADRV_0x161", 0), ("ADRV_0x1ea", 0)], 0)
    parser.update([(int(now * 1e9), messages)])
    assert {message[0] for message in messages} == {0x161, 0x1EA}
    return parser.vl


@pytest.fixture
def drive_factory(monkeypatch):
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
  clock = NS(now=1.)
  monkeypatch.setattr(carcontroller.time, "monotonic", lambda: clock.now)
  return lambda: ClusterDrive(clock)


def animation(values):
  return int(values["ADRV_0x161"]["LANE_LEFT"]), int(values["ADRV_0x161"]["LANE_RIGHT"])


def blink_hold(values):
  return int(values["ADRV_0x1ea"]["LEFT_BLINK_HOLD"]), int(values["ADRV_0x1ea"]["RIGHT_BLINK_HOLD"])


@pytest.mark.parametrize("desire", (1, 2, 3, 4))
def test_known_good_animation_fields_keep_raw_desire_lamp_and_bsd_meaning(drive_factory, desire):
  values = drive_factory().step(desire, now=1., left_blinker=True, right_bsd=True)
  hud = values["ADRV_0x161"]
  assert tuple(int(hud[name]) for name in (
    "LANELINE_LEFT", "LANELINE_RIGHT", "LCA_LEFT_ARROW", "LCA_RIGHT_ARROW",
    "LCA_LEFT_ICON", "LCA_RIGHT_ICON", "LANE_LEFT", "LANE_RIGHT",
  )) == (1, 1, 2, 0, 2, 1, int(desire in (1, 3)), int(desire in (2, 4)))
  assert blink_hold(values) == ((1, 0) if desire == 3 else (0, 1) if desire == 4 else (0, 0))


@pytest.mark.parametrize("desire", (3, 4))
def test_invalid_zero_snapshot_retains_only_recent_animation_not_blink_hold(drive_factory, desire):
  drive = drive_factory()
  expected = (1, 0) if desire == 3 else (0, 1)
  assert animation(drive.step(desire, now=1.)) == expected
  invalid = drive.step(0, now=1.05, valid=False)
  assert animation(invalid) == expected
  assert invalid["ADRV_0x1ea"]["LANE_CHANGING"] == 0
  assert blink_hold(invalid) == (0, 0)
  assert invalid["ADRV_0x161"]["LCA_LEFT_ARROW"] == 0
  assert invalid["ADRV_0x161"]["LCA_RIGHT_ARROW"] == 0
  assert animation(drive.step(now=1.249, valid=False)) == expected
  assert animation(drive.step(now=1.25, valid=False)) == expected
  assert animation(drive.step(now=1.251, valid=False)) == (0, 0)


@pytest.mark.parametrize("desire", (3, 4))
def test_invalid_high_probability_snapshot_never_holds_the_vehicle_blinker(drive_factory, desire):
  drive = drive_factory()
  drive.step(desire, now=1.)
  values = drive.step(desire, now=1.01, valid=False)
  assert animation(values) == ((1, 0) if desire == 3 else (0, 1))
  assert blink_hold(values) == (0, 0)
  assert values["ADRV_0x1ea"]["LANE_CHANGING"] == 0


def test_repeated_control_cycles_do_not_extend_last_model_receipt(drive_factory):
  drive = drive_factory()
  drive.step(3, now=1.)
  for tick in range(1, 25):
    assert animation(drive.step(now=1. + tick / 100.)) == (1, 0)
  assert animation(drive.step(now=1.251)) == (0, 0)
  assert animation(drive.step(now=1.3)) == (0, 0)


def test_repeated_updated_flag_with_old_receipt_does_not_refresh_animation(drive_factory):
  drive = drive_factory()
  drive.step(3, now=1.)
  assert animation(drive.step(3, now=1.2, receipt=1., updated=True)) == (1, 0)
  assert animation(drive.step(3, now=1.251, receipt=1., updated=True)) == (0, 0)


def test_an_actual_new_model_receipt_refreshes_the_animation_window(drive_factory):
  drive = drive_factory()
  drive.step(3, now=1.)
  assert animation(drive.step(3, now=1.2)) == (1, 0)
  assert animation(drive.step(0, now=1.4, valid=False)) == (1, 0)
  assert animation(drive.step(now=1.451, valid=False)) == (0, 0)


def test_fresh_valid_none_finishes_immediately_and_old_snapshot_cannot_rearm(drive_factory):
  drive = drive_factory()
  drive.step(3, now=1.)
  finished = drive.step(0, now=1.05)
  assert animation(finished) == (0, 0)
  assert blink_hold(finished) == (0, 0)
  assert animation(drive.step(3, now=1.1, receipt=1.05, updated=False)) == (0, 0)


@pytest.mark.parametrize("guard", ("lateral", "can", "latched"))
def test_override_clears_cache_and_requires_a_new_model_receipt(drive_factory, guard):
  drive = drive_factory()
  drive.step(3, now=1.)
  stopped = drive.step(now=1.01, active=guard != "lateral", can_valid=guard != "can",
                       lever=2 if guard == "latched" else 0)
  assert animation(stopped) == (0, 0)
  assert blink_hold(stopped) == (0, 0)
  assert animation(drive.step(now=1.02)) == (0, 0)
  assert animation(drive.step(3, now=1.05)) == (1, 0)


@pytest.mark.parametrize("guard", ("lateral", "can", "latched"))
def test_model_received_during_override_cannot_rearm_after_override_is_released(drive_factory, guard):
  drive = drive_factory()
  drive.step(3, now=1.)
  stopped = drive.step(4, now=1.01, active=guard != "lateral", can_valid=guard != "can",
                       lever=2 if guard == "latched" else 0)
  assert animation(stopped) == (0, 0)
  assert blink_hold(stopped) == (0, 0)
  assert animation(drive.step(now=1.02)) == (0, 0)
  assert animation(drive.step(4, now=1.05)) == (0, 1)


@pytest.mark.parametrize("guard", ("lateral", "can", "latched"))
def test_override_between_cluster_packets_consumes_receipt_before_next_packet(drive_factory, guard):
  drive = drive_factory()
  assert animation(drive.step(3, now=1., frame=5)) == (1, 0)
  assert drive.step(4, now=1.01, frame=6, active=guard != "lateral", can_valid=guard != "can",
                    lever=2 if guard == "latched" else 0) is None
  for tick in range(2, 5):
    assert drive.step(now=1. + tick / 100., frame=5 + tick) is None
  assert animation(drive.step(now=1.05, frame=10)) == (0, 0)
  assert animation(drive.step(4, now=1.1, frame=15)) == (0, 1)


def test_twenty_hz_model_in_hundred_hz_loop_refreshes_only_on_actual_receipts(drive_factory):
  drive = drive_factory()
  for tick in range(51):
    # A normal model publication every fifth controller tick, then a silent
    # publisher whose SubMaster sample still reports valid/alive.
    published = tick <= 20 and tick % 5 == 0
    values = drive.step(3 if published else _UNCHANGED, now=1. + tick / 100., frame=5 + tick)
    if tick % 5:
      assert values is None
    else:
      assert animation(values) == ((1, 0) if tick <= 45 else (0, 0))
  assert animation(drive.step(4, now=1.55, frame=60)) == (0, 1)


@pytest.mark.parametrize("receipt", (0., -1., 1.01))
def test_missing_or_future_receipt_never_arms_animation(drive_factory, receipt):
  assert animation(drive_factory().step(3, now=1., receipt=receipt)) == (0, 0)


def test_latched_turn_retains_known_good_turn_animation_without_lane_hold(drive_factory):
  values = drive_factory().step(1, now=1., lever=2)
  assert animation(values) == (1, 0)
  assert blink_hold(values) == (0, 0)


def test_fresh_opposite_desire_changes_animation_without_retaining_old_side(drive_factory):
  drive = drive_factory()
  drive.step(3, now=1.)
  switched = drive.step(4, now=1.05)
  assert animation(switched) == (0, 1)
  assert blink_hold(switched) == (0, 1)


def test_controller_instances_do_not_share_previous_vehicle_animation(drive_factory):
  drive = drive_factory()
  drive.step(3, now=1.)
  other = drive_factory()
  assert animation(other.step(0, now=1.01, valid=False)) == (0, 0)
  assert animation(drive.step(now=1.02, valid=False)) == (1, 0)


def test_clock_reversal_clears_animation_and_does_not_rearm_from_cached_sample(drive_factory):
  drive = drive_factory()
  drive.step(3, now=2.)
  assert animation(drive.step(now=1.9)) == (0, 0)
  assert animation(drive.step(now=2.01)) == (0, 0)
  assert animation(drive.step(3, now=2.05)) == (1, 0)
