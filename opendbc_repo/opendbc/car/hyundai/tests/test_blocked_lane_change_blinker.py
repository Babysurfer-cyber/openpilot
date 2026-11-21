"""One blocked stalk tap may extend three stock flashes to five raw flashes."""
import math
from types import SimpleNamespace as NS

import pytest

from cereal import log
from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller, carstate, hyundaicanfd
from opendbc.car.hyundai.blocked_lane_change_blinker import BlockedLaneChangeBlinker
from opendbc.car.hyundai.values import HyundaiFlags


class BlinkerFeed:
  def __init__(self, side=1, blocked=1, *, initial_count=0, initial_lamp=False):
    self.helper = BlockedLaneChangeBlinker()
    self.side = side
    self.now = 1.
    self.car = structs.CarState(canValid=True, vEgo=25., gearShifter="drive",
                                leftBlinkerStalkCount=initial_count, rightBlinkerStalkCount=initial_count,
                                leftBlinker=side == 1, rightBlinker=side == 2)
    self.model = log.ModelDataV2.new_message()
    self.model.meta.laneChangeState = "preLaneChange"
    self.model.meta.laneChangeDirection = "left" if side == 1 else "right"
    self.model.meta.laneChangeBlocked = blocked
    self.model.meta.desireState = [0.] * 8
    self.left = initial_lamp and side == 1
    self.right = initial_lamp and side == 2
    self.model_valid = self.lamp_valid = self.lateral_active = True
    self.rises = 0
    assert self.step(dt=0.) == 0

  def step(self, dt=.05, *, left=None, right=None, lever=None, blocked=None,
           state=None, direction=None, model_valid=None, lamp_valid=None,
           lateral_active=None, now=None, **car_values):
    self.now = self.now + dt if now is None else now
    old_left, old_right = self.left, self.right
    if left is not None:
      self.left = left
    if right is not None:
      self.right = right
    self.rises += int(self.left and not old_left) + int(self.right and not old_right)
    if lever is not None:
      self.car.blinkerLever = lever
    for name, value in car_values.items():
      setattr(self.car, name, value)
    if state is not None:
      self.model.meta.laneChangeState = state
    if direction is not None:
      self.model.meta.laneChangeDirection = direction
    if blocked is not None:
      self.model.meta.laneChangeBlocked = blocked
    if model_valid is not None:
      self.model_valid = model_valid
    if lamp_valid is not None:
      self.lamp_valid = lamp_valid
    if lateral_active is not None:
      self.lateral_active = lateral_active
    result = self.helper.update(
      self.now, self.car, model_meta=self.model.meta, model_valid=self.model_valid,
      left_lamp=self.left, right_lamp=self.right, lamp_valid=self.lamp_valid,
      lateral_active=self.lateral_active,
    )
    assert result in (0, 1, 2)
    return result

  def tap(self, side=None, *, release=True):
    self.side = self.side if side is None else side
    name = "leftBlinkerStalkCount" if self.side == 1 else "rightBlinkerStalkCount"
    setattr(self.car, name, (getattr(self.car, name) + 1) % 256)
    self.car.leftBlinker, self.car.rightBlinker = self.side == 1, self.side == 2
    assert self.step(.01, lever=1, left=self.side == 1, right=self.side == 2) == 0
    if release:
      assert self.step(.10, lever=0) == 0

  def lamp(self, on, *, dt=.30, **changes):
    return self.step(dt, left=on and self.side == 1, right=on and self.side == 2, **changes)

  def armed(self):
    self.tap()
    assert self.lamp(False) == self.side
    return self


@pytest.mark.parametrize("side", (1, 2))
@pytest.mark.parametrize("blocked", (1, 2))
def test_three_stock_flashes_plus_two_extra_end_at_fifth_raw_falling_edge(side, blocked):
  feed = BlinkerFeed(side, blocked)
  feed.tap()
  assert feed.lamp(False) == side
  for pulse in range(2, 6):
    assert feed.lamp(True) == side
    assert feed.lamp(False) == (side if pulse < 5 else 0)
  assert feed.rises == 5
  # Smoothed CarState blinkers remain on throughout. Neither them nor later
  # raw transitions may create another extension from the consumed stalk tap.
  assert feed.car.leftBlinker == (side == 1)
  assert feed.car.rightBlinker == (side == 2)
  for _ in range(4):
    assert feed.lamp(True) == 0
    assert feed.lamp(False) == 0


def test_no_count_change_means_no_rearm_even_after_lamps_go_off_and_block_returns():
  feed = BlinkerFeed().armed()
  for _ in range(4):
    feed.lamp(True)
    feed.lamp(False)
  assert feed.step(blocked=0, lever=0) == 0
  assert feed.step(blocked=1, lever=1) == 0
  assert feed.step(lever=0) == 0
  assert feed.lamp(True) == 0
  assert feed.lamp(False) == 0


@pytest.mark.parametrize("first,second", ((1, 1), (1, 2), (2, 1), (2, 2)))
def test_new_physical_tap_is_allowed_after_previous_request_was_consumed(first, second):
  feed = BlinkerFeed(first).armed()
  for _ in range(4):
    feed.lamp(True)
    feed.lamp(False)
  feed.model.meta.laneChangeDirection = "left" if second == 1 else "right"
  feed.tap(second)
  assert feed.lamp(False) == second


@pytest.mark.parametrize("first,second", ((1, 1), (1, 2), (2, 1), (2, 2)))
def test_new_physical_tap_during_extension_starts_its_own_five_flash_budget(first, second):
  feed = BlinkerFeed(first).armed()
  feed.model.meta.laneChangeDirection = "left" if second == 1 else "right"
  feed.tap(second)
  assert feed.lamp(False) == second
  for pulse in range(2, 6):
    assert feed.lamp(True) == second
    assert feed.lamp(False) == (second if pulse < 5 else 0)


@pytest.mark.parametrize("side", (1, 2))
def test_uint8_stalk_counter_wrap_is_one_new_tap(side):
  feed = BlinkerFeed(side, initial_count=255)
  feed.tap()
  assert getattr(feed.car, "leftBlinkerStalkCount" if side == 1 else "rightBlinkerStalkCount") == 0
  assert feed.lamp(False) == side


@pytest.mark.parametrize("side", (1, 2))
def test_boot_mid_blink_without_new_stalk_edge_never_activates(side):
  feed = BlinkerFeed(side, initial_count=37, initial_lamp=True)
  for _ in range(6):
    assert feed.lamp(False) == 0
    assert feed.lamp(True) == 0


@pytest.mark.parametrize("guard", (
  "latch", "hazard", "model_stale", "lamp_stale", "can_invalid", "lateral_disabled", "park",
  "slow", "speed_nan", "opposite_lamp", "opposite_model", "starting", "finishing", "off",
))
def test_guard_cancels_extension_without_rearming_old_stalk(guard):
  feed = BlinkerFeed().armed()
  changes = {
    "latch": {"lever": 2}, "hazard": {"left": True, "right": True},
    "model_stale": {"model_valid": False}, "lamp_stale": {"lamp_valid": False},
    "can_invalid": {"canValid": False}, "lateral_disabled": {"lateral_active": False},
    "park": {"gearShifter": "park"}, "slow": {"vEgo": 2.}, "speed_nan": {"vEgo": math.nan},
    "opposite_lamp": {"right": True}, "opposite_model": {"direction": "right"},
    "starting": {"state": "laneChangeStarting"}, "finishing": {"state": "laneChangeFinishing"},
    "off": {"state": "off"},
  }[guard]
  assert feed.step(**changes) == 0
  assert feed.step(left=False, right=False, lever=0, model_valid=True, lamp_valid=True,
                   canValid=True, lateral_active=True, gearShifter="drive", vEgo=25.,
                   direction="left", state="preLaneChange") == 0
  assert feed.lamp(True) == 0
  assert feed.lamp(False) == 0


def test_block_clearing_keeps_existing_pre_lane_change_request_until_start_or_fifth_flash():
  feed = BlinkerFeed().armed()
  assert feed.lamp(True, blocked=0) == 1
  assert feed.lamp(False) == 1
  assert feed.lamp(True) == 1
  assert feed.lamp(False) == 1
  assert feed.lamp(True) == 1
  assert feed.lamp(False) == 1
  assert feed.lamp(True) == 1
  assert feed.lamp(False) == 0


def test_new_tap_waits_for_model_pre_state_to_arrive_before_arming():
  feed = BlinkerFeed()
  feed.model.meta.laneChangeState = "off"
  feed.model.meta.laneChangeDirection = "none"
  feed.tap()
  assert feed.lamp(False) == 0
  assert feed.step(state="preLaneChange", direction="left") == 1


def test_block_can_be_observed_later_in_stock_flash_window_but_not_after_third_fall():
  feed = BlinkerFeed(blocked=0)
  feed.tap()
  assert feed.lamp(False) == 0
  assert feed.lamp(True, blocked=1) == 1

  late = BlinkerFeed(blocked=0)
  late.tap()
  for pulse in range(1, 4):
    if pulse > 1:
      assert late.lamp(True) == 0
    assert late.lamp(False) == 0
  assert late.lamp(True, blocked=1) == 0
  assert late.lamp(False) == 0


def test_held_tap_is_not_extended_and_release_after_stock_flashes_cannot_arm():
  feed = BlinkerFeed()
  feed.tap(release=False)
  for pulse in range(1, 4):
    if pulse > 1:
      assert feed.lamp(True) == 0
    assert feed.lamp(False) == 0
  assert feed.step(lever=0) == 0
  assert feed.lamp(True) == 0


@pytest.mark.parametrize("static_on", (False, True))
def test_raw_lamp_without_phase_progress_cancels_active_extension(static_on):
  feed = BlinkerFeed().armed()
  if static_on:
    assert feed.lamp(True) == 1
  assert feed.lamp(static_on, dt=1.21) == 0
  assert feed.lamp(not static_on) == 0


def test_first_raw_on_off_pair_is_required_and_smoothed_blinker_edges_do_not_count():
  feed = BlinkerFeed()
  feed.tap()
  for index in range(10):
    assert feed.step(.10, leftBlinker=bool(index % 2)) == 0
  assert feed.step(.21, leftBlinker=True) == 0
  assert feed.lamp(False) == 0


@pytest.mark.parametrize("side", (1, 2))
def test_stalk_edge_can_precede_first_raw_lamp_on_without_arming_early(side):
  feed = BlinkerFeed(side)
  setattr(feed.car, "leftBlinkerStalkCount" if side == 1 else "rightBlinkerStalkCount", 1)
  assert feed.step(lever=1) == 0
  assert feed.step(lever=0) == 0
  assert feed.lamp(True) == 0
  assert feed.lamp(False) == side


def test_counter_first_observed_after_controller_stall_is_consumed_until_another_tap():
  feed = BlinkerFeed()
  feed.car.leftBlinkerStalkCount += 1
  assert feed.step(1.21, lever=1, left=True) == 0
  assert feed.step(lever=0) == 0
  assert feed.lamp(False) == 0
  feed.tap()
  assert feed.lamp(False) == 1


def test_total_session_timeout_stops_slow_but_continuing_lamp_edges():
  feed = BlinkerFeed()
  feed.tap()
  assert feed.lamp(False, dt=.80) == 1
  assert feed.lamp(True, dt=.90) == 1
  assert feed.lamp(False, dt=.90) == 1
  assert feed.lamp(True, dt=.90) == 1
  assert feed.lamp(False, dt=.90) == 1
  assert feed.lamp(True, dt=.90) == 1
  assert feed.lamp(False, dt=.90) == 0


@pytest.mark.parametrize("ambiguous", ("counter_jump", "both_counters", "opposite_raw"))
def test_ambiguous_input_does_not_create_a_supplemental_blinker_request(ambiguous):
  feed = BlinkerFeed()
  feed.car.leftBlinkerStalkCount += 2 if ambiguous == "counter_jump" else 1
  if ambiguous == "both_counters":
    feed.car.rightBlinkerStalkCount += 1
  assert feed.step(lever=1, left=True, right=ambiguous == "opposite_raw") == 0
  assert feed.step(lever=0) == 0
  assert feed.lamp(False) == 0


class NativeBlinkerFeed:
  """Raw CAN -> stalk decoding -> real controller helper -> ADRV hold bits."""
  def __init__(self, side, lamp_message, model_clock):
    self.side, self.lamp_message = side, lamp_message
    self.model_clock = model_clock
    self.packer = CANPacker("hyundai_canfd_generated")
    self.parser = CANParser("hyundai_canfd_generated", [("BLINKER_STALKS", 0), ("BLINKERS", 0), ("BLINKERS_ALT", 0)], 0)
    self.stalk_state = NS(left_stalk_prev=False, right_stalk_prev=False,
                          left_blinker_stalk_count=0, right_blinker_stalk_count=0)
    self.out = structs.CarState(canValid=True, gearShifter="drive", vEgo=25.,
                                leftBlinker=side == 1, rightBlinker=side == 2)
    self.model = log.ModelDataV2.new_message()
    self.model.meta.laneChangeState = "preLaneChange"
    self.model.meta.laneChangeDirection = "left" if side == 1 else "right"
    self.model.meta.laneChangeBlocked = 1
    self.model.meta.desireState = [0.] * 8
    stock = {name: 0 for name in self.packer.dbc.name_to_msg["ADRV_0x1ea"].sigs}
    stock.update(COUNTER=71, LEFT_BLINK_HOLD=1, RIGHT_BLINK_HOLD=1)
    self.cs = NS(cp=self.parser, out=self.out, MD=self.model, MD_valid=True,
                 scc_control=None, lfahda_cluster=None, adrv_0x161=None, adrv_0x200=None,
                 adrv_0x1ea=stock, ccnc_0x162=None, cruise_buttons_msg=None)
    self.cp = NS(flags=HyundaiFlags.CAMERA_SCC.value | HyundaiFlags.CANFD.value, openpilotLongitudinalControl=True)
    self.control = structs.CarControl(latActive=True)
    self.controller = carcontroller.CarController.__new__(carcontroller.CarController)
    self.controller.CP = self.cp
    self.controller.sm = NS(seen={"modelV2": True}, valid={"modelV2": True}, alive={"modelV2": True},
                            recv_time={"modelV2": model_clock.now})
    self.controller.blocked_lane_change_blinker = BlockedLaneChangeBlinker()
    self.controller.blocked_blinker_source = None
    self.helper_output = self.rises = 0
    self.previous_lamp = False
    self.now = 1.
    assert self.step(False, dt=0.) == (0, 0)

  def step(self, on, *, lever=0, dt=.30, state=None, desire=None, blocked=None, model_age=0.):
    self.now += dt
    stamp = int(self.now * 1e9)
    stalks = {"LEFT_BLINKER": lever > 0 and self.side == 1, "RIGHT_BLINKER": lever > 0 and self.side == 2,
              "LEFT_BLINKER_TAP": lever == 1 and self.side == 1, "RIGHT_BLINKER_TAP": lever == 1 and self.side == 2}
    suffix = "_ALT" if self.lamp_message == "BLINKERS_ALT" else ""
    lamps = {f"LEFT_LAMP{suffix}": on and self.side == 1, f"RIGHT_LAMP{suffix}": on and self.side == 2}
    self.parser.update([(stamp, [self.packer.make_can_msg("BLINKER_STALKS", 0, stalks),
                                self.packer.make_can_msg(self.lamp_message, 0, lamps)])])
    self.stalk_state.blinker_stalks = self.parser.vl["BLINKER_STALKS"]
    carstate.CarState._update_blinker_stalks(self.stalk_state, self.out)
    left, right, valid, source = carcontroller.read_blinker_feedback(self.cs, stamp)
    assert valid and source == self.lamp_message
    assert (left, right) == (on and self.side == 1, on and self.side == 2)
    self.rises += int(on and not self.previous_lamp)
    self.previous_lamp = on
    if state is not None:
      self.model.meta.laneChangeState = state
    if blocked is not None:
      self.model.meta.laneChangeBlocked = blocked
    if desire is not None:
      self.model.meta.desire = "laneChangeLeft" if desire == 3 else "laneChangeRight"
      self.model.meta.desireState = [0.] * 8
      self.model.meta.desireState[desire] = .95
    # Receive clocks stay in local MONOTONIC even when CAN/control use a replay
    # time domain. Comparing now_nanos to recv_time would reject this feed.
    self.model_clock.now = 123_000. + self.now
    self.controller.sm.recv_time["modelV2"] = self.model_clock.now - model_age
    self.controller._update_blocked_blinker_hold(self.control, self.cs, stamp)
    self.helper_output = self.cs.blocked_blinker_hold
    return self.packet()

  def packet(self):
    messages = hyundaicanfd.create_ccnc_messages(
      self.cp, self.packer, NS(ECAN=0, CAM=2), 5, self.control, self.cs, self.control.hudControl,
      0., 0, 0, False, 0, False, 0,
    )
    assert len(messages) == 1 and messages[0][0] == 0x1EA and messages[0][2] == 0
    decoded = CANParser("hyundai_canfd_generated", [("ADRV_0x1ea", 0)], 0)
    decoded.update([(int(self.now * 1e9), messages)])
    return tuple(int(decoded.vl["ADRV_0x1ea"][key]) for key in ("LEFT_BLINK_HOLD", "RIGHT_BLINK_HOLD"))


@pytest.fixture
def native_feed(monkeypatch):
  class MemoryParams:
    def get_int(self, _key):
      return 0

  monkeypatch.setattr(hyundaicanfd, "Params", MemoryParams)
  for name, value in (("_lane_line_check", 0), ("_blinker_timer", 0),
                      ("_lever_max_level", 0), ("_lane_changed_this_session", False)):
    monkeypatch.setattr(hyundaicanfd.create_ccnc_messages, name, value, raising=False)
  clock = NS(now=123_000.)
  monkeypatch.setattr(carcontroller.time, "monotonic", lambda: clock.now)
  return lambda side, lamp_message: NativeBlinkerFeed(side, lamp_message, clock)


@pytest.mark.parametrize("side", (1, 2))
@pytest.mark.parametrize("lamp_message", ("BLINKERS", "BLINKERS_ALT"))
def test_native_raw_can_five_flash_wave_controls_actual_cluster_hold_bits(native_feed, side, lamp_message):
  feed = native_feed(side, lamp_message)
  expected = (1, 0) if side == 1 else (0, 1)
  assert feed.step(True, lever=1, dt=.01) == (0, 0)
  assert feed.step(True, lever=0, dt=.10) == (0, 0)
  assert feed.step(False) == expected
  for pulse in range(2, 6):
    assert feed.step(True) == expected
    assert feed.step(False) == (expected if pulse < 5 else (0, 0))
  assert feed.rises == 5
  assert (feed.out.leftBlinkerStalkCount, feed.out.rightBlinkerStalkCount) == ((1, 0) if side == 1 else (0, 1))
  assert feed.step(False) == (0, 0)
  assert feed.step(True) == (0, 0)


@pytest.mark.parametrize("side", (1, 2))
def test_native_lane_change_start_releases_helper_and_preserves_existing_normal_hold(native_feed, side):
  feed = native_feed(side, "BLINKERS")
  feed.step(True, lever=1, dt=.01)
  feed.step(True, dt=.10)
  expected = (1, 0) if side == 1 else (0, 1)
  assert feed.step(False) == expected
  assert feed.helper_output == side
  assert feed.step(True, state="laneChangeStarting", desire=3 if side == 1 else 4, blocked=0) == expected
  assert feed.helper_output == 0


@pytest.mark.parametrize("normal_side", (1, 2))
def test_normal_model_hold_takes_precedence_over_opposite_supplement(native_feed, normal_side):
  feed = native_feed(normal_side, "BLINKERS")
  desire = 3 if normal_side == 1 else 4
  feed.model.meta.desire = "laneChangeLeft" if normal_side == 1 else "laneChangeRight"
  feed.model.meta.desireState[desire] = .95
  feed.cs.blocked_blinker_hold = 2 if normal_side == 1 else 1
  assert feed.packet() == ((1, 0) if normal_side == 1 else (0, 1))


@pytest.mark.parametrize("fault", ("unseen", "invalid", "not_alive", "stale", "future"))
def test_real_controller_cancels_extension_on_invalid_model_receipt(native_feed, fault):
  feed = native_feed(1, "BLINKERS")
  feed.step(True, lever=1, dt=.01)
  feed.step(True, dt=.10)
  assert feed.step(False) == (1, 0)
  if fault in ("unseen", "invalid", "not_alive"):
    group = {"unseen": "seen", "invalid": "valid", "not_alive": "alive"}[fault]
    getattr(feed.controller.sm, group)["modelV2"] = False
  age = .251 if fault == "stale" else -.001 if fault == "future" else 0.
  assert feed.step(True, model_age=age) == (0, 0)
  for name in ("seen", "valid", "alive"):
    getattr(feed.controller.sm, name)["modelV2"] = True
  assert feed.step(False) == (0, 0)


def test_real_controller_does_not_count_a_lamp_source_switch_as_a_new_phase(native_feed):
  feed = native_feed(1, "BLINKERS")
  feed.step(True, lever=1, dt=.01)
  feed.step(True, dt=.10)
  assert feed.step(False) == (1, 0)
  feed.lamp_message = "BLINKERS_ALT"
  assert feed.step(True) == (0, 0)
  assert feed.step(False) == (0, 0)


@pytest.mark.parametrize("fault", ("non_canfd", "non_camera_scc", "stock_longitudinal", "missing_adrv", "invalid_can"))
def test_real_controller_cancels_on_unsupported_vehicle_path_or_can_error(native_feed, fault):
  feed = native_feed(1, "BLINKERS")
  feed.step(True, lever=1, dt=.01)
  feed.step(True, dt=.10)
  assert feed.step(False) == (1, 0)
  original_flags, original_adrv = feed.cp.flags, feed.cs.adrv_0x1ea
  if fault == "non_canfd":
    feed.cp.flags &= ~HyundaiFlags.CANFD.value
  elif fault == "non_camera_scc":
    feed.cp.flags &= ~HyundaiFlags.CAMERA_SCC.value
  elif fault == "stock_longitudinal":
    feed.cp.openpilotLongitudinalControl = False
  elif fault == "missing_adrv":
    feed.cs.adrv_0x1ea = None
  else:
    feed.out.canValid = False
  feed.controller._update_blocked_blinker_hold(feed.control, feed.cs, int(feed.now * 1e9))
  assert feed.cs.blocked_blinker_hold == 0
  feed.cp.flags, feed.cs.adrv_0x1ea = original_flags, original_adrv
  feed.cp.openpilotLongitudinalControl, feed.out.canValid = True, True
  assert feed.step(True) == (0, 0)


@pytest.mark.parametrize("lamp_message", ("BLINKERS", "BLINKERS_ALT"))
def test_native_lamp_feedback_requires_both_stalk_and_lamp_receipts_within_250_ms(lamp_message):
  packer = CANPacker("hyundai_canfd_generated")
  parser = CANParser("hyundai_canfd_generated", [("BLINKER_STALKS", 0), (lamp_message, 0)], 0)
  cs = NS(cp=parser)
  stalk = packer.make_can_msg("BLINKER_STALKS", 0, {})
  lamp = packer.make_can_msg(lamp_message, 0, {"LEFT_LAMP": 1})
  parser.update([(1_000_000_000, [stalk, lamp])])
  assert carcontroller.read_blinker_feedback(cs, 1_250_000_000) == (True, False, True, lamp_message)
  assert carcontroller.read_blinker_feedback(cs, 1_250_000_001) == (False, False, False, None)
  assert carcontroller.read_blinker_feedback(cs, 999_999_999) == (False, False, False, None)
  parser.update([(1_300_000_000, [packer.make_can_msg(lamp_message, 0, {"LEFT_LAMP": 1})])])
  assert carcontroller.read_blinker_feedback(cs, 1_300_000_000) == (False, False, False, None)
