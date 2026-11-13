"""OEM display-radar BSD handoff and real CAN input regressions."""

from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.blindspot import CornerBsdSupplement
from opendbc.car.hyundai.blindspot_inputs import (
  CORNER_MESSAGES, MAX_SAMPLE_AGE_NS, STOCK_BSD_MESSAGE, CornerSample,
  read_corner_samples, register_seen_messages,
)
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai import carstate, hyundaicanfd, interface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, HyundaiFlags


START = 1_000_000_000


def observations(timestamp, *, side="L", front=0.0, rear=0.0, source="ADRV_0x1ea"):
  f, r = ("LF", "LR") if side == "L" else ("RF", "RR")
  return {f: CornerSample(front, 2.0, timestamp, source), r: CornerSample(rear, 2.0, timestamp, source)}


@pytest.mark.parametrize("side", ("L", "R"))
def test_ego_overtake_keeps_warning_through_front_to_rear_handoff(side):
  bsd = CornerBsdSupplement()
  now = START
  for index in range(20):
    now += 10_000_000
    assert bsd.update(now, observations(now, side=side, front=4.0 - index * 0.05))[side]
  assert bsd.sides[side].origin == "front"
  # The B-pillar/side gap has no displayed object; bridge it.
  for _ in range(10):
    now += 10_000_000
    assert bsd.update(now, observations(now, side=side))[side]
  for index in range(60):
    now += 10_000_000
    assert bsd.update(now, observations(now, side=side, rear=2.0 + index * 0.05))[side]
  assert bsd.sides[side].origin == "front"
  # Previously rear=5 m immediately cancelled the front-entry episode.
  now += 10_000_000
  assert bsd.update(now, observations(now, side=side, rear=5.0))[side]
  now += 10_000_000
  assert bsd.update(now, observations(now, side=side, rear=5.05))[side]
  # Bridge remains bounded; it is not a permanently latched warning.
  assert not bsd.update(now + 2_000_000_000, {})[side]


@pytest.mark.parametrize("side", ("L", "R"))
def test_rear_entry_still_releases_when_vehicle_moves_away_ahead(side):
  bsd = CornerBsdSupplement()
  now = START
  for index in range(10):
    now += 50_000_000
    assert bsd.update(now, observations(now, side=side, rear=4.0 - index * 0.1))[side]
  assert bsd.sides[side].origin == "rear"
  for distance in (3.0, 3.2, 3.4, 8.0):
    now += 50_000_000
    result = bsd.update(now, observations(now, side=side, front=distance))
  assert not result[side]


@pytest.mark.parametrize("interval_ns", (10_000_000, 50_000_000, 100_000_000))
def test_velocity_uses_actual_fresh_message_time(interval_ns):
  bsd = CornerBsdSupplement()
  for index in range(12):
    now = START + index * interval_ns
    distance = 5.0 - index * interval_ns * 1e-9
    bsd.update(now, observations(now, front=distance))
  assert bsd.velocities["LF"] == pytest.approx(-1.0)


def test_cached_observation_does_not_replenish_timer_or_history():
  bsd = CornerBsdSupplement()
  sample = observations(START, front=3.0)
  assert bsd.update(START, sample)["L"]
  deadline = bsd.sides["L"].deadline_ns
  for index in range(1, 20):
    assert bsd.update(START + index * 10_000_000, sample)["L"]
  assert len(bsd.history["LF"]) == 1
  assert bsd.sides["L"].deadline_ns == deadline
  assert not bsd.update(START + 5_000_000_000, sample)["L"]


def test_parallel_vehicle_with_fresh_measurements_keeps_warning():
  bsd = CornerBsdSupplement()
  for index in range(150):
    now = START + index * 100_000_000
    result = bsd.update(now, observations(now, front=3.0))
    assert result == {"L": True, "R": False}


def test_distance_jump_and_source_switch_do_not_create_departure_speed():
  bsd = CornerBsdSupplement()
  bsd.update(START, observations(START, front=2.0))
  bsd.update(START + 50_000_000, observations(START + 50_000_000, front=5.0))
  assert bsd.velocities["LF"] == 0.0
  bsd.update(START + 100_000_000, observations(START + 100_000_000, front=3.0, source="CCNC_0x162"))
  assert bsd.velocities["LF"] == 0.0


def test_clock_reset_cannot_keep_an_old_warning_deadline():
  bsd = CornerBsdSupplement()
  assert bsd.update(START + 10_000_000_000, observations(START + 10_000_000_000, front=3.0))["L"]
  assert not bsd.update(START, {})["L"]


@pytest.mark.parametrize("velocity,limit", ((-10.0, 12.0), (-5.0, 12.0), (-0.5, 6.0), (0.0, 6.0), (0.5, 6.0), (5.0, 2.0), (10.0, 2.0)))
def test_existing_warning_zone_limits_are_preserved(velocity, limit):
  assert CornerBsdSupplement._front_limit(velocity) == pytest.approx(limit)


class CanFeed:
  def __init__(self, stock=True):
    self.now = START
    self.packer = CANPacker("hyundai_canfd_generated")
    self.pt = CANParser("hyundai_canfd_generated", [], 0)
    self.cam = CANParser("hyundai_canfd_generated", [], 2)
    self.pt.controls_ready = self.cam.controls_ready = True
    self.state = CarState.__new__(CarState)
    self.state.CP = SimpleNamespace(enableBsm=stock)
    self.state.corner_bsd = CornerBsdSupplement()
    self.state.adrv_0x1ea = self.state.ccnc_0x162 = None
    self.state.controls_ready_count = 500

  def step(self, specs=(), elapsed_ns=10_000_000):
    self.now += elapsed_ns
    frames = [self.packer.make_can_msg(name, bus, values) for name, bus, values in specs]
    packets = [(self.now, frames)]
    self.pt.update(packets)
    self.cam.update(packets)
    ret = structs.CarState.new_message()
    self.state._update_canfd_blindspots(ret, self.pt, self.cam)
    return ret

  def decoded(self, specs):
    self.step(specs)  # discovery precedes first decoded frame
    return self.step(specs)


def test_late_camera_stream_is_registered_outside_startup_and_remains_optional():
  feed = CanFeed()
  feed.step()
  assert "ADRV_0x1ea" not in feed.cam.vl
  ret = feed.decoded([("ADRV_0x1ea", 2, {"LF_DETECT_DISTANCE": 3.0})])
  assert ret.leftBlindspot
  assert feed.state.controls_ready_count > 200
  address = feed.cam.dbc.name_to_msg["ADRV_0x1ea"].address
  assert feed.cam.message_states[address].ignore_alive
  assert feed.state.adrv_0x1ea is feed.cam.vl["ADRV_0x1ea"]


def test_near_object_is_not_hidden_by_far_source_and_coordinates_stay_paired():
  feed = CanFeed()
  ret = feed.decoded([
    ("ADRV_0x1ea", 2, {"LF_DETECT_DISTANCE": 3.0, "LF_DETECT_LATERAL": 1.5}),
    ("CCNC_0x162", 2, {"LF_DETECT_DISTANCE": 15.0, "LF_DETECT_LATERAL": 4.0}),
  ])
  assert ret.leftBlindspot
  assert ret.leftLongDist == pytest.approx(3.0)
  assert ret.leftLatDist == pytest.approx(1.5)
  # The other source may remain present, but it cannot refresh the close one.
  for _ in range(35):
    ret = feed.step([("CCNC_0x162", 2, {"LF_DETECT_DISTANCE": 15.0, "LF_DETECT_LATERAL": 4.0})])
  assert ret.leftLongDist == pytest.approx(15.0)
  assert ret.leftLatDist == pytest.approx(4.0)


def test_stopped_distance_stream_expires_and_bsd_hold_eventually_clears():
  feed = CanFeed()
  assert feed.decoded([("ADRV_0x1ea", 2, {"LF_DETECT_DISTANCE": 3.0})]).leftBlindspot
  ret = feed.step(elapsed_ns=MAX_SAMPLE_AGE_NS + 1)
  assert ret.leftLongDist == 0.0
  assert ret.leftBlindspot
  assert not feed.step(elapsed_ns=5_000_000_000).leftBlindspot


def test_our_ecan_cluster_copy_is_not_used_as_fresh_oem_distance():
  feed = CanFeed()
  ret = feed.decoded([("ADRV_0x1ea", 0, {"LF_DETECT_DISTANCE": 2.0})])
  assert not ret.leftBlindspot
  assert ret.leftLongDist == 0.0
  assert "ADRV_0x1ea" not in feed.pt.vl


@pytest.mark.parametrize("side", ("L", "R"))
def test_real_can_front_rear_handoff_does_not_drop_bsd(side):
  feed = CanFeed()
  front, rear = ("LF", "LR") if side == "L" else ("RF", "RR")
  flag = "leftBlindspot" if side == "L" else "rightBlindspot"
  for index in range(20):
    ret = feed.step([("ADRV_0x1ea", 2, {f"{front}_DETECT_DISTANCE": 4.0 - index * 0.1})], 50_000_000)
    if index > 0:
      assert getattr(ret, flag)
  for _ in range(5):
    assert getattr(feed.step([("ADRV_0x1ea", 2, {})], 50_000_000), flag)
  for index in range(43):
    ret = feed.step([("ADRV_0x1ea", 2, {f"{rear}_DETECT_DISTANCE": 1.0 + index * 0.1})], 50_000_000)
    assert getattr(ret, flag)
  assert feed.state.corner_bsd.sides[side].origin == "front"
  assert not getattr(feed.step(elapsed_ns=4_000_000_000), flag)


@pytest.mark.parametrize("bus", (0, 2))
def test_stock_warning_cannot_be_cleared_by_supplement(bus):
  feed = CanFeed()
  ret = feed.decoded([(STOCK_BSD_MESSAGE, bus, {"FL_INDICATOR": 1, "FR_INDICATOR": 1})])
  assert ret.leftBlindspot and ret.rightBlindspot
  # Camera distance disappearing must not turn off a fresh OEM warning.
  ret = feed.step([(STOCK_BSD_MESSAGE, bus, {"FL_INDICATOR": 1, "FR_INDICATOR": 1})])
  assert ret.leftBlindspot and ret.rightBlindspot


def test_stock_source_loss_does_not_block_fresh_other_bus():
  feed = CanFeed()
  feed.decoded([(STOCK_BSD_MESSAGE, 0, {"FL_INDICATOR": 1})])
  for _ in range(40):
    ret = feed.step([(STOCK_BSD_MESSAGE, 2, {"FR_INDICATOR": 1})])
  assert not ret.leftBlindspot
  assert ret.rightBlindspot
  ret = feed.step(elapsed_ns=MAX_SAMPLE_AGE_NS + 1)
  assert not ret.leftBlindspot and not ret.rightBlindspot


@pytest.mark.parametrize("name", CORNER_MESSAGES)
@pytest.mark.parametrize("distance", (0.0, 0.1, float("nan"), float("inf"), -1.0))
def test_invalid_distance_does_not_create_near_object(name, distance):
  feed = CanFeed()
  register_seen_messages(feed.cam, CORNER_MESSAGES)
  feed.cam._add_message(name, float("nan"))
  feed.cam.vl[name].update(LF_DETECT_DISTANCE=distance, LF_DETECT_LATERAL=2.0)
  feed.cam.ts_nanos[name]["LF_DETECT_DISTANCE"] = feed.now
  assert read_corner_samples(feed.cam, feed.now)["LF"].distance == 0.0


def test_valid_measurement_needs_real_timestamp():
  feed = CanFeed()
  feed.cam._add_message("ADRV_0x1ea", float("nan"))
  feed.cam.vl["ADRV_0x1ea"]["LF_DETECT_DISTANCE"] = 2.0
  assert read_corner_samples(feed.cam, feed.now) == {}


@pytest.mark.parametrize("age_ns,accepted", ((-1, False), (0, True), (MAX_SAMPLE_AGE_NS, True), (MAX_SAMPLE_AGE_NS + 1, False)))
def test_camera_measurement_age_boundary(age_ns, accepted):
  feed = CanFeed()
  feed.decoded([("ADRV_0x1ea", 2, {"LF_DETECT_DISTANCE": 3.0})])
  samples = read_corner_samples(feed.cam, feed.now + age_ns)
  assert ("LF" in samples) == accepted


@pytest.mark.parametrize("name,distance", (("ADRV_0x1ea", 30.0), ("CCNC_0x162", 30.0)))
def test_fresh_far_distance_remains_available_without_asserting_bsd(name, distance):
  feed = CanFeed()
  ret = feed.decoded([(name, 2, {"LF_DETECT_DISTANCE": distance, "LF_DETECT_LATERAL": 3.0})])
  assert ret.leftLongDist == pytest.approx(distance)
  assert ret.leftLatDist == pytest.approx(3.0)
  assert not ret.leftBlindspot


def test_camera_only_oem_bsm_is_detected(monkeypatch):
  class Params:
    def get_int(self, _key):
      return 0
    def get_bool(self, _key):
      return False
    def put_int(self, _key, _value):
      pass
  for module in (carstate, interface, hyundaicanfd):
    monkeypatch.setattr(module, "Params", Params)
  fingerprint = gen_empty_fingerprint()
  fingerprint[CanBus(None, fingerprint).CAM][0x1ba] = 24
  cp = interface.CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
  assert cp.enableBsm


def test_real_ev6_carstate_update_runs_bsd_after_startup_discovery(monkeypatch):
  class Params:
    def get(self, key, *args, **kwargs):
      return str({0: {}, 1: {}, 2: {}, 4: {}, 5: {}, 6: {}}) if key == "FingerPrints" else None
    def get_int(self, _key):
      return 0
    def get_bool(self, _key):
      return False
  for module in (carstate, hyundaicanfd):
    monkeypatch.setattr(module, "Params", Params)
  cp = structs.CarParams.new_message(
    brand="hyundai", carFingerprint=CAR.KIA_EV6, flags=int(HyundaiFlags.CANFD),
    enableBsm=True, wheelSpeedFactor=1.0, openpilotLongitudinalControl=True,
  )
  cp.init("safetyConfigs", 1)
  state = CarState(cp)
  state.controls_ready_count = 500
  parsers = state.get_can_parsers(cp)
  for parser in parsers.values():
    parser.controls_ready = True
  packer = CANPacker("hyundai_canfd_generated")
  for index in range(3):
    now = START + index * 50_000_000
    frames = [packer.make_can_msg("ADRV_0x1ea", CanBus(cp).CAM, {
      "LF_DETECT_DISTANCE": 3.0, "LF_DETECT_LATERAL": 2.0,
    })]
    for parser in parsers.values():
      parser.update([(now, frames)])
    result = state.update(parsers)
  assert result.leftBlindspot
  assert not result.rightBlindspot
  assert result.leftLongDist == pytest.approx(3.0)
