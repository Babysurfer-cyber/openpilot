"""Auto mode can distinguish fresh optional navigation sources from caches."""

import math
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carstate
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, HyundaiFlags


DBC = "hyundai_canfd_generated"
ROAD = "HDA_INFO_4A3"
PROFILE = "NEW_MSG_4BE"
START = 2_000_000_000


def parser():
  return CANParser(DBC, [(ROAD, math.nan), (PROFILE, math.nan)], 0)


def receive(cp, timestamp, message):
  values = {"SPEED_LIMIT": 100, "LinkClass": 1} if message == ROAD else {
    "PROLONG_VALUE": 0x150, "PROLONG_OFFSET": 100, "PROLONG_PROFILE_TYPE": 16,
  }
  frame = CANPacker(DBC).make_can_msg(message, 0, values)
  cp.update([(timestamp, [frame])])


def state(cp):
  cs = CarState.__new__(CarState)
  cs.hda_info_4a3 = cp.vl[ROAD]
  return cs


@pytest.mark.parametrize("message", (ROAD, PROFILE))
@pytest.mark.parametrize("age,valid", ((0, True), (1_000_000_000, True),
                                        (1_000_000_001, False), (-1, False)))
def test_navigation_source_requires_a_recent_nonfuture_receive(message, age, valid):
  cp = parser()
  receive(cp, START, message)
  cp.update([(START + age, [])])
  assert CarState._vehicle_navi_message_recent(cp, message) == valid


def test_no_optional_navigation_samples_are_unknown_without_a_can_fault():
  cp = parser()
  cp.update([(START, [])])
  ret = structs.CarState.new_message(navDataValid=True, speedLimitDataValid=True)
  state(cp)._update_vehicle_navi_freshness(cp, ret)
  assert not ret.navDataValid and not ret.speedLimitDataValid
  assert cp.can_valid and not cp.bus_timeout


def test_navigation_not_discovered_yet_does_not_publish_valid_road_metadata():
  cp = parser()
  receive(cp, START, ROAD)
  cs = state(cp)
  cs.hda_info_4a3 = None
  ret = structs.CarState.new_message()
  cs._update_vehicle_navi_freshness(cp, ret)
  assert not ret.navDataValid and not ret.speedLimitDataValid


def test_fresh_road_does_not_make_an_old_profile_limit_valid():
  cp = parser()
  receive(cp, START, PROFILE)
  receive(cp, START + 1_000_000_001, ROAD)
  ret = structs.CarState.new_message(speedLimit=50, navSpeedLimit=100)
  cs = state(cp)
  cs._update_vehicle_navi_freshness(cp, ret, mixed_source=True)
  assert ret.navDataValid and not ret.speedLimitDataValid
  # Freshness is metadata; legacy limit values and selection are preserved.
  assert ret.speedLimit == 50 and ret.navSpeedLimit == 100


def test_fresh_profile_does_not_make_old_road_metadata_valid():
  cp = parser()
  receive(cp, START, ROAD)
  receive(cp, START + 1_000_000_001, PROFILE)
  ret = structs.CarState.new_message()
  cs = state(cp)
  cs._update_vehicle_navi_freshness(cp, ret, mixed_source=True)
  assert not ret.navDataValid and ret.speedLimitDataValid
  cs._update_vehicle_navi_freshness(cp, ret)
  assert not ret.navDataValid and not ret.speedLimitDataValid


def test_fresh_road_then_profile_preserve_independent_source_flags():
  cp = parser()
  receive(cp, START, ROAD)
  ret = structs.CarState.new_message()
  cs = state(cp)
  cs._update_vehicle_navi_freshness(cp, ret)
  assert ret.navDataValid and ret.speedLimitDataValid
  cs._update_vehicle_navi_freshness(cp, ret, mixed_source=True)
  assert ret.navDataValid and not ret.speedLimitDataValid
  receive(cp, START + 100_000_000, PROFILE)
  cs._update_vehicle_navi_freshness(cp, ret, mixed_source=True)
  assert ret.navDataValid and ret.speedLimitDataValid


def test_serialized_legacy_car_state_has_unknown_navigation_freshness():
  original = structs.CarState.new_message(navSpeedLimit=100, speedLimit=100)
  with structs.CarState.from_bytes(original.to_bytes()) as decoded:
    assert not decoded.navDataValid and not decoded.speedLimitDataValid


def ev6_parsers():
  cp = structs.CarParams.new_message(
    brand="hyundai", carFingerprint=CAR.KIA_EV6,
    flags=int(HyundaiFlags.CANFD | HyundaiFlags.CANFD_ALT_BUTTONS),
  )
  cp.init("safetyConfigs", 1)
  cs = CarState.__new__(CarState)
  cs.CP = cp
  return cs, cs.get_can_parsers_canfd(cp)


def test_canfd_navigation_registered_upfront_is_optional_when_absent():
  cs, can_parsers = ev6_parsers()
  cp = can_parsers[Bus.pt]
  assert ROAD in cp.vl
  assert cp.message_states[0x4A3].ignore_alive
  cp.update([(START + 30_000_000_000, [])])
  assert cp.can_valid and not cp.bus_timeout
  cs.hda_info_4a3 = cp.vl[ROAD]
  ret = structs.CarState.new_message()
  cs._update_vehicle_navi_freshness(cp, ret)
  assert not ret.navDataValid and not ret.speedLimitDataValid


def test_navigation_starting_after_discovery_updates_existing_road_cache(monkeypatch):
  cs, can_parsers = ev6_parsers()
  cp = can_parsers[Bus.pt]
  cp.enable_capture = False
  cp.update([(START, [])])
  cs.controls_ready_count = 122
  cs.hda_info_4a3 = None
  monkeypatch.setattr(carstate, "Params", lambda: SimpleNamespace(get_bool=lambda _key: True))

  # Discovery's one-time cache binding succeeds before navigation sends 4A3.
  cs.monitor_fingerprint(can_parsers, canfd=True)
  assert cs.controls_ready_count == 123
  assert cs.hda_info_4a3 is cp.vl[ROAD]
  ret = structs.CarState.new_message()
  cs._update_vehicle_navi_freshness(cp, ret)
  assert not ret.navDataValid

  # No second discovery is necessary after the initial capture window ends.
  cs.controls_ready_count = 201
  receive(cp, START + 60_000_000_000, ROAD)
  cs.monitor_fingerprint(can_parsers, canfd=True)
  assert cs.hda_info_4a3["SPEED_LIMIT"] == 100
  assert cs.hda_info_4a3["LinkClass"] == 1
  cs._update_vehicle_navi_freshness(cp, ret)
  assert ret.navDataValid and ret.speedLimitDataValid
