import time

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai import interface, radar_interface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.tests.test_corner_radar_180 import (
  Feed, car_params as corner_180_car_params, raw_object, settings, wire_payload,
)
from opendbc.car.hyundai.values import CAR, HyundaiExtFlags


DBC = "hyundai_canfd_corner_radar_235_generated"
ADDRESSES = range(0x235, 0x249)


def car_params(*, mixed=False):
  cp = corner_180_car_params(corner=mixed)
  cp.extFlags |= int(HyundaiExtFlags.CORNER_RADAR_OBJECTS_235)
  return cp


def corner_points(result, source="corner235"):
  assert result is not None
  return [point for point in result.points if point.measured and str(point.radarSource) == source]


@pytest.mark.parametrize("bus", [0, 1, 4, 5])
def test_wire_decode_identity_age_and_motion(bus):
  # 0x235 uses the same 128-bit object layout as the first 0x180 slot.
  data = wire_payload(raw_object(46, age=23, x=500, y=1998, vx=1900, vy=520, ax=-30))
  parser = CANParser(DBC, [(f"CORNER_RADAR_235_OBJECTS_{address:x}", 33) for address in ADDRESSES], bus)
  assert parser.update([(time.monotonic_ns(), [(0x235, data, bus)])]) == {0x235}
  expected = {
    "OBJ_QUAL_LEVEL": 90, "OBJ_AGE": 23, "OBJ_OBJECT_ID": 46,
    "OBJ_REL_POS_X": 25.0, "OBJ_REL_POS_Y": -2.5,
    "OBJ_REL_VEL_X": -5.0, "OBJ_REL_VEL_Y": 1.0, "OBJ_REL_ACCEL_X": -1.5,
  }
  for name, value in expected.items():
    assert parser.vl[0x235][name] == pytest.approx(value)
  assert CANPacker(DBC).make_can_msg(0x235, bus, expected) == (0x235, data, bus)


@pytest.mark.parametrize("hda2", [False, True])
@pytest.mark.parametrize("defect", [None, "missing", "dlc", "bus"])
def test_fingerprint_requires_complete_32_byte_bank(settings, hda2, defect):
  settings["CanfdHDA2"] = int(hda2)
  fingerprint = gen_empty_fingerprint()
  buses = CanBus(None, fingerprint, hda2)
  fingerprint[buses.ACAN].update({address: 32 for address in ADDRESSES})
  if defect == "missing":
    del fingerprint[buses.ACAN][0x240]
  elif defect == "dlc":
    fingerprint[buses.ACAN][0x240] = 16
  elif defect == "bus":
    fingerprint[buses.ECAN] = fingerprint[buses.ACAN]
    fingerprint[buses.ACAN] = {}
  cp = interface.CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
  assert bool(cp.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_235) == (defect is None)
  assert not cp.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_180


@pytest.mark.parametrize("enabled", [0, 1, 2])
def test_235_parser_obeys_existing_corner_setting(settings, enabled):
  settings["EnableCornerRadar"] = enabled
  radar = radar_interface.RadarInterface(car_params())
  assert (radar.rcp_corner_objects is not None) == (enabled > 0)
  assert radar.rcp_corner_objects_180 is None


def test_twenty_objects_publish_at_existing_cadence(settings):
  radar = radar_interface.RadarInterface(car_params())
  feed = Feed(radar)
  bus = CanBus(radar.CP).ACAN
  messages = [(address, wire_payload(raw_object(index + 1, x=400 + index * 100)), bus)
              for index, address in enumerate(ADDRESSES)]
  outputs = [feed.step(messages if index % 3 == 0 else ()) for index in range(20)]
  assert [index + 1 for index, result in enumerate(outputs) if result is not None] == [5, 10, 15, 20]
  points = corner_points(outputs[-1])
  assert len(points) == 20
  assert len({point.trackId for point in points}) == 20
  assert min(point.trackId for point in points) >= 1000
  assert not outputs[-1].errors.canError


def test_235_slot_handoff_and_recycled_identity(settings):
  radar = radar_interface.RadarInterface(car_params())
  feed = Feed(radar)
  bus = CanBus(radar.CP).ACAN
  identity = corner_points(feed.publish([(0x235, wire_payload(raw_object(17)), bus)]))[0].trackId
  duplicate = corner_points(feed.publish([
    (0x235, wire_payload(raw_object(17, age=21)), bus),
    (0x236, wire_payload(raw_object(17, x=402, age=22)), bus),
  ]))
  assert len(duplicate) == 1
  assert duplicate[0].trackId == identity
  moved = corner_points(feed.publish([
    (0x235, bytes(32), bus), (0x236, wire_payload(raw_object(17, x=404, age=23)), bus),
  ]))
  assert len(moved) == 1
  assert moved[0].trackId == identity
  recycled = corner_points(feed.publish([(0x236, wire_payload(raw_object(17, x=404, age=1)), bus)]))
  assert recycled[0].trackId != identity


def test_mixed_banks_keep_sensor_identities_and_expiry_separate(settings):
  radar = radar_interface.RadarInterface(car_params(mixed=True))
  feed = Feed(radar)
  bus = CanBus(radar.CP).ACAN
  first = feed.publish([
    (0x180, wire_payload(raw_object(9)), bus),
    (0x235, wire_payload(raw_object(9)), bus),
  ])
  first_180 = corner_points(first, "corner180")[0]
  first_235 = corner_points(first)[0]
  assert first_180.trackId != first_235.trackId

  survivors = [(0x184, wire_payload(raw_object(10, x=1200)), bus),
               (0x248, wire_payload(raw_object(10, x=1200)), bus)]
  for _ in range(12):
    result = feed.publish(survivors)
  active = corner_points(result) + corner_points(result, "corner180")
  assert len(active) == 2
  assert len({point.trackId for point in active}) == 2
  assert all(point.trackId not in {first_180.trackId, first_235.trackId} for point in active)
  assert all(point.dRel == pytest.approx(60.0) for point in active)
  # Refreshing 0x180 cannot keep a cached 0x235 object alive, or vice versa.
  for _ in range(12):
    result = feed.publish([survivors[0]])
  assert corner_points(result) == []
  assert len(corner_points(result, "corner180")) == 1


def test_missing_235_dbc_disables_only_235_parser(settings, monkeypatch, tmp_path):
  monkeypatch.setattr(radar_interface, "DBC_PATH", str(tmp_path))
  radar = radar_interface.RadarInterface(car_params())
  assert radar.rcp_corner_objects is None
  assert radar.radar_off_can


@pytest.mark.parametrize("hda2", [False, True])
def test_fingerprint_automatically_enables_both_detected_banks(settings, hda2):
  settings["CanfdHDA2"] = int(hda2)
  fingerprint = gen_empty_fingerprint()
  bus = CanBus(None, fingerprint, hda2).ACAN
  fingerprint[bus].update({address: 32 for address in ADDRESSES})
  fingerprint[bus].update({address: 32 for address in range(0x180, 0x185)})
  cp = interface.CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
  radar = radar_interface.RadarInterface(cp)
  assert radar.rcp_corner_objects is not None
  assert radar.rcp_corner_objects_180 is not None
  assert radar.rcp_corner_objects.bus == bus
  assert radar.rcp_corner_objects_180.bus == bus


@pytest.mark.parametrize("native_state", [0, 1, 2, 3])
def test_group2_reports_native_state_separately_from_track_age(settings, native_state):
  settings["EnableRadarTracks"] = 1
  cp = corner_180_car_params(available=True, corner=False)
  radar = radar_interface.RadarInterface(cp)
  packer = CANPacker("hyundai_canfd_radar_generated")
  bus = CanBus(cp).ACAN
  messages = [packer.make_can_msg(address, bus, {
    "VALID_CNT": 20, "VALID": native_state, "LONG_DIST": 40,
    "LAT_DIST": 1, "REL_SPEED": -2,
  }) for address in range(radar.radar_start_addr, radar.trigger_msg_tracks + 1)]
  result = Feed(radar).step(messages)
  points = [point for point in result.points if str(point.radarSource) == "frontRadar"]
  assert points
  assert all(point.measured and point.trackState == native_state for point in points)


def test_group1_age_counters_do_not_synthesize_confirmed_state(settings):
  settings["EnableRadarTracks"] = 1
  cp = corner_180_car_params(available=True, corner=False)
  cp.extFlags |= int(HyundaiExtFlags.RADAR_GROUP1)
  radar = radar_interface.RadarInterface(cp)
  packer = CANPacker("hyundai_canfd_radar_generated")
  bus = CanBus(cp).ACAN
  messages = [packer.make_can_msg(address, bus, {
    "VALID_CNT1": 200, "VALID_CNT2": 220, "LONG_DIST1": 40, "LONG_DIST2": 50,
  }) for address in range(radar.radar_start_addr, radar.trigger_msg_tracks + 1)]
  result = Feed(radar).step(messages)
  points = [point for point in result.points if str(point.radarSource) == "frontRadar"]
  assert len(points) == 32
  assert all(point.measured and point.trackState == 0 for point in points)


@pytest.mark.parametrize("native_state", [3, 4])
def test_legacy_reports_actual_sensor_state(settings, native_state):
  settings["EnableRadarTracks"] = 1
  cp = corner_180_car_params(available=True, corner=False)
  cp.flags = 0
  cp.carFingerprint = CAR.HYUNDAI_SONATA
  radar = radar_interface.RadarInterface(cp)
  packer = CANPacker("hyundai_kia_mando_front_radar_generated")
  messages = [packer.make_can_msg(address, 1, {"STATE": native_state, "LONG_DIST": 40})
              for address in range(radar.radar_start_addr, radar.trigger_msg_tracks + 1)]
  result = Feed(radar).step(messages)
  points = [point for point in result.points if str(point.radarSource) == "frontRadar"]
  assert len(points) == 32
  assert all(point.measured and point.trackState == native_state for point in points)


def test_disabled_corner_feature_retains_existing_filtered_measurements(settings):
  settings["EnableRadarTracks"] = 1
  settings["EnableCornerRadar"] = 0
  cp = corner_180_car_params(available=True)
  radar = radar_interface.RadarInterface(cp)
  reference = radar_interface.RadarInterface(cp)
  feed = Feed(radar)
  packer = CANPacker("hyundai_canfd_radar_generated")
  bus = CanBus(cp).ACAN

  def measurements(result):
    return [(point.trackId, point.dRel, point.yRel, point.vRel, point.yvRel,
             point.aLead, point.jLead, point.trackState)
            for point in result.points if point.measured]

  for cycle in range(25):
    messages = [packer.make_can_msg(address, bus, {
      "VALID_CNT": 20, "VALID": 2, "LONG_DIST": 40 - cycle * 0.2,
      "LAT_DIST": 1 - cycle * 0.01, "REL_SPEED": -2 - cycle * 0.4,
    }) for address in range(radar.radar_start_addr, radar.trigger_msg_tracks + 1)]
    result = feed.step(messages, filtered=True)
    expected = radar_interface.RadarInterfaceBase.update_carrot(
      reference, 20.0, 0.0, feed.now * 1e-9, [(feed.now, messages)],
    )
    assert (result is None) == (expected is None)
    if result is not None:
      assert measurements(result) == measurements(expected)


def test_enabled_corner_observer_retains_front_identity_during_real_braking(settings):
  settings["EnableRadarTracks"] = 1
  cp = car_params()
  cp.radarUnavailable = False
  radar = radar_interface.RadarInterface(cp)
  feed = Feed(radar)
  packer = CANPacker("hyundai_canfd_radar_generated")
  bus = CanBus(cp).ACAN
  for cycle in range(30):
    messages = [packer.make_can_msg(address, bus, {
      "VALID_CNT": 20, "VALID": 2, "LONG_DIST": 40 - cycle * 0.2,
      "LAT_DIST": 1, "REL_SPEED": -2 - cycle * 0.4,
    }) for address in range(radar.radar_start_addr, radar.trigger_msg_tracks + 1)]
    result = feed.step(messages, filtered=True)
  points = [point for point in result.points if point.measured and str(point.radarSource) == "frontRadar"]
  assert points
  assert all(point.trackState == 2 and point.aLead < -1 for point in points)
  assert all(radar.tracks[point.trackId].cnt >= 20 for point in points)
