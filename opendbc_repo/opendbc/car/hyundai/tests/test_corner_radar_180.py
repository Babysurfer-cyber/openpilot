import math
import time

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai import hyundaicanfd, interface, radar_interface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, HyundaiExtFlags, HyundaiFlags


DBC = "hyundai_canfd_corner_radar_180_generated"
ADDRESSES = range(0x180, 0x185)


@pytest.fixture
def settings(monkeypatch):
  values = {"EnableCornerRadar": 1, "EnableRadarTracks": 0}

  class TestParams:
    def get_int(self, key):
      return values.get(key, 0)

    def get_bool(self, key):
      return bool(values.get(key, 0))

    def put_int(self, key, value):
      values[key] = value

  for module in (radar_interface, interface, hyundaicanfd):
    monkeypatch.setattr(module, "Params", TestParams)
  return values


def car_params(*, available=False, corner=True, hda2=False):
  cp = structs.CarParams.new_message()
  cp.carFingerprint = CAR.KIA_EV6
  cp.flags = int(HyundaiFlags.CANFD | (HyundaiFlags.CANFD_HDA2 if hda2 else 0))
  cp.extFlags = int(HyundaiExtFlags.CORNER_RADAR_OBJECTS_180) if corner else 0
  cp.radarUnavailable = not available
  cp.radarTimeStep = 0.05
  return cp


def wire_payload(slot1=None, slot2=None):
  """Encode the published wire layout independently of CANPacker and the DBC."""
  payload = 0
  for index, slot in enumerate((slot1, slot2)):
    if slot is None:
      continue
    raw = {
      "quality": (24, 7), "age": (32, 8), "id": (44, 7), "x": (64, 13),
      "y": (78, 12), "vx": (91, 12), "vy": (104, 10), "ax": (115, 9),
    }
    for name, (start, width) in raw.items():
      payload |= (slot[name] & ((1 << width) - 1)) << (start + 128 * index)
  return payload.to_bytes(32, "little")


def raw_object(object_id=1, *, x=400, y=2108, age=20, quality=90, vx=1960, vy=500, ax=0):
  return {"quality": quality, "age": age, "id": object_id, "x": x, "y": y, "vx": vx, "vy": vy, "ax": ax}


class Feed:
  def __init__(self, radar):
    self.radar = radar
    self.now = time.monotonic_ns()

  def step(self, messages=(), *, filtered=False):
    self.now += 10_000_000
    packets = [(self.now, list(messages))]
    if filtered:
      return self.radar.update_carrot(20.0, 0.0, self.now * 1e-9, packets)
    return self.radar.update(packets)

  def publish(self, messages=(), *, filtered=False):
    for index in range(5):
      result = self.step(messages if index == 0 else (), filtered=filtered)
      if result is not None:
        return result
    return None


def corner_points(result):
  assert result is not None
  return [point for point in result.points if point.measured and str(point.radarSource) == "corner180"]


@pytest.mark.parametrize("bus", [0, 1, 4, 5])
def test_wire_decode_and_pack_both_slots(bus):
  first = raw_object(37, x=400, y=1998, vx=1900, vy=520, ax=-30)
  second = raw_object(81, x=600, y=2108, vx=2020, vy=480, ax=50)
  data = wire_payload(first, second)
  parser = CANParser(DBC, [(f"CORNER_RADAR_180_OBJECTS_{address:x}", 33) for address in ADDRESSES], bus)
  assert parser.update([(time.monotonic_ns(), [(0x180, data, bus)])]) == {0x180}
  signals = parser.vl[0x180]
  expected = {
    "SLOT1_QUAL_LEVEL": 90, "SLOT1_AGE": 20, "SLOT1_OBJECT_ID": 37,
    "SLOT1_REL_POS_X": 20.0, "SLOT1_REL_POS_Y": -2.5,
    "SLOT1_REL_VEL_X": -5.0, "SLOT1_REL_VEL_Y": 1.0, "SLOT1_REL_ACCEL_X": -1.5,
    "SLOT2_QUAL_LEVEL": 90, "SLOT2_AGE": 20, "SLOT2_OBJECT_ID": 81,
    "SLOT2_REL_POS_X": 30.0, "SLOT2_REL_POS_Y": 3.0,
    "SLOT2_REL_VEL_X": 1.0, "SLOT2_REL_VEL_Y": -1.0, "SLOT2_REL_ACCEL_X": 2.5,
  }
  for name, value in expected.items():
    assert signals[name] == pytest.approx(value)
  assert CANPacker(DBC).make_can_msg(0x180, bus, expected) == (0x180, data, bus)


@pytest.mark.parametrize("hda2", [False, True])
@pytest.mark.parametrize("defect", [None, "missing", "dlc", "bus"])
def test_fingerprint_requires_complete_32_byte_bank(settings, hda2, defect):
  settings["CanfdHDA2"] = int(hda2)
  fingerprint = gen_empty_fingerprint()
  bus = CanBus(None, fingerprint, hda2).ACAN
  fingerprint[bus].update({address: 32 for address in ADDRESSES})
  if defect == "missing":
    del fingerprint[bus][0x182]
  elif defect == "dlc":
    fingerprint[bus][0x182] = 16
  elif defect == "bus":
    fingerprint[CanBus(None, fingerprint, hda2).ECAN] = fingerprint[bus]
    fingerprint[bus] = {}
  cp = interface.CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
  assert bool(cp.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_180) == (defect is None)


@pytest.mark.parametrize("enabled,has_bank,canfd", [(0, True, True), (1, False, True), (1, True, False), (1, True, True)])
def test_parser_enable_gate(settings, enabled, has_bank, canfd):
  settings["EnableCornerRadar"] = enabled
  cp = car_params(corner=has_bank)
  if not canfd:
    cp.flags = 0
    cp.carFingerprint = CAR.HYUNDAI_SONATA
  radar = radar_interface.RadarInterface(cp)
  assert (radar.rcp_corner_objects_180 is not None) == bool(enabled and has_bank and canfd)


def test_missing_dbc_disables_optional_parser(settings, monkeypatch, tmp_path):
  monkeypatch.setattr(radar_interface, "DBC_PATH", str(tmp_path))
  radar = radar_interface.RadarInterface(car_params())
  assert radar.rcp_corner_objects_180 is None
  assert radar.radar_off_can


def test_ten_objects_publish_without_front_radar_or_scc(settings):
  radar = radar_interface.RadarInterface(car_params())
  feed = Feed(radar)
  bus = CanBus(radar.CP).ACAN
  messages = [(address, wire_payload(raw_object(2 * index + 1, x=400 + index * 200),
                                     raw_object(2 * index + 2, x=500 + index * 200, y=1988)), bus)
              for index, address in enumerate(ADDRESSES)]
  outputs = [feed.step(messages if index % 3 == 0 else ()) for index in range(20)]
  assert [index + 1 for index, result in enumerate(outputs) if result is not None] == [5, 10, 15, 20]
  points = corner_points(outputs[-1])
  assert len(points) == 10
  assert len({point.trackId for point in points}) == 10
  assert min(point.trackId for point in points) >= 1000
  assert not outputs[-1].errors.canError
  assert not radar.radar_tracks


def test_slot_handoff_deduplication_and_object_id_reuse(settings):
  radar = radar_interface.RadarInterface(car_params())
  feed = Feed(radar)
  bus = CanBus(radar.CP).ACAN
  first = corner_points(feed.publish([(0x180, wire_payload(raw_object(17)), bus)]))[0]
  identity = first.trackId
  # A one-cycle duplicate during handoff must remain one physical track.
  duplicate = corner_points(feed.publish([(0x180, wire_payload(raw_object(17, age=21), raw_object(17, x=402, age=22)), bus)]))
  assert len(duplicate) == 1
  assert duplicate[0].trackId == identity
  handed_off = corner_points(feed.publish([(0x180, wire_payload(None, raw_object(17, x=404, age=23)), bus)]))
  assert len(handed_off) == 1
  assert handed_off[0].trackId == identity
  # Recycled native ID and reset age is a new object, even at the same position.
  recycled = corner_points(feed.publish([(0x180, wire_payload(raw_object(17, x=404, age=1)), bus)]))[0]
  assert recycled.trackId != identity
  # Native IDs are not globally unique, so distant simultaneous objects stay separate.
  distant = corner_points(feed.publish([(0x180, wire_payload(raw_object(17, x=400, age=2), raw_object(17, x=1200, age=2)), bus)]))
  assert len(distant) == 2
  assert len({point.trackId for point in distant}) == 2


@pytest.mark.parametrize("obj,expected", [
  (raw_object(quality=0), False), (raw_object(vx=0), False),
  (raw_object(x=4000), False), (raw_object(y=4095), False),
  (raw_object(x=0, y=2108), True), (raw_object(x=0, y=2048), False),
  (raw_object(), True),
])
def test_object_validity_and_clipped_side_objects(settings, obj, expected):
  radar = radar_interface.RadarInterface(car_params())
  result = Feed(radar).publish([(0x180, wire_payload(obj), CanBus(radar.CP).ACAN)])
  assert bool(corner_points(result)) == expected


def test_stale_address_expires_while_other_addresses_refresh(settings):
  radar = radar_interface.RadarInterface(car_params())
  feed = Feed(radar)
  bus = CanBus(radar.CP).ACAN
  result = feed.publish([(0x180, wire_payload(raw_object(1)), bus), (0x184, wire_payload(raw_object(2, x=1200)), bus)])
  assert len(corner_points(result)) == 2
  survivor = max(corner_points(result), key=lambda point: point.dRel).trackId
  for _ in range(12):
    result = feed.publish([(0x184, wire_payload(raw_object(2, x=1200)), bus)])
  assert [point.trackId for point in corner_points(result)] == [survivor]
  # Cached addresses must not be reintroduced by a different message's update.
  for _ in range(12):
    result = feed.publish()
  assert corner_points(result) == []


def test_optional_scc_clears_after_reception_stops(settings):
  radar = radar_interface.RadarInterface(car_params())
  feed = Feed(radar)
  bus = CanBus(radar.CP)
  scc = CANPacker("hyundai_canfd_generated").make_can_msg("SCC_CONTROL", bus.ECAN, {
    "ACC_ObjDist": 50, "ACC_ObjRelSpd": -1,
  })
  corners = [(address, wire_payload(raw_object(index + 1, x=400 + index * 200)), bus.ACAN)
             for index, address in enumerate(ADDRESSES)]
  for _ in range(3):
    result = feed.publish(corners + [scc])
  assert any(point.measured and str(point.radarSource) == "scc" for point in result.points)
  for _ in range(15):
    result = feed.publish(corners)
  assert not any(point.measured and str(point.radarSource) == "scc" for point in result.points)
  assert len(corner_points(result)) == 5
  assert not result.errors.canError


def test_corner_frames_do_not_change_front_radar_clock_or_measurements(settings):
  settings["EnableRadarTracks"] = 1
  cp = car_params(available=True)
  enabled = radar_interface.RadarInterface(cp)
  settings["EnableCornerRadar"] = 0
  baseline = radar_interface.RadarInterface(cp)
  packer = CANPacker("hyundai_canfd_radar_generated")
  bus = CanBus(cp).ACAN
  messages = [packer.make_can_msg(f"RADAR_TRACK_{address:x}", bus, {
    "VALID_CNT": 20, "LONG_DIST": 45 + index, "LAT_DIST": 0.8,
    "REL_SPEED": -2, "REL_ACCEL": 0.5, "LAT_SPEED": 0.1,
  }) for index, address in enumerate(range(enabled.radar_start_addr, enabled.trigger_msg_tracks + 1))]
  result = Feed(enabled).step(messages + [(0x180, wire_payload(raw_object()), bus)])
  reference = Feed(baseline).step(messages)
  def front(data):
    return [(point.trackId, point.dRel, point.yRel, point.vRel, point.measured)
            for point in data.points if str(point.radarSource) == "frontRadar"]

  assert front(result) == front(reference)
  assert len(corner_points(result)) == 1
  assert Feed(enabled).step([(0x180, wire_payload(raw_object(age=21)), bus)]) is None


def test_filtered_corner_snapshot_uses_current_track_and_survives_handoff(settings):
  radar = radar_interface.RadarInterface(car_params())
  feed = Feed(radar)
  bus = CanBus(radar.CP).ACAN
  result = None
  for cycle in range(12):
    first = raw_object(7, x=400 + cycle * 2, y=2108 - cycle, age=20 + cycle)
    data = wire_payload(first, None) if cycle < 8 else wire_payload(None, first)
    result = feed.publish([(0x180, data, bus)], filtered=True)
  point = corner_points(result)[0]
  assert len(radar.corner_object_180_point_ids) == 1
  assert all(math.isfinite(value) for value in (point.aLead, point.jLead, point.yRel, point.yvRel))
  assert point.yRel == pytest.approx(radar.pts[point.trackId].yRel)
  assert point.yvRel == pytest.approx(radar.pts[point.trackId].yvRel)
  assert radar.tracks[point.trackId].cnt >= 6
