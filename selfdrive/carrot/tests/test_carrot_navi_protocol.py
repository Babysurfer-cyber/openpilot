"""Real WebSocket v2 navigation sessions reach the native typed control parser."""
import asyncio
from contextlib import asynccontextmanager
from types import SimpleNamespace

from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer
import pytest

from cereal import log
from openpilot.selfdrive.carrot import carrot_navi as protocol
from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload
from openpilot.selfdrive.carrot.carrot_navi_control import parse_carrot_navi_control


JSON_NAMES = (
  "vehicle", "guidance_current", "guidance_next", "lane_current", "lane_ahead", "speed",
  "traffic_signal", "crossroad", "route", "navigation_status", "app_status", "camera_state", "composition_state",
)
IMAGE_NAMES = (
  "tbt_current_compact", "tbt_current_full", "tbt_next", "traffic_signal", "lane_top", "lane_bottom",
  "safety_primary", "safety_secondary", "safety_section", "crossroad_minimized", "crossroad_expanded",
  "center_tbt_icon", "center_tbt_text", "center_tbt_fee",
)
CATALOG = tuple(("json", name) for name in JSON_NAMES) + tuple(("image", name) for name in IMAGE_NAMES) + (("render", "map_main"),)


def requirements():
  return {
    "type": "requirements_query", "protocol_version": 2, "catalog_revision": 1,
    "timestamp_ms": 1_783_920_000_000, "app_version": "11.2.3.3740",
    "limits": {"max_binary_frame_bytes": 8 * 1024 * 1024, "max_total_bitrate_kbps": 12_000},
    "streams": [{"kind": kind, "name": name, "schema_version": 1} for kind, name in CATALOG],
  }


def update(manifest, name, value, *, sequence=1, present=True, reason="source_absent", **changes):
  stream = next(item for item in manifest["streams"] if item["kind"] == "json" and item["name"] == name)
  result = {
    "type": "item_update", "protocol_version": 2, "session_id": manifest["session_id"],
    "manifest_revision": manifest["revision"], "schema_version": 1,
    "kind": "json", "name": name, "stream_handle": stream["stream_handle"],
    "sequence": sequence, "source_timestamp_ms": 1_783_920_000_000,
    "sent_at_ms": 1_783_920_000_001, "present": present, "value": value,
  }
  if not present:
    result["reason"] = reason
  result.update(changes)
  return result


async def until(predicate):
  async def wait():
    while not predicate():
      await asyncio.sleep(.001)
  await asyncio.wait_for(wait(), timeout=2.)


@asynccontextmanager
async def receiver_client():
  receiver = protocol.CarrotNaviReceiver()
  client = TestClient(TestServer(protocol.create_app(receiver)))
  try:
    await client.start_server()
    yield receiver, client
  finally:
    await client.close()


async def negotiate(client):
  control = await client.ws_connect("/api/navi/ws/v2/control/11.2.3.3740")
  await control.send_json(requirements())
  manifest = await control.receive_json(timeout=2.)
  assert manifest["type"] == "subscription_manifest"
  await control.send_json({
    "type": "manifest_applied", "protocol_version": 2, "session_id": manifest["session_id"],
    "revision": manifest["revision"], "timestamp_ms": 1_783_920_000_010,
  })
  return control, manifest


async def open_item(client, manifest, name):
  return await client.ws_connect(f"/api/navi/ws/v2/json/{manifest['session_id']}/{name}")


async def send_item(receiver, socket, message):
  count = receiver.health()["received_count"]
  await socket.send_json(message)
  await until(lambda: receiver.health()["received_count"] > count)


def typed_control(receiver, *, publish_ns=None):
  payload = build_carrot_navi_payload(receiver.cereal_snapshot(), publish_mono_ns=publish_ns)
  event = log.Event.new_message(valid=True)
  event.init("carrotNavi")
  event.carrotNavi = payload
  with log.Event.from_bytes(event.to_bytes()) as received:
    return received.carrotNavi.to_dict(), parse_carrot_navi_control(received.carrotNavi)


async def rejected(socket, *, code="json_stream_error"):
  error = await socket.receive_json(timeout=2.)
  assert error["type"] == "protocol_error"
  assert error["protocol_version"] == 2
  assert error["code"] == code
  closing = await socket.receive(timeout=2.)
  assert closing.type in (WSMsgType.CLOSE, WSMsgType.CLOSED)
  return error


def test_websocket_manifest_has_complete_catalog_but_only_json_subscriptions():
  async def scenario():
    async with receiver_client() as (receiver, client):
      _, manifest = await negotiate(client)
      streams = manifest["streams"]
      assert manifest["protocol_version"] == 2 and manifest["revision"] >= 1
      assert 8 <= len(manifest["session_id"]) <= 64
      assert len(streams) == 28
      assert {(item["kind"], item["name"]) for item in streams} == set(CATALOG)
      assert len({item["stream_handle"] for item in streams}) == 28
      assert all(item["stream_handle"] > 0 and item["schema_version"] == 1 for item in streams)
      assert {(item["kind"], item["name"]) for item in streams if item["enabled"]} == {("json", name) for name in JSON_NAMES}
      for item in streams:
        if item["kind"] == "json":
          assert item["params"]["delivery_mode"] == "on_change_with_heartbeat"
          assert item["params"]["interval_ms"] == 1000
      health = await (await client.get("/health")).json()
      assert health["connected"] and health["session_id"] == manifest["session_id"]
      assert receiver.cereal_snapshot()["connected"]
  asyncio.run(scenario())


@pytest.mark.parametrize("defect", ("missing", "duplicate", "schema", "revision"))
def test_incomplete_or_unsupported_catalog_cannot_create_a_session(defect):
  async def scenario():
    async with receiver_client() as (receiver, client):
      control = await client.ws_connect("/api/navi/ws/v2/control/11.2.3.3740")
      query = requirements()
      if defect == "missing":
        query["streams"].pop()
      elif defect == "duplicate":
        query["streams"][-1] = dict(query["streams"][0])
      elif defect == "schema":
        query["streams"][0]["schema_version"] = 2
      else:
        query["catalog_revision"] = 2
      await control.send_json(query)
      error = await control.receive_json(timeout=2.)
      assert error["type"] == "protocol_error" and error["code"] == "invalid_control_message"
      assert receiver.cereal_snapshot()["session_id"] == ""
      await control.send_json(requirements())
      assert (await control.receive_json(timeout=2.))["type"] == "subscription_manifest"
  asyncio.run(scenario())


def test_all_json_item_streams_reach_native_cereal_and_navigation_control():
  async def scenario():
    values = {
      "vehicle": {"lat": 37.5, "lon": 127.1, "heading_deg": 92., "speed_kph": 57.5, "road_name": "판교로"},
      "guidance_current": {"distance_m": 120, "turn_type": 12, "main_text": "판교", "near_direction": "서울", "far_direction": "부산"},
      "guidance_next": {"distance_m": 450, "turn_type": 1, "main_text": "다음 안내"},
      "lane_current": {"count": 3, "current_lane": 1, "available": [0, 1, 1], "road_category": 1},
      "lane_ahead": [{"count": 2, "distance_m": 500, "available": [1, 0]}],
      "speed": {"current_kph": 57.5, "road_limit_kph": 80,
                "sdi": {"type": 7, "distance_m": 210, "speed_limit_kph": 60},
                "sdi_secondary": {"type": 22, "distance_m": 350, "speed_limit_kph": 30},
                "section": {"active": True, "speed_limit_kph": 70, "remaining_distance_m": 1000.}},
      "traffic_signal": {"visible": True, "distance_m": 85, "source": "its", "lights": {"red": {"on": True, "remain_sec": 7}}},
      "crossroad": {"visible": True, "distance_m": 85, "image_code": 12},
      "route": {"remain_distance_m": 1500, "remain_time_sec": 180,
                "polyline": [{"lat": 37.5, "lon": 127.1}, {"lat": 37.51, "lon": 127.11}]},
      "navigation_status": {"mode": "guidance", "guidance_active": True, "off_route": False, "route_present": True},
      "app_status": {"foreground": True}, "camera_state": {}, "composition_state": {},
    }
    async with receiver_client() as (receiver, client):
      _, manifest = await negotiate(client)
      for name in JSON_NAMES:
        socket = await open_item(client, manifest, name)
        await send_item(receiver, socket, update(manifest, name, values[name]))
      assert receiver.health()["received_count"] == 13
      payload, parsed = typed_control(receiver)
      assert parsed is not None and parsed.session_id == manifest["session_id"]
      assert parsed.vehicle.present and (parsed.vehicle.latitude, parsed.vehicle.longitude) == (37.5, 127.1)
      assert parsed.vehicle.speed_kph == 57.5 and parsed.vehicle.road_name == "판교로"
      assert parsed.current.present and (parsed.current.distance_m, parsed.current.turn_type) == (120, 12)
      assert parsed.current.main_text == "판교" and parsed.next.distance_m == 450
      assert parsed.speed.present and parsed.speed.road_limit_kph == 80
      assert parsed.speed.sdi_present and (parsed.speed.sdi_type, parsed.speed.sdi_distance_m, parsed.speed.sdi_speed_limit_kph) == (7, 210, 60)
      assert parsed.speed.secondary_sdi_present and parsed.speed.secondary_sdi_type == 22
      assert parsed.speed.section_active and parsed.speed.section_remaining_distance_m == 1000
      assert parsed.route.present and parsed.route.polyline == ((37.5, 127.1), (37.51, 127.11))
      assert parsed.traffic.present and parsed.traffic.lamp == "red" and parsed.traffic.remain_sec == 7
      assert parsed.guidance_active and not parsed.off_route and parsed.road_category == 1
      assert payload["laneAhead"][0]["count"] == 2 and payload["crossroad"]["imageCode"] == 12
  asyncio.run(scenario())


@pytest.mark.parametrize("name", ("speed", "guidance_current", "route", "traffic_signal"))
def test_explicit_absence_removes_previous_typed_value_without_reusing_it(name):
  async def scenario():
    async with receiver_client() as (receiver, client):
      _, manifest = await negotiate(client)
      socket = await open_item(client, manifest, name)
      await send_item(receiver, socket, update(manifest, name, {"road_limit_kph": 80, "distance_m": 120, "visible": True}))
      await send_item(receiver, socket, update(manifest, name, None, sequence=2, present=False))
      snapshot = receiver.cereal_snapshot()["items"][name]
      assert not snapshot["present"] and snapshot["value"] is None and snapshot["sequence"] == 2
      _, parsed = typed_control(receiver)
      item = {"speed": parsed.speed, "guidance_current": parsed.current, "route": parsed.route, "traffic_signal": parsed.traffic}[name]
      assert not item.present and item.sequence == 2
      if name == "speed":
        assert item.road_limit_kph is None and not item.sdi_present
      elif name == "guidance_current":
        assert item.distance_m == 0 and item.turn_type == -1
      elif name == "route":
        assert item.polyline == ()
      else:
        assert item.lamp == "" and not item.visible
  asyncio.run(scenario())


@pytest.mark.parametrize("sequence", (-1, True, "invalid", "2", 1.5, 2 ** 64, 0, 1))
def test_invalid_or_repeated_item_sequence_is_rejected_without_mutating_navigation(sequence):
  async def scenario():
    async with receiver_client() as (receiver, client):
      _, manifest = await negotiate(client)
      socket = await open_item(client, manifest, "speed")
      await send_item(receiver, socket, update(manifest, "speed", {"road_limit_kph": 80}))
      count = receiver.health()["received_count"]
      await socket.send_json(update(manifest, "speed", {"road_limit_kph": 30}, sequence=sequence))
      await rejected(socket)
      assert receiver.health()["received_count"] == count
      assert typed_control(receiver)[1].speed.road_limit_kph == 80
      replacement = await open_item(client, manifest, "speed")
      await send_item(receiver, replacement, update(manifest, "speed", {"road_limit_kph": 60}, sequence=2))
      assert typed_control(receiver)[1].speed.road_limit_kph == 60
  asyncio.run(scenario())


@pytest.mark.parametrize("defect", ("session", "revision", "handle", "absent_value", "absent_reason", "missing_value"))
def test_mismatched_envelopes_and_malformed_tombstones_do_not_replace_valid_data(defect):
  async def scenario():
    async with receiver_client() as (receiver, client):
      _, manifest = await negotiate(client)
      socket = await open_item(client, manifest, "speed")
      await send_item(receiver, socket, update(manifest, "speed", {"road_limit_kph": 80}))
      bad = update(manifest, "speed", {"road_limit_kph": 30}, sequence=2)
      if defect == "session":
        bad["session_id"] = "obsolete-session"
      elif defect == "revision":
        bad["manifest_revision"] += 1
      elif defect == "handle":
        bad["stream_handle"] = 999
      elif defect == "absent_value":
        bad.update(present=False, reason="source_absent")
      elif defect == "absent_reason":
        bad.update(present=False, value=None)
      else:
        del bad["value"]
      await socket.send_json(bad)
      await rejected(socket)
      assert typed_control(receiver)[1].speed.road_limit_kph == 80
  asyncio.run(scenario())


def test_control_close_invalidates_items_and_reconnect_uses_a_fresh_session():
  async def scenario():
    async with receiver_client() as (receiver, client):
      control, first = await negotiate(client)
      old = await open_item(client, first, "speed")
      await send_item(receiver, old, update(first, "speed", {"road_limit_kph": 80}, sequence=100))
      await control.close()
      await until(lambda: not receiver.cereal_snapshot()["connected"])
      assert receiver.cereal_snapshot()["session_id"] == ""
      assert typed_control(receiver)[1] is None
      _, second = await negotiate(client)
      assert second["session_id"] != first["session_id"]
      assert not typed_control(receiver)[1].speed.present
      socket = await open_item(client, second, "speed")
      await send_item(receiver, socket, update(second, "speed", {"road_limit_kph": 60}, sequence=1))
      await old.send_json(update(first, "speed", {"road_limit_kph": 30}, sequence=101))
      await rejected(old)
      assert typed_control(receiver)[1].speed.road_limit_kph == 60
  asyncio.run(scenario())


def test_second_phone_cannot_replace_active_control_owner_or_its_session():
  async def scenario():
    async with receiver_client() as (receiver, client):
      _, manifest = await negotiate(client)
      socket = await open_item(client, manifest, "speed")
      await send_item(receiver, socket, update(manifest, "speed", {"road_limit_kph": 80}))
      challenger = await client.ws_connect("/api/navi/ws/v2/control/other-phone")
      await challenger.send_json(requirements())
      await rejected(challenger, code="receiver_busy")
      assert receiver.cereal_snapshot()["connected"]
      assert typed_control(receiver)[1].session_id == manifest["session_id"]
      assert typed_control(receiver)[1].speed.road_limit_kph == 80
  asyncio.run(scenario())


def test_pending_second_control_socket_cannot_keep_disconnected_owners_data_alive():
  async def scenario():
    async with receiver_client() as (receiver, client):
      owner, manifest = await negotiate(client)
      socket = await open_item(client, manifest, "speed")
      await send_item(receiver, socket, update(manifest, "speed", {"road_limit_kph": 80}))
      pending = await client.ws_connect("/api/navi/ws/v2/control/pending-phone")
      await owner.close()
      await until(lambda: not receiver.cereal_snapshot()["connected"])
      assert receiver.cereal_snapshot()["session_id"] == ""
      assert typed_control(receiver)[1] is None
      await pending.send_json(requirements())
      next_manifest = await pending.receive_json(timeout=2.)
      assert next_manifest["type"] == "subscription_manifest"
      assert next_manifest["session_id"] != manifest["session_id"]
      assert not typed_control(receiver)[1].speed.present
  asyncio.run(scenario())


def test_local_receipt_expiry_is_not_extended_by_fresh_publishes_and_preserves_wire_sequence(monkeypatch):
  clock = SimpleNamespace(ns=10_000_000_000)
  monkeypatch.setattr(protocol.time, "monotonic_ns", lambda: clock.ns)

  async def scenario():
    async with receiver_client() as (receiver, client):
      _, manifest = await negotiate(client)
      socket = await open_item(client, manifest, "speed")
      await send_item(receiver, socket, update(manifest, "speed", {"road_limit_kph": 80, "sdi": {"type": 7}}, sequence=8))
      for age_s in (1., 2., 2.999):
        clock.ns = 10_000_000_000 + int(age_s * 1e9)
        if age_s == 1.:
          vehicle = await open_item(client, manifest, "vehicle")
          await send_item(receiver, vehicle, update(manifest, "vehicle", {"lat": 37.5, "lon": 127.1}))
        payload, parsed = typed_control(receiver, publish_ns=clock.ns)
        assert parsed.speed.present and parsed.speed.road_limit_kph == 80
        assert payload["speed"]["meta"]["receivedMonoTimeNanos"] == 10_000_000_000
      original_generation = receiver.cereal_snapshot()["generation"]
      clock.ns = 13_001_000_000
      payload, parsed = typed_control(receiver, publish_ns=clock.ns)
      assert payload["connected"] and not parsed.speed.present and parsed.speed.road_limit_kph is None
      assert not parsed.speed.sdi_present and parsed.speed.sequence == 8
      assert parsed.vehicle.present  # Each item expires from its own receipt.
      expired = receiver.cereal_snapshot()
      assert expired["generation"] > original_generation
      assert expired["items"]["speed"]["value"] is None
      assert receiver.cereal_snapshot()["generation"] == expired["generation"]
      # The next app packet increments the original wire sequence, even though
      # the receiver emitted an absence when the previous local receipt aged out.
      await send_item(receiver, socket, update(manifest, "speed", {"road_limit_kph": 60}, sequence=9))
      assert typed_control(receiver)[1].speed.road_limit_kph == 60
  asyncio.run(scenario())


def test_optional_route_traffic_sections_survive_websocket_and_cereal_roundtrip():
  async def scenario():
    async with receiver_client() as (receiver, client):
      control, manifest = await negotiate(client)
      route_ws = await open_item(client, manifest, "route")
      status_ws = await open_item(client, manifest, "navigation_status")
      await send_item(receiver, status_ws, update(manifest, "navigation_status", {
        "guidance_active": True, "off_route": False, "route_present": True,
      }))
      await send_item(receiver, route_ws, update(manifest, "route", {
        "total_distance_m": 20000, "remain_distance_m": 12600,
        "traffic_sections": [
          {"start_distance_m": 0, "length_m": 5000, "congestion": "good"},
          {"start_distance_m": 5000, "length_m": 3000, "congestion": "bad"},
          {"start_distance_m": 8000, "length_m": 4000, "congestion": "very_bad"},
        ],
      }))
      raw, parsed = typed_control(receiver)
      assert parsed is not None and parsed.guidance_active
      assert raw["route"]["trafficPresent"] is True
      assert raw["route"]["trafficSections"] == [
        {"startDistanceM": 0, "lengthM": 5000, "congestionCode": 1},
        {"startDistanceM": 5000, "lengthM": 3000, "congestionCode": 2},
        {"startDistanceM": 8000, "lengthM": 4000, "congestionCode": 3},
      ]
      # A new route without traffic must NOT reuse congestion from the old one.
      await send_item(receiver, route_ws, update(manifest, "route", {
        "total_distance_m": 25000, "remain_distance_m": 18000,
      }, sequence=2))
      raw, _ = typed_control(receiver)
      assert raw["route"]["trafficPresent"] is False
      assert raw["route"]["trafficSections"] == []
      await control.close()
  asyncio.run(scenario())


def test_route_congestion_validation_rejects_inconsistent_sections_in_cereal():
  async def scenario():
    async with receiver_client() as (receiver, client):
      _, manifest = await negotiate(client)
      route_ws = await open_item(client, manifest, "route")
      await send_item(receiver, route_ws, update(manifest, "route", {
        "total_distance_m": 20000, "remain_distance_m": 15000,
        "traffic_sections": [
          {"start_distance_m": 1000, "length_m": 3000, "congestion": "good"},
          {"start_distance_m": 2000, "length_m": 3000, "congestion": "very_bad"},
        ],
      }))
      raw, _ = typed_control(receiver)
      assert raw["route"]["trafficPresent"] is False
      assert raw["route"]["trafficSections"] == []
  asyncio.run(scenario())
