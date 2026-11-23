#!/usr/bin/env python3
"""CarrotNavi v2 JSON receiver, adapted from carrot-wip 5c970aa51678.

Keep the complete application catalog while subscribing only to navigation
data consumed by this branch. Image rendering and the web UI are independent.
"""
from __future__ import annotations

import argparse
import asyncio
import copy
from dataclasses import dataclass, replace
import errno
import ipaddress
import json
import secrets
import socket
import threading
import time
from typing import Any

from aiohttp import WSMsgType, web


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 7714
DISCOVERY_PORT = 7705
DISCOVERY_INTERVAL_S = 1.0
PROTOCOL_VERSION = 2
CATALOG_REVISION = 1
MAX_MESSAGE_BYTES = 8 * 1024 * 1024
JSON_FRESHNESS_SECONDS = 3.0
CONTROL_WEBSOCKET_HEARTBEAT_S = 5.0
STREAM_WEBSOCKET_HEARTBEAT_S = 10.0
BIND_RETRY_COUNT = 10
BIND_RETRY_INTERVAL_S = 0.5

JSON_NAMES = (
  "vehicle", "guidance_current", "guidance_next", "lane_current",
  "lane_ahead", "speed", "traffic_signal", "crossroad", "route",
  "navigation_status", "app_status", "camera_state", "composition_state",
)
CLUSTER_JSON_NAMES = frozenset(JSON_NAMES)
IMAGE_NAMES = (
  "tbt_current_compact", "tbt_current_full", "tbt_next", "traffic_signal",
  "lane_top", "lane_bottom", "safety_primary", "safety_secondary",
  "safety_section", "crossroad_minimized", "crossroad_expanded",
  "center_tbt_icon", "center_tbt_text", "center_tbt_fee",
)
RENDER_NAMES = ("map_main",)
CATALOG = tuple(
  [("json", name) for name in JSON_NAMES]
  + [("image", name) for name in IMAGE_NAMES]
  + [("render", name) for name in RENDER_NAMES]
)
CATALOG_SET = frozenset(CATALOG)


def now_ms() -> int:
  return time.time_ns() // 1_000_000


def detect_advertise_ip(bind_host: str) -> str:
  if bind_host not in ("", "0.0.0.0", "::"):
    return bind_host
  probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    probe.connect(("8.8.8.8", 80))
    return str(probe.getsockname()[0])
  except OSError:
    try:
      return socket.gethostbyname(socket.gethostname())
    except OSError:
      return "127.0.0.1"
  finally:
    probe.close()


def _interface_ipv4_addresses() -> tuple[tuple[str, str | None, str | None], ...]:
  try:
    import psutil
  except ImportError:
    return ()
  return tuple(
    (address.address, address.netmask, address.broadcast)
    for addresses in psutil.net_if_addrs().values()
    for address in addresses
    if address.family == socket.AF_INET and not address.address.startswith("127.")
  )


def discovery_targets(advertise_ip: str | None = None) -> tuple[tuple[str, str], ...]:
  targets = []
  for address, netmask, broadcast in _interface_ipv4_addresses():
    if advertise_ip is not None and address != advertise_ip:
      continue
    if not broadcast and netmask:
      try:
        broadcast = str(ipaddress.ip_network(f"{address}/{netmask}", strict=False).broadcast_address)
      except ValueError:
        broadcast = None
    targets.append((address, broadcast or "255.255.255.255"))
  if not targets:
    address = advertise_ip or detect_advertise_ip(DEFAULT_HOST)
    if not address.startswith("127."):
      targets.append((address, "255.255.255.255"))
  return tuple(dict.fromkeys(targets))


class CarrotNaviDiscoveryBeacon:
  def __init__(self, advertise_ip: str | None = None, interval_s: float = DISCOVERY_INTERVAL_S) -> None:
    self.advertise_ip = advertise_ip
    self.interval_s = max(0.2, float(interval_s))
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None

  def start(self) -> None:
    if self._thread is None:
      self._stop.clear()
      self._thread = threading.Thread(target=self._run, name="carrot_navi_discovery", daemon=True)
      self._thread.start()

  def stop(self) -> None:
    self._stop.set()
    if self._thread is not None:
      self._thread.join(timeout=1.0)
    self._thread = None

  def broadcast_once(self) -> None:
    for source_ip, broadcast_ip in discovery_targets(self.advertise_ip):
      body = json.dumps({"ip": source_ip, "navi_debug": 0}).encode("utf-8")
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind((source_ip, 0))
        sock.sendto(body, (broadcast_ip, DISCOVERY_PORT))
      except OSError:
        continue
      finally:
        sock.close()

  def _run(self) -> None:
    while not self._stop.is_set():
      try:
        self.broadcast_once()
      except OSError:
        pass
      self._stop.wait(self.interval_s)


def build_manifest(session_id: str, revision: int = 1) -> dict[str, Any]:
  streams = []
  for handle, (kind, name) in enumerate(CATALOG, start=1):
    if kind == "json":
      params = {"delivery_mode": "on_change_with_heartbeat", "interval_ms": 1000,
                "stale_timeout_ms": int(JSON_FRESHNESS_SECONDS * 1000)}
    elif kind == "image":
      params = {"format": "png", "max_fps": 1, "stale_timeout_ms": 15000}
      if name.startswith("crossroad_"):
        params["theme"] = "auto"
    else:
      params = {"composition": "map_route_vehicle", "codec": "h264", "width": 960,
                "height": 540, "fps": 5, "h264_bitrate_kbps": 1500, "map_theme": "auto"}
    streams.append({"kind": kind, "name": name, "schema_version": 1,
                    "stream_handle": handle, "enabled": kind == "json", "params": params})
  return {"type": "subscription_manifest", "protocol_version": PROTOCOL_VERSION,
          "timestamp_ms": now_ms(), "session_id": session_id, "revision": revision,
          "metrics_enabled": False, "streams": streams}


def parse_json_object(raw: str) -> dict[str, Any]:
  value = json.loads(raw)
  if not isinstance(value, dict):
    raise ValueError("message must be a JSON object")
  return value


@dataclass(frozen=True)
class ItemRecord:
  name: str
  sequence: int
  source_timestamp_ms: int
  received_mono_ns: int
  present: bool
  value: Any
  reason: str = ""

  def summary(self) -> dict[str, Any]:
    return {"present": self.present, "sequence": self.sequence,
            "source_timestamp_ms": self.source_timestamp_ms,
            "received_mono_ns": self.received_mono_ns,
            "value": copy.deepcopy(self.value) if self.present else None,
            "reason": self.reason}


class CarrotNaviReceiver:
  def __init__(self, port: int = DEFAULT_PORT) -> None:
    self._lock = threading.RLock()
    self._port = port
    self._session_id: str | None = None
    self._app_version = ""
    self._manifest: dict[str, Any] | None = None
    self._manifest_by_key: dict[str, dict[str, Any]] = {}
    self._records: dict[str, ItemRecord] = {}
    self._control_connections = 0
    self._state_generation = 0
    self._state_changed = threading.Event()
    self._received_count = 0
    self._last_peer = "-"
    self._last_error: str | None = None
    self._cereal_publish_count = 0
    self._cereal_error: str | None = None

  def negotiate(self, requirements: dict[str, Any], app_version: str) -> dict[str, Any]:
    if requirements.get("type") != "requirements_query" or requirements.get("protocol_version") != PROTOCOL_VERSION:
      raise ValueError("invalid v2 requirements query")
    revision = requirements.get("catalog_revision")
    if isinstance(revision, bool) or revision != CATALOG_REVISION:
      raise ValueError("unsupported v2 catalog revision")
    offered = requirements.get("streams")
    if not isinstance(offered, list) or len(offered) != len(CATALOG):
      raise ValueError("app v2 catalog does not contain exactly 28 items")
    keys = []
    for stream in offered:
      if not isinstance(stream, dict) or stream.get("schema_version") != 1:
        raise ValueError("invalid v2 catalog entry")
      keys.append((str(stream.get("kind")), str(stream.get("name"))))
    if len(set(keys)) != len(keys) or set(keys) != CATALOG_SET:
      raise ValueError("app v2 catalog does not match receiver catalog")
    with self._lock:
      self._session_id = secrets.token_hex(8)
      self._app_version = app_version[:96]
      self._manifest = build_manifest(self._session_id)
      self._manifest_by_key = {f"{s['kind']}:{s['name']}": s for s in self._manifest["streams"]}
      self._records.clear()
      self._last_error = None
      self._mark_state_changed_locked()
      return copy.deepcopy(self._manifest)

  def control_connected(self) -> None:
    with self._lock:
      self._control_connections += 1
      self._mark_state_changed_locked()

  def control_disconnected(self) -> None:
    with self._lock:
      self._control_connections = max(0, self._control_connections - 1)
      if self._control_connections == 0:
        self._session_id = None
        self._manifest = None
        self._manifest_by_key.clear()
        self._records.clear()
      self._mark_state_changed_locked()

  def record_control(self, payload: dict[str, Any], peer: str) -> None:
    if payload.get("protocol_version") != PROTOCOL_VERSION:
      raise ValueError("unsupported v2 control protocol")
    with self._lock:
      self._last_peer = peer
      if payload.get("type") == "protocol_error":
        self._last_error = str(payload.get("message", payload.get("code", "error")))[:256]

  def stream_config(self, session_id: str, kind: str, name: str) -> dict[str, Any]:
    with self._lock:
      return copy.deepcopy(self._validate_stream_locked(session_id, kind, name, {}))

  def record_json(self, session_id: str, name: str, envelope: dict[str, Any], peer: str) -> None:
    if (envelope.get("type") != "item_update" or envelope.get("protocol_version") != PROTOCOL_VERSION
        or envelope.get("session_id") != session_id or envelope.get("kind") != "json" or envelope.get("name") != name):
      raise ValueError("v2 JSON envelope/path mismatch")
    with self._lock:
      self._validate_stream_locked(session_id, "json", name, envelope)
      sequence = self._nonnegative_int(envelope, "sequence")
      source_timestamp = self._nonnegative_int(envelope, "source_timestamp_ms")
      self._nonnegative_int(envelope, "sent_at_ms")
      present = envelope.get("present")
      if not isinstance(present, bool) or "value" not in envelope:
        raise ValueError("v2 JSON item must contain boolean present and value")
      value = envelope["value"]
      reason = ""
      if present:
        if name == "lane_ahead":
          if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
            raise ValueError("v2 JSON lane_ahead must be an array of objects")
        elif not isinstance(value, dict):
          raise ValueError(f"v2 JSON {name} value must be an object")
      else:
        reason = envelope.get("reason")
        if value is not None or not isinstance(reason, str) or not 0 < len(reason) <= 64:
          raise ValueError("absent JSON item must contain value=null and a reason")
      key = f"json:{name}"
      previous = self._records.get(key)
      if previous is not None and sequence <= previous.sequence:
        raise ValueError("stale v2 sequence")
      self._records[key] = ItemRecord(name, sequence, source_timestamp, time.monotonic_ns(),
                                      present, copy.deepcopy(value) if present else None, reason)
      self._received_count += 1
      self._last_peer = peer
      self._last_error = None
      self._mark_state_changed_locked()

  def _expire_records_locked(self) -> None:
    now = time.monotonic_ns()
    for key, record in tuple(self._records.items()):
      age = now - record.received_mono_ns
      if record.present and not 0 <= age <= int(JSON_FRESHNESS_SECONDS * 1e9):
        # Keep the wire sequence: synthesizing an increment would reject the
        # application's next legitimate update. Consumers also compare present.
        self._records[key] = replace(record, present=False, value=None, reason="expired")
        self._mark_state_changed_locked()

  def cereal_snapshot(self) -> dict[str, Any]:
    with self._lock:
      self._expire_records_locked()
      return {"generation": self._state_generation, "session_id": self._session_id or "",
              "connected": self._control_connections > 0 and self._session_id is not None,
              "items": {r.name: r.summary() for r in self._records.values()}}

  def wait_for_state_change(self, timeout: float) -> bool:
    changed = self._state_changed.wait(max(0.0, timeout))
    self._state_changed.clear()
    return changed

  def record_cereal_publish(self, error: str | None = None) -> None:
    with self._lock:
      if error is None:
        self._cereal_publish_count += 1
      self._cereal_error = str(error)[:256] if error is not None else None

  def health(self) -> dict[str, Any]:
    snapshot = self.cereal_snapshot()
    with self._lock:
      return {"protocol_version": PROTOCOL_VERSION, "port": self._port,
              "connected": snapshot["connected"], "session_id": snapshot["session_id"],
              "app_version": self._app_version, "received_count": self._received_count,
              "error": self._last_error, "cereal_publish_count": self._cereal_publish_count,
              "cereal_error": self._cereal_error}

  def latest(self) -> dict[str, Any]:
    return {**self.health(), **self.cereal_snapshot()}

  def fail(self, error: str, peer: str) -> None:
    with self._lock:
      self._last_error = str(error)[:256]
      self._last_peer = peer

  def _validate_stream_locked(self, session_id: str, kind: str, name: str,
                              identity: dict[str, Any]) -> dict[str, Any]:
    if session_id != self._session_id:
      raise ValueError("stale v2 session")
    stream = self._manifest_by_key.get(f"{kind}:{name}")
    if stream is None or not stream["enabled"]:
      raise ValueError("v2 stream is not enabled")
    if identity:
      if self._nonnegative_int(identity, "manifest_revision") != self._manifest["revision"]:
        raise ValueError("stale v2 manifest revision")
      if self._nonnegative_int(identity, "stream_handle") != stream["stream_handle"]:
        raise ValueError("v2 stream handle mismatch")
      if "schema_version" in identity and self._nonnegative_int(identity, "schema_version") != 1:
        raise ValueError("v2 item schema mismatch")
    return stream

  @staticmethod
  def _nonnegative_int(value: dict[str, Any], key: str) -> int:
    candidate = value.get(key)
    if type(candidate) is not int or not 0 <= candidate <= 0xffffffffffffffff:
      raise ValueError(f"invalid v2 {key}")
    return candidate

  def _mark_state_changed_locked(self) -> None:
    self._state_generation += 1
    self._state_changed.set()


RECEIVER_KEY = web.AppKey("carrot_navi_receiver", CarrotNaviReceiver)
WEBSOCKETS_KEY = web.AppKey("carrot_navi_websockets", set)
CONTROL_OWNER_KEY = web.AppKey("carrot_navi_control_owner", object)


@dataclass
class ControlOwner:
  socket: web.WebSocketResponse | None = None


def _protocol_error(code: str, message: str) -> dict[str, Any]:
  return {"type": "protocol_error", "protocol_version": PROTOCOL_VERSION, "timestamp_ms": now_ms(),
          "code": code, "recoverable": True, "message": str(message)[:256]}


async def _safe_send_json(ws: web.WebSocketResponse, payload: dict[str, Any]) -> bool:
  try:
    if not ws.closed:
      await ws.send_json(payload)
      return True
  except (ConnectionError, RuntimeError):
    pass
  return False


async def ws_control(request: web.Request) -> web.WebSocketResponse:
  receiver = request.app[RECEIVER_KEY]
  owner = request.app[CONTROL_OWNER_KEY]
  ws = web.WebSocketResponse(heartbeat=CONTROL_WEBSOCKET_HEARTBEAT_S,
                            max_msg_size=MAX_MESSAGE_BYTES, compress=False)
  await ws.prepare(request)
  request.app[WEBSOCKETS_KEY].add(ws)
  negotiated = False
  try:
    async for message in ws:
      try:
        if message.type != WSMsgType.TEXT:
          raise ValueError("v2 control accepts JSON text only")
        payload = parse_json_object(message.data)
        if payload.get("type") == "requirements_query":
          if owner.socket is not None and owner.socket is not ws and not owner.socket.closed:
            await _safe_send_json(ws, _protocol_error("receiver_busy", "another navigation control connection is active"))
            await ws.close(code=1013, message=b"navigation receiver busy")
            break
          manifest = receiver.negotiate(payload, request.match_info["version"])
          owner.socket = ws
          if not negotiated:
            receiver.control_connected()
            negotiated = True
          if not await _safe_send_json(ws, manifest):
            break
        else:
          if owner.socket is not ws:
            raise ValueError("negotiate navigation control before sending events")
          receiver.record_control(payload, request.remote or "-")
      except (TypeError, ValueError) as exc:
        receiver.fail(str(exc), request.remote or "-")
        if not await _safe_send_json(ws, _protocol_error("invalid_control_message", str(exc))):
          break
  finally:
    if owner.socket is ws:
      owner.socket = None
    if negotiated:
      receiver.control_disconnected()
    request.app[WEBSOCKETS_KEY].discard(ws)
  return ws


async def ws_json(request: web.Request) -> web.WebSocketResponse:
  receiver = request.app[RECEIVER_KEY]
  session_id, name = request.match_info["session_id"], request.match_info["name"]
  ws = web.WebSocketResponse(heartbeat=STREAM_WEBSOCKET_HEARTBEAT_S,
                            max_msg_size=MAX_MESSAGE_BYTES, compress=False)
  await ws.prepare(request)
  request.app[WEBSOCKETS_KEY].add(ws)
  try:
    receiver.stream_config(session_id, "json", name)
    async for message in ws:
      if message.type != WSMsgType.TEXT:
        raise ValueError("v2 JSON item stream accepts text only")
      receiver.record_json(session_id, name, parse_json_object(message.data), request.remote or "-")
  except (TypeError, ValueError) as exc:
    receiver.fail(str(exc), request.remote or "-")
    await _safe_send_json(ws, _protocol_error("json_stream_error", str(exc)))
    await ws.close(code=1008, message=b"invalid JSON stream")
  finally:
    request.app[WEBSOCKETS_KEY].discard(ws)
  return ws


async def index(request: web.Request) -> web.Response:
  return web.json_response({"name": "Carrot Navi Receiver", "protocol_version": PROTOCOL_VERSION,
                            "port": DEFAULT_PORT, "health": "/health", "latest": "/api/navi/latest"})


async def health(request: web.Request) -> web.Response:
  return web.json_response(request.app[RECEIVER_KEY].health())


async def latest(request: web.Request) -> web.Response:
  return web.json_response(request.app[RECEIVER_KEY].latest())


async def traffic_diagnostic(request: web.Request) -> web.Response:
  # Display-only: no coordinates or route names are exposed in this summary.
  from openpilot.selfdrive.carrot.traffic_diagnostic import summarize_traffic
  return web.json_response(summarize_traffic(request.app[RECEIVER_KEY].latest()))


async def _close_websockets(app: web.Application) -> None:
  await asyncio.gather(*(ws.close(code=1001, message=b"receiver shutdown")
                         for ws in tuple(app[WEBSOCKETS_KEY])), return_exceptions=True)


def create_app(receiver: CarrotNaviReceiver | None = None) -> web.Application:
  app = web.Application(client_max_size=MAX_MESSAGE_BYTES)
  app[RECEIVER_KEY] = receiver or CarrotNaviReceiver()
  app[WEBSOCKETS_KEY] = set()
  app[CONTROL_OWNER_KEY] = ControlOwner()
  app.router.add_get("/", index)
  app.router.add_get("/health", health)
  app.router.add_get("/api/navi/latest", latest)
  app.router.add_get("/api/navi/traffic-diagnostic", traffic_diagnostic)
  app.router.add_get("/api/navi/ws/v2/control/{version}", ws_control)
  app.router.add_get("/api/navi/ws/v2/json/{session_id}/{name}", ws_json)
  app.on_shutdown.append(_close_websockets)
  return app


def run_receiver_app(receiver: CarrotNaviReceiver, host: str, port: int) -> None:
  for retry in range(BIND_RETRY_COUNT + 1):
    try:
      web.run_app(create_app(receiver), host=host, port=port, print=None)
      return
    except OSError as exc:
      if exc.errno != errno.EADDRINUSE or retry == BIND_RETRY_COUNT:
        raise
      time.sleep(BIND_RETRY_INTERVAL_S)


def main() -> None:
  parser = argparse.ArgumentParser(description="Carrot Navi WebSocket JSON receiver")
  parser.add_argument("--host", default=DEFAULT_HOST)
  parser.add_argument("--port", type=int, default=DEFAULT_PORT)
  parser.add_argument("--advertise-ip", default=None)
  parser.add_argument("--no-beacon", action="store_true")
  parser.add_argument("--no-cereal", action="store_true")
  args = parser.parse_args()
  receiver = CarrotNaviReceiver(args.port)
  advertise_ip = args.advertise_ip or (args.host if args.host not in ("", "0.0.0.0", "::") else None)
  beacon = None if args.no_beacon else CarrotNaviDiscoveryBeacon(advertise_ip)
  publisher = None
  if not args.no_cereal:
    from openpilot.selfdrive.carrot.carrot_navi_cereal import CarrotNaviCerealPublisher
    publisher = CarrotNaviCerealPublisher(receiver)
    publisher.start()
  if beacon is not None:
    beacon.start()
  try:
    run_receiver_app(receiver, args.host, args.port)
  finally:
    if beacon is not None:
      beacon.stop()
    if publisher is not None:
      publisher.stop()


if __name__ == "__main__":
  main()
