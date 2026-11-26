"""Only the explicitly marked telemetry may appear in the HUD-only channel."""
from openpilot.selfdrive.carrot.navi_signal_hud import build_signal_hud


def sample():
  return {"schema_version": 1, "source": "naver_traffic_signal_live_data",
          "vehicle_control_allowed": False, "direction_match_verified": False,
          "signals": [{"group_index": 0, "signal_index": 0,
                       "direction": "STRAIGHT", "state": "RED", "remaining_seconds": 25}]}


def snap(kind="traffic_signal", value=None, *, present=True, connected=True):
  if value is None:
    value = sample()
  return {"connected": connected, "items": {kind: {"present": present, "value": value,
          "sequence": 4, "source_timestamp_ms": 1000, "received_mono_ns": 5000000000}}}


def test_direct():
  x = build_signal_hud(snap())
  assert x["meta"]["present"]
  assert x["candidates"] == [{"direction": "STRAIGHT", "state": "RED", "remainingValid": True, "remainingSec": 25}]
  assert x["directionMatchVerified"] is False


def test_nested_route():
  assert build_signal_hud(snap("route", {"traffic_signal": sample()}))["candidates"][0]["remainingSec"] == 25


def test_nested_speed():
  assert build_signal_hud(snap("speed", {"traffic_signal": sample()}))["meta"]["present"]


def test_not_connected_or_expired():
  assert build_signal_hud(snap(connected=False))["candidates"] == []
  assert build_signal_hud(snap(present=False))["candidates"] == []


def test_multiple_valid():
  data = sample()
  data["signals"].append({"group_index": 0, "signal_index": 1, "direction": "LEFT",
                          "state": "GREEN", "remaining_seconds": 15})
  assert len(build_signal_hud(snap(value=data))["candidates"]) == 2


def test_more_than_two_hidden_not_trimmed():
  data = sample()
  data["signals"] *= 3
  assert build_signal_hud(snap(value=data))["candidates"] == []


def test_invalid_colour_direction():
  data = sample()
  data["signals"][0]["state"] = "BLUE"
  assert build_signal_hud(snap(value=data))["candidates"] == []
  data["signals"][0]["state"] = "RED"
  data["signals"][0]["direction"] = "BAD"
  assert build_signal_hud(snap(value=data))["candidates"] == []


def test_no_guess_countdown():
  data = sample()
  data["signals"][0]["remaining_seconds"] = -1
  assert build_signal_hud(snap(value=data))["candidates"][0]["remainingValid"] is False
  data["signals"][0]["remaining_seconds"] = "24"
  assert build_signal_hud(snap(value=data))["candidates"][0]["remainingValid"] is False


def test_reject_missing_or_control_allowed():
  data = sample()
  data["vehicle_control_allowed"] = True
  assert build_signal_hud(snap(value=data))["candidates"] == []
  data["vehicle_control_allowed"] = False
  data.pop("direction_match_verified")
  assert build_signal_hud(snap(value=data))["candidates"] == []


def test_malformed_group_and_duplicate():
  data = sample()
  data["signals"][0]["group_index"] = -1
  assert build_signal_hud(snap(value=data))["candidates"] == []
  data = sample()
  data["signals"].append(dict(data["signals"][0]))
  assert build_signal_hud(snap(value=data))["candidates"] == []


def test_legacy_payload_unaffected():
  data = {"visible": True, "lights": {"red": {"on": True, "remain_sec": 12}}}
  assert build_signal_hud(snap(value=data))["meta"]["present"] is False


def test_malformed_values_cannot_crash_publisher():
  data = sample()
  data["signals"][0]["direction"] = []
  assert build_signal_hud(snap(value=data))["candidates"] == []
  data["signals"][0]["direction"] = "STRAIGHT"
  data["signals"][0]["state"] = {}
  assert build_signal_hud(snap(value=data))["candidates"] == []
  data = sample()
  data["schema_version"] = True
  assert build_signal_hud(snap(value=data))["candidates"] == []


def test_missing_mono_rejected():
  s = snap()
  s["items"]["traffic_signal"]["received_mono_ns"] = 0
  assert build_signal_hud(s)["meta"]["present"] is False
