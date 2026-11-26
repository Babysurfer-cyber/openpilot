"""Stock CarrotNavi legacy/V2 signals must remain display-only and fail closed."""
import json
from pathlib import Path

from openpilot.selfdrive.carrot.navi_signal_legacy import (
  FRESH_NS, lamps_from_sinf, lamps_from_ssinf, lamps_from_v2_lights,
  read_legacy_signal_hud, signal_hud_from_lamps, write_legacy_signal,
)
from openpilot.selfdrive.carrot.navi_signal_hud import build_signal_hud
from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload


def _record(data, *, connected=True, present=True):
  return {"connected": connected, "items": {"traffic_signal": {
    "present": present, "value": data, "sequence": 7,
    "received_mono_ns": 8_000_000_000, "source_timestamp_ms": 1234}}}


def test_sinf_red_and_left_indicators():
  lamps = lamps_from_sinf({"redLightOn": True, "leftLightOn": True,
                           "redLightRemainTime": 25, "leftLightRemainTime": 15})
  assert [(x["direction"], x["state"]) for x in lamps] == [
    ("STRAIGHT", "RED"), ("LEFT", "GREEN")]
  assert [x["remainingSec"] for x in lamps] == [25, 15]


def test_sinf_straight_green_without_red():
  assert lamps_from_sinf({"greenLightOn": True, "greenLightRemainTime": 9})[0]["state"] == "GREEN"
  assert lamps_from_sinf({"redLightOn": True, "greenLightOn": True}) == []
  assert lamps_from_sinf({"rightLightOn": True}) == []
  assert lamps_from_sinf("broken") == []


def test_sinf_missing_seconds_does_not_invent_countdown():
  lamps = lamps_from_sinf({"redLightOn": True, "redLightRemainTime": "none"})
  assert not lamps[0]["remainingValid"]
  assert lamps[0]["remainingSec"] == 0


def test_ssinf_directional_red_green_yellow():
  lamps = lamps_from_ssinf({
    "straight": "RED_LIGHT_ON", "straight_remain_time": 20,
    "left": "GREEN_LIGHT_ON", "left_remain_time": 11,
  })
  assert [(l["direction"], l["state"]) for l in lamps] == [
    ("STRAIGHT", "RED"), ("LEFT", "GREEN")]
  assert lamps_from_ssinf({"left": "YELLOW_LIGHT_ON"})[0]["state"] == "YELLOW"
  assert lamps_from_ssinf({"left": "NONE", "straight": "OFF"}) == []


def test_stock_v2_lights_and_movements():
  data = {"visible": True, "lights": {
    "red": {"on": True, "remain_sec": 20},
    "left": {"on": True, "remain_sec": 15},
  }}
  lamps = lamps_from_v2_lights(data)
  assert [l["state"] for l in lamps] == ["RED", "GREEN"]
  assert lamps_from_v2_lights({**data, "visible": False}) == []
  conflicting = {"visible": True, "lights": {
    "red": {"on": True}, "green": {"on": True}}}
  assert lamps_from_v2_lights(conflicting) == []
  directional = {"visible": True, "movements": {
    "straight": {"state": "GREEN_LIGHT_ON", "remain_sec": 7},
    "left": {"state": "RED_LIGHT_ON", "remain_sec": 12},
  }}
  assert [l["state"] for l in lamps_from_v2_lights(directional)] == ["GREEN", "RED"]


def test_stock_v2_to_existing_four_lamp_hud():
  signal = {"visible": True, "lights": {
    "red": {"on": True, "remain_sec": 20},
    "left": {"on": True, "remain_sec": 15},
  }}
  hud = build_signal_hud(_record(signal))
  assert hud["meta"]["present"] and not hud["directionMatchVerified"]
  assert len(hud["candidates"]) == 2
  assert hud["meta"]["receivedMonoTimeNanos"] == 8_000_000_000
  assert build_signal_hud(_record(signal, connected=False))["meta"]["present"] is False
  assert build_signal_hud(_record(signal, present=False))["meta"]["present"] is False


def test_no_legacy_signals_promoted_into_traffic_state():
  old = {"connected": False, "items": {}}
  lamps = lamps_from_sinf({"redLightOn": True, "redLightRemainTime": 15})
  legacy = signal_hud_from_lamps(lamps, 8_000_000_000, timestamp_ms=1234)
  payload = build_carrot_navi_payload(old, legacy_signal_hud=legacy)
  assert payload["connected"] is False
  assert payload["signalHud"]["meta"]["present"]
  assert payload["trafficSignal"]["meta"]["present"] is False
  assert payload["signalHud"]["candidates"][0]["state"] == "RED"


def test_v2_source_wins_over_simultaneous_legacy_packet():
  current = _record({"visible": True, "lights": {
    "green": {"on": True, "remain_sec": 12}}})
  legacy = signal_hud_from_lamps(
    lamps_from_sinf({"redLightOn": True, "redLightRemainTime": 25}),
    7_000_000_000)
  payload = build_carrot_navi_payload(current, legacy_signal_hud=legacy)
  assert payload["signalHud"]["candidates"][0]["state"] == "GREEN"


def test_atomic_ipc_and_expiry(tmp_path: Path):
  path = tmp_path / "legacy-signal.json"
  write_legacy_signal("ssinf", {"straight": "RED_LIGHT_ON",
                               "straight_remain_time": 18}, path=path)
  raw = json.loads(path.read_text())
  received = raw["received_mono_ns"]
  hud = read_legacy_signal_hud(path, now_ns=received + 100)
  assert hud is not None and hud["candidates"][0]["remainingSec"] == 18
  assert read_legacy_signal_hud(path, now_ns=received + FRESH_NS + 1) is None
  assert read_legacy_signal_hud(path, now_ns=received - 1) is None
  write_legacy_signal("sinf", {"greenLightOn": True, "greenLightRemainTime": 9}, path=path)
  assert read_legacy_signal_hud(path) is not None


def test_invalid_packet_hides_previous_signal(tmp_path: Path):
  path = tmp_path / "legacy.json"
  write_legacy_signal("sinf", {"redLightOn": True, "redLightRemainTime": 11}, path)
  assert read_legacy_signal_hud(path)
  write_legacy_signal("sinf", {"redLightOn": False}, path)
  assert read_legacy_signal_hud(path) is None


def test_reject_tampered_or_overlarge_tmpfs_record(tmp_path: Path):
  path = tmp_path / "legacy.json"
  path.write_text("{" + "x" * 5000)
  assert read_legacy_signal_hud(path) is None
  path.write_text('{"received_mono_ns": 1, "candidates": "invalid"}')
  assert read_legacy_signal_hud(path, now_ns=2) is None
  path.write_text("garbled")
  assert read_legacy_signal_hud(path) is None
