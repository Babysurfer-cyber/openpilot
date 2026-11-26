"""Read-only CarrotNM traffic-signal candidates for the comma HUD.

Does not alter the legacy trafficSignal object used by carrot controls. A
candidate is a *reported* signal, not a verified signal for our lane.
"""
from __future__ import annotations

from typing import Any

ALLOWED_DIRECTIONS = frozenset(("STRAIGHT", "LEFT", "RIGHT", "U_TURN"))
ALLOWED_STATES = frozenset(("RED", "YELLOW", "GREEN", "RED_BLINK", "YELLOW_BLINK", "GREEN_BLINK"))
MAX_SIGNALS = 2  # No arbitrary first-N selection when several junctions are reported.


def _dict(value: Any) -> dict[str, Any]:
  return value if isinstance(value, dict) else {}


def _record(snapshot: dict[str, Any], name: str) -> dict[str, Any]:
  return _dict(_dict(snapshot.get("items")).get(name))


def _empty() -> dict[str, Any]:
  return {"meta": {"present": False, "sequence": 0, "sourceTimestampMillis": 0,
                   "receivedMonoTimeNanos": 0},
          "directionMatchVerified": False, "candidates": []}


def build_signal_hud(snapshot: dict[str, Any]) -> dict[str, Any]:
  """Use the independent V2 signal channel, or a Bridge-injected nested payload.

  The source stream's monotonic arrival timestamp is retained for display TTL.
  A retransmitted / stale / absent record cannot be made fresh by UI repaints.
  """
  output = _empty()
  if snapshot.get("connected") is not True:
    return output

  for name in ("traffic_signal", "route", "guidance_current", "guidance_next",
               "navigation_status"):
    record = _record(snapshot, name)
    if record.get("present") is not True:
      continue
    value = _dict(record.get("value"))
    telemetry = _dict(value.get("traffic_signal"))
    if not telemetry and name == "traffic_signal":
      telemetry = value
    if (type(telemetry.get("schema_version")) is not int or telemetry["schema_version"] != 1 or
        telemetry.get("source") != "naver_traffic_signal_live_data" or
        telemetry.get("vehicle_control_allowed") is not False or
        type(telemetry.get("direction_match_verified")) is not bool):
      continue
    raw_signals = telemetry.get("signals")
    if not isinstance(raw_signals, list) or not 1 <= len(raw_signals) <= MAX_SIGNALS:
      continue
    candidates: list[dict[str, Any]] = []
    seen: set[tuple[str, int, int]] = set()
    for raw in raw_signals:
      item = _dict(raw)
      direction = item.get("direction")
      state = item.get("state")
      group = item.get("group_index")
      index = item.get("signal_index")
      if (type(direction) is not str or type(state) is not str or
          direction not in ALLOWED_DIRECTIONS or state not in ALLOWED_STATES or
          type(group) is not int or group < 0 or group > 7 or
          type(index) is not int or index < 0 or index > 11 or
          (direction, group, index) in seen):
        candidates = []
        break
      seen.add((direction, group, index))
      remaining = item.get("remaining_seconds")
      remaining_valid = type(remaining) is int and 0 <= remaining <= 600
      candidates.append({"direction": direction, "state": state,
                         "remainingValid": remaining_valid,
                         "remainingSec": remaining if remaining_valid else 0})
    if not candidates:
      continue
    received = record.get("received_mono_ns")
    sequence = record.get("sequence")
    timestamp = record.get("source_timestamp_ms")
    if (type(received) is not int or received <= 0 or
        type(sequence) is not int or sequence < 0 or
        type(timestamp) is not int or timestamp < 0):
      continue
    output.update({
      "meta": {"present": True, "sequence": sequence,
               "sourceTimestampMillis": timestamp,
               "receivedMonoTimeNanos": received},
      "directionMatchVerified": telemetry["direction_match_verified"],
      "candidates": candidates,
    })
    return output
  return output
