"""Read-only bridge from stock CarrotNavi sinf/ssinf to the existing HUD strip.

Legacy TCP/HTTP packets are received in carrot_man, whereas the UI consumes the
separate carrotNavi cereal publisher.  An atomic, short-lived tmpfs record lets
the two processes share observations without modifying the APK or controls.
This module NEVER writes carrotMan.trafficState or DETECT commands.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any


LEGACY_SIGNAL_PATH = Path("/dev/shm/carrot_navi_legacy_signal_hud.json")
FRESH_NS = 2_500_000_000  # Match the HUD's 2.5 s receive-time limit.
MAX_FILE_BYTES = 4096
ALLOWED_STATES = frozenset(("RED", "YELLOW", "GREEN"))
ALLOWED_DIRECTIONS = frozenset(("STRAIGHT", "LEFT"))


def _seconds(value: Any) -> tuple[bool, int]:
  # No guessed countdown from a missing/invalid field.
  if type(value) is bool or value is None:
    return False, 0
  try:
    number = float(value)
  except (TypeError, ValueError, OverflowError):
    return False, 0
  if not math.isfinite(number) or not 0 <= number <= 600 or number != int(number):
    return False, 0
  return True, int(number)


def _lamp(direction: str, state: str, seconds: Any, index: int) -> dict[str, Any]:
  valid, remaining = _seconds(seconds)
  return {"direction": direction, "state": state, "remainingValid": valid,
          "remainingSec": remaining, "groupIndex": 0, "signalIndex": index}


def lamps_from_sinf(data: Any) -> list[dict[str, Any]]:
  """Legacy summary: only observed ON flags; red is a general stop indication.

  Never claim that the red flag is a verified lane-specific signal.
  Refuse conflicting red+straight-green flags rather than select a winner.
  """
  if not isinstance(data, dict):
    return []
  red = data.get("redLightOn") is True
  green = data.get("greenLightOn") is True
  left = data.get("leftLightOn") is True
  if red and green:
    return []
  lamps = []
  if red:
    lamps.append(_lamp("STRAIGHT", "RED", data.get("redLightRemainTime"), 0))
  elif green:
    lamps.append(_lamp("STRAIGHT", "GREEN", data.get("greenLightRemainTime"), 0))
  if left:
    lamps.append(_lamp("LEFT", "GREEN", data.get("leftLightRemainTime"), 1))
  return lamps


def lamps_from_ssinf(data: Any) -> list[dict[str, Any]]:
  """Detailed legacy directional states; ignore unknown or OFF states."""
  if not isinstance(data, dict):
    return []
  states = {"RED_LIGHT_ON": "RED", "YELLOW_LIGHT_ON": "YELLOW",
            "GREEN_LIGHT_ON": "GREEN"}
  lamps = []
  for index, (key, direction) in enumerate((("straight", "STRAIGHT"), ("left", "LEFT"))):
    state = data.get(key)
    if not isinstance(state, str):
      continue
    color = states.get(state.upper())
    if color is not None:
      lamps.append(_lamp(direction, color, data.get(key + "_remain_time"), index))
  return lamps


def lamps_from_v2_lights(data: Any) -> list[dict[str, Any]]:
  """Unmodified CarrotNavi v2 traffic_signal.lights fallback.

  The app's red light is global; treat it only as an unverified red indicator.
  Explicit per-direction V2 movements override that global projection.
  """
  if not isinstance(data, dict) or data.get("visible") is not True:
    return []
  movements = data.get("movements")
  if isinstance(movements, dict):
    values = {}
    mapped = {"RED_LIGHT_ON": "RED", "YELLOW_LIGHT_ON": "YELLOW",
              "GREEN_LIGHT_ON": "GREEN"}
    for index, (key, direction) in enumerate((("straight", "STRAIGHT"), ("left", "LEFT"))):
      item = movements.get(key)
      if not isinstance(item, dict):
        continue
      state = item.get("state")
      if isinstance(state, str) and state.upper() in mapped:
        values[direction] = _lamp(direction, mapped[state.upper()], item.get("remain_sec"), index)
    if values:
      return list(values.values())

  lights = data.get("lights")
  if not isinstance(lights, dict):
    return []
  def on(name: str) -> bool:
    item = lights.get(name)
    return isinstance(item, dict) and item.get("on") is True
  def remaining(name: str) -> Any:
    item = lights.get(name)
    return item.get("remain_sec") if isinstance(item, dict) else None

  red, green, left = on("red"), on("green"), on("left")
  if red and green:
    return []
  lamps = []
  if red:
    lamps.append(_lamp("STRAIGHT", "RED", remaining("red"), 0))
  elif green:
    lamps.append(_lamp("STRAIGHT", "GREEN", remaining("green"), 0))
  if left:
    lamps.append(_lamp("LEFT", "GREEN", remaining("left"), 1))
  return lamps


def _validated_candidates(lamps: Any) -> list[dict[str, Any]]:
  if not isinstance(lamps, list) or not 1 <= len(lamps) <= 2:
    return []
  seen = set()
  output = []
  for item in lamps:
    if not isinstance(item, dict):
      return []
    direction, state = item.get("direction"), item.get("state")
    if (direction not in ALLOWED_DIRECTIONS or state not in ALLOWED_STATES or
        direction in seen or type(item.get("remainingValid")) is not bool or
        type(item.get("remainingSec")) is not int or
        not 0 <= item["remainingSec"] <= 600):
      return []
    seen.add(direction)
    output.append(_lamp(direction, state,
                        item["remainingSec"] if item["remainingValid"] else None,
                        0 if direction == "STRAIGHT" else 1))
  return output


def signal_hud_from_lamps(lamps: Any, received_ns: Any,
                          sequence: int = 0, timestamp_ms: int = 0) -> dict[str, Any] | None:
  candidates = _validated_candidates(lamps)
  if not candidates or type(received_ns) is not int or received_ns <= 0:
    return None
  if type(sequence) is not int or sequence < 0 or type(timestamp_ms) is not int or timestamp_ms < 0:
    return None
  return {"meta": {"present": True, "sequence": sequence,
                   "sourceTimestampMillis": timestamp_ms,
                   "receivedMonoTimeNanos": received_ns},
          "directionMatchVerified": False, "candidates": candidates}


def write_legacy_signal(kind: str, payload: Any, path: Path = LEGACY_SIGNAL_PATH) -> None:
  """Record only a fresh observation. Empty candidates immediately hide stale data."""
  if kind not in ("sinf", "ssinf"):
    return
  lamps = lamps_from_sinf(payload) if kind == "sinf" else lamps_from_ssinf(payload)
  data = {"received_mono_ns": time.monotonic_ns(),
          "source_timestamp_ms": time.time_ns() // 1_000_000,
          "candidates": _validated_candidates(lamps)}
  temp_name = None
  try:
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                     prefix=".carrot_legacy_signal_", delete=False) as stream:
      temp_name = stream.name
      json.dump(data, stream, separators=(",", ":"))
    os.replace(temp_name, path)
  finally:
    if temp_name is not None:
      try:
        os.unlink(temp_name)
      except FileNotFoundError:
        pass


def read_legacy_signal_hud(path: Path = LEGACY_SIGNAL_PATH,
                           now_ns: int | None = None) -> dict[str, Any] | None:
  try:
    with path.open("rb") as stream:
      raw = stream.read(MAX_FILE_BYTES + 1)
    if len(raw) > MAX_FILE_BYTES:
      return None
    obj = json.loads(raw)
    if not isinstance(obj, dict):
      return None
    received = obj.get("received_mono_ns")
    now = time.monotonic_ns() if now_ns is None else now_ns
    if type(received) is not int or not 0 <= now - received <= FRESH_NS:
      return None
    return signal_hud_from_lamps(obj.get("candidates"), received,
                                  timestamp_ms=obj.get("source_timestamp_ms", 0))
  except (OSError, ValueError, TypeError, OverflowError, UnicodeDecodeError):
    return None
