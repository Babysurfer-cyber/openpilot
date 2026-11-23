"""Strict, display-only congestion sections from an optional route JSON extension.

CarrotNM 1.30 does not currently send this field. Never derive congestion
from progress, speed-limit cameras, TBT, or traffic-signal information.

The optional wire field lives inside the *same* route snapshot:
  traffic_sections: [{start_distance_m: int, length_m: int,
                      congestion: "good"|"bad"|"very_bad"|"none"}]
All distances are measured from the START of that particular route, in meters.
"""
from __future__ import annotations

import math
from typing import Any

MAX_TRAFFIC_SECTIONS = 128
MAX_ROUTE_DISTANCE_M = 2_000_000
_CONGESTION_CODES = {
  "none": 0,
  "good": 1,
  "bad": 2,
  "verybad": 3,
  "very_bad": 3,
}


def _meters(value: Any) -> int | None:
  if type(value) not in (int, float) or not math.isfinite(value):
    return None
  if value < 0 or value > MAX_ROUTE_DISTANCE_M or int(value) != value:
    return None
  return int(value)


def extract_route_traffic(route: Any) -> tuple[bool, list[dict[str, int]]]:
  """Accept only a complete, ordered and bounded traffic section snapshot.

  present=False means no usable congestion data; the HUD stays neutral.
  No previously received data are reused on a reroute or expired route.
  """
  if not isinstance(route, dict):
    return False, []
  raw = route.get("traffic_sections")
  total = _meters(route.get("total_distance_m"))
  if (not isinstance(raw, list) or not 0 < len(raw) <= MAX_TRAFFIC_SECTIONS
      or total is None or total == 0):
    return False, []

  segments: list[dict[str, int]] = []
  previous_end = 0
  for item in raw:
    if not isinstance(item, dict):
      return False, []
    start = _meters(item.get("start_distance_m"))
    length = _meters(item.get("length_m"))
    congestion = item.get("congestion")
    if (start is None or length is None or length == 0
        or not isinstance(congestion, str)):
      return False, []
    code = _CONGESTION_CODES.get(congestion.strip().lower())
    if code is None or start < previous_end or start + length > total:
      return False, []
    segments.append({"startDistanceM": start, "lengthM": length, "congestionCode": code})
    previous_end = start + length
  return True, segments
