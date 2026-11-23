"""Validate optional CarrotNM route congestion segments (display only).

Patched CarrotNM 1.30 puts traffic_sections inside the existing json/route value.
Never infer congestion from route progress or reuse traffic after it disappears.
"""
from __future__ import annotations

import math
from typing import Any

MAX_ROUTE_DISTANCE_M = 2_000_000
MAX_TRAFFIC_SECTIONS = 128

_CONGESTION_CODES = {
  "none": 0,
  "good": 1,
  "bad": 2,
  "verybad": 3,
  "very_bad": 3,
}


def _meters(value: Any) -> int | None:
  # Avoid bool coercion, fractional distances, infinities and overflow.
  if type(value) is int:
    return value if 0 <= value <= MAX_ROUTE_DISTANCE_M else None
  if type(value) is float and math.isfinite(value):
    return int(value) if 0 <= value <= MAX_ROUTE_DISTANCE_M and value.is_integer() else None
  return None


def extract_route_traffic(route: Any) -> tuple[bool, list[dict[str, int]]]:
  """Return validated sections measured in meters from the route's start.

  Invalid, absent, unordered, overlapping or out-of-bounds input is rejected
  entirely. Valid gaps (including None congestion) are drawn neutral gray.
  """
  if not isinstance(route, dict):
    return False, []
  total_m = _meters(route.get("total_distance_m"))
  raw = route.get("traffic_sections")
  if total_m is None or total_m <= 0 or not isinstance(raw, list) or not 1 <= len(raw) <= MAX_TRAFFIC_SECTIONS:
    return False, []

  sections: list[dict[str, int]] = []
  previous_end = 0
  for item in raw:
    if not isinstance(item, dict):
      return False, []
    start_m = _meters(item.get("start_distance_m"))
    length_m = _meters(item.get("length_m"))
    congestion = item.get("congestion")
    if start_m is None or length_m is None or length_m == 0 or not isinstance(congestion, str):
      return False, []
    code = _CONGESTION_CODES.get(congestion.strip().casefold())
    if code is None or start_m < previous_end or start_m + length_m > total_m:
      return False, []
    sections.append({"startDistanceM": start_m, "lengthM": length_m, "congestionCode": code})
    previous_end = start_m + length_m

  return True, sections
