"""Read-only CarrotNM traffic/bar diagnostics, omitting coordinates and route names."""
from __future__ import annotations

import math
from typing import Any

from openpilot.selfdrive.carrot.route_traffic import extract_route_traffic


def _object(value: Any) -> dict:
  return value if isinstance(value, dict) else {}


def _distance(value: Any) -> int | None:
  if type(value) is int and 0 <= value <= 2_000_000:
    return value
  if type(value) is float and math.isfinite(value) and value.is_integer() and 0 <= value <= 2_000_000:
    return int(value)
  return None


def summarize_traffic(snapshot: dict[str, Any]) -> dict[str, Any]:
  """Explain why the existing route bar can/cannot show position and colors."""
  items = _object(snapshot.get("items"))
  route_record = _object(items.get("route"))
  status_record = _object(items.get("navigation_status"))
  route = _object(route_record.get("value")) if route_record.get("present") else {}
  status = _object(status_record.get("value")) if status_record.get("present") else {}
  speed_record = _object(items.get("speed"))
  speed = _object(speed_record.get("value")) if speed_record.get("present") else {}
  camera = _object(speed.get("sdi"))
  secondary_camera = _object(speed.get("sdi_secondary"))
  total = _distance(route.get("total_distance_m"))
  remaining = _distance(route.get("remain_distance_m"))
  connected = snapshot.get("connected") is True
  route_received = route_record.get("present") is True
  status_received = status_record.get("present") is True

  blockers = []
  if not connected:
    blockers.append("websocket_control_not_connected")
  if not status_received:
    blockers.append("navigation_status_missing_or_stale")
  if status.get("guidance_active") is not True:
    blockers.append("guidance_not_active")
  if status.get("off_route") is not False:
    blockers.append("route_off_or_unknown")
  if status.get("route_present") is not True:
    blockers.append("navigation_route_present_false")
  if not route_received:
    blockers.append("route_item_missing_or_stale")
  if total is None or total <= 0 or remaining is None or remaining > total:
    blockers.append("route_distance_invalid_or_missing")

  raw_traffic = route.get("traffic_sections")
  traffic_received = isinstance(raw_traffic, list)
  traffic_valid, sections = extract_route_traffic(route)
  if not traffic_valid:
    blockers.append("traffic_sections_missing_or_invalid")
  publisher_count = snapshot.get("cereal_publish_count")
  publisher_error = snapshot.get("cereal_error")
  if isinstance(publisher_count, int) and publisher_count == 0:
    blockers.append("carrotNavi_cereal_not_published")
  if publisher_error:
    blockers.append("carrotNavi_cereal_publish_error")

  position_ready = not any(x in blockers for x in (
    "websocket_control_not_connected", "navigation_status_missing_or_stale",
    "guidance_not_active", "route_off_or_unknown",
    "navigation_route_present_false", "route_item_missing_or_stale",
    "route_distance_invalid_or_missing",
    "carrotNavi_cereal_not_published", "carrotNavi_cereal_publish_error",
  ))
  traffic_ready = position_ready and traffic_valid

  # Diagnose whether the sender actually covers the entire route; valid
  # traffic_sections are allowed to contain gaps, which the HUD draws gray.
  lengths = {"good": 0, "bad": 0, "very_bad": 0, "none": 0}
  covered_m = 0
  last_section_end_m = None
  first_section_start_m = None
  if traffic_valid:
    names = {0: "none", 1: "good", 2: "bad", 3: "very_bad"}
    for section in sections:
      start = section["startDistanceM"]
      end = start + section["lengthM"]
      if first_section_start_m is None:
        first_section_start_m = start
      last_section_end_m = end
      covered_m += section["lengthM"]
      lengths[names[section["congestionCode"]]] += section["lengthM"]
  uncovered_m = total - covered_m if traffic_valid and total is not None else None
  covered_percent = round(100.0 * covered_m / total, 1) if traffic_valid and total else None

  return {
    "connected": connected,
    "route_item_present": route_received,
    "navigation_status_present": status_received,
    "guidance_active": status.get("guidance_active") is True,
    "off_route": status.get("off_route"),
    "route_present": status.get("route_present"),
    "total_distance_m": total,
    "remain_distance_m": remaining,
    "traffic_source": route.get("traffic_source", "(not sent)"),
    "traffic_probe": route.get("traffic_probe", "(not sent)"),
    # v5 APK: compare validated coverage from all three Naver SDK APIs.
    # -1 means a candidate could not be extracted or was invalid.
    "traffic_guidance_covered_m": route.get("traffic_guidance_covered_m"),
    "traffic_route_sections_covered_m": route.get("traffic_route_sections_covered_m"),
    "traffic_route_traffics_covered_m": route.get("traffic_route_traffics_covered_m"),
    "traffic_selected_covered_m": route.get("traffic_selected_covered_m"),
    # Input-side SDI values for diagnosing repeated fixed-camera warnings.
    # These are read-only app messages, not the car's CAN camera signals.
    "speed_item_present": speed_record.get("present") is True,
    "sdi_present": bool(camera),
    "sdi_type": camera.get("type"),
    "sdi_distance_m": camera.get("distance_m"),
    "sdi_speed_limit_kph": camera.get("speed_limit_kph"),
    "secondary_sdi_present": bool(secondary_camera),
    "secondary_sdi_type": secondary_camera.get("type"),
    "secondary_sdi_distance_m": secondary_camera.get("distance_m"),
    "road_limit_kph": speed.get("road_limit_kph"),
    "traffic_sections_received": len(raw_traffic) if traffic_received else None,
    "traffic_sections_valid": len(sections) if traffic_valid else 0,
    "traffic_lengths_m": lengths if traffic_valid else None,
    "traffic_covered_m": covered_m if traffic_valid else None,
    "traffic_covered_percent": covered_percent,
    "traffic_uncovered_m": uncovered_m,
    "traffic_first_section_start_m": first_section_start_m,
    "traffic_last_section_end_m": last_section_end_m,
    "cereal_publish_count": publisher_count,
    "cereal_error": publisher_error,
    "bar_position_expected": position_ready,
    "bar_traffic_colors_expected": traffic_ready,
    "reasons": blockers,
    "note": "Read-only summary. No GPS coordinates, route names, or vehicle control.",
  }
