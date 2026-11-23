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
    "traffic_sections_received": len(raw_traffic) if traffic_received else None,
    "traffic_sections_valid": len(sections) if traffic_valid else 0,
    "cereal_publish_count": publisher_count,
    "cereal_error": publisher_error,
    "bar_position_expected": position_ready,
    "bar_traffic_colors_expected": traffic_ready,
    "reasons": blockers,
    "note": "Read-only summary. No GPS coordinates, route names, or vehicle control.",
  }
