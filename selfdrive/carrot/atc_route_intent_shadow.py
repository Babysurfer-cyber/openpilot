"""Read-only evidence for a possible mismatch with external turn guidance.

This module MUST NOT modify ATC targets, turn guidance, or vehicle control.
Model straightness alone cannot establish driving intent.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

TURN_SIDE = {1: 1, 2: -1, 3: 1, 4: -1}  # model y: left +, right -
MIN_LANE_PROB = 0.90
MAX_ROUTE_ANCHOR_ERROR_M = 5.0
MIN_POST_TURN_M = 20.0
MIN_ROUTE_DEFLECTION_DEG = 40.0
MAX_MODEL_DEFLECTION_DEG = 12.0
MIN_LATERAL_DISAGREEMENT_M = 10.0
REQUIRED_STABLE_FRAMES = 20  # ~1 second at the 20Hz CarrotMan loop


@dataclass(frozen=True)
class IntentEvidence:
  state: str
  reason: str
  consecutive_frames: int = 0


def _finite(value: Any) -> float:
  try:
    number = float(value)
  except (TypeError, ValueError, OverflowError):
    return math.nan
  return number if math.isfinite(number) else math.nan


def _xy_path(x: Any, y: Any) -> list[tuple[float, float]]:
  try:
    points = [(_finite(a), _finite(b)) for a, b in zip(x, y, strict=True)]
  except (TypeError, ValueError):
    return []
  if len(points) < 3 or any(not math.isfinite(a) or not math.isfinite(b) for a, b in points):
    return []
  if abs(points[0][0]) > 3.0 or any(points[i + 1][0] <= points[i][0] for i in range(len(points) - 1)):
    return []
  return points


def _point_at_x(points: list[tuple[float, float]], x: float) -> float | None:
  if not points or x < points[0][0] or x > points[-1][0]:
    return None
  for a, b in zip(points, points[1:]):
    if a[0] <= x <= b[0]:
      ratio = (x - a[0]) / (b[0] - a[0])
      return a[1] + (b[1] - a[1]) * ratio
  return None


def _point_along(points: list[tuple[float, float]], distance: float) -> tuple[float, float] | None:
  if distance < 0:
    return None
  remaining = distance
  for a, b in zip(points, points[1:]):
    segment = math.hypot(b[0] - a[0], b[1] - a[1])
    if segment <= 0:
      continue
    if remaining <= segment:
      t = remaining / segment
      return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
    remaining -= segment
  return None


def _route_relative(navi: Any) -> list[tuple[float, float]]:
  vehicle = navi.vehicle
  lat0, lon0 = _finite(vehicle.latitude), _finite(vehicle.longitude)
  heading = _finite(vehicle.heading_deg)
  if not all(map(math.isfinite, (lat0, lon0, heading))) or abs(lat0) > 80 or abs(lon0) > 180:
    return []
  rad = math.radians(heading)
  cos_lat = math.cos(math.radians(lat0))
  result = []
  try:
    coords = navi.route.polyline
    if len(coords) < 3 or len(coords) > 256:
      return []
    for lat, lon in coords:
      lat, lon = _finite(lat), _finite(lon)
      if not math.isfinite(lat) or not math.isfinite(lon) or abs(lat) > 85 or abs(lon) > 180:
        return []
      east = (lon - lon0) * 111_195 * cos_lat
      north = (lat - lat0) * 111_195
      result.append((east * math.sin(rad) + north * math.cos(rad),
                     -east * math.cos(rad) + north * math.sin(rad)))
  except (TypeError, ValueError, AttributeError):
    return []
  best = (math.inf, -1, (0.0, 0.0))
  for i, (a, b) in enumerate(zip(result, result[1:])):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-6:
      continue
    t = max(0.0, min(1.0, -(a[0] * dx + a[1] * dy) / length_sq))
    q = (a[0] + t * dx, a[1] + t * dy)
    d = math.hypot(*q)
    if d < best[0]:
      best = (d, i, q)
  if best[0] > MAX_ROUTE_ANCHOR_ERROR_M or best[1] < 0:
    return []
  return [best[2], *result[best[1] + 1:]]


def _can_correlate_lane(model: Any, model_path: list[tuple[float, float]], distance_m: float) -> bool:
  try:
    if len(model.laneLines) != 4 or len(model.laneLineProbs) != 4:
      return False
    if any(_finite(model.laneLineProbs[i]) < MIN_LANE_PROB for i in (1, 2)):
      return False
    left = _xy_path(model.laneLines[1].x, model.laneLines[1].y)
    right = _xy_path(model.laneLines[2].x, model.laneLines[2].y)
  except (AttributeError, TypeError, ValueError):
    return False
  for x in (8.0, distance_m + 12.0):
    yl, yr, yc = _point_at_x(left, x), _point_at_x(right, x), _point_at_x(model_path, x)
    if any(value is None for value in (yl, yr, yc)):
      return False
    width = yl - yr
    if not 2.6 <= width <= 4.5 or not yr < yc < yl or abs(yc - (yl + yr) * 0.5) > 0.7:
      return False
  return True


def mismatch_evidence(navi: Any, car_state: Any, model: Any, turn_info: int, distance_m: float) -> str:
  """Return evidence, NEVER a command to cancel a turn."""
  if navi is None or model is None or car_state is None or turn_info not in TURN_SIDE:
    return "unavailable"
  if navi.off_route or not navi.guidance_active or not (navi.current.present and navi.route.present and navi.vehicle.present):
    return "unavailable"
  distance_m = _finite(distance_m)
  if not 12.0 <= distance_m <= 130.0:
    return "outside_preview"
  # A tap means lane-change input, not a turn. Both tap and latched signal
  # veto evidence. A latched signal passes through tap for ~0.1s.
  if getattr(car_state, "blinkerLever", 0) != 0 or getattr(car_state, "leftBlinker", False) or getattr(car_state, "rightBlinker", False):
    return "stalk_or_lamp_active"
  try:
    if str(model.meta.laneChangeState) not in ("off", "0"):
      return "lane_change_active"
    model_path = _xy_path(model.position.x, model.position.y)
  except (AttributeError, TypeError, ValueError):
    return "model_missing"
  if not model_path:
    return "model_missing"
  post = distance_m + MIN_POST_TURN_M
  model_early = _point_along(model_path, 8.0)
  model_late = _point_along(model_path, post)
  if model_early is None or model_late is None or model_late[0] < distance_m + 10.0:
    return "model_not_far_enough"
  if not _can_correlate_lane(model, model_path, distance_m):
    return "lane_not_confident"
  route = _route_relative(navi)
  if not route:
    return "route_unusable"
  route_pre = _point_along(route, max(0.0, distance_m - 10.0))
  route_late = _point_along(route, post)
  if route_pre is None or route_late is None:
    return "route_not_far_enough"
  side = TURN_SIDE[turn_info]
  route_delta = route_late[1] - route_pre[1]
  if side * route_delta < 8.0:
    return "route_direction_unverified"
  route_heading = math.degrees(math.atan2(route_late[1] - route_pre[1], route_late[0] - route_pre[0]))
  model_heading = math.degrees(math.atan2(model_late[1] - model_early[1], model_late[0] - model_early[0]))
  if abs(route_heading) < MIN_ROUTE_DEFLECTION_DEG:
    return "route_direction_unverified"
  if abs(model_heading) > MAX_MODEL_DEFLECTION_DEG:
    return "model_not_straight"
  if abs(route_late[1] - model_late[1]) < MIN_LATERAL_DISAGREEMENT_M:
    return "separation_too_small"
  return "strong_route_model_disagreement"


class AtcRouteIntentShadow:
  """Consecutive evidence for diagnostics only. No actuator output."""
  def __init__(self) -> None:
    self._identity: tuple[Any, ...] | None = None
    self._streak = 0
    self._last_observed_s: float | None = None

  def observe(self, navi: Any, car_state: Any, model: Any, turn_info: int, distance_m: float) -> IntentEvidence:
    now_s = time.monotonic()
    if self._last_observed_s is not None and (now_s < self._last_observed_s or now_s - self._last_observed_s > 0.20):
      self._streak = 0
    self._last_observed_s = now_s
    identity = (navi.session_id, navi.current.sequence, turn_info) if navi is not None and navi.current.present else None
    if identity != self._identity:
      self._identity = identity
      self._streak = 0
    reason = mismatch_evidence(navi, car_state, model, turn_info, distance_m)
    if reason != "strong_route_model_disagreement":
      self._streak = 0
      return IntentEvidence("keep_atc", reason)
    self._streak = min(REQUIRED_STABLE_FRAMES, self._streak + 1)
    if self._streak < REQUIRED_STABLE_FRAMES:
      return IntentEvidence("keep_atc", "insufficient_consecutive_evidence", self._streak)
    return IntentEvidence("candidate_only", reason, self._streak)
