"""Mode-5 cruise set-speed caps for confirmed upcoming slow and congested route sections.

Naver route traffic predicts general road flow; it is NOT an obstacle sensor.
Only the requested cruise set speed is capped. Normal lead, collision and
vehicle acceleration/braking limits remain authoritative.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

SLOW_TRIGGER_DISTANCE_M = 200
JAM_TRIGGER_DISTANCE_M = 300
SLOW_MAX_SET_SPEED_KPH = 90.0
JAM_MAX_SET_SPEED_KPH = 70.0
SERVICE_TIMEOUT_S = 1.0
ITEM_TIMEOUT_S = 3.0
MAX_SECTIONS = 128

# Naver congestionCode: 0 unknown, 1 good, 2 slow, 3 congested.
TRAFFIC_CAPS = {
  2: (SLOW_TRIGGER_DISTANCE_M, SLOW_MAX_SET_SPEED_KPH),
  3: (JAM_TRIGGER_DISTANCE_M, JAM_MAX_SET_SPEED_KPH),
}


@dataclass(frozen=True)
class JamAhead:
  key: tuple[str, int, int, int, int]
  route_sequence: int
  distance_m: int
  cap_kph: float


def _fresh(meta: Any, now_s: float) -> bool:
  if getattr(meta, "present", False) is not True:
    return False
  received = getattr(meta, "receivedMonoTimeNanos", 0)
  if type(received) is not int or received <= 0:
    return False
  age = now_s - received * 1e-9
  return 0 <= age <= ITEM_TIMEOUT_S


def _imminent_route_traffic(sm: Any, *, now_s: float | None, allowed_codes: tuple[int, ...]) -> JamAhead | None:
  """Select the most restrictive section approaching within its own distance.

  Validate the entire current route before using any traffic section. Route
  section offsets are measured from the beginning of the active Naver route.
  """
  if now_s is None:
    now_s = time.monotonic()
  try:
    service = "carrotNavi"
    if not (sm.seen[service] and sm.alive[service] and sm.valid[service]):
      return None
    if not 0 <= now_s - sm.recv_time[service] <= SERVICE_TIMEOUT_S:
      return None
    nav = sm[service]
    if not (nav.connected and nav.sessionId):
      return None
    status = nav.navigationStatus
    route = nav.route
    if not (_fresh(status.meta, now_s) and status.guidanceActive and not status.offRoute
            and status.routePresent and _fresh(route.meta, now_s) and route.trafficPresent):
      return None

    total = route.totalDistanceM
    remaining = route.remainingDistanceM
    if (type(total) is not int or type(remaining) is not int
        or not 0 < total <= 2_000_000 or not 0 <= remaining <= total):
      return None
    progress = total - remaining

    sections = route.trafficSections
    if not 1 <= len(sections) <= MAX_SECTIONS:
      return None
    last_end = 0
    near: JamAhead | None = None
    for section in sections:
      start, length, code = section.startDistanceM, section.lengthM, section.congestionCode
      if (type(start) is not int or type(length) is not int or type(code) is not int
          or start < last_end or length <= 0 or start + length > total or code not in (0, 1, 2, 3)):
        return None
      last_end = start + length
      if code not in allowed_codes:
        continue
      trigger_m, cap_kph = TRAFFIC_CAPS[code]
      distance = start - progress
      if 0 <= distance <= trigger_m:
        candidate = JamAhead((str(nav.sessionId), total, start, length, code),
                             int(route.meta.sequence), distance, cap_kph)
        # Congestion (70 km/h) wins over slow (90 km/h) if both are ahead.
        # Within the same severity, choose the nearest upcoming section.
        if near is None or (candidate.cap_kph, candidate.distance_m) < (near.cap_kph, near.distance_m):
          near = candidate
    return near
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return None


def imminent_route_traffic(sm: Any, *, now_s: float | None = None) -> JamAhead | None:
  """Find slow within 200m or congestion within 300m, if verified."""
  return _imminent_route_traffic(sm, now_s=now_s, allowed_codes=(2, 3))


def imminent_route_jam(sm: Any, *, now_s: float | None = None) -> JamAhead | None:
  """Compatibility helper: only congestion within 300m."""
  return _imminent_route_traffic(sm, now_s=now_s, allowed_codes=(3,))


class RouteCongestionCruiseCap:
  """Latch non-increasing route caps (90 then optionally 70) per cruise session.

  Confirm each distinct severity/section via two different current route
  messages. Never automatically restore speed when traffic improves or
  navigation disappears. A manual/remote speed choice releases the cap until
  cruise is disengaged and engaged again.
  """
  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.active = False
    self.cap_kph: float | None = None
    self.manual_override = False
    self.candidate_key: tuple[str, int, int, int, int] | None = None
    self.candidate_sequence: int | None = None

  def _clear_candidate(self) -> None:
    self.candidate_key = None
    self.candidate_sequence = None

  def update(self, target_kph: float, ego_kph: float, sm: Any, *, engaged: bool,
             manual_speed_change: bool = False, gas_pressed: bool = False,
             now_s: float | None = None) -> tuple[float, bool]:
    """Return (possibly capped set speed, whether it was lowered this update)."""
    if not engaged:
      self.reset()
      return target_kph, False
    if manual_speed_change:
      self.active = False
      self.cap_kph = None
      self.manual_override = True
      self._clear_candidate()
      return target_kph, False
    if self.manual_override:
      return target_kph, False

    current_target = min(target_kph, self.cap_kph) if self.cap_kph is not None else target_kph

    # No minimum ego-speed requirement: the map threshold is solely distance.
    # Keep a previously confirmed cap even when navigation/inputs are stale.
    if (gas_pressed or not math.isfinite(ego_kph) or not math.isfinite(target_kph)):
      self._clear_candidate()
      return current_target, False

    section = imminent_route_traffic(sm, now_s=now_s)
    if section is None or (self.cap_kph is not None and section.cap_kph >= self.cap_kph):
      self._clear_candidate()
      return current_target, False

    if self.candidate_key == section.key:
      if self.candidate_sequence != section.route_sequence:
        self.cap_kph = section.cap_kph if self.cap_kph is None else min(self.cap_kph, section.cap_kph)
        self.active = True
        self._clear_candidate()
        updated_target = min(target_kph, self.cap_kph)
        return updated_target, updated_target < current_target - 1e-6
    else:
      self.candidate_key = section.key
      self.candidate_sequence = section.route_sequence
    return current_target, False
