"""Optional mode-5 cruise set-speed cap on a confirmed upcoming map congestion section.

Naver route congestion is predictive, NOT an obstacle/lead-vehicle sensor.
This helper changes only the requested cruise set speed. It cannot command
brakes, resume cruise, or override ordinary lead and collision protections.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

TRIGGER_DISTANCE_M = 100
TRIGGER_EGO_KPH = 90.0
MAX_SET_SPEED_KPH = 80.0
SERVICE_TIMEOUT_S = 1.0
ITEM_TIMEOUT_S = 3.0
MAX_SECTIONS = 128


@dataclass(frozen=True)
class JamAhead:
  key: tuple[str, int, int, int]
  route_sequence: int
  distance_m: int


def _fresh(meta: Any, now_s: float) -> bool:
  if getattr(meta, "present", False) is not True:
    return False
  received = getattr(meta, "receivedMonoTimeNanos", 0)
  if type(received) is not int or received <= 0:
    return False
  age = now_s - received * 1e-9
  return 0 <= age <= ITEM_TIMEOUT_S


def imminent_route_jam(sm: Any, *, now_s: float | None = None) -> JamAhead | None:
  """Return a verified very_bad section starting 0..100m ahead, else None.

  Route distances are from the start of the *current* Naver route. The route
  and guidance status must both be fresh, live, and mutually consistent.
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
      distance = start - progress
      if code == 3 and 0 <= distance <= TRIGGER_DISTANCE_M and near is None:
        near = JamAhead((str(nav.sessionId), total, start, length), int(route.meta.sequence), distance)
    return near
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return None


class RouteCongestionCruiseCap:
  """One reduction per engaged cruise session; no automatic speed restoration.

  A second distinct route heartbeat must confirm the same approaching section.
  Explicit driver/remote set-speed input releases the cap and prevents repeated
  unwanted reductions until cruise is disengaged and engaged again.
  """
  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.active = False
    self.manual_override = False
    self.candidate_key: tuple[str, int, int, int] | None = None
    self.candidate_sequence: int | None = None

  def update(self, target_kph: float, ego_kph: float, sm: Any, *, engaged: bool,
             manual_speed_change: bool = False, gas_pressed: bool = False,
             now_s: float | None = None) -> tuple[float, bool]:
    """Return (possibly reduced set speed, newly_reduced).

    Unknown/stale route traffic never initiates a reduction. Once reduced,
    loss of navigation will not silently accelerate back to the old target.
    """
    if not engaged:
      self.reset()
      return target_kph, False
    if manual_speed_change:
      self.active = False
      self.manual_override = True
      self.candidate_key = None
      self.candidate_sequence = None
      return target_kph, False
    if self.manual_override:
      return target_kph, False
    if self.active:
      return min(target_kph, MAX_SET_SPEED_KPH), False
    if (gas_pressed or not math.isfinite(ego_kph) or ego_kph < TRIGGER_EGO_KPH
        or not math.isfinite(target_kph) or target_kph <= MAX_SET_SPEED_KPH):
      self.candidate_key = None
      self.candidate_sequence = None
      return target_kph, False

    jam = imminent_route_jam(sm, now_s=now_s)
    if jam is None:
      self.candidate_key = None
      self.candidate_sequence = None
      return target_kph, False
    if self.candidate_key == jam.key:
      if self.candidate_sequence != jam.route_sequence:
        self.active = True
        return min(target_kph, MAX_SET_SPEED_KPH), True
    else:
      self.candidate_key = jam.key
      self.candidate_sequence = jam.route_sequence
    return target_kph, False
