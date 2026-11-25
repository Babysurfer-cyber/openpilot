"""Conservative adjacent-lane lead handoff during an active lane change.

The radar matcher owns leadOne. This helper only suggests a measured side
track for leadTwo, and never changes cruise speed or acceleration limits.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


MIN_HISTORY_S = 0.20
MAX_FRAME_GAP_S = 0.15
MAX_ASSIST_S = 4.0
LANE_MARGIN_M = 0.20
LANE_WIDTH_MIN_M = 2.4
LANE_WIDTH_MAX_M = 5.2


@dataclass
class _SideHistory:
  first_s: float
  last_s: float
  count: int
  point: Any


def _side_of_point(model: Any, point: Any, expected_side: str | None = None) -> str | None:
  # SCC's yRel is unreliable and must never establish adjacent-lane occupancy.
  source = str(getattr(point, "source", ""))
  if source != "frontRadar" and not source.startswith("corner"):
    return None
  if not getattr(point, "measured", False):
    return None
  # Negative/synthetic track IDs cannot establish a reliable vehicle identity.
  if not isinstance(getattr(point, "track_id", None), int) or point.track_id < 0:
    return None
  # Native front-radar state 1 is a tentative track; do not promote it to leadTwo.
  if source == "frontRadar" and getattr(point, "radar_track_state", 0) == 1:
    return None
  try:
    radar_values = (point.d_rel, point.y_rel, point.v_rel, point.v_lead, point.yv_rel)
    if not all(math.isfinite(float(v)) for v in radar_values) or not 0.0 < point.d_rel < 80.0:
      return None
  except (ValueError, TypeError):
    return None
  lines = getattr(model, "laneLines", ())
  probs = getattr(model, "laneLineProbs", ())
  if len(lines) < 4 or len(probs) < 4:
    return None
  try:
    confidence = tuple(float(p) for p in probs[:4])
  except (TypeError, ValueError):
    return None
  if not all(math.isfinite(p) and 0.0 <= p <= 1.0 for p in confidence):
    return None
  position_x = float(point.d_rel) + 1.52  # camera and front-radar origin offset
  try:
    boundaries = [float(np.interp(position_x, line.x, line.y)) for line in lines[:4]]
  except (ValueError, TypeError):
    return None
  if not all(math.isfinite(y) for y in boundaries):
    return None
  # Outlier lane-line geometry can misclassify a car several lanes away.
  # Require a plausible ego lane and plausible selected adjacent lane.
  ego_width = boundaries[2] - boundaries[1]
  if not LANE_WIDTH_MIN_M <= ego_width <= LANE_WIDTH_MAX_M:
    return None
  # Camera lane lines use right-positive coordinates, radar uses left-positive.
  y = -float(point.y_rel)
  left_valid = (min(confidence[0], confidence[1]) >= 0.5
                and LANE_WIDTH_MIN_M <= boundaries[1] - boundaries[0] <= LANE_WIDTH_MAX_M)
  right_valid = (min(confidence[2], confidence[3]) >= 0.5
                 and LANE_WIDTH_MIN_M <= boundaries[3] - boundaries[2] <= LANE_WIDTH_MAX_M)
  if expected_side == "left":
    # During a lane change the same car can move from adjacent to current lane.
    # Keep its identity, but reject broken lane geometry or an opposite-lane car.
    return "left" if left_valid and y < boundaries[2] - LANE_MARGIN_M else None
  if expected_side == "right":
    return "right" if right_valid and y > boundaries[1] + LANE_MARGIN_M else None
  if left_valid and boundaries[0] + LANE_MARGIN_M < y < boundaries[1] - LANE_MARGIN_M:
    return "left"
  if right_valid and boundaries[2] + LANE_MARGIN_M < y < boundaries[3] - LANE_MARGIN_M:
    return "right"
  return None


def _continuous(previous: Any, point: Any, dt: float) -> bool:
  return (0.0 < dt <= MAX_FRAME_GAP_S
          and point.measured
          and not (point.source == "frontRadar" and getattr(point, "radar_track_state", 0) == 1)
          and abs(point.d_rel - (previous.d_rel + previous.v_rel * dt)) <= 2.5
          and abs(point.y_rel - (previous.y_rel + previous.yv_rel * dt)) <= 0.8
          and abs(point.v_lead - previous.v_lead) <= 3.0)


class LaneChangeSpeedAssist:
  """Latch a stable adjacent track before lane change; suggest it only during starting."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._history: dict[tuple[str, int], tuple[str, _SideHistory]] = {}
    self._state = "off"
    self._pending_direction: str | None = None
    self._identity: tuple[str, int] | None = None
    self._last_point: Any | None = None
    self._last_s: float | None = None
    self._started_s: float | None = None

  def _observe(self, time_s: float, model: Any, points: tuple[Any, ...]) -> None:
    fresh: dict[tuple[str, int], tuple[str, _SideHistory]] = {}
    for point in points:
      side = _side_of_point(model, point)
      if side is None or point.v_lead < 2.0 or not math.isfinite(point.v_rel):
        continue
      identity = (point.source, point.track_id)
      previous = self._history.get(identity)
      if (previous is not None and previous[0] == side
          and _continuous(previous[1].point, point, time_s - previous[1].last_s)):
        history = _SideHistory(previous[1].first_s, time_s, previous[1].count + 1, point)
      else:
        history = _SideHistory(time_s, time_s, 1, point)
      fresh[identity] = (side, history)
    self._history = fresh

  def update(self, time_s: float, v_ego: float, model: Any, points: tuple[Any, ...]) -> Any | None:
    meta = getattr(model, "meta", None)
    state = str(getattr(meta, "laneChangeState", "off"))
    direction = str(getattr(meta, "laneChangeDirection", "none"))
    if not math.isfinite(time_s) or not math.isfinite(v_ego):
      self.reset()
      return None

    if state in ("off", "preLaneChange"):
      if self._state not in ("off", "preLaneChange") or (state == "off" and self._state != "off"):
        self.reset()
      if state == "preLaneChange":
        if direction not in ("left", "right"):
          self.reset()
          return None
        # A pending direction change must not borrow a previous side's history.
        if self._state == "preLaneChange" and self._pending_direction != direction:
          self._history.clear()
        self._pending_direction = direction
      else:
        self._pending_direction = None
      self._observe(time_s, model, points)
      self._state = state
      self._identity = None
      self._last_point = None
      self._last_s = None
      return None

    if state != "laneChangeStarting" or direction not in ("left", "right"):
      self.reset()
      return None
    # Direction is latched in preLaneChange and cannot change during the maneuver.
    if self._pending_direction != direction:
      self.reset()
      self._state = state
      return None

    if self._state != "laneChangeStarting":
      # Never infer an intent from the lamp alone. A real FSM transition is required.
      if self._state != "preLaneChange":
        self.reset()
        self._state = state
        return None
      eligible = [
        (identity, history.point)
        for identity, (side, history) in self._history.items()
        if side == direction and history.count >= 3
        and time_s - history.first_s >= MIN_HISTORY_S
        and 0.0 <= time_s - history.last_s <= MAX_FRAME_GAP_S
        and history.point.v_rel <= 0.0
      ]
      self._identity = min(eligible, key=lambda pair: pair[1].d_rel)[0] if eligible else None
      self._started_s = time_s
      self._last_point = self._history[self._identity][1].point if self._identity is not None else None
      self._last_s = self._history[self._identity][1].last_s if self._identity is not None else None
    self._state = state
    if self._identity is None or self._started_s is None or time_s - self._started_s > MAX_ASSIST_S:
      return None
    point = next((p for p in points if (p.source, p.track_id) == self._identity), None)
    if (point is None or self._last_point is None or self._last_s is None
        or not _continuous(self._last_point, point, time_s - self._last_s)):
      self._identity = None  # Never reacquire a different car mid-maneuver.
      return None
    # Revalidate the active direction's lane geometry every frame. A target
    # latched before the maneuver must not survive loss of lane perception.
    if _side_of_point(model, point, expected_side=direction) != direction:
      self._identity = None
      return None
    self._last_point, self._last_s = point, time_s
    # Do not request acceleration to catch an adjacent car; preserve normal MPC.
    if (v_ego < 5.0 or point.v_rel > 0.0 or point.v_lead < 2.0
        or not max(8.0, v_ego * 0.5) < point.d_rel < min(80.0, v_ego * 2.5)):
      return None
    return point


def side_lead_may_replace(candidate: dict[str, Any], existing: dict[str, Any] | None) -> bool:
  """Never replace an existing leadTwo unless the side lead dominates in range and speed."""
  if existing is None or not existing.get("status"):
    return True
  return (float(candidate["dRel"]) + 0.5 < float(existing["dRel"])
          and float(candidate["vLead"]) <= float(existing["vLead"]))
