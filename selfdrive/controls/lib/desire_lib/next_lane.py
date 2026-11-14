"""Physical stalk events and lane-line crossing observations for queued changes."""
import math
from numbers import Integral, Real

from openpilot.common.realtime import DT_MDL
from .constants import BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT


class StalkPressTracker:
  def __init__(self):
    self.previous = None
    self.invalidated = False

  def reset(self):
    self.previous = None
    self.invalidated = False

  def update(self, carstate):
    self.invalidated = False
    values = (getattr(carstate, "leftBlinkerStalkCount", None),
              getattr(carstate, "rightBlinkerStalkCount", None))
    if not all(isinstance(v, Integral) and not isinstance(v, bool) and 0 <= v <= 255 for v in values):
      self.previous = None
      self.invalidated = True
      return BLINKER_NONE

    current = tuple(int(v) for v in values)
    previous, self.previous = self.previous, current
    if not getattr(carstate, "canValid", True):
      self.invalidated = True
      return BLINKER_NONE
    if previous is None:
      return BLINKER_NONE

    delta = tuple((new - old) % 256 for new, old in zip(current, previous, strict=True))
    if delta == (1, 0):
      return BLINKER_LEFT
    if delta == (0, 1):
      return BLINKER_RIGHT
    self.invalidated = delta != (0, 0)
    return BLINKER_NONE


class LaneCrossing:
  CROSS_NEAR_M = 1.0
  CROSS_JUMP_M = 1.5
  MIN_LANE_WIDTH_M = 2.0
  MAX_LANE_WIDTH_M = 6.0

  def __init__(self):
    self.reset()

  def reset(self):
    self.since = -1.
    self.closest = None

  @classmethod
  def _inner_lines(cls, model):
    try:
      lines = model.laneLines
      if len(lines) < 3:
        return None
      samples = (lines[1].y, lines[2].y)
      if not all(len(y) > 0 for y in samples):
        return None
      values = tuple(y[0] for y in samples)
      if not all(isinstance(y, Real) and not isinstance(y, bool) and math.isfinite(y) for y in values):
        return None
      left, right = (float(y) for y in values)
      width = right - left
      if (not cls.MIN_LANE_WIDTH_M <= width <= cls.MAX_LANE_WIDTH_M
          or max(abs(left), abs(right)) > cls.MAX_LANE_WIDTH_M):
        return None
      return left, right
    except (AttributeError, IndexError, TypeError, ValueError):
      return None

  def update(self, model, left):
    if self.since >= 0:
      self.since += DT_MDL
      return self.since

    inner = self._inner_lines(model)
    if inner is None:
      self.closest = None
      return self.since

    distance = abs(inner[0 if left else 1])
    self.closest = distance if self.closest is None else min(self.closest, distance)
    # Model relabeling can span several frames, so compare to the closest
    # approach, not only the immediately previous target-side line.
    if self.closest < self.CROSS_NEAR_M and distance - self.closest > self.CROSS_JUMP_M:
      self.since = 0.
    return self.since
