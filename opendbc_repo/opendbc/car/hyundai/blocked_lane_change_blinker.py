"""Give one blocked physical tap two extra flashes, using raw lamp phases."""

import math

from opendbc.car.common.conversions import Conversions as CV


STOCK_BLINKS = 3
TOTAL_BLINKS = 5
MAX_SESSION_TIME_S = 6.0
LAMP_PROGRESS_TIMEOUT_S = 1.2
LANE_CHANGE_SPEED_MIN = 25 * CV.KPH_TO_MS


def _finite_number(value):
  if type(value) not in (int, float):
    return False
  try:
    return math.isfinite(value)
  except OverflowError:
    return False


def _counter(carstate, name):
  value = getattr(carstate, name, None)
  return value if type(value) is int and 0 <= value < 256 else None


def _enum_number(value, names):
  raw = getattr(value, "raw", value)
  if type(raw) is int:
    return raw if raw in names.values() else None
  return names.get(raw) if isinstance(raw, str) else None


class BlockedLaneChangeBlinker:
  """Bound a pending one-touch request without generating a new request.

  The caller supplies fresh, raw turn-lamp bits rather than the filtered carState
  blinkers. A held or fully latched lever never arms this extension. Once spent
  or cancelled, a session needs a new physical stalk counter increment to reset.
  """

  def __init__(self):
    self._last_counts = None
    self._last_update_s = None
    self._direction = 0
    self._consumed = True
    self._armed = False
    self._budget_spent = False
    self._max_lever_level = 0
    self._seen_pre = False
    self._start_s = 0.0
    self._last_progress_s = 0.0
    self._lamp_on = False
    self._blink_count = 0
    self._fall_count = 0

  def _consume(self):
    self._consumed = True
    self._armed = False

  def _start(self, direction, now_s, lever, lamp):
    self._direction = direction
    self._consumed = False
    self._armed = False
    self._budget_spent = False
    self._max_lever_level = lever
    self._seen_pre = False
    self._start_s = self._last_progress_s = now_s
    self._lamp_on = lamp
    # The CAN stalk edge can arrive while the first raw ON bit is already set.
    self._blink_count = int(lamp)
    self._fall_count = 0

  def update(self, now_s, carstate, *, model_meta, model_valid, left_lamp,
             right_lamp, lamp_valid, lateral_active):
    counts = (_counter(carstate, "leftBlinkerStalkCount"),
              _counter(carstate, "rightBlinkerStalkCount"))
    if None in counts:
      self._last_counts = None
      self._consume()
      return 0

    previous_counts = self._last_counts
    self._last_counts = counts
    # First observations, including nonzero counts on startup, are a baseline.
    if previous_counts is None:
      self._last_update_s = now_s if _finite_number(now_s) else None
      self._consume()
      return 0

    deltas = tuple((current - previous) % 256 for current, previous in zip(counts, previous_counts))
    new_direction = 1 if deltas == (1, 0) else 2 if deltas == (0, 1) else 0
    if deltas != (0, 0):
      self._consume()
      if not new_direction:
        return 0

    lever = getattr(carstate, "blinkerLever", None)
    speed = getattr(carstate, "vEgo", None)
    state = _enum_number(getattr(model_meta, "laneChangeState", None),
                         {"off": 0, "preLaneChange": 1, "laneChangeStarting": 2, "laneChangeFinishing": 3})
    direction = _enum_number(getattr(model_meta, "laneChangeDirection", None),
                             {"none": 0, "left": 1, "right": 2})
    blocked = getattr(model_meta, "laneChangeBlocked", None)
    valid = (
      _finite_number(now_s)
      and type(lever) is int and lever in (0, 1, 2)
      and _finite_number(speed) and speed >= LANE_CHANGE_SPEED_MIN
      and getattr(carstate, "canValid", False) and getattr(carstate, "gearShifter", None) == "drive"
      and model_valid and lamp_valid and lateral_active
      and state is not None and direction is not None
      and type(blocked) is int and blocked in (0, 1, 2)
      and not (left_lamp and right_lamp)
    )
    if not valid:
      self._consume()
      return 0

    if self._last_update_s is not None and now_s < self._last_update_s:
      self._last_update_s = now_s
      self._consume()
      return 0
    gap = 0.0 if self._last_update_s is None else now_s - self._last_update_s
    self._last_update_s = now_s
    if gap > LAMP_PROGRESS_TIMEOUT_S:
      # A changed counter may belong to a tap made during the missing interval.
      # Its already-completed flashes cannot be counted, so consume it too.
      self._consume()
      return 0

    if new_direction:
      self._start(new_direction, now_s, lever, bool(left_lamp if new_direction == 1 else right_lamp))
    if self._consumed:
      return 0

    self._max_lever_level = max(self._max_lever_level, lever)
    opposite_lamp = right_lamp if self._direction == 1 else left_lamp
    if (self._max_lever_level == 2 or opposite_lamp
        or now_s - self._start_s >= MAX_SESSION_TIME_S
        or state in (2, 3)):
      self._consume()
      return 0

    # Allow the model a frame to observe a new stalk edge. Once it has entered
    # this direction's pending state, returning OFF is a cancellation, not retry.
    if state == 1 and direction == self._direction:
      self._seen_pre = True
    elif self._seen_pre or (state == 1 and direction not in (0, self._direction)):
      self._consume()
      return 0

    lamp = bool(left_lamp if self._direction == 1 else right_lamp)
    if lamp != self._lamp_on:
      self._last_progress_s = now_s
      self._lamp_on = lamp
      if lamp:
        self._blink_count = min(TOTAL_BLINKS, self._blink_count + 1)
      else:
        self._fall_count += 1
    elif now_s - self._last_progress_s >= LAMP_PROGRESS_TIMEOUT_S:
      self._consume()
      return 0

    if self._fall_count >= TOTAL_BLINKS:
      self._consume()
      return 0

    if self._armed:
      # Keep the pending request alive while BSD/the line restriction clears;
      # the normal lane-change hold takes over as soon as STARTING is observed.
      if state == 1 and direction == self._direction and lever == 0:
        return self._direction
      self._consume()
      return 0

    if self._fall_count >= STOCK_BLINKS:
      # Never restart a stopped OEM sequence with a late blocked observation.
      self._consume()
      return 0

    if (not self._budget_spent and self._fall_count > 0
        and self._max_lever_level == 1 and lever == 0
        and state == 1 and direction == self._direction and blocked in (1, 2)):
      self._armed = self._budget_spent = True
      return self._direction
    return 0
