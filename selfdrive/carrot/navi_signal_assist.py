"""Mode-3 navigation reference and its own stopped-car veto.

This consumes the direct custom navigation observation, not the HUD's selected
countdown or its colour priority. A green reference never grants acceleration
permission and never releases another signal detector's stop.
"""
from dataclasses import dataclass
import math
import time


SOURCE = 'naver_traffic_signal_live_data'
MAX_SOURCE_AGE = 1.5
# carrotNavi publishes at 2 Hz; the vehicle-context checks remain separate.
MAX_SERVICE_AGE = 1.0
MAX_ROAD_AGE = 10.
RED_CONFIRM = .2
GREEN_CONFIRM = .7


def _get(obj, name, default=None):
  if isinstance(obj, dict):
    return obj.get(name, default)
  return getattr(obj, name, default)


def _uint(value):
  return value if type(value) is int and 0 <= value < 2**64 else None


def _meta(meta):
  if _get(meta, 'present') is not True:
    return None
  values = tuple(_uint(_get(meta, name)) for name in
                 ('sequence', 'sourceTimestampMillis', 'receivedMonoTimeNanos'))
  return values if all(value is not None for value in values) and values[1] > 0 and values[2] > 0 else None


@dataclass(frozen=True)
class NaviSignalDecision:
  available: bool = False
  color: str = 'unknown'
  confirmed: bool = False
  hold: bool = False
  warn: bool = False
  released: bool = False
  reason: str = 'disabled'
  sequence: int = -1
  session: str = ''


class NaviSignalAssist:
  def __init__(self):
    self.mode = None
    self.mode_since = math.inf
    self.last_now = -math.inf
    self.cooldown = -math.inf
    self.reset()

  def reset(self):
    self.hold = False
    self.warned = False
    self.direction = None
    self.key = None
    self.release_key = None
    self.sequence = -1
    self.received = -math.inf
    self.source_stamp = -1
    self.last_distinct_receipt = -math.inf
    self.color = 'unknown'
    self.pending = 'unknown'
    self.pending_since = -math.inf
    self.count = 0
    self.confirmed = False

  def _clear_votes(self, *, clear_release=False):
    self.pending = 'unknown'
    self.pending_since = -math.inf
    self.count = 0
    self.confirmed = False
    if clear_release:
      self.release_key = None

  def _unknown(self, reason):
    self._clear_votes(clear_release=True)
    return NaviSignalDecision(hold=self.hold, reason=reason)

  @staticmethod
  def _observation(navi, now, direction='STRAIGHT'):
    """Require provenance shared with the direct traffic_signal item.

    signalHud alone can contain legacy/global or nested-route observations.
    trafficSignal.visible is intentionally false in the current custom APK.
    """
    if (_uint(_get(navi, 'schemaVersion')) != 1 or _get(navi, 'connected') is not True
        or not isinstance(_get(navi, 'sessionId'), str) or not _get(navi, 'sessionId')):
      return None, 'disconnected'
    direct = _get(navi, 'trafficSignal')
    hud = _get(navi, 'signalHud')
    direct_meta = _meta(_get(direct, 'meta'))
    hud_meta = _meta(_get(hud, 'meta'))
    if (_get(direct, 'source') != SOURCE or _get(direct, 'visible') is not False
        or direct_meta is None or direct_meta != hud_meta):
      return None, 'not_direct_source'
    receipt = direct_meta[2] / 1e9
    if not 0 <= now - receipt <= MAX_SOURCE_AGE:
      return None, 'source_stale'
    status = _get(navi, 'navigationStatus')
    status_meta = _meta(_get(status, 'meta'))
    if (status_meta is None or not 0 <= now - status_meta[2] / 1e9 <= MAX_SOURCE_AGE
        or _get(status, 'mode') != 'guidance' or _get(status, 'guidanceActive') is not True
        or _get(status, 'offRoute') is not False):
      return None, 'not_live_guidance'
    vehicle = _get(navi, 'vehicle')
    if _get(_get(vehicle, 'meta'), 'present') is True and _get(vehicle, 'virtualGps') is not False:
      return None, 'virtual_gps'
    lane = _get(navi, 'laneCurrent')
    lane_meta = _get(lane, 'meta')
    lane_receipt = _uint(_get(lane_meta, 'receivedMonoTimeNanos'))
    if (_get(lane_meta, 'present') is True and lane_receipt is not None and lane_receipt > 0
        and 0 <= now - lane_receipt / 1e9 <= MAX_ROAD_AGE
        and _get(lane, 'roadCategoryValid') is True and _get(lane, 'roadCategory') in (0, 1)):
      return None, 'motorway'
    candidates = _get(hud, 'candidates')
    try:
      if candidates is None or not 1 <= len(candidates) <= 4:
        return None, 'no_candidates'
      if direction not in ('STRAIGHT', 'LEFT'):
        return None, 'unsupported_movement'
      by_direction = {}
      group = None
      directions = set()
      for candidate in candidates:
        candidate_direction, state = _get(candidate, 'direction'), _get(candidate, 'state')
        index, candidate_group = _uint(_get(candidate, 'signalIndex')), _uint(_get(candidate, 'groupIndex'))
        if (candidate_direction not in ('STRAIGHT', 'LEFT', 'RIGHT', 'U_TURN')
            or state not in ('RED', 'YELLOW', 'GREEN', 'RED_BLINK', 'YELLOW_BLINK', 'GREEN_BLINK')
            or index is None or index > 11 or candidate_group is None or candidate_group > 7):
          return None, 'malformed_candidates'
        if candidate_direction in directions:
          return None, 'duplicate_direction'
        directions.add(candidate_direction)
        if group is not None and group != candidate_group:
          return None, 'mixed_groups'
        group = candidate_group
        by_direction[candidate_direction] = (state, index)
      selected = by_direction.get(direction)
      reason = 'navigation_reference'
      if direction == 'LEFT':
        if selected is not None and selected[0] not in ('RED', 'YELLOW', 'GREEN'):
          # Blinking is not an explicit protected-left permission.
          selected = ('RED', selected[1])
          reason = 'left_turn_not_allowed'
        elif selected is None:
          straight = by_direction.get('STRAIGHT')
          if straight is None or straight[0] not in ('RED', 'YELLOW', 'GREEN'):
            return None, 'no_left_signal'
          # This is an internal permission veto, never a fabricated wire lamp.
          selected = ('RED', -1)
          reason = 'left_turn_not_allowed'
      if selected is None:
        return None, 'no_straight_signal'
      if selected[0] not in ('RED', 'YELLOW', 'GREEN'):
        return None, 'blinking_straight'
    except (AttributeError, TypeError, ValueError, OverflowError):
      return None, 'malformed_candidates'
    state, _ = selected
    # signalIndex is an array position, not a physical signal identity. Other
    # indications may appear before the same movement without changing it.
    key = (str(_get(navi, 'sessionId')), group, direction)
    return (key, state.lower(), *direct_meta, receipt), reason

  def update(self, now, navi, *, mode, valid, received_s, enabled, drive, speed,
             gas, brake, turning, departure, direction='STRAIGHT'):
    """Return an observation and optional veto; never a green permission."""
    finite_now = isinstance(now, (int, float)) and math.isfinite(now)
    if mode != self.mode:
      self.reset()
      self.mode, self.mode_since = mode, now if finite_now else math.inf
      self.cooldown = -math.inf
    if finite_now and not math.isfinite(self.mode_since):
      self.mode_since = now
    if mode != 3 or not enabled or not drive:
      self.reset()
      self.last_now = now if finite_now else -math.inf
      return NaviSignalDecision()
    if not finite_now or now < self.last_now:
      self.last_now = now if finite_now else -math.inf
      return self._unknown('clock_discontinuity')
    self.last_now = now
    if direction not in ('STRAIGHT', 'LEFT'):
      return self._unknown('unsupported_movement')
    if direction != self.direction:
      # Physical intent retires only this helper's previous movement veto.
      # It creates neither a release event nor a green acceleration permission.
      self.reset()
      self.direction = direction
    if gas:
      self.reset()
      self.cooldown = now + 10.
      return NaviSignalDecision(reason='driver_gas')
    if not isinstance(speed, (int, float)) or not math.isfinite(speed):
      return self._unknown('invalid_speed')
    if abs(speed) > 5 / 3.6:
      # Moving references remain usable for bounded preparation, but a stop
      # from a previous stop/creep phase cannot reappear after departure.
      self.hold = self.warned = False
      self.release_key = None
    if now < self.cooldown:
      return NaviSignalDecision(reason='driver_cooldown')
    if (not valid or not isinstance(received_s, (int, float)) or not math.isfinite(received_s)
        or not 0 <= now - received_s <= MAX_SERVICE_AGE):
      return self._unknown('invalid_context')
    if turning:
      return self._unknown('turning')
    try:
      observation, reason = self._observation(navi, now, direction)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      return self._unknown('malformed')
    if observation is None:
      return self._unknown(reason)
    key, color, sequence, stamp, receipt_ns, receipt = observation
    if receipt < self.mode_since:
      return self._unknown('pre_mode_observation')
    if key != self.key:
      self._clear_votes(clear_release=True)
      self.key, self.sequence, self.source_stamp = key, -1, -1
      self.received = self.last_distinct_receipt = -math.inf
    if sequence < self.sequence or receipt < self.received or stamp < self.source_stamp:
      # Reacquire from this lower baseline on a subsequent genuine source
      # update, but do not inherit release rights across a clock/sequence jump.
      self.sequence, self.source_stamp, self.received = sequence, stamp, receipt
      self.last_distinct_receipt = -math.inf
      return self._unknown('reordered_source')
    repeated = sequence == self.sequence or stamp == self.source_stamp
    if repeated:
      if (color != self.color or (sequence == self.sequence and (receipt != self.received or stamp != self.source_stamp))
          or not 0 <= now - self.last_distinct_receipt <= MAX_SOURCE_AGE):
        return self._unknown('repeated_or_stale_source')
    else:
      gap = receipt - self.last_distinct_receipt
      if color != self.pending or gap > MAX_SOURCE_AGE:
        self._clear_votes(clear_release=gap > MAX_SOURCE_AGE)
        self.pending, self.pending_since = color, receipt
      self.count += 1
      self.last_distinct_receipt = receipt
      self.color = color
      duration = GREEN_CONFIRM if color == 'green' else RED_CONFIRM
      self.confirmed = self.count >= 2 and receipt - self.pending_since >= duration - 1e-9
    self.sequence, self.source_stamp, self.received = sequence, stamp, receipt
    released = False
    if self.confirmed and color in ('red', 'yellow') and abs(speed) <= .3:
      self.hold = True
      self.release_key = key
    elif self.confirmed and color == 'green' and self.hold and self.release_key == key:
      self.hold = self.warned = False
      self.release_key = None
      released = True
    warn = self.hold and departure and not brake and not self.warned
    self.warned |= warn
    return NaviSignalDecision(True, color, self.confirmed, self.hold, warn, released,
                              reason, sequence, key[0])


class NaviSignalRuntime:
  """An optional subscription, outside plannerd's required service checks."""
  def __init__(self, sm_factory=None):
    self.sm_factory = sm_factory
    self.sm = None

  def read(self, mode, now=None):
    if mode != 3:
      self.sm = None
      return None, False, 0., time.monotonic() if now is None else now
    try:
      if self.sm is None:
        if self.sm_factory is None:
          from cereal import messaging
          self.sm = messaging.SubMaster(['carrotNavi'])
        else:
          self.sm = self.sm_factory()
      self.sm.update(0)
      current = time.monotonic() if now is None else now
      valid = bool(self.sm.seen['carrotNavi'] and self.sm.valid['carrotNavi'] and self.sm.alive['carrotNavi'])
      return self.sm['carrotNavi'], valid, self.sm.recv_time['carrotNavi'], current
    except (AttributeError, ImportError, KeyError, OSError, RuntimeError, TypeError, ValueError):
      return None, False, 0., time.monotonic() if now is None else now
