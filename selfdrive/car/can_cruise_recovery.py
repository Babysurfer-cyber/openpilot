"""One bounded cruise request after a transient CAN-only disengagement."""
import math


class CanErrorCruiseRecovery:
  STABLE_TIME_S = 0.2
  MAX_WAIT_S = 3.0
  INPUT_INHIBIT_S = 0.5

  def __init__(self):
    self.previous_enabled = False
    self.last_speed_kph = None
    self.pending_since = None
    self.healthy_since = None
    self.target_speed_kph = None
    self.used = False
    self.manual_enable_until = -math.inf
    self.input_inhibit_until = -math.inf
    self.last_now = None
    self.status = "idle"

  def _clear(self, status):
    self.pending_since = self.healthy_since = None
    self.target_speed_kph = None
    self.status = status

  def update(self, *, now_s, enabled, set_speed_kph, can_fault, blocked, ready,
             allowed, driver_input, manual_enable=False):
    was_enabled = self.previous_enabled
    self.previous_enabled = enabled
    if not math.isfinite(now_s) or (self.last_now is not None and now_s < self.last_now):
      self._clear("clock invalid")
      self.used = True
      self.manual_enable_until = -math.inf
      return None
    self.last_now = now_s

    if driver_input and (not manual_enable or blocked or not allowed):
      self.manual_enable_until = -math.inf
    if manual_enable and not enabled and allowed and not blocked:
      self.manual_enable_until = now_s + 1.0
    if enabled and not was_enabled:
      self.last_speed_kph = None
      if now_s <= self.manual_enable_until and allowed and not blocked:
        self.used = False
      self.manual_enable_until = -math.inf

    if driver_input:
      self.last_speed_kph = None
      self.input_inhibit_until = now_s + self.INPUT_INHIBIT_S
      self._clear("driver input")
      return None
    if not allowed or blocked:
      self.last_speed_kph = None
      self.manual_enable_until = -math.inf
      self._clear("disabled" if not allowed else "blocked")
      return None

    if enabled:
      if not can_fault:
        if not math.isfinite(set_speed_kph) or not 0 < set_speed_kph < 200:
          self.last_speed_kph = None
        elif ready:
          self.last_speed_kph = float(set_speed_kph)
      self._clear("engaged")
      return None

    if (was_enabled and can_fault and not self.used and self.last_speed_kph is not None
        and now_s >= self.input_inhibit_until):
      self.pending_since = now_s
      self.target_speed_kph = self.last_speed_kph
      self.healthy_since = None
      self.status = "waiting"

    if self.pending_since is None:
      return None
    if now_s - self.pending_since > self.MAX_WAIT_S:
      self._clear("expired")
      self.used = True
      return None
    if can_fault or not ready:
      self.healthy_since = None
      return None
    if self.healthy_since is None:
      self.healthy_since = now_s
    if now_s - self.healthy_since < self.STABLE_TIME_S:
      return None

    target = self.target_speed_kph
    self._clear("requested")
    self.used = True
    return target
