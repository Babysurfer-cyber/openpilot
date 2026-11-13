"""Coordinate a parked blinker experiment without changing shared CarParams."""
from dataclasses import dataclass

from opendbc.car.hyundai.blinker_test import BlinkerHandoffTest
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags


REQUEST = "HyundaiBlinkerTestSafetyRequest"
ACK = "HyundaiBlinkerTestSafetyAck"
STATUS = "HyundaiBlinkerTestStatus"


@dataclass(frozen=True)
class PandaObservation:
  configs: tuple
  received: float
  valid: bool = True
  controls_allowed: bool = False


@dataclass
class RuntimeOutput:
  blocked: bool
  messages: list
  reset_controller: bool = False


def supports_blinker_runtime(cp, ecan):
  configs = cp.safetyConfigs
  return bool(cp.brand == "hyundai" and cp.carFingerprint == CAR.KIA_EV6 and cp.flags & HyundaiFlags.CANFD
          and cp.flags & HyundaiFlags.CAMERA_SCC and ecan == 0 and not cp.passive and not cp.dashcamOnly
          and not cp.flags & HyundaiFlags.BLINKER_TEST
          and len(configs) == 1 and str(configs[0].safetyModel) == "hyundaiCanfd"
          and not configs[0].safetyParam & HyundaiSafetyFlags.BLINKER_TEST
          and configs[0].safetyParam & HyundaiSafetyFlags.EV_GAS
          and configs[0].safetyParam & HyundaiSafetyFlags.CAMERA_SCC)


class LiveBlinkerTest:
  def __init__(self, cp, params):
    self.params = params
    self.normal_configs = tuple((str(c.safetyModel), c.safetyParam) for c in cp.safetyConfigs)
    self.test_configs = tuple((model, param | HyundaiSafetyFlags.BLINKER_TEST.value) for model, param in self.normal_configs)
    self.test = BlinkerHandoffTest()
    self.phase = "normal"
    self.selection = 0
    self.selection_since = None
    self.safe_since = None
    self.session = None
    self.request = None
    self.requested_at = None
    self.pause_requested = False
    self.session_requested = False
    self.status = None
    self.blocked_at = None
    self.completed_at = None

  @property
  def blocking(self):
    return self.phase != "normal"

  def observe(self, packets, now):
    if self.selection or self.blocking:
      self.test.observe(packets, now)

  def _status(self, value):
    if value != self.status:
      self.status = value
      self.params.put_nonblocking(STATUS, value)

  @staticmethod
  def _text(value):
    return value.decode() if isinstance(value, bytes) else value

  def _request_mode(self, mode, now):
    self.request = f"{mode}:{self.session}"
    self.requested_at = now
    self.params.put(REQUEST, self.request)
    self.safe_since = None

  def _panda_matches(self, observation, configs, now, after=None):
    return (observation is not None and observation.valid and observation.configs == configs
            and not observation.controls_allowed and 0 <= now - observation.received <= 0.3
            and (after is None or observation.received > after))

  def _acknowledged(self, observation, configs, now):
    return (self._text(self.params.get(ACK)) == self.request
            and self._panda_matches(observation, configs, now, self.requested_at))

  def _safe_to_restore(self, now):
    if not self.test._stock_ready(now):
      return False
    if not self.session_requested:
      return True  # No diagnostic session or ECU pause was ever requested.
    terminal = self.test.state in ("done", "failed")
    if not terminal:
      return False
    if not self.pause_requested:
      # The session attempt never reached communication-control. Fresh neutral
      # stock RX proves traffic continued; do not call this a confirmed recovery.
      return True
    return (self.test.restore_enable_ok and self.test.restore_default_ok
            and all(self.test.rx[(addr, 0)].time > self.test.recovery_started for addr in (0x165, 0x16A)))

  def update(self, selection, now, observation, *, enabled=False, can_valid=True):
    selection = selection if selection in (1, 2) else 0
    if selection != self.selection:
      self.selection, self.selection_since = selection, now
    parked = self.test._parked(now) and can_valid and not enabled
    out = []
    if self.phase == "normal":
      if not selection:
        return RuntimeOutput(False, [])
      ready = (parked and self.test._stock_ready(now) and not any(self.test._lamps())
               and self._panda_matches(observation, self.normal_configs, now))
      self.safe_since = (now if self.safe_since is None else self.safe_since) if ready else None
      if self.safe_since is None or now - self.safe_since < 0.3 or now - self.selection_since < 0.3:
        self._status("waiting: parked P, brake, parking brake, controls off and fresh normal Panda required")
        return RuntimeOutput(False, [])  # Moving selection never interrupts driving.
      self.session = str(round(now * 1e9))
      self.blocked_at = now
      self.phase = "entering"
      self._request_mode("test", now)
      self._status("switching Panda to parked test mode; driving output blocked")
      return RuntimeOutput(True, [])

    if self.phase == "entering":
      if self._acknowledged(observation, self.test_configs, now):
        self.phase = "running"
        self.test.first_update = None
        self._status("Panda test mode confirmed; checking parked inputs")
      elif now - self.requested_at > 3.:
        self._status("Panda test-mode confirmation missing; driving output remains blocked; select 0 to recover")
      if selection == 0 and parked and self._safe_to_restore(now):
        self.phase = "restoring"
        self._request_mode("normal", now)
      return RuntimeOutput(True, [])

    if self.phase == "running":
      panda_ready = self._panda_matches(observation, self.test_configs, now)
      # Underlying ECU cleanup remains live when CAN/control/health disappears.
      effective = selection if panda_ready else 0
      out = self.test.update(effective, now, enabled=enabled, can_valid=can_valid and panda_ready)
      for addr, dat, _ in out:
        if addr == 0x7B1 and dat[1:3] == b"\x10\x03":
          self.session_requested = True
        if addr == 0x7B1 and dat[1:4] == b"\x28\x01\x01":
          self.pause_requested = True
      self._status(self.test.status + "; select 0 before changing direction or retrying")
      if selection == 0 and parked and self._safe_to_restore(now):
        self.phase = "restoring"
        self._request_mode("normal", now)
        self._status("parking ECU traffic verified; restoring normal Panda mode")
      return RuntimeOutput(True, out)

    if self.phase == "restoring":
      # Selection changes cannot rearm while a transition is in flight. Exact
      # token + fresh hardware readback, parked data and a neutral settle period
      # are required before a normal controller can transmit again.
      ready = (selection == 0 and parked and self._safe_to_restore(now)
               and self._acknowledged(observation, self.normal_configs, now))
      self.safe_since = (now if self.safe_since is None else self.safe_since) if ready else None
      if self.safe_since is not None and now - self.safe_since >= 0.3:
        self.phase = "normal"
        self.test = BlinkerHandoffTest()
        self.pause_requested = self.session_requested = False
        self.safe_since = None
        self._status("stopped; normal Panda mode and fresh parked CAN confirmed")
        # This cycle stays neutral; the following cycle can use normal control.
        return RuntimeOutput(True, [], reset_controller=True)
      if now - self.requested_at > 3.:
        self._status("normal Panda/parked CAN confirmation missing; driving output remains blocked")
      return RuntimeOutput(True, [])
    raise RuntimeError(f"Unknown blinker runtime phase: {self.phase}")
