"""EV6 SPAS2 indicator controller *planning core* (no vehicle transmit).

Adapted conceptually from ajouatom/openpilot joongyu01/carrot-wip-dev:
SPAS stream validation, acknowledged APRK session, source silence,
and explicit restoration. Unlike the donor this module intentionally has
NO CAN/UDS transmitter, no packet builder and no host/panda integration.
Events named WOULD_* are planning telemetry, NEVER CAN messages.
"""
from dataclasses import dataclass

SPAS_LENGTHS = {0x165: 24, 0x16A: 32}
APRK_RESPONSE = 0x7B9
STOCK_MAX_AGE_S = .100
POST_SESSION_MAX_AGE_S = .050
ACK_TIMEOUT_S = .400
QUIET_S = .100
RESTORE_TIMEOUT_S = 1.000
PLAN_MAX_DURATION_S = .350  # planning duration, NOT a validated safe TX time


def crc16_xmodem(data):
  crc = 0
  for byte in data:
    crc ^= byte << 8
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
  return crc


def check_stock_frame(addr, data):
  if addr not in SPAS_LENGTHS or len(data) != SPAS_LENGTHS[addr]:
    return False
  xor = {24: 0x819D, 32: 0x9F5B}[len(data)]
  expected = crc16_xmodem(data[2:] + addr.to_bytes(2, 'little')) ^ xor
  return int.from_bytes(data[:2], 'little') == expected


@dataclass(frozen=True)
class SafetyInputs:
  park: bool
  brake: bool
  standstill: bool
  controls_inactive: bool
  driver_lever_idle: bool
  bus_healthy: bool
  independent_bus_monitor: bool = False

  def safe(self):
    return (self.park and self.brake and self.standstill and self.controls_inactive and
            self.driver_lever_idle and self.bus_healthy and self.independent_bus_monitor)


@dataclass
class StockStream:
  last_time: float = -1.
  counter: int = -1
  consecutive: int = 0
  after_ack: bool = False


class EV6SpasControlPlan:
  """Event-only state machine. No CanData, packets, CAN access or TX outputs.

  All inputs must come from a trusted *independent isolated-bench* capture.
  Callers must NOT treat simulated eligibility as authority to send on a car.
  """

  def __init__(self, bus=0):
    self.bus = bus
    self.phase = 'idle'
    self.reason = ''
    self.stock = {addr: StockStream() for addr in SPAS_LENGTHS}
    self.request_id = None
    self.side = None
    self.last_request_id = None
    self.request_at = -1.
    self.changed_at = -1.
    self.last_time = -1.
    self.session_ack_at = -1.
    self.disable_ack_at = -1.
    self.simulated_active_at = -1.
    self.restore_enable_ack = False
    self.restore_default_ack = False
    self.response = None
    self.events = []

  def _enter(self, phase, now):
    self.phase, self.changed_at, self.response = phase, now, None

  def _record_time(self, now):
    if not isinstance(now, (int, float)) or not 0 <= now < float('inf'):
      self.fault('invalid_time')
      return False
    if self.last_time >= 0 and now < self.last_time:
      self.fault('nonmonotonic_time')
      return False
    self.last_time = now
    return True

  def _emit(self, name):
    self.events.append(name)

  def _drain(self):
    events, self.events = self.events, []
    return events

  def stock_ready(self, now):
    return all(s.consecutive >= 10 and 0 <= now-s.last_time <= STOCK_MAX_AGE_S
               for s in self.stock.values())

  def _post_session_stock_ready(self, now):
    return self.session_ack_at >= 0 and all(
      s.after_ack and s.consecutive >= 2 and
      self.session_ack_at < s.last_time <= now and
      0 <= now - s.last_time <= POST_SESSION_MAX_AGE_S
      for s in self.stock.values())

  def observe_stock(self, addr, bus, data, now):
    if not self._record_time(now):
      return
    if addr not in SPAS_LENGTHS or bus != self.bus:
      return
    st = self.stock[addr]
    if not isinstance(data, bytes) or not check_stock_frame(addr, data):
      st.consecutive, st.after_ack = 0, False
      if self.phase not in ('idle', 'locked') and not self.phase.startswith('restore_'):
        self._restore(now, 'invalid_stock_frame')
      return
    counter = data[2]
    advances = st.last_time >= 0 and 0 < now-st.last_time < .100 and counter == (st.counter + 1) % 256
    st.consecutive = min(st.consecutive + 1, 20) if advances else 1
    st.counter, st.last_time = counter, now
    if self.session_ack_at >= 0 and now > self.session_ack_at and advances:
      st.after_ack = True
    if self.phase == 'eligible':
      self._restore(now, 'original_ecu_resumed')
    elif self.phase == 'quiet' and now-self.disable_ack_at >= QUIET_S:
      self._restore(now, 'stock_still_transmitting')

  def observe_reply(self, addr, bus, data, now):
    if not self._record_time(now):
      return
    if addr != APRK_RESPONSE or bus != self.bus or self.phase in ('idle', 'locked'):
      return
    if not isinstance(data, bytes) or len(data) != 8 or not 2 <= data[0] <= 7:
      self._restore(now, 'invalid_ecu_reply')
      return
    body = data[1:1 + data[0]]
    if body.startswith(b'\x7f'):
      self._restore(now, 'negative_ecu_reply')
      return
    self.response = (now, body)

  def _ack(self, prefix):
    return (self.response is not None and
            self.changed_at <= self.response[0] <= self.changed_at + ACK_TIMEOUT_S and
            self.response[1].startswith(prefix))

  def request(self, request_id, side, now, inputs):
    if not self._record_time(now):
      return self._drain()
    if self.phase != 'idle':
      self._emit('REQUEST_DENIED_NOT_IDLE')
    elif not isinstance(request_id, str) or not 0 < len(request_id) <= 100 or request_id == self.last_request_id:
      self._emit('REQUEST_DENIED_NONCE')
    elif side not in ('left', 'right') or not isinstance(inputs, SafetyInputs) or not inputs.safe():
      self._emit('REQUEST_DENIED_SAFETY')
    elif not self.stock_ready(now):
      self._emit('REQUEST_DENIED_STOCK_NOT_READY')
    else:
      self.last_request_id = self.request_id = request_id
      self.side, self.request_at, self.reason = side, now, ''
      self.session_ack_at = self.disable_ack_at = self.simulated_active_at = -1.
      self.restore_enable_ack = self.restore_default_ack = False
      for stream in self.stock.values():
        stream.after_ack = False
      self._enter('session', now)
      self._emit('WOULD_REQUEST_EXTENDED_SESSION')
    return self._drain()

  def _restore(self, now, reason):
    if self.phase in ('idle', 'locked') or self.phase.startswith('restore_'):
      return
    self.reason = reason
    self._enter('restore_enable', now)
    self._emit('WOULD_REQUEST_COMMUNICATION_ENABLE')

  def fault(self, reason='independent_bus_fault'):
    now = self.last_time if self.last_time >= 0 else 0.
    self._restore(now, reason)
    if self.phase == 'idle':
      self.phase, self.reason = 'locked', reason

  def cancel(self, now):
    if self._record_time(now):
      self._restore(now, 'cancelled')
    return self._drain()

  def tick(self, now, inputs):
    if not self._record_time(now):
      return self._drain()
    if self.phase == 'idle' or self.phase == 'locked':
      return self._drain()
    if not isinstance(inputs, SafetyInputs) or not inputs.safe():
      self._restore(now, 'unsafe_inputs')
    if self.phase == 'session':
      if self._ack(b'\x50\x03'):
        self.session_ack_at = now
        self._enter('post_session_stock', now)
      elif now - self.changed_at >= ACK_TIMEOUT_S:
        self._restore(now, 'extended_session_timeout')
    elif self.phase == 'post_session_stock':
      if self._post_session_stock_ready(now):
        self._enter('disable', now)
        self._emit('WOULD_REQUEST_COMMUNICATION_DISABLE')
      elif now - self.changed_at >= .200:
        self._restore(now, 'no_post_session_stock_proof')
    elif self.phase == 'disable':
      if self._ack(b'\x68\x03'):
        self.disable_ack_at = now
        self._enter('quiet', now)
      elif now - self.changed_at >= ACK_TIMEOUT_S:
        self._restore(now, 'disable_timeout')
    elif self.phase == 'quiet':
      if now-self.changed_at >= QUIET_S:
        if all(now-s.last_time >= QUIET_S for s in self.stock.values()):
          self.simulated_active_at = now
          self._enter('eligible', now)
          self._emit('SIMULATED_HANDOFF_ONLY_NO_CAN_TX')
        else:
          self._restore(now, 'stock_not_quiet')
    elif self.phase == 'eligible':
      if now-self.simulated_active_at >= PLAN_MAX_DURATION_S:
        self._restore(now, 'plan_duration_expired')
    elif self.phase == 'restore_enable':
      if self._ack(b'\x68\x00'):
        self.restore_enable_ack = True
        self._enter('restore_default', now)
        self._emit('WOULD_REQUEST_DEFAULT_SESSION')
      elif now-self.changed_at >= ACK_TIMEOUT_S:
        self._enter('restore_default', now)
        self._emit('WOULD_REQUEST_DEFAULT_SESSION')
    elif self.phase == 'restore_default':
      if self._ack(b'\x50\x01'):
        self.restore_default_ack = True
        self._enter('restore_stock', now)
      elif now-self.changed_at >= ACK_TIMEOUT_S:
        self._enter('restore_stock', now)
    elif self.phase == 'restore_stock':
      if self.restore_enable_ack and self.restore_default_ack and self.stock_ready(now):
        self._enter('idle', now)
        self.request_id = self.side = None
        self._emit('SIMULATED_RESTORE_CONFIRMED_NO_TX')
      elif now-self.changed_at >= RESTORE_TIMEOUT_S:
        self.reason = 'restore_unconfirmed'
        self._enter('locked', now)
        self._emit('RESTORE_UNCONFIRMED_LOCKOUT')
    return self._drain()
