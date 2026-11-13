"""Parked EV6 experiment: short SPAS request, explicit ECU recovery, ADRV hold.

This is a test-only boot, not a lane-change controller. A CAN lamp indication
does not establish physical actuation or parking-assist health on the vehicle.
"""
from dataclasses import dataclass

from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum


FRESH = 0.1
BODY_FRESH = 0.3  # the stalk/lamp messages have an unverified checksum
SPAS = (0x165, 0x16A)
CRC_LENGTHS = {0x35: 32, 0xA0: 24, 0x175: 24, 0x165: 24, 0x16A: 32, 0x1EA: 32}


def with_checksum(addr, data):
  result = bytearray(data)
  result[:2] = hkg_can_fd_checksum(addr, None, result).to_bytes(2, "little")
  return bytes(result)


@dataclass
class Observation:
  data: bytes
  time: float
  counter: int
  count: int


class BlinkerHandoffTest:
  def __init__(self):
    self.state = "waiting"
    self.status = "waiting: P, brake, parking brake, neutral stalk and fresh CAN required"
    self.rx = {}
    self.ack = None
    self.entered = 0.
    self.started = None
    self.safe_since = None
    self.first_update = None
    self.side = 0
    self.snapshot = {}
    self.counters = {}
    self.last_spas = -1.
    self.last_hold = -1.
    self.last_lamp = None
    self.pulse_started = None
    self.failed = False
    self.stop_reason = ""
    self.restore_enable_ok = False
    self.restore_default_ok = False
    self.recovery_started = None
    self.recovery_attempt = 0
    self.pending_clear = False
    self.clear_sent = False

  def observe(self, can_packets, now):
    # Only physical RX buses. Panda TX echoes (src >= 128) cannot acknowledge
    # requests or satisfy the resumed-stock checks.
    for _, messages in can_packets:
      for addr, dat, bus in messages:
        dat = bytes(dat)
        if addr == 0x7B9 and bus == 0 and len(dat) == 8:
          size = dat[0]
          if 1 <= size <= 7:
            self.ack = (dat[1:1 + size], now)
          continue
        if (bus == 0 and addr in (*CRC_LENGTHS, 0x3C1, 0x413) and addr != 0x1EA) or (bus == 2 and addr == 0x1EA):
          key = (addr, bus)
          if addr in CRC_LENGTHS:
            if len(dat) != CRC_LENGTHS[addr] or with_checksum(addr, dat) != dat:
              self.rx.pop(key, None)
              continue
            counter, modulus = dat[2], 256
          else:
            if len(dat) != 8:
              self.rx.pop(key, None)
              continue
            counter, modulus = dat[1] >> 4, 16
          old = self.rx.get(key)
          count = old.count + 1 if old and counter == (old.counter + 1) % modulus and 0 <= now - old.time <= BODY_FRESH else 1
          self.rx[key] = Observation(dat, now, counter, count)

  def _fresh(self, addr, now, bus=0, max_age=FRESH):
    value = self.rx.get((addr, bus))
    return value is not None and value.count >= 2 and 0 <= now - value.time <= max_age

  def _parked(self, now):
    if not all(self._fresh(addr, now) for addr in (0x35, 0xA0, 0x175)):
      return False
    if not all(self._fresh(addr, now, max_age=BODY_FRESH) for addr in (0x3C1, 0x413)):
      return False
    acc, wheels, tcs, stalk = (self.rx[(a, 0)].data for a in (0x35, 0xA0, 0x175, 0x3C1))
    return (acc[24] & 7 == 0 and acc[5] == 0 and wheels[8:16] == bytes(8)
            and bool(tcs[10] & 2) and (tcs[10] >> 6) & 3 == 1
            and not (stalk[3] & 0x40 or stalk[4] & 0x51))

  def _lamps(self):
    data = self.rx[(0x413, 0)].data
    # Match CarState's USE_ALT_LAMP selection, without its debounce latch.
    return (bool(data[7] & 8), bool(data[7] & 32)) if data[7] & 64 else (bool(data[2] & 16), bool(data[2] & 64))

  def _stock_ready(self, now):
    return (all(self._fresh(addr, now) for addr in SPAS)
            and self.rx[(0x16A, 0)].data[16] & 0xE0 == 0
            and self._fresh(0x1EA, now, 2)
            and self.rx[(0x1EA, 2)].data[3] & 0x60 == 0)

  def _transition(self, state, now, status=None):
    self.state, self.entered = state, now
    self.ack = None
    self.status = status or state

  def _diag(self, payload):
    return (0x7B1, bytes([len(payload)]) + payload + bytes(7 - len(payload)), 0)

  def _acknowledged(self, prefix):
    return self.ack is not None and self.ack[1] >= self.entered and self.ack[0].startswith(prefix)

  def _recover(self, now, reason, failed=False):
    self.failed |= failed
    self.stop_reason = reason
    self.pending_clear |= failed or reason == "stopped"
    self.recovery_started = now
    self.recovery_attempt += 1
    self.restore_enable_ok = self.restore_default_ok = False
    self._transition("restore_enable", now, "restoring parking ECU: " + reason)
    return [self._diag(b"\x28\x00\x01")]

  def _hold(self, now, direction):
    # Leave transport/scheduling margin above Panda's 50ms minimum. A zero
    # request can bypass this interval to release immediately.
    if (direction and now - self.last_hold < 0.06 - 1e-6) or not self._fresh(0x1EA, now, 2):
      return []
    data = bytearray(self.rx[(0x1EA, 2)].data)
    data[3] = (data[3] & ~0x60) | (0x20 if direction == 1 else 0x40 if direction == 2 else 0)
    self.last_hold = now
    return [(0x1EA, with_checksum(0x1EA, data), 0)]

  def _spas(self, now, direction):
    # Panda independently enforces >=20ms. Nominal 30ms gives USB/CAN transport
    # jitter some room instead of advancing a counter on a rejected frame.
    if now - self.last_spas < 0.03 - 1e-6:
      return []
    self.last_spas = now
    result = []
    for addr in SPAS:
      data = bytearray(self.snapshot[addr])
      self.counters[addr] = (self.counters[addr] + 1) % 256
      data[2] = self.counters[addr]
      if addr == 0x16A:
        data[16] = (data[16] & ~0xE0) | ((3 if direction == 1 else 4 if direction == 2 else 0) << 5)
      result.append((addr, with_checksum(addr, data), 0))
    return result

  def update(self, requested, now, enabled=False, can_valid=True):
    if self.first_update is None:
      self.first_update = now
    requested = requested if requested in (1, 2) else 0
    parked = self._parked(now) and can_valid and not enabled
    out = []
    active = self.state not in ("waiting", "done", "failed")
    # Cleanup is allowed even if movement, stale input or the physical lever
    # interrupts the test. Do not restart an experiment within this test boot.
    if active and (requested != self.side or not parked):
      self.pending_clear = True
      if self.state not in ("restore_enable", "restore_default", "resume"):
        out += self._recover(now, "stopped" if requested == 0 else "conditions lost", failed=requested != 0)
    if active and parked and self._lamps()[2 - self.side]:
      if self.state in ("restore_enable", "restore_default", "resume"):
        self.failed = self.pending_clear = True
        self.stop_reason = "opposite lamp/hazards detected"
      else:
        out += self._recover(now, "opposite lamp/hazards detected", failed=True)
    if self.state == "waiting":
      ready = requested and parked and self._stock_ready(now) and not any(self._lamps())
      self.safe_since = (now if self.safe_since is None else self.safe_since) if ready else None
      if self.safe_since is not None and now - self.safe_since >= 0.3:
        self.side, self.started = requested, now
        self.snapshot = {addr: self.rx[(addr, 0)].data for addr in SPAS}
        self.counters = {addr: self.snapshot[addr][2] for addr in SPAS}
        self._transition("session", now, "requesting parking ECU extended session")
        out.append(self._diag(b"\x10\x03"))
      elif requested == 0 or now - self.first_update > 30.:
        self._transition("done", now, "not started; select 0 and restart before another test")
    elif self.state == "session":
      if self._acknowledged(b"\x50\x03"):
        self._transition("disable", now, "requesting parking ECU TX pause")
        out.append(self._diag(b"\x28\x01\x01"))
      elif now - self.entered > 0.5:
        out += self._recover(now, "no extended-session acknowledgement", failed=True)
    elif self.state == "disable":
      if self._acknowledged(b"\x68\x01"):
        self._transition("silence", now, "waiting for both stock SPAS streams to stop")
      elif now - self.entered > 0.5:
        out += self._recover(now, "no communication-control acknowledgement", failed=True)
    elif self.state == "silence":
      if any((addr, 0) not in self.rx for addr in SPAS):
        out += self._recover(now, "invalid stock SPAS observation", failed=True)
      elif all(now - self.rx[(addr, 0)].time >= 0.1 for addr in SPAS) and now - self.entered >= 0.1:
        # The ECU continued publishing during the diagnostic exchange. Preserve
        # its final body/counter, not the earlier preflight snapshot.
        self.snapshot = {addr: self.rx[(addr, 0)].data for addr in SPAS}
        self.counters = {addr: self.snapshot[addr][2] for addr in SPAS}
        self.pulse_started = now
        self._transition("pulse", now, "SPAS initial lamp request")
        out += self._spas(now, self.side)
        out += self._hold(now, self.side)
      elif now - self.entered > 0.5:
        out += self._recover(now, "stock SPAS did not stop", failed=True)
    elif self.state == "pulse":
      lamps = self._lamps() if parked else (False, False)
      if any((addr, 0) not in self.rx for addr in SPAS):
        out += self._recover(now, "invalid stock SPAS observation", failed=True)
      elif any(self.rx[(addr, 0)].time > self.pulse_started for addr in SPAS):
        out += self._recover(now, "stock SPAS resumed during pulse", failed=True)
      elif lamps[self.side - 1] and not lamps[2 - self.side]:
        # Stop all host SPAS as soon as CAN lamp feedback arrives. Hold overlaps
        # recovery; success still requires positive recovery ACKs and stock RX.
        self.last_lamp = now
        out += self._recover(now, "lamp feedback received")
      elif now - self.entered >= 0.6:
        out += self._recover(now, "no initial lamp feedback", failed=True)
      else:
        out += self._spas(now, self.side)
    elif self.state == "restore_enable":
      if self._acknowledged(b"\x68\x00") or now - self.entered > 0.5:
        self.restore_enable_ok = self._acknowledged(b"\x68\x00")
        self._transition("restore_default", now, "returning parking ECU to default session")
        out.append(self._diag(b"\x10\x01"))
    elif self.state == "restore_default":
      if self._acknowledged(b"\x50\x01") or now - self.entered > 0.5:
        self.restore_default_ok = self._acknowledged(b"\x50\x01")
        self._transition("resume", now, "verifying restored stock SPAS")
    elif self.state == "resume":
      resumed = (all(self._fresh(addr, now) and self.rx[(addr, 0)].time > self.recovery_started for addr in SPAS)
                 and self.rx[(0x16A, 0)].data[16] & 0xE0 == 0)
      if resumed and self.restore_enable_ok and self.restore_default_ok:
        if self.failed or self.pending_clear:
          self._transition("failed" if self.failed else "done", now, self.stop_reason + "; parking ECU RX restored")
          self.pending_clear = True
        else:
          self.last_lamp = now
          self._transition("holding", now, "holding; parking ECU RX restored (physical lamps/parking health unverified)")
      elif now - self.entered > 0.6:
        if self.recovery_attempt < 3:
          out += self._recover(now, "parking ECU recovery unconfirmed", failed=True)
        else:
          self.pending_clear = True
          self._transition("failed", now, "parking ECU recovery UNCONFIRMED; stop test and inspect vehicle")
    elif self.state == "holding":
      lamps = self._lamps()
      if lamps[self.side - 1] and not lamps[2 - self.side]:
        self.last_lamp = now
      if not self._stock_ready(now) or lamps[2 - self.side] or now - self.last_lamp > 1.5:
        out += self._recover(now, "hold feedback/stock RX lost", failed=True)

    if self.started is not None and now - self.started > 4. and self.state in ("session", "disable", "silence", "pulse"):
      out += self._recover(now, "initial request exceeded time budget", failed=True)
    holding = self.state in ("pulse", "restore_enable", "restore_default", "resume", "holding")
    if self.started is not None and now - self.started > 4. and self.state != "holding":
      self.pending_clear = True
    if holding and not self.pending_clear and not self.failed and parked and self.pulse_started is not None:
      out += self._hold(now, self.side)
    if self.pending_clear and not self.clear_sent:
      clear = self._hold(now, 0)
      out += clear
      self.clear_sent = bool(clear)
    return out
