"""CAN-level simulations; these cannot validate EV6 lamp or parking health."""
import pytest

from opendbc.can import CANPacker
from opendbc.car.hyundai.blinker_test import BlinkerHandoffTest, SPAS, with_checksum


class HandoffRig:
  def __init__(self, direction=1):
    self.test = BlinkerHandoffTest()
    self.packer = CANPacker("hyundai_canfd_generated")
    self.time = 10.
    self.request = direction
    self.count = {}
    self.sent = []
    self.stock = True
    self.lamps = (False, False)
    self.changes = {}

  def message(self, name, values=None, bus=0):
    msg = self.packer.dbc.name_to_msg[name]
    counter = (self.count.get(msg.address, -1) + 1) % 256
    self.count[msg.address] = counter
    fields = {"COUNTER": counter, "COUNTER_ALT": counter % 16}
    fields.update(values or {})
    return self.packer.make_can_msg(name, bus, fields)

  def tick(self, dt=0.02, ack=None, enabled=False, can_valid=True, omit=(), invalid_spas=False):
    self.time += dt
    fields = {
      "ACCELERATOR": {"GEAR": 0, "ACCELERATOR_PEDAL": 0},
      "WHEEL_SPEEDS": {},
      "TCS": {"DriverBraking": 1, "ESC_PrkBrkActvSta": 1},
      "BLINKER_STALKS": {},
      "BLINKERS": {"LEFT_LAMP": self.lamps[0], "RIGHT_LAMP": self.lamps[1]},
      "ADRV_0x1ea": {"HDA_MODE2": 3, "NEW_SIGNAL_1": 1},
    }
    for name, values in self.changes.items():
      fields[name].update(values)
    messages = [self.message(name, values, 2 if name == "ADRV_0x1ea" else 0)
                for name, values in fields.items() if name not in omit]
    if self.stock:
      messages += [self.message("SPAS1", {"BYTE3": 0xA5}),
                   self.message("SPAS2", {"BYTE3": 0x4B, "BYTE20": 0x67})]
    if invalid_spas:
      messages.append((0x165, b"\xff" * 24, 0))
    if ack is not None:
      messages.append((0x7B9, bytes([len(ack)]) + ack + bytes(7 - len(ack)), 0))
    self.test.observe([(int(self.time * 1e9), messages)], self.time)
    out = self.test.update(self.request, self.time, enabled=enabled, can_valid=can_valid)
    self.sent.extend(out)
    return out

  def reach_session(self):
    for _ in range(40):
      self.tick()
      if self.test.state == "session":
        return
    raise AssertionError(self.test.status)

  def reach_pulse(self):
    self.reach_session()
    out = self.tick(ack=b"\x50\x03\x00\x32\x01\xf4")
    assert any(dat[1:4] == b"\x28\x01\x01" for addr, dat, _ in out if addr == 0x7B1)
    assert self.test.state == "disable"
    self.stock = False
    self.tick(ack=b"\x68\x01")
    for _ in range(10):
      out = self.tick()
      if self.test.state == "pulse":
        return out
    raise AssertionError(self.test.status)

  def recover(self):
    assert self.test.state == "restore_enable"
    self.stock = True
    self.tick(ack=b"\x68\x00")
    assert self.test.state == "restore_default"
    self.tick(ack=b"\x50\x01")
    assert self.test.state == "resume"
    self.tick()

  def reach_hold(self):
    self.reach_pulse()
    self.lamps = (self.request == 1, self.request == 2)
    out = self.tick()
    assert self.test.state == "restore_enable"
    assert not any(addr in SPAS for addr, _, _ in out)
    self.recover()
    assert self.test.state == "holding"


def nonzero_commands(messages):
  return [(addr, dat) for addr, dat, _ in messages
          if (addr == 0x16A and dat[16] & 0xE0) or (addr == 0x1EA and dat[3] & 0x60)]


@pytest.mark.parametrize("side,spas_value,hold_mask", [(1, 3, 0x20), (2, 4, 0x40)])
def test_short_initial_request_then_restored_ecu_and_indefinite_hold(side, spas_value, hold_mask):
  rig = HandoffRig(side)
  out = rig.reach_pulse()
  assert {addr for addr, _, _ in out} == {0x165, 0x16A, 0x1EA}
  for addr, dat, bus in out:
    assert bus == 0
    assert with_checksum(addr, dat) == dat
    if addr == 0x16A:
      assert (dat[16] >> 5) & 7 == spas_value
  rig.lamps = (side == 1, side == 2)
  out = rig.tick()
  assert rig.test.state == "restore_enable"
  assert not any(addr in SPAS for addr, _, _ in out)
  rig.recover()
  assert rig.test.state == "holding"
  # Model alternating light feedback while the parked ECU is publishing normally.
  held = []
  for i in range(500):
    lit = (i // 20) % 2 == 0
    rig.lamps = (lit and side == 1, lit and side == 2)
    held.extend(rig.tick())
  assert rig.test.state == "holding"
  assert held and all(addr == 0x1EA for addr, _, _ in held)
  assert all(dat[3] & 0x60 == hold_mask for _, dat, _ in held)
  rig.request = 0
  stopped = rig.tick()
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(stopped)
  rig.recover()
  assert rig.test.state == "done"
  rig.request = 3 - side
  for _ in range(30):
    assert not nonzero_commands(rig.tick())
  assert rig.test.state == "done"  # one experiment per test boot


def test_no_session_ack_never_disables_ecu_or_requests_lamps():
  rig = HandoffRig()
  rig.reach_session()
  for _ in range(30):
    rig.tick()
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(rig.sent)
  assert not any(dat[1:4] == b"\x28\x01\x01" for addr, dat, _ in rig.sent if addr == 0x7B1)
  assert any(dat[1:4] == b"\x28\x00\x01" for addr, dat, _ in rig.sent if addr == 0x7B1)


def test_no_disable_ack_never_requests_lamps():
  rig = HandoffRig()
  rig.reach_session()
  rig.tick(ack=b"\x50\x03")
  for _ in range(30):
    rig.tick()
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(rig.sent)


def test_stock_spas_must_go_silent_before_host_spas():
  rig = HandoffRig()
  rig.reach_session()
  rig.tick(ack=b"\x50\x03")
  rig.tick(ack=b"\x68\x01")
  for _ in range(30):
    rig.tick()
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(rig.sent)


def test_tx_echoes_cannot_acknowledge_or_supply_stock_observations():
  test = BlinkerHandoffTest()
  rig = HandoffRig()
  rig.reach_session()
  test.observe([(0, [(addr, dat, bus + 128) for addr, dat, bus in rig.sent])], rig.time)
  assert not test.rx and test.ack is None
  rig.test.observe([(0, [(0x7B9, b"\x02\x50\x03\x00\x00\x00\x00\x00", 128)])], rig.time)
  rig.tick()
  assert rig.test.state == "session"


@pytest.mark.parametrize("changes", [
  {"ACCELERATOR": {"GEAR": 5}},
  {"ACCELERATOR": {"ACCELERATOR_PEDAL": 1}},
  {"WHEEL_SPEEDS": {"WHEEL_SPEED_1": 0.03125}},
  {"TCS": {"DriverBraking": 0}},
  {"TCS": {"ESC_PrkBrkActvSta": 0}},
  {"BLINKER_STALKS": {"LEFT_BLINKER": 1}},
  {"BLINKER_STALKS": {"RIGHT_BLINKER": 1}},
  {"BLINKER_STALKS": {"LEFT_BLINKER_TAP": 1}},
  {"BLINKER_STALKS": {"RIGHT_BLINKER_TAP": 1}},
])
def test_guard_loss_ends_hold_and_still_attempts_explicit_recovery(changes):
  rig = HandoffRig()
  rig.reach_hold()
  rig.changes = changes
  out = rig.tick(dt=0.06)
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(out)
  assert any(addr == 0x7B1 and dat[1:4] == b"\x28\x00\x01" for addr, dat, _ in out)


@pytest.mark.parametrize("enabled,can_valid", [(True, True), (False, False)])
def test_control_or_can_invalid_aborts(enabled, can_valid):
  rig = HandoffRig()
  rig.reach_hold()
  assert not nonzero_commands(rig.tick(enabled=enabled, can_valid=can_valid))
  assert rig.test.state == "restore_enable"


def test_stale_inputs_abort_without_extending_hold():
  rig = HandoffRig()
  rig.reach_hold()
  out = rig.tick(dt=0.12, omit=("WHEEL_SPEEDS",))
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(out)


def test_hazards_during_hold_abort():
  rig = HandoffRig()
  rig.reach_hold()
  rig.lamps = (True, True)
  out = rig.tick(dt=0.06)
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(out)


def test_hold_feedback_cannot_succeed_without_positive_recovery_acks():
  rig = HandoffRig()
  rig.reach_pulse()
  rig.lamps = (True, False)
  rig.tick()
  rig.stock = True
  for _ in range(300):
    rig.tick()
    assert rig.test.state != "holding"
  assert rig.test.state == "failed"
  assert "UNCONFIRMED" in rig.test.status


def test_payloads_preserve_stock_body_and_have_valid_checksums():
  rig = HandoffRig()
  out = rig.reach_pulse()
  for addr, dat, _ in out:
    assert with_checksum(addr, dat) == dat
    if addr in SPAS:
      baseline = bytearray(rig.test.snapshot[addr])
      actual = bytearray(dat)
      baseline[:3] = actual[:3] = bytes(3)
      if addr == 0x16A:
        baseline[16] &= ~0xE0
        actual[16] &= ~0xE0
      assert actual == baseline
      assert dat[2] == (rig.test.snapshot[addr][2] + 1) % 256
    else:
      baseline = bytearray(rig.test.rx[(0x1EA, 2)].data)
      actual = bytearray(dat)
      baseline[:2] = actual[:2] = bytes(2)
      baseline[3] &= ~0x60
      actual[3] &= ~0x60
      assert actual == baseline


def test_bad_crc_does_not_arm_test():
  rig = HandoffRig()
  for _ in range(30):
    rig.tick(invalid_spas=True)
  assert rig.test.state == "waiting"
  assert not rig.sent


def test_corrupted_spas_during_pulse_recovers_instead_of_crashing():
  rig = HandoffRig()
  rig.reach_pulse()
  out = rig.tick(invalid_spas=True)
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(out)


def test_hazards_during_pulse_abort_immediately():
  rig = HandoffRig()
  rig.reach_pulse()
  rig.lamps = (True, True)
  out = rig.tick()
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(out)


@pytest.mark.parametrize("changes", [
  {"ACCELERATOR": {"GEAR": 5}},
  {"TCS": {"DriverBraking": 0}},
  {"TCS": {"ESC_PrkBrkActvSta": 0}},
  {"BLINKER_STALKS": {"LEFT_BLINKER": 1}},
  {"ADRV_0x1ea": {"RIGHT_BLINK_HOLD": 1}},
])
def test_unmet_initial_preflight_never_sends_diagnostics(changes):
  rig = HandoffRig()
  rig.changes = changes
  for _ in range(35):
    rig.tick()
  assert rig.test.state == "waiting"
  assert not rig.sent


def test_initial_lamps_already_active_do_not_arm():
  rig = HandoffRig()
  rig.lamps = (True, False)
  for _ in range(35):
    rig.tick()
  assert rig.test.state == "waiting"
  assert not rig.sent


def test_negative_diagnostic_reply_does_not_advance():
  rig = HandoffRig()
  rig.reach_session()
  rig.tick(ack=b"\x7f\x10\x22")
  assert rig.test.state == "session"
  for _ in range(30):
    rig.tick()
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(rig.sent)


def test_no_initial_lamp_feedback_ends_short_pulse():
  rig = HandoffRig()
  rig.reach_pulse()
  for _ in range(35):
    out = rig.tick()
    if rig.test.state == "restore_enable":
      assert not nonzero_commands(out)
      break
  assert rig.test.state == "restore_enable"
  assert rig.time - rig.test.pulse_started < 0.65
  assert rig.test.failed and rig.test.pending_clear


def test_restored_stock_stream_without_lamp_feedback_ends_hold():
  rig = HandoffRig()
  rig.reach_hold()
  rig.lamps = (False, False)
  for _ in range(85):
    out = rig.tick()
    if rig.test.state == "restore_enable":
      assert not nonzero_commands(out)
      break
  assert rig.test.state == "restore_enable"
  assert rig.test.failed


def test_returning_stock_spas_during_pulse_aborts_before_another_host_spas():
  rig = HandoffRig()
  rig.reach_pulse()
  rig.stock = True
  out = rig.tick()
  assert rig.test.state == "restore_enable"
  assert not any(addr in SPAS for addr, _, _ in out)
  assert not nonzero_commands(out)


def test_counter_replay_is_not_a_fresh_parked_input():
  rig = HandoffRig()
  rig.reach_hold()
  # Replaying a correct CRC frame with a non-advancing counter invalidates its
  # two-consecutive-sample prerequisite even though host receipt time is new.
  old = rig.test.rx[(0x35, 0)].data
  rig.time += 0.02
  rig.test.observe([(0, [(0x35, old, 0)])], rig.time)
  out = rig.test.update(1, rig.time)
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(out)


def test_alternate_lamp_selection_drives_the_same_handoff():
  rig = HandoffRig(2)
  rig.reach_pulse()
  rig.changes["BLINKERS"] = {"USE_ALT_LAMP": 1, "RIGHT_LAMP_ALT": 1}
  rig.tick()
  assert rig.test.state == "restore_enable"
  rig.recover()
  assert rig.test.state == "holding"


def test_zero_mode_never_sends_anything():
  rig = HandoffRig(0)
  for _ in range(50):
    assert not rig.tick()
  assert rig.test.state == "done"


def test_opposite_lamp_during_ecu_restore_clears_hold_immediately():
  rig = HandoffRig()
  rig.reach_pulse()
  rig.lamps = (True, False)
  rig.tick()
  assert rig.test.state == "restore_enable"
  rig.lamps = (True, True)
  out = rig.tick(dt=0.06)
  assert not nonzero_commands(out)
  assert rig.test.pending_clear


def test_pulse_counter_uses_last_stock_frame_before_silence_not_session_start():
  rig = HandoffRig()
  rig.reach_session()
  initial = {addr: rig.test.rx[(addr, 0)].counter for addr in SPAS}
  rig.tick(ack=b"\x50\x03")
  final_stock = {addr: rig.test.rx[(addr, 0)].data for addr in SPAS}
  assert all(final_stock[addr][2] != initial[addr] for addr in SPAS)
  rig.stock = False
  rig.tick(ack=b"\x68\x01")
  for _ in range(10):
    out = rig.tick()
    if rig.test.state == "pulse":
      break
  assert rig.test.state == "pulse"
  for addr, data, _ in out:
    if addr in SPAS:
      assert data[2] == (final_stock[addr][2] + 1) % 256
