"""Run the Python handoff against the actual Panda C safety hooks.

The simulated ECU supplies CAN and diagnostic acknowledgements. These tests
prove software contract compatibility, not physical EV6 lamp/parking behavior.
"""
import ctypes

import pytest

from opendbc.car.hyundai.tests.test_blinker_test import HandoffRig, nonzero_commands
from opendbc.car.hyundai.blinker_test import with_checksum
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class CheckedHandoffRig(HandoffRig):
  def __init__(self, direction=1, hda2=False):
    super().__init__(direction)
    self.safety = libsafety_py.libsafety
    self.param = 1024 | 1 | 8 | (16 if hda2 else 0)
    assert self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.param) == 0
    self.safety.init_tests()
    self.checked = []
    self._raw_observe = self.test.observe
    self._raw_update = self.test.update
    self.test.observe = self.observe_checked
    self.test.update = self.update_checked

  def observe_checked(self, packets, now):
    self.safety.set_timer(round(now * 1e6) & 0xFFFFFFFF)
    for _, messages in packets:
      for addr, dat, bus in messages:
        if bus < 128:
          assert self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat)), (hex(addr), bus)
    self._raw_observe(packets, now)

  def update_checked(self, requested, now, **kwargs):
    self.safety.set_timer(round(now * 1e6) & 0xFFFFFFFF)
    result = self._raw_update(requested, now, **kwargs)
    for addr, dat, bus in result:
      assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)), (
        self.test.state, self.test.status, hex(addr), dat.hex(), now)
      self.checked.append((addr, dat, bus))
    assert not self.safety.get_controls_allowed()
    return result


class DriverOrderHandoffRig(CheckedHandoffRig):
  """FDCAN order: forward a copy, then RX the untouched physical input."""
  def __init__(self, direction=1):
    super().__init__(direction)
    self.library = ctypes.CDLL(libsafety_py.libsafety_fn)
    self.forward = self.library.safety_fwd_hook
    self.forward.argtypes = [ctypes.c_void_p]
    self.forward.restype = ctypes.c_int
    self.buffered = ctypes.c_bool.in_dll(self.library, "safety_tx_buffered_for_fwd")
    self.forwarded_adrv = []
    self.wire_tx = []
    self.host_buffered = []
    self.last_source_adrv = -1.
    self.source_adrv_counter = 0
    self.jitter = False
    self.last_delivery_delayed = False

  def observe_checked(self, packets, now):
    self.safety.set_timer(round(now * 1e6) & 0xFFFFFFFF)
    physical_packets = []
    for stamp, messages in packets:
      physical_messages = []
      for addr, dat, bus in messages:
        if addr == 0x1EA and bus == 2:
          # Simulate a genuine 20Hz camera source while card runs at 100Hz.
          if now - self.last_source_adrv < 0.05 - 1e-6:
            continue
          self.last_source_adrv = now
          self.source_adrv_counter = (self.source_adrv_counter + 1) % 256
          body = bytearray(dat)
          body[2] = self.source_adrv_counter
          dat = with_checksum(addr, body)
        original = libsafety_py.make_CANPacket(addr, bus, dat)
        forwarded = libsafety_py.make_CANPacket(addr, bus, dat)
        destination = self.forward(int(libsafety_py.ffi.cast("uintptr_t", forwarded)))
        assert destination == (2 if bus == 0 else 0)
        # can_send(skip_tx_hook=true) resets the buffer-command flag and emits
        # only the forwarded copy; its FDCAN RX source remains unchanged.
        self.buffered.value = False
        self.safety.can_set_checksum(forwarded)
        emitted = bytes(forwarded[0].data[0:len(dat)])
        self.wire_tx.append((addr, emitted, destination))
        if addr == 0x1EA:
          self.forwarded_adrv.append((bytes(dat), emitted))
        assert bytes(original[0].data[0:len(dat)]) == dat
        assert self.safety.safety_rx_hook(original), (hex(addr), bus)
        physical_messages.append((addr, bytes(dat), bus))
      physical_packets.append((stamp, physical_messages))
    self._raw_observe(physical_packets, now)

  def update_checked(self, requested, now, **kwargs):
    result = self._raw_update(requested, now, **kwargs)
    delay = 0.
    if self.jitter and result:
      self.last_delivery_delayed = not self.last_delivery_delayed
      delay = 0.008 if self.last_delivery_delayed else 0.
    self.safety.set_timer(round((now + delay) * 1e6) & 0xFFFFFFFF)
    for addr, dat, bus in result:
      self.buffered.value = False
      assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)), (
        self.test.state, hex(addr), dat.hex(), now, delay)
      self.checked.append((addr, dat, bus))
      if self.buffered.value:
        assert addr == 0x1EA
        self.host_buffered.append((addr, dat, bus))
        self.buffered.value = False
      else:
        self.wire_tx.append((addr, dat, bus))
        assert addr != 0x1EA  # host snapshots are never duplicate wire frames
    assert not self.safety.get_controls_allowed()
    return result


@pytest.mark.parametrize("side", [1, 2])
@pytest.mark.parametrize("hda2", [False, True])
def test_real_python_output_is_accepted_by_real_safety_for_handoff_hold_and_stop(side, hda2):
  rig = CheckedHandoffRig(side, hda2)
  rig.reach_hold()
  held = []
  for i in range(500):
    lit = (i // 20) % 2 == 0
    rig.lamps = (lit and side == 1, lit and side == 2)
    held += rig.tick()
  assert rig.test.state == "holding"
  assert held and all(addr == 0x1EA for addr, _, _ in held)
  rig.request = 0
  assert not nonzero_commands(rig.tick())
  rig.recover()
  assert rig.test.state == "done"
  assert len(rig.checked) > 100


@pytest.mark.parametrize("change", [
  {"ACCELERATOR": {"GEAR": 5}},
  {"WHEEL_SPEEDS": {"WHEEL_SPEED_1": 0.03125}},
  {"TCS": {"DriverBraking": 0}},
  {"TCS": {"ESC_PrkBrkActvSta": 0}},
  {"BLINKER_STALKS": {"RIGHT_BLINKER": 1}},
])
def test_guard_abort_cleanup_remains_accepted_by_firmware(change):
  rig = CheckedHandoffRig()
  rig.reach_hold()
  last_hold = next(msg for msg in reversed(rig.checked) if msg[0] == 0x1EA and msg[1][3] & 0x60)
  rig.changes = change
  out = rig.tick(dt=0.06)
  assert not nonzero_commands(out)
  assert rig.test.state == "restore_enable"
  addr, dat, bus = last_hold
  assert not rig.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
  rig.recover()
  assert rig.test.state == "failed"


def test_no_session_ack_never_sends_lamp_commands_and_recovery_is_allowed():
  rig = CheckedHandoffRig()
  rig.reach_session()
  for _ in range(30):
    rig.tick()
  assert rig.test.state == "restore_enable"
  assert not nonzero_commands(rig.checked)
  rig.recover()
  assert rig.test.state == "failed"


def test_no_initial_feedback_releases_lamps_and_recovers_with_real_safety():
  rig = CheckedHandoffRig()
  rig.reach_pulse()
  for _ in range(35):
    rig.tick()
    if rig.test.state == "restore_enable":
      break
  assert rig.test.state == "restore_enable"
  rig.recover()
  assert rig.test.state == "failed"


@pytest.mark.parametrize("rig_class", [HandoffRig, CheckedHandoffRig])
@pytest.mark.parametrize("failed", [False, True])
def test_terminal_states_stop_all_host_tx_after_a_single_clear(rig_class, failed):
  rig = rig_class()
  rig.reach_hold()
  first = len(rig.sent)
  if failed:
    rig.lamps = (True, True)
  else:
    rig.request = 0
  rig.tick()
  rig.recover()
  assert rig.test.state == ("failed" if failed else "done")
  clears = [dat for addr, dat, _ in rig.sent[first:] if addr == 0x1EA and not dat[3] & 0x60]
  assert len(clears) == 1
  for _ in range(100):
    assert not rig.tick()


@pytest.mark.parametrize("side,mask", [(1, 0x20), (2, 0x40)])
def test_fdcan_forward_before_rx_keeps_latest_counter_body_and_original_host_input(side, mask):
  rig = DriverOrderHandoffRig(side)
  rig.reach_hold()
  first = len(rig.forwarded_adrv)
  for i in range(250):
    rig.changes["ADRV_0x1ea"] = {"HDA_MODE2": i % 4, "NEW_SIGNAL_1": (i // 4) % 2}
    rig.tick(dt=0.01)
  assert rig.test.state == "holding"
  assert len(rig.forwarded_adrv) - first >= 45
  for original, forwarded in rig.forwarded_adrv[first:]:
    assert original[3] & 0x60 == 0
    assert forwarded[3] & 0x60 == mask
    assert original[2] == forwarded[2]
    assert with_checksum(0x1EA, forwarded) == forwarded
    baseline, actual = bytearray(original), bytearray(forwarded)
    baseline[:2] = actual[:2] = bytes(2)
    baseline[3] &= ~0x60
    actual[3] &= ~0x60
    assert baseline == actual
  assert rig.host_buffered
  # Original physical RX is what card observes, not the modified forwarded copy.
  assert rig.test.rx[(0x1EA, 2)].data[3] & 0x60 == 0
  rig.request = 0
  rig.tick(dt=0.01)
  rig.recover()
  first = len(rig.forwarded_adrv)
  for _ in range(15):
    assert not rig.tick(dt=0.01)
  assert rig.forwarded_adrv[first:]
  assert all(original == forwarded for original, forwarded in rig.forwarded_adrv[first:])


def test_short_spas_pulse_tolerates_eight_millisecond_delivery_jitter():
  rig = DriverOrderHandoffRig()
  rig.reach_pulse()
  rig.jitter = True
  first = len(rig.checked)
  for _ in range(35):
    rig.tick(dt=0.01)
  assert rig.test.state == "pulse"
  # Delivery delay alternates 8ms/0ms. Host nominal 30ms leaves a 22ms
  # shortest arrival gap above firmware's independent 20ms minimum.
  assert sum(addr == 0x16A for addr, _, _ in rig.checked[first:]) >= 10
  rig.lamps = (True, False)
  rig.tick(dt=0.01)
  rig.recover()
  assert rig.test.state == "holding"
