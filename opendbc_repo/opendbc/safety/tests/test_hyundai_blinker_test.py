"""Parked experiment only: exercise the real Panda C hooks without a vehicle."""
import ctypes
import unittest

from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class TestHyundaiBlinkerTest(unittest.TestCase):
  PARAM = 1024 | 1 | 8  # BLINKER_TEST, EV gas, camera SCC; ECAN=0

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.PARAM)
    self.safety.init_tests()
    self.now = 10000
    self.counter = {}
    self.stock = {}
    self.sent_spas = {}
    # This branch changed the C forwarding API to CANPacket_t*. Its shared
    # Python wrapper still declares the old two-integer API; use the actual ABI.
    self.forward = ctypes.CDLL(libsafety_py.libsafety_fn).safety_fwd_hook
    self.forward.argtypes = [ctypes.c_void_p]
    self.forward.restype = ctypes.c_int
    self.refresh()
    self.advance(10000)
    self.refresh()

  def advance(self, us):
    self.now = (self.now + us) & 0xFFFFFFFF
    self.safety.set_timer(self.now)

  @staticmethod
  def checksum(addr, data):
    data = bytearray(data)
    data[:2] = hkg_can_fd_checksum(addr, None, data).to_bytes(2, 'little')
    return data

  @staticmethod
  def packet(addr, data, bus=0):
    return libsafety_py.make_CANPacket(addr, bus, data)

  def rx(self, addr, data, bus=0):
    return self.safety.safety_rx_hook(self.packet(addr, data, bus))

  def tx(self, addr, data, bus=0):
    ctypes.c_bool.in_dll(ctypes.CDLL(libsafety_py.libsafety_fn), 'safety_tx_buffered_for_fwd').value = False
    return self.safety.safety_tx_hook(self.packet(addr, data, bus))

  def forwarded_adrv(self, data=None, bus=2):
    if data is None:
      data = bytearray(self.stock[0x1EA])
      data[2] = (data[2] + 1) & 255
      data = self.checksum(0x1EA, data)
    packet = self.packet(0x1EA, data, bus)
    destination = self.forward(int(libsafety_py.ffi.cast('uintptr_t', packet)))
    return destination, bytes(packet[0].data[0:32])

  def fwd(self, addr, bus):
    packet = self.packet(addr, bytes(32), bus)
    return self.forward(int(libsafety_py.ffi.cast('uintptr_t', packet)))

  def refresh(self, stock=True, **changes):
    definitions = {0x35: 32, 0xA0: 24, 0x175: 24, 0x3C1: 8, 0x413: 8, 0x1EA: 32}
    if stock:
      definitions.update({0x165: 24, 0x16A: 32})
    for addr, length in definitions.items():
      self.counter[addr] = (self.counter.get(addr, -1) + 1) & (15 if length == 8 else 255)
      data = bytearray(length)
      if length == 8:
        data[1] = self.counter[addr] << 4
      else:
        data[2] = self.counter[addr]
      if addr == 0x35:
        data[24] = changes.get('gear', 0)
        data[5] = changes.get('gas', 0)
      if addr == 0xA0:
        data[8:10] = changes.get('speed', 0).to_bytes(2, 'little')
      if addr == 0x175:
        data[10] = (2 if changes.get('brake', True) else 0) | (64 if changes.get('parking_brake', True) else 0)
      if addr == 0x3C1:
        data[3] |= 64 if changes.get('stalk', False) else 0
      if addr == 0x413:
        data[2] |= 16 if changes.get('lamp', False) else 0
      if addr in (0x165, 0x16A, 0x1EA):
        data[4] = 0x53  # An opaque body byte must always survive the copy.
      if length != 8:
        data = self.checksum(addr, data)
      self.stock[addr] = data
      self.rx(addr, data, 2 if addr == 0x1EA else 0)

  def diag(self, service, sub):
    data = bytes([3, service, sub, 1, 0, 0, 0, 0]) if service == 0x28 else bytes([2, service, sub, 0, 0, 0, 0, 0])
    return self.tx(0x7B1, data)

  def ack(self, service, sub):
    self.rx(0x7B9, bytes([2, service, sub, 0, 0, 0, 0, 0]))

  def disabled(self):
    self.assertTrue(self.diag(0x10, 3))
    self.ack(0x50, 3)
    self.assertTrue(self.diag(0x28, 1))
    self.ack(0x68, 1)
    self.advance(100000)
    self.refresh(stock=False)

  def spas(self, side=3, addr=0x16A, mutate=False):
    data = bytearray(self.stock[addr])
    counter = self.sent_spas.get(addr, data[2])
    data[2] = (counter + 1) & 255
    if addr == 0x16A:
      data[16] = (data[16] & 0x1F) | (side << 5)
    if mutate:
      data[4] ^= 1
    result = self.tx(addr, self.checksum(addr, data))
    if result:
      self.sent_spas[addr] = data[2]
    return result

  def hold(self, side=3, mutate=False, counter_delta=0):
    data = bytearray(self.stock[0x1EA])
    data[3] = (data[3] & 0x9F) | {0: 0, 3: 0x20, 4: 0x40, 7: 0x60}[side]
    data[2] = (data[2] + counter_delta) & 255
    if mutate:
      data[4] ^= 1
    return self.tx(0x1EA, self.checksum(0x1EA, data))

  def holding(self, side=3):
    self.disabled()
    self.assertTrue(self.spas(side))
    self.assertTrue(self.hold(side))
    lamp = bytearray(self.stock[0x413])
    lamp[1] = (lamp[1] + 0x10) & 0xF0
    self.counter[0x413] = (self.counter[0x413] + 1) & 15
    lamp[2] |= 0x10 if side == 3 else 0x40
    self.rx(0x413, lamp)
    self.assertTrue(self.diag(0x28, 0))
    self.ack(0x68, 0)
    self.assertTrue(self.diag(0x10, 1))
    self.ack(0x50, 1)
    self.refresh()

  def test_normal_actuation_is_never_allowed(self):
    for allowed in (False, True):
      self.safety.set_controls_allowed(allowed)
      for addr, length in [(0x50, 16), (0x110, 32), (0x12A, 16), (0x1A0, 32), (0x1CF, 8), (0x7D0, 8)]:
        self.assertFalse(self.tx(addr, bytes(length)))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_uds_requires_real_ordered_acknowledgements(self):
    self.assertFalse(self.diag(0x28, 1))
    self.ack(0x50, 3)  # An unsolicited response cannot grant a session.
    self.assertFalse(self.diag(0x28, 1))
    self.assertTrue(self.diag(0x10, 3))
    self.assertFalse(self.diag(0x28, 1))
    self.ack(0x50, 3)
    self.assertTrue(self.diag(0x28, 1))
    self.assertFalse(self.spas())
    self.ack(0x68, 1)
    self.assertFalse(self.spas())  # Require 100ms stock silence after the ACK.
    self.advance(100000)
    self.refresh(stock=False)
    self.assertTrue(self.spas())

  def test_all_parked_inputs_are_required(self):
    for change in ({'gear': 5}, {'gas': 1}, {'speed': 1}, {'brake': False}, {'parking_brake': False}, {'stalk': True}, {'lamp': True}):
      with self.subTest(change=change):
        self.setUp()
        self.refresh(**change)
        self.assertFalse(self.diag(0x10, 3))

  def test_diagnostics_are_narrowly_scoped(self):
    for data in (b'\x02\x3e\x00' + bytes(5), b'\x03\x28\x83\x01' + bytes(4), b'\x02\x11\x01' + bytes(5),
                 b'\x02\x10\x03\x00\x01' + bytes(3)):
      self.assertFalse(self.tx(0x7B1, data))
    self.assertFalse(self.tx(0x7B1, b'\x02\x10\x03' + bytes(5), bus=1))

  def test_spas_body_direction_checksum_counter_and_rate_limits(self):
    self.disabled()
    self.assertFalse(self.spas(mutate=True))
    for side in (1, 2, 5, 6, 7):
      self.assertFalse(self.spas(side))
    self.assertTrue(self.spas(addr=0x165))
    self.assertTrue(self.spas())
    self.assertFalse(self.spas())
    self.advance(20000)
    self.assertFalse(self.spas(4))
    self.assertTrue(self.spas())

  def test_hold_is_masked_to_matching_side_and_stock_body(self):
    self.assertFalse(self.hold())
    self.disabled()
    self.assertTrue(self.spas())
    self.assertFalse(self.hold(4))
    self.assertFalse(self.hold(7))
    self.assertFalse(self.hold(mutate=True))
    self.assertFalse(self.hold(counter_delta=1))
    self.assertTrue(self.hold())
    self.assertFalse(self.hold())
    self.advance(50000)
    self.refresh(stock=False)
    self.assertTrue(self.hold())

  def test_no_infinite_hold_without_restoration_and_stock_resume(self):
    self.disabled()
    self.assertTrue(self.spas())
    self.advance(4000001)
    self.refresh(stock=False)
    self.assertFalse(self.hold())
    self.assertTrue(self.diag(0x28, 0))
    self.assertTrue(self.diag(0x10, 1))

  def test_pulse_has_two_second_hard_limit(self):
    self.disabled()
    self.assertTrue(self.spas())
    self.advance(2000001)
    self.refresh(stock=False)
    self.assertFalse(self.spas())
    self.assertTrue(self.spas(0))

  def test_hold_continues_only_after_acknowledged_restore_and_stock_resume(self):
    self.holding()
    for _ in range(60):
      self.advance(100000)
      self.refresh()
      self.assertTrue(self.hold())
    self.assertTrue(self.hold(0))

  def test_input_loss_latches_abort_and_cleanup_remains_possible(self):
    self.holding()
    self.advance(50000)
    self.refresh(gear=5)
    self.assertFalse(self.hold())
    self.refresh()
    self.assertFalse(self.hold())
    self.assertTrue(self.hold(0))
    self.assertTrue(self.diag(0x28, 0))
    self.assertTrue(self.diag(0x10, 1))

  def test_counter_replay_cannot_refresh_guard(self):
    self.holding()
    self.rx(0x3C1, self.stock[0x3C1])
    self.advance(50000)
    self.assertFalse(self.hold())

  def test_source_crc_failure_cannot_freshen_hold(self):
    self.holding()
    self.advance(50000)
    self.refresh()
    data = bytearray(self.stock[0x1EA])
    data[4] ^= 1
    self.rx(0x1EA, data, 2)
    self.assertFalse(self.hold())
    self.assertTrue(self.hold(0))

  def test_missing_enable_ack_cannot_grant_indefinite_hold(self):
    self.disabled()
    self.assertTrue(self.spas())
    self.assertTrue(self.diag(0x28, 0))
    self.assertTrue(self.diag(0x10, 1))
    self.ack(0x50, 1)
    self.refresh()
    self.advance(50000)
    self.refresh()
    self.assertFalse(self.hold())

  def test_restore_stock_rx_cannot_bypass_session_deadline(self):
    self.holding()
    self.assertTrue(self.diag(0x28, 0))
    self.ack(0x68, 0)
    self.assertTrue(self.diag(0x10, 1))
    self.ack(0x50, 1)
    self.advance(4000001)
    # Supply SPAS first: a phase transition must not bypass the expired timer
    # before the next ordinary guard frame arrives.
    for addr in (0x165, 0x16A):
      data = bytearray(self.stock[addr])
      data[2] = (data[2] + 1) & 255
      self.rx(addr, self.checksum(addr, data))
    self.refresh()
    self.assertFalse(self.hold())

  def test_no_lamp_feedback_cannot_enter_indefinite_hold(self):
    self.disabled()
    self.assertTrue(self.spas())
    self.assertTrue(self.diag(0x28, 0))
    self.ack(0x68, 0)
    self.assertTrue(self.diag(0x10, 1))
    self.ack(0x50, 1)
    self.refresh()
    self.assertFalse(self.hold())

  def test_hold_requires_fresh_neutral_stock_spas(self):
    self.holding()
    for _ in range(3):
      self.advance(50000)
      self.refresh(stock=False)
    self.assertFalse(self.hold())
    self.assertEqual(self.fwd(0x1EA, 2), 0)

  def test_opposite_lamp_or_hazards_abort_in_every_active_phase(self):
    for phase in ('extended', 'pulse', 'hold'):
      with self.subTest(phase=phase):
        self.setUp()
        if phase == 'hold':
          self.holding()
        else:
          self.disabled()
          if phase == 'pulse':
            self.assertTrue(self.spas())
        data = bytearray(self.stock[0x413])
        data[1] = ((data[1] + 0x10) & 0xF0)
        data[2] |= 0x40  # right lamp; this experiment requests left
        self.rx(0x413, data)
        self.advance(50000)
        self.assertFalse(self.hold())
        self.assertTrue(self.diag(0x28, 0))

  def test_stock_spas_resuming_during_pulse_aborts(self):
    self.disabled()
    self.assertTrue(self.spas())
    self.refresh()
    self.assertFalse(self.hold())
    self.assertTrue(self.diag(0x28, 0))

  def test_forward_only_replaces_fresh_active_hold(self):
    self.assertEqual(self.fwd(0x1EA, 2), 0)
    self.holding()
    destination, data = self.forwarded_adrv()
    self.assertEqual(destination, 0)
    self.assertEqual(data[3] & 0x60, 0x20)
    for bus, dest in ((0, 2), (1, -1), (2, 0)):
      for addr in (0x50, 0x1A0, 0x165, 0x16A, 0x175, 0x7B9):
        self.assertEqual(self.fwd(addr, bus), dest)
    self.assertTrue(self.hold(0))
    self.assertEqual(self.fwd(0x1EA, 2), 0)
    self.assertEqual(self.forwarded_adrv()[1][3] & 0x60, 0)

  def test_forward_recovers_when_host_or_input_disappears(self):
    self.holding()
    self.advance(150001)
    self.refresh()
    self.assertEqual(self.fwd(0x1EA, 2), 0)
    self.assertEqual(self.forwarded_adrv()[1][3] & 0x60, 0)
    self.setUp()
    self.holding()
    self.advance(100001)
    self.assertEqual(self.fwd(0x1EA, 2), 0)
    self.assertEqual(self.forwarded_adrv()[1][3] & 0x60, 0)

  def test_host_hold_and_off_are_commands_without_duplicate_wire_transmission(self):
    self.holding()
    self.advance(60000)
    self.refresh()
    self.assertTrue(self.hold())
    flag = ctypes.c_bool.in_dll(ctypes.CDLL(libsafety_py.libsafety_fn), 'safety_tx_buffered_for_fwd')
    self.assertTrue(flag.value)
    self.assertTrue(self.hold(0))
    self.assertTrue(flag.value)

  def test_forward_preserves_current_stock_body_counter_and_checksum(self):
    for side, hold in ((3, 0x20), (4, 0x40)):
      with self.subTest(side=side):
        self.setUp()
        self.holding(side)
        data = bytearray(self.stock[0x1EA])
        data[2] = (data[2] + 1) & 255
        data[3] = 0x9F
        data[4:] = bytes(range(28))
        data = self.checksum(0x1EA, data)
        destination, transformed = self.forwarded_adrv(data)
        self.assertEqual(destination, 0)
        self.assertEqual(transformed[2], data[2])
        self.assertEqual(transformed[3], data[3] | hold)
        self.assertEqual(transformed[4:], data[4:])
        self.assertEqual(transformed, self.checksum(0x1EA, transformed))

  def test_forward_does_not_modify_bad_crc_duplicate_counter_or_other_bus(self):
    self.holding()
    original = bytearray(self.stock[0x1EA])
    bad_crc = bytearray(original)
    bad_crc[4] ^= 1
    bad_counter = bytearray(original)
    bad_counter[2] = (bad_counter[2] + 3) & 255
    for frame in (original, bad_crc, self.checksum(0x1EA, bad_counter)):
      self.assertEqual(self.forwarded_adrv(frame)[1], frame)
    frame = bytearray(original)
    frame[2] = (frame[2] + 1) & 255
    frame = self.checksum(0x1EA, frame)
    self.assertEqual(self.forwarded_adrv(frame, bus=0), (2, frame))

  def test_recent_host_snapshot_is_accepted_after_camera_advances(self):
    self.holding()
    old = bytearray(self.stock[0x1EA])
    old[3] |= 0x20
    old = self.checksum(0x1EA, old)
    for _ in range(2):
      self.advance(30000)
      self.refresh()
    self.assertTrue(self.tx(0x1EA, old))
    self.advance(60000)
    self.refresh()
    self.assertFalse(self.tx(0x1EA, old))  # Evicted and over 100ms old.

  def test_recent_snapshot_rejects_body_or_counter_not_seen_in_camera_history(self):
    self.holding()
    self.advance(60000)
    self.refresh()
    self.assertFalse(self.hold(mutate=True))
    self.assertFalse(self.hold(counter_delta=1))
    self.assertTrue(self.hold())

  def test_unsigned_timer_wrap(self):
    self.now = 0xFFFF0000
    self.safety.set_timer(self.now)
    self.refresh()
    self.holding()
    self.advance(50000)
    self.refresh()
    self.assertTrue(self.hold())

  def test_safety_reset_clears_all_experiment_state(self):
    self.holding()
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.PARAM)
    self.assertFalse(self.hold())
    self.assertFalse(self.diag(0x10, 3))
    self.assertEqual(self.fwd(0x1EA, 2), 0)

  def test_cleanup_does_not_allow_another_experiment_without_safety_reset(self):
    self.holding()
    self.assertTrue(self.hold(0))
    self.assertTrue(self.diag(0x28, 0))
    self.ack(0x68, 0)
    self.assertTrue(self.diag(0x10, 1))
    self.ack(0x50, 1)
    self.refresh()
    self.assertFalse(self.diag(0x10, 3))

  def test_test_parameter_does_not_leak_into_normal_mode(self):
    self.holding()
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, 1 | 8)
    self.assertFalse(self.diag(0x28, 0))
    self.assertFalse(self.spas())

  def test_unsupported_test_parameters_cannot_fall_back_to_driving(self):
    for param in (1024, 1024 | 1, 1024 | 8):
      with self.subTest(param=param):
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, param)
        self.safety.set_controls_allowed(True)
        for addr, length in ((0x12A, 16), (0x50, 16), (0x1A0, 32), (0x1CF, 8), (0x7B1, 8), (0x16A, 32)):
          self.assertFalse(self.tx(addr, bytes(length)))
        self.assertFalse(self.safety.get_controls_allowed())


if __name__ == '__main__':
  unittest.main()
