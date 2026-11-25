#!/usr/bin/env python3
"""Panda safety regression for the existing EV6 P-only 0x3C1 laboratory trial.

These tests use the *compiled* panda safety C hooks, not an imitation of
the guard logic. They create packets in memory; no vehicle CAN I/O.
The experimental 0x3C1 message must never be used on public roads.
"""
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


def _crc16_xmodem(data):
  crc = 0
  for b in data:
    crc ^= b << 8
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
  return crc


def _frame(address, data, bus=0):
  return libsafety_py.make_CANPacket(address, bus, data)


def _hyundai_canfd(address, length, counter=0, *, gear=0, braking=False, bus=0):
  """Minimal checksum-valid test frames. 0x35 gear is in byte24 bits[2:0]."""
  assert length in (24, 32)
  data = bytearray(length)
  data[2] = counter & 0xFF
  if address == 0x35:
    data[24] = gear & 0x7
  if address == 0x175 and braking:
    data[10] |= 0x02  # bit81: DriverBraking
  xor = {24: 0x819D, 32: 0x9F5B}[length]
  checksum = _crc16_xmodem(data[2:] + address.to_bytes(2, "little")) ^ xor
  data[0:2] = checksum.to_bytes(2, "little")
  return _frame(address, bytes(data), bus=bus)


# Captured real EV6 neutral stalk and matching human left-tap signature:
# 36 20 30 00 00 00 00 00 -> 35 30 30 40 10 00 00 00.
STOCK_STALK = bytes.fromhex("3620300000000000")
LEFT_TAP = bytes.fromhex("3530304010000000")


class TestEV6StalkParkGuard(unittest.TestCase):
  """Run with the same compiled libsafety hooks as the rest of panda tests."""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.flags = (HyundaiSafetyFlags.CANFD_LKA_STEERING |
                  HyundaiSafetyFlags.CAMERA_SCC |
                  HyundaiSafetyFlags.EV_GAS |
                  HyundaiSafetyFlags.EV6_BLINKER_STALK_TEST)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.flags)
    self.safety.init_tests()
    self.safety.set_timer(1_000_000)

  def _rx(self, packet):
    return self.safety.safety_rx_hook(packet)

  def _tx_left(self):
    return self.safety.safety_tx_hook(_frame(0x3C1, LEFT_TAP))

  def _prime(self, gear=0, *, correct_crc=True, gear_bus=0, include_stalk=True):
    gear_frame = _hyundai_canfd(0x35, 32, gear=gear, bus=gear_bus)
    if not correct_crc:
      # Modifying a checksum field invalidates this 32-byte stock frame.
      damaged = bytearray(gear_frame.data[:32])
      damaged[0] ^= 0xFF
      gear_frame = _frame(0x35, bytes(damaged), bus=gear_bus)
    self._rx(gear_frame)
    self._rx(_hyundai_canfd(0x175, 24, braking=True))
    if include_stalk:
      self._rx(_frame(0x3C1, STOCK_STALK))

  def test_valid_park_brake_stalk_allows_only_original_trial(self):
    self._prime(gear=0)
    self.assertTrue(self._tx_left())

  def test_drive_neutral_reverse_not_authorized_at_standstill(self):
    for gear in (5, 6, 7):
      with self.subTest(gear=gear):
        self.setUp()
        self._prime(gear=gear)
        self.assertFalse(self._tx_left())

  def test_no_gear_frame_is_not_park(self):
    self._rx(_hyundai_canfd(0x175, 24, braking=True))
    self._rx(_frame(0x3C1, STOCK_STALK))
    self.assertFalse(self._tx_left())

  def test_wrong_bus_gear_is_not_park(self):
    self._prime(gear=0, gear_bus=1)
    self.assertFalse(self._tx_left())

  def test_invalid_crc_gear_is_not_park(self):
    self._prime(gear=0, correct_crc=False)
    self.assertFalse(self._tx_left())

  def test_gear_switch_from_p_to_d_revokes_permission(self):
    self._prime(gear=0)
    self._rx(_hyundai_canfd(0x35, 32, counter=1, gear=5))
    self.assertFalse(self._tx_left())

  def test_stale_p_gear_frame_does_not_authorize(self):
    self._prime(gear=0)
    self.safety.set_timer(1_120_001)
    # Refresh stalk only, so the failure must be from the expired gear
    # evidence rather than the stalk freshness guard.
    self._rx(_frame(0x3C1, STOCK_STALK))
    self.assertFalse(self._tx_left())

  def test_brake_release_revokes_permission(self):
    self._prime(gear=0)
    self._rx(_hyundai_canfd(0x175, 24, counter=1, braking=False))
    self.assertFalse(self._tx_left())

  def test_missing_stock_stalk_blocks(self):
    self._prime(gear=0, include_stalk=False)
    self.assertFalse(self._tx_left())

  def test_spas_and_diagnostic_ids_remain_disallowed(self):
    self._prime(gear=0)
    for address, size in ((0x165, 24), (0x16A, 32), (0x7B1, 8)):
      with self.subTest(address=hex(address)):
        self.assertFalse(self.safety.safety_tx_hook(_frame(address, b"\x00" * size)))


if __name__ == "__main__":
  unittest.main()
