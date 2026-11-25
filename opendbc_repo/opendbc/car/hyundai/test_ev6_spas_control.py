"""No-CAN, synthetic replay for EV6 SPAS control planning core."""
import unittest
from ev6_spas_control import (APRK_RESPONSE, EV6SpasControlPlan, SafetyInputs,
                              SPAS_LENGTHS, crc16_xmodem, check_stock_frame)


def sample(addr, counter=0, control=0):
  size = SPAS_LENGTHS[addr]
  data = bytearray(size)
  data[2] = counter
  if addr == 0x16A:
    data[16] = control << 5
  xor = {24: 0x819D, 32: 0x9F5B}[size]
  data[:2] = (crc16_xmodem(data[2:] + addr.to_bytes(2, 'little')) ^ xor).to_bytes(2, 'little')
  return bytes(data)


def reply(b1, b2):
  return bytes((2, b1, b2, 0, 0, 0, 0, 0))


SAFE = SafetyInputs(True, True, True, True, True, True, True)
BAD = SafetyInputs(True, True, True, True, True, True, False)


class TestPlan(unittest.TestCase):
  def setUp(self):
    self.c = EV6SpasControlPlan()
    self.prime()

  def prime(self):
    for i in range(12):
      t = 1.0 + i * .02
      for a in SPAS_LENGTHS:
        self.c.observe_stock(a, 0, sample(a, i), t)

  def start(self):
    return self.c.request('once', 'left', 1.24, SAFE)

  def session_ack(self):
    self.c.observe_reply(APRK_RESPONSE, 0, reply(0x50, 3), 1.25)
    self.c.tick(1.25, SAFE)

  def after_session(self):
    self.session_ack()
    for i in (12, 13):
      t = 1.26 + (i-12) * .02
      for a in SPAS_LENGTHS:
        self.c.observe_stock(a, 0, sample(a, i), t)
    return self.c.tick(1.29, SAFE)

  def disable_ack(self):
    self.after_session()
    self.c.observe_reply(APRK_RESPONSE, 0, reply(0x68, 3), 1.30)
    self.c.tick(1.30, SAFE)

  def test_crc_check_rejects_tampered_sample(self):
    self.assertTrue(check_stock_frame(0x16A, sample(0x16A, 1, 3)))
    altered = bytearray(sample(0x16A, 1))
    altered[16] = 0x60
    self.assertFalse(check_stock_frame(0x16A, altered))

  def test_start_emits_intent_only(self):
    self.assertEqual(self.start(), ['WOULD_REQUEST_EXTENDED_SESSION'])
    self.assertEqual(self.c.phase, 'session')
    self.assertFalse(hasattr(self.c, 'send'))

  def test_no_independent_monitor_denied(self):
    self.assertEqual(self.c.request('once', 'left', 1.24, BAD), ['REQUEST_DENIED_SAFETY'])

  def test_unsafe_driver_or_controls_denied(self):
    for kw in ('park','brake','standstill','controls_inactive','driver_lever_idle','bus_healthy'):
      with self.subTest(kw=kw):
        st = vars(SAFE).copy()
        st[kw] = False
        self.assertEqual(self.c.request('bad'+kw, 'left', 1.24, SafetyInputs(**st)), ['REQUEST_DENIED_SAFETY'])

  def test_wrong_bus_does_not_provide_stock(self):
    p = EV6SpasControlPlan()
    for i in range(12):
      for a in SPAS_LENGTHS:
        p.observe_stock(a, 2, sample(a, i), 1+i*.02)
    self.assertEqual(p.request('once', 'left', 1.24, SAFE), ['REQUEST_DENIED_STOCK_NOT_READY'])

  def test_no_ack_times_out_instead_of_fallback_tx(self):
    self.start()
    events = self.c.tick(1.65, SAFE)
    self.assertIn('WOULD_REQUEST_COMMUNICATION_ENABLE', events)
    self.assertEqual(self.c.phase, 'restore_enable')
    self.assertNotIn('SIMULATED_HANDOFF_ONLY_NO_CAN_TX', events)

  def test_unverified_post_session_stock_blocks_disable(self):
    self.start()
    self.session_ack()
    events = self.c.tick(1.46, SAFE)
    self.assertIn('WOULD_REQUEST_COMMUNICATION_ENABLE', events)
    self.assertNotIn('WOULD_REQUEST_COMMUNICATION_DISABLE', events)

  def test_disable_intent_after_fresh_post_ack_streams(self):
    self.start()
    self.assertIn('WOULD_REQUEST_COMMUNICATION_DISABLE', self.after_session())

  def test_negative_disable_reply_causes_restore(self):
    self.start()
    self.after_session()
    self.c.observe_reply(APRK_RESPONSE, 0, b'\x03\x7f\x28\x22\x00\x00\x00\x00', 1.3)
    self.assertIn('WOULD_REQUEST_COMMUNICATION_ENABLE', self.c.tick(1.31, SAFE))
    self.assertEqual(self.c.reason, 'negative_ecu_reply')

  def test_wrong_response_bus_is_ignored(self):
    self.start()
    self.c.observe_reply(APRK_RESPONSE, 2, reply(0x50, 3), 1.25)
    self.assertEqual(self.c.tick(1.25, SAFE), [])
    self.assertEqual(self.c.phase, 'session')

  def test_successful_synthetic_handoff_is_no_tx_only(self):
    self.start()
    self.disable_ack()
    events = self.c.tick(1.50, SAFE)
    self.assertEqual(events, ['SIMULATED_HANDOFF_ONLY_NO_CAN_TX'])
    self.assertEqual(self.c.phase, 'eligible')

  def test_stock_resuming_revokes_eligibility(self):
    self.start()
    self.disable_ack()
    self.c.tick(1.50, SAFE)
    self.c.observe_stock(0x16A, 0, sample(0x16A, 14), 1.51)
    self.assertIn('WOULD_REQUEST_COMMUNICATION_ENABLE', self.c.tick(1.52, SAFE))
    self.assertEqual(self.c.reason, 'original_ecu_resumed')

  def test_cancellation_requests_restore(self):
    self.start()
    self.assertEqual(self.c.cancel(1.27), ['WOULD_REQUEST_COMMUNICATION_ENABLE'])
    self.assertEqual(self.c.phase, 'restore_enable')

  def test_no_restore_ack_leads_to_persistent_lockout(self):
    self.start()
    self.c.cancel(1.25)
    self.assertIn('WOULD_REQUEST_DEFAULT_SESSION', self.c.tick(1.66, SAFE))
    self.c.tick(2.07, SAFE)
    self.assertIn('RESTORE_UNCONFIRMED_LOCKOUT', self.c.tick(3.10, SAFE))
    self.assertEqual(self.c.request('again', 'right', 3.11, SAFE), ['REQUEST_DENIED_NOT_IDLE'])

  def test_non_monotonic_time_stops_plan(self):
    self.start()
    events = self.c.tick(1.20, SAFE)
    self.assertIn('WOULD_REQUEST_COMMUNICATION_ENABLE', events)
    self.assertEqual(self.c.reason, 'nonmonotonic_time')

  def test_expired_eligibility_stops_without_repeating(self):
    self.start()
    self.disable_ack()
    self.c.tick(1.50, SAFE)
    events = self.c.tick(1.86, SAFE)
    self.assertEqual(events, ['WOULD_REQUEST_COMMUNICATION_ENABLE'])
    self.assertEqual(self.c.reason, 'plan_duration_expired')


if __name__ == '__main__':
  unittest.main()
