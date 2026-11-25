"""Offline tests for EV6 APRK software bench. Zero vehicle or CAN access."""
import unittest

from ev6_aprk_handoff_bench import AprkHandoffBench, Context, RESPONSE, summarize_oem_report

POS_SESSION = bytes.fromhex('0350030000000000')
POS_DISABLE = bytes.fromhex('0368030000000000')
POS_ENABLE = bytes.fromhex('0368000000000000')
POS_DEFAULT = bytes.fromhex('0350010000000000')
NEGATIVE = bytes.fromhex('037f102200000000')


def initial():
  bench = AprkHandoffBench()
  for i in range(10):
    bench.stock(20*i, 0x165, i)
    bench.stock(20*i, 0x16A, i)
  return bench


def after_disable():
  bench = initial()
  assert bench.start(190, Context())
  bench.response(210, RESPONSE, 0, POS_SESSION)
  # Genuine stream liveness after session ACK (PR #45 pre-disable gate).
  bench.stock(212, 0x165, 10)
  bench.stock(214, 0x16A, 10)
  bench.response(230, RESPONSE, 0, POS_DISABLE)
  assert bench.phase == 'wait_quiet'
  return bench


class TestEV6AprkHandoffBench(unittest.TestCase):
  def test_mock_success_still_has_zero_can_tx(self):
    b = after_disable()
    b.tick(310, Context())
    self.assertEqual(b.phase, 'wait_quiet')
    b.tick(330, Context())
    self.assertEqual(b.phase, 'lease_eligible')
    b.stop(360)
    self.assertEqual(b.phase, 'wait_restore_enable')
    b.response(365, RESPONSE, 0, POS_ENABLE)
    self.assertEqual(b.phase, 'wait_restore_default')
    b.response(370, RESPONSE, 0, POS_DEFAULT)
    self.assertEqual(b.phase, 'wait_restore_stock')
    for i in range(10):
      b.stock(380+i*20, 0x165, 10+i)
      b.stock(380+i*20, 0x16A, 10+i)
    b.tick(560, Context())
    self.assertEqual(b.phase, 'complete')
    self.assertEqual(b.actual_can_tx, 0)
    self.assertFalse(b.result()['real_ecu_response_verified'])

  def test_missing_session_ack_blocks(self):
    b = initial()
    self.assertTrue(b.start(190, Context()))
    b.tick(600, Context())
    self.assertEqual(b.phase, 'wait_restore_enable')
    self.assertEqual(b.reason, 'no_extended_session_ack')
    self.assertFalse(any('ELIGIBLE' in e['event'] for e in b.audit))

  def test_negative_uds_blocks(self):
    b = initial()
    b.start(190, Context())
    b.response(210, RESPONSE, 0, NEGATIVE)
    self.assertEqual(b.reason, 'ecu_negative_response')

  def test_wrong_bus_ignored(self):
    b = initial()
    b.start(190, Context())
    b.response(210, RESPONSE, 1, POS_SESSION)
    self.assertEqual(b.phase, 'wait_extended')
    b.tick(600, Context())
    self.assertEqual(b.reason, 'no_extended_session_ack')

  def test_wrong_response_order_ignored(self):
    b = initial()
    b.start(190, Context())
    b.response(210, RESPONSE, 0, POS_DISABLE)
    self.assertEqual(b.phase, 'wait_extended')

  def test_disable_ack_timeout_blocks(self):
    b = initial()
    b.start(190, Context())
    b.response(210, RESPONSE, 0, POS_SESSION)
    b.tick(620, Context())
    self.assertEqual(b.reason, 'no_communication_disable_ack')

  def test_stock_never_quiet_blocks(self):
    b = after_disable()
    for i in range(1, 6):
      now = 230+20*i
      b.stock(now, 0x165, 9+i)
      b.stock(now, 0x16A, 9+i)
      b.tick(now, Context())
    self.assertEqual(b.phase, 'wait_restore_enable')
    self.assertEqual(b.reason, 'stock_after_disable_grace')

  def test_late_stock_cannot_create_a_delayed_false_lease(self):
    b = after_disable()
    b.stock(330, 0x165, 10)
    self.assertEqual(b.phase, 'wait_restore_enable')
    self.assertEqual(b.reason, 'stock_after_disable_grace')
    b.tick(460, Context())
    self.assertFalse(any(e['event'] == 'BENCH_ONLY_ELIGIBLE_NO_CAN_TRANSMISSION'
                         for e in b.audit))
    self.assertEqual(b.actual_can_tx, 0)

  def test_invalid_stock_during_disable_quiet_is_not_silence(self):
    b = after_disable()
    b.stock(240, 0x165, 10, crc_ok=False)
    self.assertEqual(b.phase, 'wait_restore_enable')
    self.assertEqual(b.reason, 'stock_contract_failed')
    self.assertFalse(any('ELIGIBLE' in e['event'] for e in b.audit))

  def test_disable_ack_without_post_session_stock_is_not_proof(self):
    b = initial()
    b.start(190, Context())
    b.response(210, RESPONSE, 0, POS_SESSION)
    b.response(230, RESPONSE, 0, POS_DISABLE)
    self.assertEqual(b.phase, 'wait_restore_enable')
    self.assertEqual(b.reason, 'oem_spas_not_live_after_session_ack')
    self.assertFalse(any('ELIGIBLE' in e['event'] for e in b.audit))

  def test_disable_ack_with_only_one_stock_stream_is_not_proof(self):
    b = initial()
    b.start(190, Context())
    b.response(210, RESPONSE, 0, POS_SESSION)
    b.stock(212, 0x165, 10)
    b.response(230, RESPONSE, 0, POS_DISABLE)
    self.assertEqual(b.phase, 'wait_restore_enable')
    self.assertEqual(b.reason, 'oem_spas_not_live_after_session_ack')

  def test_bad_stock_during_session_handoff_revokes(self):
    b = initial()
    b.start(190, Context())
    b.response(210, RESPONSE, 0, POS_SESSION)
    b.stock(212, 0x165, 10, crc_ok=False)
    self.assertEqual(b.phase, 'wait_restore_enable')
    self.assertEqual(b.reason, 'stock_contract_failed')

  def test_stock_resumes_revokes(self):
    b = after_disable()
    b.tick(330, Context())
    b.stock(350, 0x165, 10)
    self.assertEqual(b.reason, 'stock_resumed_during_lease')

  def test_brake_release_revokes(self):
    b = after_disable()
    b.tick(330, Context())
    b.tick(335, Context(brake=False))
    self.assertEqual(b.reason, 'foot_brake_required')

  def test_ineligible_vehicle_conditions(self):
    for ctx in (Context(park=False), Context(stopped=False),
                Context(gas=True), Context(driver_lever=True),
                Context(lamp_on=True), Context(can_healthy=False),
                Context(relay_healthy=False), Context(brake=False)):
      b = initial()
      self.assertFalse(b.start(190, ctx))
      self.assertEqual(b.phase, 'idle')
      self.assertEqual(b.actual_can_tx, 0)

  def test_stock_crc_failure(self):
    b = initial()
    b.stock(185, 0x165, 10, crc_ok=False)
    self.assertFalse(b.start(190, Context()))
    self.assertEqual(b.reason, 'oem_spas_not_valid_and_fresh')

  def test_stale_stock_disallows_start(self):
    b = initial()
    self.assertFalse(b.start(500, Context()))
    self.assertEqual(b.reason, 'oem_spas_not_valid_and_fresh')

  def test_simulated_lease_expires(self):
    b = after_disable()
    b.tick(330, Context())
    b.tick(1330, Context())
    self.assertEqual(b.reason, 'lease_duration_limit')

  def test_restore_missing_confirmation_not_success(self):
    b = initial()
    b.start(190, Context())
    b.tick(600, Context())
    b.tick(1001, Context())
    self.assertEqual(b.phase, 'restore_unconfirmed')

  def test_repeated_start_never_replays(self):
    b = initial()
    self.assertTrue(b.start(190, Context()))
    self.assertFalse(b.start(191, Context()))
    self.assertEqual(sum(e['event'] == 'WOULD_REQUEST_EXTENDED_SESSION' for e in b.audit), 1)

  def test_nonmonotonic_clock_rejected(self):
    b = initial()
    with self.assertRaises(ValueError):
      b.tick(100, Context())

  def test_passive_report_cannot_prove_session_support(self):
    report = {'cases': {name: {'spas1_frames': 10, 'spas2_frames': 10,
                               'spas1_crc_invalid': 0, 'spas2_crc_invalid': 0,
                               'uds_7b9_rx_frames': 0} for name in ('left', 'right', 'idle')}}
    result = summarize_oem_report(report)
    self.assertFalse(result['external_spas_send_safe'])
    self.assertIn('UNKNOWN', result['ecu_accepts_external_session'])


if __name__ == '__main__':
  unittest.main()
