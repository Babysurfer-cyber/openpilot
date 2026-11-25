#!/usr/bin/env python3
"""EV6 APRK/SPAS software bench. No real CAN, UDS or vehicle interface.

All WOULD_* audit entries are hypothetical. A simulated positive ECU
response proves only the state machine, not that an EV6 accepts it.

  python tools/signal_analysis/ev6_aprk_handoff_bench.py --report report.json
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path

SPAS = (0x165, 0x16A)
RESPONSE = 0x7B9
SESSION_TIMEOUT_MS = 400
QUIET_MS = 100
STOCK_FRESH_MS = 100
POST_SESSION_STOCK_WINDOW_MS = 50
LEASE_MAX_MS = 1000


@dataclass(frozen=True)
class Context:
  # Openpilot lateral control may remain enabled; never use it as TX proof.
  park: bool = True
  stopped: bool = True
  brake: bool = True
  gas: bool = False
  driver_lever: bool = False
  lamp_on: bool = False
  can_healthy: bool = True
  relay_healthy: bool = True

  def denial(self) -> str | None:
    for name, ok in (
      ('not_in_park', self.park),
      ('not_stopped', self.stopped),
      ('foot_brake_required', self.brake),
      ('accelerator_active', not self.gas),
      ('driver_lever_active', not self.driver_lever),
      ('indicator_already_on', not self.lamp_on),
      ('can_unhealthy', self.can_healthy),
      ('relay_unhealthy', self.relay_healthy),
    ):
      if not ok:
        return name
    return None


class AprkHandoffBench:
  """Independent fail-closed mock. Never produces CAN packets."""

  def __init__(self):
    self.phase = 'idle'
    self.last_ms = -1
    self.phase_ms = 0
    self.lease_ms = 0
    self.last_stock = {addr: -1 for addr in SPAS}
    self.session_ack_ms = -1
    self.stock_after_session_ack = {addr: -1 for addr in SPAS}
    self.disable_request_stock_verified = False
    self.last_counter = {addr: None for addr in SPAS}
    self.consecutive = {addr: 0 for addr in SPAS}
    self.reason = None
    self.restore_enable_ack = False
    self.restore_default_ack = False
    self.audit = []
    self.actual_can_tx = 0

  def _event(self, now_ms: int, name: str):
    self.audit.append({'time_ms': now_ms, 'event': name})

  def _time_ok(self, now_ms: int):
    if now_ms < self.last_ms:
      raise ValueError('non-monotonic bench timestamp')
    self.last_ms = now_ms

  def stock(self, now_ms: int, addr: int, counter: int, crc_ok: bool = True, bus: int = 0):
    self._time_ok(now_ms)
    if addr not in SPAS or bus != 0:
      return
    if not crc_ok or not 0 <= counter <= 255:
      self.consecutive[addr] = 0
      # Never silently treat malformed OEM telemetry during a handoff as
      # proof of silence. An invalid observed stock frame is not "no frame".
      if self.phase in ('wait_post_session_stock', 'wait_disable', 'wait_quiet', 'lease_eligible'):
        self._abort(now_ms, 'stock_contract_failed')
      return
    old_ms, old_counter = self.last_stock[addr], self.last_counter[addr]
    consecutive = (old_ms >= 0 and 0 <= now_ms - old_ms < STOCK_FRESH_MS and
                   old_counter is not None and counter == (old_counter + 1) % 256)
    self.consecutive[addr] = min(20, self.consecutive[addr] + 1) if consecutive else 1
    self.last_stock[addr] = now_ms
    self.last_counter[addr] = counter
    if self.phase == 'wait_post_session_stock':
      # Do not even *propose* the diagnostic disable request until BOTH
      # original streams are still live and advancing after session ACK.
      if (not continuous or now_ms <= self.session_ack_ms or
          now_ms - self.session_ack_ms > POST_SESSION_STOCK_WINDOW_MS):
        self._abort(now_ms, 'stock_contract_failed_after_session_ack')
        return
      self.stock_after_session_ack[addr] = now_ms
      both_live = all(self.stock_after_session_ack[a] > self.session_ack_ms and
                      0 <= now_ms - self.stock_after_session_ack[a] <= POST_SESSION_STOCK_WINDOW_MS
                      for a in SPAS)
      if both_live:
        self.disable_request_stock_verified = True
        self.phase, self.phase_ms = 'wait_disable', now_ms
        self._event(now_ms, 'WOULD_REQUEST_COMMUNICATION_DISABLE')
    if self.phase == 'lease_eligible':
      self._abort(now_ms, 'stock_resumed_during_lease')
    if self.phase == 'wait_quiet':
      # Match the passive PR #45 trace gate: OEM SPAS continuing at or
      # beyond the 100 ms disable grace is a failed handoff. Do not grant a
      # lease merely because the stream happens to stop at some later time.
      if now_ms - self.phase_ms >= QUIET_MS:
        self._abort(now_ms, 'stock_after_disable_grace')
      else:
        self._event(now_ms, 'OEM_STILL_TRANSMITTING')

  def stock_ready(self, now_ms: int) -> bool:
    return all(self.consecutive[a] >= 10 and self.last_stock[a] >= 0 and
               0 <= now_ms - self.last_stock[a] < STOCK_FRESH_MS for a in SPAS)

  def start(self, now_ms: int, ctx: Context) -> bool:
    self._time_ok(now_ms)
    if self.phase != 'idle':
      return False
    denial = ctx.denial()
    if denial is None and not self.stock_ready(now_ms):
      denial = 'oem_spas_not_valid_and_fresh'
    if denial:
      self.reason = denial
      self._event(now_ms, 'BLOCKED_' + denial)
      return False
    self.reason = None
    self.phase, self.phase_ms = 'wait_extended', now_ms
    self._event(now_ms, 'WOULD_REQUEST_EXTENDED_SESSION')
    return True

  def response(self, now_ms: int, addr: int, bus: int, payload: bytes):
    self._time_ok(now_ms)
    if (addr != RESPONSE or bus != 0 or len(payload) != 8 or
        self.phase not in ('wait_extended', 'wait_post_session_stock', 'wait_disable', 'wait_restore_enable', 'wait_restore_default')):
      return
    if not 0 <= now_ms - self.phase_ms < SESSION_TIMEOUT_MS:
      return
    n = payload[0]
    if not 2 <= n <= 7:
      return
    response = bytes(payload[1:1+n])
    if response.startswith(b'\x7f'):
      if self.phase in ('wait_restore_enable', 'wait_restore_default'):
        # _abort() deliberately does not re-enter restore from a restore
        # state. A negative ECU reply during restore must instead be
        # terminal: no subsequent positive reply may claim restoration.
        self.reason = 'ecu_negative_response_during_restore'
        self.phase, self.phase_ms = 'restore_unconfirmed', now_ms
        self._event(now_ms, 'RESTORE_UNCONFIRMED_DO_NOT_ACTUATE')
      else:
        self._abort(now_ms, 'ecu_negative_response')
    elif self.phase == 'wait_extended' and response.startswith(b'\x50\x03'):
      self.session_ack_ms = now_ms
      self.stock_after_session_ack = {addr: -1 for addr in SPAS}
      self.disable_request_stock_verified = False
      self.phase, self.phase_ms = 'wait_post_session_stock', now_ms
      self._event(now_ms, 'AWAIT_OEM_SPAS_AFTER_SESSION_ACK')
    elif self.phase == 'wait_disable' and response.startswith(b'\x68\x03'):
      # Verified at the *mock disable-request* transition, not at its
      # potentially delayed ACK. Check preserved independent evidence.
      if not self.disable_request_stock_verified:
        self._abort(now_ms, 'oem_spas_not_live_before_disable_request')
      else:
        self.phase, self.phase_ms = 'wait_quiet', now_ms
        self._event(now_ms, 'AWAIT_OEM_SPAS1_AND_SPAS2_QUIET')
    elif self.phase == 'wait_restore_enable' and response.startswith(b'\x68\x00'):
      self.restore_enable_ack = True
      self.phase, self.phase_ms = 'wait_restore_default', now_ms
      self._event(now_ms, 'WOULD_REQUEST_DEFAULT_SESSION')
    elif self.phase == 'wait_restore_default' and response.startswith(b'\x50\x01'):
      self.restore_default_ack = True
      self.phase, self.phase_ms = 'wait_restore_stock', now_ms
      self._event(now_ms, 'AWAIT_STOCK_RESUMPTION')

  def _abort(self, now_ms: int, reason: str):
    if self.phase in ('idle', 'wait_restore_enable', 'wait_restore_default', 'wait_restore_stock',
                      'complete', 'restore_unconfirmed'):
      return
    self.reason = reason
    self.phase, self.phase_ms = 'wait_restore_enable', now_ms
    self.restore_enable_ack = self.restore_default_ack = False
    self._event(now_ms, 'REVOKED_' + reason)
    self._event(now_ms, 'WOULD_REQUEST_COMMUNICATION_ENABLE')

  def stop(self, now_ms: int, reason: str = 'bench_trial_ended'):
    self._time_ok(now_ms)
    self._abort(now_ms, reason)

  def tick(self, now_ms: int, ctx: Context):
    self._time_ok(now_ms)
    if self.phase in ('idle', 'complete', 'restore_unconfirmed'):
      return
    if self.phase in ('wait_restore_enable', 'wait_restore_default', 'wait_restore_stock'):
      if (self.phase == 'wait_restore_stock' and self.restore_enable_ack and
          self.restore_default_ack and self.stock_ready(now_ms)):
        self.phase = 'complete'
        self._event(now_ms, 'SIMULATED_TWO_RESTORE_ACKS_AND_STOCK_RESUMPTION')
      elif now_ms - self.phase_ms >= SESSION_TIMEOUT_MS:
        self.phase = 'restore_unconfirmed'
        self._event(now_ms, 'RESTORE_UNCONFIRMED_DO_NOT_ACTUATE')
      return
    denial = ctx.denial()
    if denial:
      self._abort(now_ms, denial)
    elif self.phase == 'wait_extended' and now_ms - self.phase_ms >= SESSION_TIMEOUT_MS:
      self._abort(now_ms, 'no_extended_session_ack')
    elif self.phase == 'wait_post_session_stock' and now_ms - self.session_ack_ms > POST_SESSION_STOCK_WINDOW_MS:
      self._abort(now_ms, 'oem_spas_not_live_after_session_ack')
    elif self.phase == 'wait_disable' and now_ms - self.phase_ms >= SESSION_TIMEOUT_MS:
      self._abort(now_ms, 'no_communication_disable_ack')
    elif self.phase == 'wait_quiet':
      quiet = (now_ms - self.phase_ms >= QUIET_MS and
               all(self.last_stock[a] >= 0 and now_ms - self.last_stock[a] >= QUIET_MS for a in SPAS))
      if quiet:
        self.phase, self.lease_ms = 'lease_eligible', now_ms
        self._event(now_ms, 'BENCH_ONLY_ELIGIBLE_NO_CAN_TRANSMISSION')
      elif now_ms - self.phase_ms >= SESSION_TIMEOUT_MS:
        self._abort(now_ms, 'oem_spas_not_quiet')
    elif self.phase == 'lease_eligible' and now_ms - self.lease_ms >= LEASE_MAX_MS:
      self._abort(now_ms, 'lease_duration_limit')

  def result(self) -> dict:
    return {'phase': self.phase, 'reason': self.reason, 'actual_can_tx': self.actual_can_tx,
            'bench_only': True, 'real_ecu_response_verified': False, 'audit': self.audit}


def summarize_oem_report(report: dict) -> dict:
  cases = report.get('cases') or {}
  names = ('left', 'right', 'idle')
  if not all(name in cases for name in names):
    raise ValueError('OEM report requires left, right and idle')
  for name in names:
    item = cases[name]
    if not item.get('spas1_frames') or not item.get('spas2_frames'):
      raise ValueError(f'{name}: factory SPAS missing')
    if item.get('spas1_crc_invalid') or item.get('spas2_crc_invalid'):
      raise ValueError(f'{name}: factory SPAS CRC failed')
  return {'oem_stream_valid': True,
          'oem_spas1_frames': sum(cases[k]['spas1_frames'] for k in names),
          'oem_spas2_frames': sum(cases[k]['spas2_frames'] for k in names),
          'uds_response_frames_in_recordings': sum(cases[k].get('uds_7b9_rx_frames', 0) for k in names),
          'ecu_accepts_external_session': 'UNKNOWN: no external UDS requests in recordings',
          'oem_silence_after_communication_disable': 'UNKNOWN: not tested on vehicle',
          'external_spas_send_safe': False,
          'next_physical_step': 'Isolated ECU/CAN bench after independent safety review'}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--report', type=Path, help='PR #43 offline OEM report JSON')
  parser.add_argument('--output', type=Path, help='write software bench summary JSON')
  args = parser.parse_args()
  result = {'kind': 'EV6_APRK_HANDOFF_SOFTWARE_BENCH',
            'actual_can_tx': 0, 'vehicle_untouched': True}
  if args.report:
    result['recorded_oem_evidence'] = summarize_oem_report(json.loads(args.report.read_text('utf-8')))
  model = AprkHandoffBench()
  for i in range(10):
    model.stock(i*20, 0x165, i)
    model.stock(i*20, 0x16A, i)
  ctx = Context()
  model.start(190, ctx)
  model.response(210, RESPONSE, 0, bytes.fromhex('0350030000000000'))
  # The OEM stream must remain live after entering extended session.
  model.stock(212, 0x165, 10)
  model.stock(214, 0x16A, 10)
  model.response(230, RESPONSE, 0, bytes.fromhex('0368030000000000'))
  model.tick(300, ctx)
  model.tick(330, ctx)
  model.stop(360)
  model.response(365, RESPONSE, 0, bytes.fromhex('0368000000000000'))
  model.response(370, RESPONSE, 0, bytes.fromhex('0350010000000000'))
  for i in range(10):
    model.stock(380+i*20, 0x165, 10+i)
    model.stock(380+i*20, 0x16A, 10+i)
  model.tick(560, ctx)
  result['positive_mock_lease'] = model.result()
  content = json.dumps(result, ensure_ascii=False, indent=2)
  if args.output:
    args.output.write_text(content+'\n', encoding='utf-8')
    print('Wrote', args.output)
  print(content)


if __name__ == '__main__':
  main()
