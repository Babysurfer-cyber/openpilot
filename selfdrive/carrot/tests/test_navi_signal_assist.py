"""Direct navigation observations must remain distinct from display projections."""
from types import SimpleNamespace as NS

import pytest
from cereal import log

from openpilot.selfdrive.carrot.navi_signal_assist import NaviSignalAssist, NaviSignalRuntime


SOURCE = 'naver_traffic_signal_live_data'
BASE_WALL_MS = 1_800_000_000_000


def packet(receipt, state='RED', *, sequence=None, source_stamp=None, session='navigation', candidates=None):
  event = log.Event.new_message()
  event.init('carrotNavi')
  navi = event.carrotNavi
  navi.schemaVersion, navi.connected, navi.sessionId = 1, True, session
  sequence = round(receipt * 1000) if sequence is None else sequence
  source_stamp = BASE_WALL_MS + round(receipt * 1000) if source_stamp is None else source_stamp
  meta = dict(present=True, sequence=sequence, sourceTimestampMillis=source_stamp,
              receivedMonoTimeNanos=round(receipt * 1e9))
  navi.trafficSignal = dict(meta=meta, source=SOURCE)
  navi.signalHud = dict(meta=meta, directionMatchVerified=False,
    candidates=candidates if candidates is not None else [candidate('STRAIGHT', state)])
  navi.navigationStatus = dict(meta=meta, mode='guidance', guidanceActive=True, offRoute=False)
  navi.vehicle = dict(meta=meta, virtualGps=False)
  navi.laneCurrent = dict(meta=meta, roadCategory=8, roadCategoryValid=True)
  return navi


def candidate(direction, state, *, index=0, group=0):
  return dict(direction=direction, state=state, signalIndex=index, groupIndex=group,
              remainingValid=True, remainingSec=25)


def step(helper, now, navi, **changes):
  context = dict(mode=3, valid=True, received_s=now, enabled=True, drive=True,
                 speed=0., gas=False, brake=False, turning=False, departure=True)
  context.update(changes)
  return helper.update(now, navi, **context)


def red_hold(helper=None):
  helper = NaviSignalAssist() if helper is None else helper
  first = step(helper, 10., packet(10.))
  assert first.available and not first.confirmed and not first.hold
  confirmed = step(helper, 10.3, packet(10.3))
  assert confirmed.available and confirmed.confirmed and confirmed.hold
  return helper, confirmed


@pytest.mark.parametrize('state,color', [('RED', 'red'), ('YELLOW', 'yellow'), ('GREEN', 'green')])
def test_native_direct_source_and_current_straight_signal(state, color):
  helper = NaviSignalAssist()
  navi = packet(10., state)
  assert not navi.trafficSignal.visible  # Custom APK deliberately omits stock light fields.
  result = step(helper, 10., navi.as_reader())
  assert result.available and result.color == color and not result.confirmed
  assert result.session == 'navigation'
  assert not navi.signalHud.directionMatchVerified


def test_display_green_priority_cannot_replace_the_straight_red_reference():
  navi = packet(10., candidates=[candidate('LEFT', 'GREEN', index=1), candidate('STRAIGHT', 'RED')])
  result = step(NaviSignalAssist(), 10., navi)
  assert result.available and result.color == 'red'


@pytest.mark.parametrize('straight,left,color', [('RED', 'GREEN', 'green'), ('GREEN', 'RED', 'red'),
  ('GREEN', 'YELLOW', 'yellow')])
def test_explicit_left_reference_selects_the_protected_left_signal(straight, left, color):
  navi = packet(10., candidates=[candidate('STRAIGHT', straight), candidate('LEFT', left, index=1)])
  result = step(NaviSignalAssist(), 10., navi, direction='LEFT')
  assert result.available and result.color == color


@pytest.mark.parametrize('state', ['RED', 'YELLOW', 'GREEN'])
def test_left_without_a_protected_left_lamp_is_a_stop_reference_when_straight_is_present(state):
  result = step(NaviSignalAssist(), 10., packet(10., state), direction='LEFT')
  assert result.available and result.color == 'red' and result.reason == 'left_turn_not_allowed'
  assert not result.confirmed and not result.released


@pytest.mark.parametrize('state', ['RED_BLINK', 'YELLOW_BLINK', 'GREEN_BLINK'])
def test_blinking_left_signal_never_grants_a_green_reference(state):
  navi = packet(10., candidates=[candidate('STRAIGHT', 'GREEN'), candidate('LEFT', state, index=1)])
  result = step(NaviSignalAssist(), 10., navi, direction='LEFT')
  assert result.available and result.color == 'red' and result.reason == 'left_turn_not_allowed'


def test_left_reference_without_either_relevant_direction_falls_back_to_unknown():
  navi = packet(10., candidates=[candidate('RIGHT', 'GREEN')])
  result = step(NaviSignalAssist(), 10., navi, direction='LEFT')
  assert not result.available and result.color == 'unknown' and not result.hold


def test_changing_reference_direction_retires_only_the_navigation_owned_stop_without_green_permission():
  helper, _ = red_hold()
  for now in (10.4, 11.2):
    navi = packet(now, candidates=[candidate('STRAIGHT', 'RED'), candidate('LEFT', 'GREEN', index=1)])
    result = step(helper, now, navi, direction='LEFT')
    assert not result.hold and not result.released
  assert result.confirmed and result.color == 'green'
  # Returning to STRAIGHT cannot inherit either the previous stop or a green permission.
  for now in (11.3, 12.1):
    result = step(helper, now, packet(now, 'GREEN'))
    assert not result.hold and not result.released
  assert result.confirmed and not hasattr(result, 'green_confirmed')


def test_confirmed_protected_left_green_releases_only_its_own_missing_left_stop():
  helper = NaviSignalAssist()
  for now in (10., 10.3):
    result = step(helper, now, packet(now, 'GREEN'), direction='LEFT')
  assert result.confirmed and result.hold and result.color == 'red'
  for now in (10.4, 11.2):
    navi = packet(now, candidates=[candidate('STRAIGHT', 'RED'), candidate('LEFT', 'GREEN', index=1)])
    result = step(helper, now, navi, direction='LEFT')
    if now == 10.4:
      assert result.hold and not result.confirmed and not result.released
  assert result.confirmed and result.color == 'green' and not result.hold and result.released


@pytest.mark.parametrize('direction', ['STRAIGHT', 'LEFT'])
def test_array_reordering_does_not_change_the_same_movements_red_to_green_transition(direction):
  helper = NaviSignalAssist()
  for now in (10., 10.3):
    result = step(helper, now, packet(now, candidates=[candidate(direction, 'RED')]), direction=direction)
  assert result.hold
  other = 'LEFT' if direction == 'STRAIGHT' else 'STRAIGHT'
  for now in (10.4, 11.2):
    navi = packet(now, candidates=[candidate(other, 'RED'), candidate(direction, 'GREEN', index=1)])
    result = step(helper, now, navi, direction=direction)
  assert result.confirmed and result.color == 'green' and result.released and not result.hold


@pytest.mark.parametrize('case', [
  'schema', 'disconnected', 'session', 'source', 'direct_absent', 'hud_absent',
  'receipt_mismatch', 'sequence_mismatch', 'stamp_mismatch', 'legacy_global_red', 'stock_fallback_hybrid',
  'status_absent', 'status_stale', 'status_future', 'off_route', 'inactive_guidance',
  'simulation', 'virtual_gps', 'no_straight', 'blink', 'duplicate', 'mixed_groups',
])
def test_unusable_or_ambiguous_source_never_becomes_a_reference(case):
  navi = packet(10.)
  if case == 'schema': navi.schemaVersion = 0
  elif case == 'disconnected': navi.connected = False
  elif case == 'session': navi.sessionId = ''
  elif case == 'source': navi.trafficSignal.source = 'legacy'
  elif case == 'direct_absent': navi.trafficSignal.meta.present = False
  elif case == 'hud_absent': navi.signalHud.meta.present = False
  elif case == 'receipt_mismatch': navi.signalHud.meta.receivedMonoTimeNanos += 1
  elif case == 'sequence_mismatch': navi.signalHud.meta.sequence += 1
  elif case == 'stamp_mismatch': navi.signalHud.meta.sourceTimestampMillis += 1
  elif case == 'legacy_global_red':
    navi.trafficSignal.meta.present = False
    navi.signalHud.directionMatchVerified = False
  elif case == 'stock_fallback_hybrid':
    # Retained custom metadata must not make a subsequent stock projection a direct signal.
    navi.trafficSignal.visible = True
  elif case == 'status_absent': navi.navigationStatus.meta.present = False
  elif case == 'status_stale': navi.navigationStatus.meta.receivedMonoTimeNanos = 8_000_000_000
  elif case == 'status_future': navi.navigationStatus.meta.receivedMonoTimeNanos = 10_100_000_000
  elif case == 'off_route': navi.navigationStatus.offRoute = True
  elif case == 'inactive_guidance': navi.navigationStatus.guidanceActive = False
  elif case == 'simulation': navi.navigationStatus.mode = 'simulation'
  elif case == 'virtual_gps': navi.vehicle.virtualGps = True
  elif case == 'no_straight': navi.signalHud.candidates = [candidate('LEFT', 'GREEN')]
  elif case == 'blink': navi.signalHud.candidates[0].state = 'RED_BLINK'
  elif case == 'duplicate': navi.signalHud.candidates = [candidate('STRAIGHT', 'RED'), candidate('STRAIGHT', 'GREEN', index=1)]
  elif case == 'mixed_groups': navi.signalHud.candidates = [candidate('STRAIGHT', 'RED'), candidate('LEFT', 'GREEN', index=1, group=1)]
  result = step(NaviSignalAssist(), 10., navi)
  assert not result.available and not result.confirmed and not result.hold


@pytest.mark.parametrize('changes', [dict(mode=0), dict(mode=1), dict(mode=2), dict(valid=False),
  dict(enabled=False), dict(drive=False), dict(turning=True), dict(gas=True),
  dict(received_s=8.9), dict(received_s=10.01), dict(speed=float('nan'))])
def test_vehicle_mode_and_service_context_gates(changes):
  result = step(NaviSignalAssist(), 10., packet(10.), **changes)
  assert not result.available and not result.hold


@pytest.mark.parametrize('age,available', [(0., True), (1., True), (1.000001, False)])
def test_service_receipt_age_boundary(age, available):
  result = step(NaviSignalAssist(), 10., packet(10.), received_s=10. - age)
  assert result.available is available


def test_source_age_boundary_is_independent_of_fresh_service_publication():
  helper = NaviSignalAssist()
  first = packet(10.)
  assert step(helper, 10., first).available
  assert step(helper, 11.5, first, received_s=11.5).available
  assert not step(helper, 11.500001, first, received_s=11.500001).available


def test_republishing_the_same_observation_does_not_manufacture_votes_or_freshness():
  helper = NaviSignalAssist()
  stamp = BASE_WALL_MS + 10_000
  for now, sequence in [(10., 1), (10.3, 2), (11., 3)]:
    result = step(helper, now, packet(now, sequence=sequence, source_stamp=stamp))
    assert result.available and not result.confirmed and not result.hold
  result = step(helper, 11.6, packet(11.6, sequence=4, source_stamp=stamp))
  assert not result.available and not result.confirmed and not result.hold


def test_repeated_service_frame_does_not_count_as_multiple_red_observations():
  helper = NaviSignalAssist()
  navi = packet(10.)
  for now in (10., 10.1, 10.3, 10.7):
    result = step(helper, now, navi)
    assert result.available and not result.confirmed and not result.hold


def test_real_publication_cadence_does_not_clear_confirmation_between_service_updates():
  helper = NaviSignalAssist()
  releases = 0
  for frame in range(101):
    now = 10. + frame * .05
    source = 10. + frame // 20  # SDK reports one new observation per second.
    service = 10. + (frame // 10) * .5  # Cereal publication is 2 Hz.
    state = 'RED' if source < 13. else 'GREEN'
    result = step(helper, now, packet(source, state), received_s=service)
    assert result.available, (frame, result)
    if 20 <= frame < 60:
      assert result.confirmed and result.hold
    if 60 <= frame < 80:
      assert result.hold and not result.released
    if frame >= 80:
      assert result.confirmed and not result.hold
    releases += result.released
  assert releases == 1


def test_red_latch_release_uses_distinct_green_observations_and_only_its_own_key():
  helper, first = red_hold()
  assert first.warn
  assert not step(helper, 10.35, packet(10.3)).warn
  green = step(helper, 10.4, packet(10.4, 'GREEN'))
  assert green.available and green.hold and not green.released and not green.confirmed
  green = step(helper, 11.2, packet(11.2, 'GREEN'))
  assert green.confirmed and green.released and not green.hold
  assert not hasattr(green, 'green_confirmed')  # No acceleration permission field.


@pytest.mark.parametrize('loss', ['missing', 'invalid', 'stale', 'future', 'other_session', 'reordered'])
def test_source_loss_or_new_source_cannot_release_a_latched_red(loss):
  helper, _ = red_hold()
  now, navi, changes = 10.5, packet(10.5, 'GREEN'), {}
  if loss == 'missing': navi = None
  elif loss == 'invalid': changes['valid'] = False
  elif loss == 'stale': navi = packet(8., 'GREEN')
  elif loss == 'future': navi = packet(10.6, 'GREEN')
  elif loss == 'other_session':
    for t in (10.5, 11.3): result = step(helper, t, packet(t, 'GREEN', session='new'))
    assert result.available and result.confirmed and result.hold and not result.released
    return
  elif loss == 'reordered': navi = packet(10.2, 'GREEN')
  result = step(helper, now, navi, **changes)
  assert result.hold and not result.released


def test_mode_reentry_requires_a_post_enable_observation():
  helper, _ = red_hold()
  assert not step(helper, 10.4, packet(10.4), mode=2).hold
  result = step(helper, 10.5, packet(10.4))
  assert not result.available and not result.hold
  result = step(helper, 10.6, packet(10.6))
  assert result.available and not result.confirmed


@pytest.mark.parametrize('condition', ['gas', 'departure', 'mode_off'])
def test_only_explicit_override_or_departure_retires_stop_memory(condition):
  helper, _ = red_hold()
  changes = {'gas': True} if condition == 'gas' else {'speed': 5 / 3.6 + .001} if condition == 'departure' else {'mode': 0}
  result = step(helper, 10.4, None, **changes)
  assert not result.hold
  if condition == 'gas':
    assert step(helper, 11., packet(11.)).reason == 'driver_cooldown'


def test_brake_does_not_turn_an_existing_red_latch_into_release_permission():
  helper, _ = red_hold()
  result = step(helper, 10.4, packet(10.4), brake=True)
  assert result.hold and not result.warn and not result.released


def test_native_schema_roundtrip_preserves_provenance_and_observation_order():
  navi = packet(10., candidates=[candidate('LEFT', 'GREEN', index=1), candidate('STRAIGHT', 'RED')])
  event = log.Event.new_message()
  event.carrotNavi = navi
  with log.Event.from_bytes(event.to_bytes()) as decoded:
    result = step(NaviSignalAssist(), 10., decoded.carrotNavi)
    assert result.available and result.color == 'red'


@pytest.mark.parametrize('direction,left,color', [
  ('STRAIGHT', 'GREEN', 'red'), ('LEFT', 'GREEN', 'green'),
  ('LEFT', 'RED', 'red'), ('LEFT', None, 'red'),
])
def test_apk_json_through_production_payload_and_native_cereal_selects_the_movement(direction, left, color):
  from openpilot.selfdrive.carrot.carrot_navi_cereal import build_carrot_navi_payload

  helper = NaviSignalAssist()
  for now in (10., 11.):
    signals = [dict(direction='STRAIGHT', state='RED' if left else 'GREEN', group_index=0, signal_index=0)]
    if left:
      signals.append(dict(direction='LEFT', state=left, group_index=0, signal_index=1))

    def record(value):
      return dict(present=True, value=value, sequence=round(now * 1000),
                  source_timestamp_ms=BASE_WALL_MS + round(now * 1000), received_mono_ns=round(now * 1e9))

    snapshot = dict(connected=True, session_id='actual-custom-payload', generation=1, items={
      'traffic_signal': record(dict(schema_version=1, source=SOURCE, vehicle_control_allowed=False,
                                   direction_match_verified=False, signals=signals)),
      'navigation_status': record(dict(mode='guidance', guidance_active=True, off_route=False)),
    })
    event = log.Event.new_message()
    event.init('carrotNavi')
    event.carrotNavi = build_carrot_navi_payload(snapshot, publish_mono_ns=round(now * 1e9))
    with log.Event.from_bytes(event.to_bytes()) as decoded:
      result = step(helper, now, decoded.carrotNavi, direction=direction)
  assert result.available and result.confirmed and result.color == color
  assert result.hold is (color == 'red')


@pytest.mark.parametrize('category,valid_category,suspended', [(0, True, True), (1, True, True),
  (0, False, False), (8, True, False)])
def test_explicit_motorway_guard_does_not_confuse_unknown_zero_with_a_motorway(category, valid_category, suspended):
  navi = packet(10.)
  navi.laneCurrent.roadCategory = category
  navi.laneCurrent.roadCategoryValid = valid_category
  result = step(NaviSignalAssist(), 10., navi)
  assert result.available is (not suspended)


def test_same_wire_sequence_cannot_carry_a_new_source_observation():
  helper = NaviSignalAssist()
  assert step(helper, 10., packet(10., sequence=1)).available
  result = step(helper, 10.3, packet(10.3, sequence=1))
  assert not result.available and not result.confirmed and not result.hold


def test_nonfinite_clock_on_mode_entry_still_requires_post_enable_data_after_recovery():
  helper = NaviSignalAssist()
  assert not step(helper, float('nan'), None).available
  result = step(helper, 10., packet(9.9))
  assert not result.available and not result.hold
  assert step(helper, 10.1, packet(10.1)).available


class RuntimeInputs(dict):
  def __init__(self, receipt=10., *, valid=True):
    super().__init__(carrotNavi=packet(receipt))
    self.seen = self.alive = self.freq_ok = {'carrotNavi': True}
    self.valid = {'carrotNavi': valid}
    self.recv_time = {'carrotNavi': receipt}
    self.updates = []

  def update(self, timeout):
    self.updates.append(timeout)


def test_optional_runtime_is_lazy_and_never_subscribes_outside_mode_three():
  calls = []
  sm = RuntimeInputs()
  runtime = NaviSignalRuntime(sm_factory=lambda: calls.append('created') or sm)
  assert runtime.read(2, 10.)[:3] == (None, False, 0.)
  assert not calls
  navi, valid, receipt, now = runtime.read(3, 10.)
  assert navi is sm['carrotNavi'] and valid and receipt == now == 10.
  assert calls == ['created'] and sm.updates == [0]
  assert runtime.read(0, 10.1)[:3] == (None, False, 0.)
  assert runtime.sm is None


def test_optional_runtime_missing_service_returns_unavailable():
  runtime = NaviSignalRuntime(sm_factory=lambda: {})
  navi, valid, receipt, now = runtime.read(3, 10.)
  assert navi is None and not valid and receipt == 0. and now == 10.


def test_fresh_optional_service_is_usable_before_frequency_tracker_warms_up():
  sm = RuntimeInputs()
  sm.freq_ok = {'carrotNavi': False}
  navi, valid, receipt, now = NaviSignalRuntime(sm_factory=lambda: sm).read(3, 10.)
  assert navi is sm['carrotNavi'] and valid and receipt == now == 10.
