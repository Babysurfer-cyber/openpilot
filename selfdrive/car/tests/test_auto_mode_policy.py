"""Auto-mode policies through real button handling and the full cruise update."""

import pytest

from cereal import car, custom, log
from openpilot.selfdrive.car.tests.test_can_cruise_recovery import integrated_drive


@pytest.fixture
def auto_drive(integrated_drive, monkeypatch):
  drive = integrated_drive
  helper = drive.helper
  drive.settings = {'MyDrivingMode': 5}
  original_get_int = helper.params.get_int
  monkeypatch.setattr(helper.params, 'get_int',
                      lambda key: drive.settings.get(key, original_get_int(key)))
  helper.update_params(True)
  helper.current_driving_mode = 5
  helper.frame = 1001
  helper.enabled_last = True
  drive.state.navDataValid = drive.state.speedLimitDataValid = True
  # Establish an ongoing mode-5 session using the public update path. Individual
  # cases can then seed a prior driver set speed without testing mode entry too.
  configure(drive, speed=110., pure=100., mixed=100.)
  step(drive, .5)
  return drive


def configure(drive, *, speed=100., pure=100., mixed=50., prior=100., offset=10.):
  helper = drive.helper
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = speed
  helper.auto_prev_limit = prior
  helper.auto_speed_offset = offset
  helper.nRoadLimitSpeed = pure
  drive.state.navSpeedLimit = pure
  drive.state.speedLimit = mixed
  drive.state.navLinkClass = 1
  drive.state.navTollExist = 0
  drive.state.mapSource = 0
  return helper


def step(drive, now):
  drive.step(now, full_update=True)
  assert 'Auto Mode Error:' not in drive.helper.log
  return float(drive.helper.v_cruise_kph)


def tap(drive, now, button='accelCruise'):
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type=button, pressed=True)]
  step(drive, now)
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type=button, pressed=False)]
  speed = step(drive, now + .01)
  drive.state.buttonEvents = []
  return speed


@pytest.mark.parametrize('link_class', (0, 1))
def test_automatic_drop_below_90_keeps_same_road_source(auto_drive, link_class):
  drive = auto_drive
  configure(drive, speed=100., pure=70., mixed=50., prior=100.)
  drive.state.navLinkClass = link_class
  assert step(drive, 1.) == 85.
  for index in range(1, 30):
    assert step(drive, 1. + index * .01) == 85.
  assert drive.helper.auto_prev_limit == 70.


def test_cancel_press_and_hold_block_auto_changes_before_enabled_feedback(auto_drive):
  drive = auto_drive
  configure(drive, speed=50., pure=100., prior=60.)
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='cancel', pressed=True)]
  assert step(drive, 1.) == 50.
  drive.state.buttonEvents = []
  for index in range(1, 5):
    assert step(drive, 1. + index * .01) == 50.
  assert drive.control.enabled


def test_stale_navigation_cannot_revive_an_old_limit_after_reconnecting_with_zero(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=100., pure=100., prior=100.)
  assert step(drive, 1.) == 100.
  drive.state.navDataValid = False
  assert step(drive, 1.01) == 100.
  drive.state.navDataValid = True
  drive.state.navSpeedLimit = 0.
  helper.auto_prev_limit = 80.
  assert step(drive, 1.02) == 100.
  assert helper.auto_prev_limit == 80.


def test_ordinary_right_lane_change_does_not_authorize_mixed_mainline_limit(auto_drive):
  drive = auto_drive
  configure(drive)
  assert step(drive, 1.) == 100.
  drive.state.rightBlinker = True
  for index in range(1, 20):
    assert step(drive, 1. + index * .01) == 100.
  drive.state.rightBlinker = False
  assert step(drive, 1.21) == 100.
  assert drive.helper.auto_prev_limit == 100.


def test_confirmed_ic_or_jc_still_allows_mixed_limit(auto_drive):
  drive = auto_drive
  configure(drive)
  drive.state.navLinkClass = 2
  assert step(drive, 1.) == 75.
  assert drive.helper.auto_prev_limit == 50.
  assert step(drive, 1.01) == 75.


def test_missing_pure_limit_never_falls_back_to_adjacent_road_on_mainline(auto_drive):
  drive = auto_drive
  configure(drive, pure=0., mixed=30.)
  assert step(drive, 1.) == 100.
  assert step(drive, 1.01) == 100.


def test_toll_reduction_has_priority_and_restores_without_manual_input(auto_drive):
  drive = auto_drive
  helper = configure(drive)
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 80.
  for index in range(1, 20):
    assert step(drive, 1. + index * .01) == 80.
  assert not helper.tollgate_speed_changed
  drive.state.navTollExist = 0
  assert step(drive, 1.21) == 100.


def test_real_speed_button_during_toll_suppresses_automatic_restore(auto_drive):
  drive = auto_drive
  helper = configure(drive)
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 80.
  manual_speed = tap(drive, 1.1)
  assert manual_speed > 80.
  assert helper.tollgate_speed_changed
  drive.state.navTollExist = 0
  assert step(drive, 1.2) == manual_speed


def test_toll_entry_never_increases_an_already_low_set_speed(auto_drive):
  drive = auto_drive
  configure(drive, speed=20., pure=20., mixed=20., prior=20.)
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 20.


def test_automatic_toll_restore_obeys_110_cap(auto_drive):
  drive = auto_drive
  configure(drive, speed=120., mixed=100.)
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 95.
  assert step(drive, 1.01) == 95.
  drive.state.navTollExist = 0
  assert step(drive, 1.02) == 110.


def test_other_automatic_acceleration_cannot_override_mode5_target(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=120., pure=100., mixed=100., prior=80.)
  helper.autoSpeedUptoRoadSpeedLimit = 1.2
  drive.sm.data['radarState'].leadOne = dict(status=True, dRel=20., vLeadK=36.)
  assert step(drive, 1.) == 110.
  for index in range(1, 20):
    assert step(drive, 1. + index * .01) == 110.


def test_model_speed_acceleration_cannot_override_mode5_target(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=120., pure=120., mixed=120., prior=80.)
  helper.applyModelSpeed = 1.
  helper.model_v_kph = 150.
  assert step(drive, 1.) == 110.
  assert step(drive, 1.01) == 110.


def test_manual_camera_release_lasts_until_the_warning_session_ends(auto_drive):
  drive = auto_drive
  helper = configure(drive, pure=80., mixed=80., prior=80.)
  drive.state.mapSource = 2
  assert step(drive, 1.) == 100.
  assert helper.camera_pending
  manual_speed = tap(drive, 1.1)
  assert manual_speed == 101.
  for index in range(1, 20):
    assert step(drive, 1.12 + index * .01) == manual_speed
    assert not helper.camera_pending
  drive.state.mapSource = 0
  step(drive, 1.4)
  drive.state.mapSource = 2
  step(drive, 1.41)
  assert helper.camera_pending


def test_manual_offset_uses_actual_result_after_button_mode_mapping(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=100., pure=100., mixed=100., prior=100.)
  drive.settings['CruiseButtonMode'] = 2
  helper._cruise_button_mode = 2
  assert tap(drive, 1.) == 110.
  assert helper.auto_speed_offset == 10.


def test_manual_set_speed_above_110_is_not_overwritten_without_an_auto_event(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=110., pure=100., mixed=100., prior=100.)
  drive.settings['CruiseButtonMode'] = 2
  helper._cruise_button_mode = 2
  assert tap(drive, 1.) == 120.
  assert step(drive, 1.1) == 120.
  drive.state.navSpeedLimit = 90.
  assert step(drive, 1.11) <= 110.


def test_invalid_can_does_not_consume_a_new_road_limit(auto_drive):
  drive = auto_drive
  helper = configure(drive, pure=70., mixed=50., prior=100.)
  drive.state.canValid = False
  assert step(drive, 1.) == 100.
  assert helper.auto_prev_limit == 100.
  drive.state.canValid = True
  assert step(drive, 1.01) == 85.


def test_synthetic_carrot_restore_resynchronizes_without_learning_an_offset(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=50., pure=60., mixed=60., prior=100., offset=0.)
  helper.carrot_cruise_active = helper.prev_carrot_active = True
  helper._v_cruise_kph_at_brake = 100.
  helper.carrot_cruise_min_timer = 0
  helper.desiredSource = 'VTRUN'
  helper.desiredSpeed = 55.
  drive.state.vEgo = drive.state.vEgoCluster = 15.
  assert step(drive, 1.) == 60.
  assert not helper.carrot_cruise_active
  assert helper.auto_speed_offset == 0.
  assert helper.auto_prev_limit == 60.


@pytest.mark.parametrize('valid_field', ('navDataValid', 'speedLimitDataValid'))
def test_unknown_navigation_data_is_not_consumed_as_a_new_mixed_limit(auto_drive, valid_field):
  drive = auto_drive
  helper = configure(drive, pure=70., mixed=50., prior=100.)
  drive.state.navLinkClass = 2
  setattr(drive.state, valid_field, False)
  assert step(drive, 1.) == 100.
  assert helper.auto_prev_limit == 100.
  setattr(drive.state, valid_field, True)
  assert step(drive, 1.01) == 75.


def test_mainline_pure_limit_does_not_require_unrelated_mixed_data(auto_drive):
  drive = auto_drive
  configure(drive, pure=70., mixed=50., prior=100.)
  drive.state.speedLimitDataValid = False
  assert step(drive, 1.) == 85.


def test_disabled_cruise_defers_new_limit_until_engagement(auto_drive):
  drive = auto_drive
  helper = configure(drive, pure=70., mixed=50., prior=100.)
  drive.control.enabled = False
  assert step(drive, 1.) == 100.
  assert helper.auto_prev_limit != 70.
  drive.control.enabled = True
  assert step(drive, 1.01) == 85.
  assert helper.auto_prev_limit == 70.


def test_mode_reentry_resynchronizes_even_when_the_road_limit_did_not_change(auto_drive):
  drive = auto_drive
  helper = configure(drive)
  assert step(drive, 1.) == 100.
  drive.settings['MyDrivingMode'] = helper.current_driving_mode = 3
  step(drive, 1.01)
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 70.
  drive.settings['MyDrivingMode'] = helper.current_driving_mode = 5
  assert step(drive, 1.02) == 110.


def test_manual_toll_entry_frame_takes_priority_and_disables_later_restore(auto_drive):
  drive = auto_drive
  helper = configure(drive)
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='accelCruise', pressed=True)]
  assert step(drive, 1.) == 100.
  drive.state.navTollExist = 1
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='accelCruise', pressed=False)]
  manual_speed = step(drive, 1.01)
  assert manual_speed == 101.
  drive.state.buttonEvents = []
  assert step(drive, 1.02) == manual_speed
  drive.state.navTollExist = 0
  assert step(drive, 1.03) == manual_speed
  assert helper.auto_speed_offset <= 10.


def test_explicit_remote_speed_during_toll_cancels_restore_without_offset_learning(auto_drive):
  drive = auto_drive
  helper = configure(drive, offset=0.)
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 80.
  drive.sm.data['carrotMan'] = custom.CarrotMan(
    nRoadLimitSpeed=100, desiredSpeed=250, carrotCmdIndex=1,
    carrotCmd='SPEED', carrotArg='75',
  )
  drive.sm.alive['carrotMan'] = True
  assert step(drive, 1.1) == 75.
  assert helper.tollgate_speed_changed
  assert helper.auto_speed_offset == 0.
  drive.state.navTollExist = 0
  assert step(drive, 1.2) == 75.


def test_lfa_coasting_restore_resynchronizes_without_learning_an_offset(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=50., pure=60., mixed=60., prior=100., offset=0.)
  drive.settings['LfaButtonMode'] = helper._lfa_button_mode = 2
  helper.carrot_cruise_active = helper.prev_carrot_active = True
  helper._v_cruise_kph_at_brake = 100.
  helper.carrot_cruise_min_timer = 500
  helper.desiredSource = ''
  drive.state.vEgo = drive.state.vEgoCluster = 15.
  assert tap(drive, 1., 'lfaButton') == 60.
  assert not helper.carrot_cruise_active
  assert helper.auto_speed_offset == 0.


def test_button_at_engagement_does_not_overwrite_the_learned_offset(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=50., pure=60., mixed=60., prior=60., offset=7.)
  drive.control.enabled = False
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='accelCruise', pressed=True)]
  step(drive, 1.)
  drive.control.enabled = True
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='accelCruise', pressed=False)]
  step(drive, 1.01)
  assert helper.auto_speed_offset == 7.


def test_can_recovery_preserves_trusted_manual_speed_through_reengagement(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=110., pure=100., mixed=100., prior=100.)
  drive.settings['CruiseButtonMode'] = helper._cruise_button_mode = 2
  assert tap(drive, 1.) == 120.
  # A button shortly before the fault correctly inhibits automatic recovery;
  # this gap makes the fault a later, independently recoverable interruption.
  assert step(drive, 1.6) == 120.
  can_error = log.OnroadEvent(name='canError', noEntry=True, immediateDisable=True)
  assert drive.step(1.7, enabled=False, can_valid=False, events=[can_error],
                    engageable=False, full_update=True) == 0
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 60.
  assert drive.step(1.9, can_valid=True, engageable=True, full_update=True) == 0
  assert drive.step(2.101, full_update=True) == 1
  assert helper.v_cruise_kph == 120.
  assert drive.step(2.11, enabled=True, full_update=True) == 0
  assert helper.v_cruise_kph == 120.


def test_coast_restore_requested_during_toll_finishes_sync_after_toll_exit(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=50., pure=100., mixed=100.)
  helper.carrot_cruise_active = helper.prev_carrot_active = True
  helper._v_cruise_kph_at_brake = 100.
  helper.carrot_cruise_min_timer = 500
  helper.desiredSource = ''
  drive.state.vEgo = drive.state.vEgoCluster = 55. / 3.6
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 40.
  assert helper.v_cruise_before_tollgate == 50.

  helper.desiredSource = 'VTRUN'
  helper.desiredSpeed = 55.
  helper.carrot_cruise_min_timer = 0
  assert step(drive, 1.01) == 40.
  assert not helper.carrot_cruise_active
  assert helper.auto_sync_speed == 40.
  assert helper.auto_speed_offset == 10.

  drive.state.navTollExist = 0
  assert step(drive, 1.02) == 110.
  assert helper.auto_sync_speed is None
  assert not helper.auto_mode_needs_sync


def test_explicit_remote_go_during_toll_cancels_restore_without_learning(auto_drive):
  drive = auto_drive
  helper = configure(drive, mixed=100., offset=3.)
  drive.settings['CruiseButtonMode'] = helper._cruise_button_mode = 2
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 80.
  drive.sm.data['carrotMan'] = custom.CarrotMan(
    nRoadLimitSpeed=100, desiredSpeed=250, carrotCmdIndex=1,
    carrotCmd='CRUISE', carrotArg='GO',
  )
  drive.sm.alive['carrotMan'] = True
  assert step(drive, 1.1) == 90.
  assert helper.tollgate_speed_changed
  assert helper.auto_speed_offset == 3.
  drive.state.navTollExist = 0
  assert step(drive, 1.2) == 90.


def test_manual_camera_override_survives_navigation_loss_and_recovery(auto_drive):
  drive = auto_drive
  helper = configure(drive, pure=80., mixed=80., prior=80.)
  drive.state.mapSource = 2
  assert step(drive, 1.) == 100.
  drive.state.navDataValid = False
  assert tap(drive, 1.1) == 101.
  assert helper.camera_pending
  drive.state.navDataValid = True
  assert step(drive, 1.2) == 101.
  assert not helper.camera_pending


def test_navigation_limit_memory_expires_while_navigation_is_unknown(auto_drive):
  drive = auto_drive
  helper = configure(drive, pure=80., mixed=50., prior=100.)
  drive.state.mapSource = 2
  assert step(drive, 1.) == 100.
  assert helper.memorized_nav_limit == 80.
  drive.state.navSpeedLimit = 0.
  drive.state.navDataValid = False
  for index in range(1, 102):
    assert step(drive, 1. + index * .01) == 100.
  assert helper.nav_limit_timer == 0
  drive.state.navDataValid = True
  drive.state.mapSource = 0
  assert step(drive, 2.1) == 100.
  assert helper.auto_prev_limit == 100.


def test_cancel_release_cannot_apply_a_new_automatic_limit(auto_drive):
  drive = auto_drive
  helper = configure(drive)
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='cancel', pressed=True)]
  assert step(drive, 1.) == 100.
  drive.state.navSpeedLimit = 70.
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='cancel', pressed=False)]
  assert step(drive, 1.01) == 100.
  assert helper._cruise_cancel_state
  assert helper.auto_prev_limit == 0.


def test_mode_reentry_while_cruise_enabled_does_not_restart_offset_learning_delay(auto_drive):
  drive = auto_drive
  helper = configure(drive, offset=3.)
  drive.settings['MyDrivingMode'] = helper.current_driving_mode = 3
  step(drive, 1.)
  drive.settings['MyDrivingMode'] = helper.current_driving_mode = 5
  step(drive, 1.01)
  manual_speed = tap(drive, 1.1)
  assert helper.auto_speed_offset == min(10., max(0., manual_speed - 100.))
  assert helper.auto_speed_offset != 3.


def test_can_recovery_during_toll_does_not_apply_the_reduction_twice(auto_drive):
  drive = auto_drive
  helper = configure(drive, mixed=100.)
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 80.
  can_error = log.OnroadEvent(name='canError', noEntry=True, immediateDisable=True)
  assert drive.step(1.1, enabled=False, can_valid=False, events=[can_error],
                    engageable=False, full_update=True) == 0
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 60.
  assert drive.step(1.3, can_valid=True, engageable=True, full_update=True) == 0
  assert drive.step(1.501, full_update=True) == 1
  assert helper.v_cruise_kph == 80.
  assert drive.step(1.51, enabled=True, full_update=True) == 0
  assert helper.v_cruise_kph == 80.
  assert step(drive, 1.52) == 80.


def test_new_pure_limit_at_toll_exit_takes_priority_over_the_old_backup(auto_drive):
  drive = auto_drive
  helper = configure(drive, pure=100., mixed=50.)
  drive.state.navTollExist = 1
  assert step(drive, 1.) == 80.
  drive.state.navSpeedLimit = 60.
  drive.state.navTollExist = 0
  assert step(drive, 1.01) == 70.
  assert helper.auto_prev_limit == 60.
  assert step(drive, 1.02) == 70.


@pytest.mark.parametrize('braking_field', ('brakePressed', 'regenBraking'))
def test_braking_blocks_auto_speed_change_before_enabled_state_catches_up(auto_drive, braking_field):
  drive = auto_drive
  configure(drive, speed=50., pure=100., mixed=100., prior=60.)
  setattr(drive.state, braking_field, True)
  assert drive.control.enabled
  assert step(drive, 1.) == 50.


def test_low_speed_mainline_classification_survives_later_unknown_road_class(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=40., pure=100., mixed=50., prior=60.)
  assert step(drive, 1.) == 110.
  drive.state.navLinkClass = 0
  assert step(drive, 1.01) == 110.
  assert helper.auto_prev_limit == 100.


def test_ic_mixed_source_survives_unknown_class_even_after_a_high_auto_target(auto_drive):
  drive = auto_drive
  helper = configure(drive, speed=110., pure=70., mixed=100., prior=80.)
  drive.state.navLinkClass = 2
  assert step(drive, 1.) == 110.
  drive.state.navLinkClass = 0
  assert step(drive, 1.01) == 110.
  assert helper.auto_prev_limit == 100.


def test_300m_route_congestion_uses_auto_mode_notice_once(auto_drive, monkeypatch):
  """Confirmed jam uses the existing AUTO chime event without announcement text."""
  drive = auto_drive
  helper = configure(drive, speed=110., pure=100., mixed=100., prior=100.)
  drive.state.vEgo = drive.state.vEgoCluster = 25.  # 90 km/h
  chimes = []
  monkeypatch.setattr(helper, '_auto_mode_chime', lambda: chimes.append(True))

  nav = custom.CarrotNaviState.new_message(
    schemaVersion=1, sessionId='test-congestion-session', connected=True,
    navigationStatus={
      'meta': {'present': True, 'sequence': 1, 'receivedMonoTimeNanos': 1_000_000_000},
      'guidanceActive': True, 'offRoute': False, 'routePresent': True,
    },
    route={
      'meta': {'present': True, 'sequence': 1, 'receivedMonoTimeNanos': 1_000_000_000},
      'totalDistanceM': 20_000, 'remainingDistanceM': 11_300, 'trafficPresent': True,
      'trafficSections': [
        {'startDistanceM': 0, 'lengthM': 9_000, 'congestionCode': 1},
        {'startDistanceM': 9_000, 'lengthM': 5_000, 'congestionCode': 3},
        {'startDistanceM': 14_000, 'lengthM': 6_000, 'congestionCode': 1},
      ],
    },
  )
  drive.sm.data['carrotNavi'] = nav
  drive.sm.seen = {'carrotNavi': True}
  drive.sm.alive['carrotNavi'] = drive.sm.valid['carrotNavi'] = True

  assert step(drive, 1.0) > 80.
  assert not chimes

  nav.route.meta.sequence = 2
  nav.route.meta.receivedMonoTimeNanos = 1_100_000_000
  assert step(drive, 1.1) == 80.
  assert len(chimes) == 1  # No new text or announcement
  assert helper.log == 'Auto congestion cap: 80 km/h'

  nav.route.meta.sequence = 3
  nav.route.meta.receivedMonoTimeNanos = 1_200_000_000
  assert step(drive, 1.2) == 80.
  assert len(chimes) == 1  # No repeated chime or AUTO blink.



def _capture_mode5_alerts(helper, monkeypatch):
  """Spy on the existing prompt/chime pathways without filesystem side effects."""
  alerts = []

  def prompt(message):
    helper._auto_mode_notification_sent = True
    alerts.append(('prompt', message))

  def chime():
    helper._auto_mode_notification_sent = True
    alerts.append(('chime', None))

  monkeypatch.setattr(helper, '_auto_mode_prompt', prompt)
  monkeypatch.setattr(helper, '_auto_mode_chime', chime)
  return alerts


def test_auto_mode_chimes_once_on_toll_exit(auto_drive, monkeypatch):
  drive = auto_drive
  helper = configure(drive, speed=120., pure=100., mixed=100., prior=100.)
  alerts = _capture_mode5_alerts(helper, monkeypatch)

  drive.state.navTollExist = 1
  assert step(drive, 1.0) == 95.
  assert [kind for kind, _ in alerts] == ['prompt']  # Existing entry beep.

  assert step(drive, 1.01) == 95.
  assert len(alerts) == 1

  drive.state.navTollExist = 0
  assert step(drive, 1.02) == 110.
  assert [kind for kind, _ in alerts] == ['prompt', 'chime']  # Added exit beep.
  assert helper.log.startswith('Auto target change:')  # HUD AUTO blink.

  assert step(drive, 1.03) == 110.
  assert len(alerts) == 2  # No repeat at steady speed.


def test_mode5_camera_release_uses_existing_beep_once(auto_drive, monkeypatch):
  drive = auto_drive
  helper = configure(drive, speed=110., pure=80., mixed=50., prior=100.)
  alerts = _capture_mode5_alerts(helper, monkeypatch)
  drive.state.mapSource = 2
  drive.state.navSectionValid = True
  drive.state.navSectionActive = False

  assert step(drive, 1.0) == 110.
  assert not alerts  # Camera approach holds the current target.

  drive.state.navSectionActive = True
  assert step(drive, 1.01) == 95.
  assert [kind for kind, _ in alerts] == ['prompt']
  assert step(drive, 1.02) == 95.
  assert len(alerts) == 1  # The fallback must not play a second chime.


def test_manual_speed_change_does_not_fire_auto_mode_beep(auto_drive, monkeypatch):
  drive = auto_drive
  helper = configure(drive, speed=100., pure=100., mixed=100., prior=100.)
  alerts = _capture_mode5_alerts(helper, monkeypatch)
  manual_speed = tap(drive, 1.0)
  assert manual_speed > 100.
  assert not alerts



@pytest.mark.parametrize(('initial_mode', 'next_mode', 'disengage_chimes'), (
  (5, 3, 1),  # AUTO -> NORM plays the requested tici_disengaged.wav
  (3, 5, 0),  # NORM -> AUTO must retain its separate entry chime
  (2, 3, 0),  # Other modes switching to NORM must not play AUTO-off
))
def test_gap_long_press_auto_to_norm_uses_disengage_chime_once(
  auto_drive, monkeypatch, initial_mode, next_mode, disengage_chimes,
):
  drive = auto_drive
  helper = drive.helper
  drive.settings['MyDrivingMode'] = initial_mode
  helper.current_driving_mode = initial_mode

  def persist_mode(name, value):
    if name == 'MyDrivingMode':
      drive.settings[name] = value

  monkeypatch.setattr(helper.params, 'put_int_nonblocking', persist_mode)
  played = []
  monkeypatch.setattr(helper, '_auto_mode_off_chime', lambda: played.append(True))

  # At the gap-button long-press threshold, _prepare_buttons generates only
  # one gapAdjustCruise event, even if the driver continues holding the button.
  helper.button_prev = car.CarState.ButtonEvent.Type.gapAdjustCruise
  helper.button_cnt = helper._cruise_button_long_delay + 30
  helper.long_pressed = False
  step(drive, 1.0)

  assert drive.settings['MyDrivingMode'] == next_mode
  assert helper.current_driving_mode == next_mode
  assert len(played) == disengage_chimes

  for i in range(1, 8):
    step(drive, 1.0 + i * .01)
  assert len(played) == disengage_chimes

  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='gapAdjustCruise', pressed=False)]
  step(drive, 1.1)
  drive.state.buttonEvents = []
  assert len(played) == disengage_chimes  # No duplicate on button release.


def test_gap_short_press_does_not_play_auto_off_chime(auto_drive, monkeypatch):
  drive = auto_drive
  helper = drive.helper
  played = []
  monkeypatch.setattr(helper, '_auto_mode_off_chime', lambda: played.append(True))
  helper.button_prev = car.CarState.ButtonEvent.Type.gapAdjustCruise
  helper.button_cnt = 1
  helper.long_pressed = False
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='gapAdjustCruise', pressed=False)]
  step(drive, 1.0)

  assert helper.current_driving_mode == 5
  assert not played
