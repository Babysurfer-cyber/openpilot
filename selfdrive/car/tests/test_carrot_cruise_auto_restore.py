"""Regression coverage for automatic Carrot Cruise speed restoration.

A coast/Carrot Cruise session may reduce the temporary setpoint. On a normal
automatic exit, mode 5 must not replace the saved driver's set speed with
that lower temporary speed. Active camera, section and tollgate holds retain
their existing precedence.
"""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.cruise import ButtonType, GearShifter, VCruiseCarrot


def make_helper():
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper._auto_mode_active_last = True
  helper.enabled_last = True
  helper.auto_engage_timer = 0
  helper.auto_mode_needs_sync = False
  helper.auto_sync_speed = None
  helper.auto_baseline_pending = False
  helper.auto_prev_limit = 80.0
  helper.auto_can_resume = False
  helper.prev_is_tollgate = False
  helper.auto_source_high = True
  helper.nav_limit_timer = 0
  helper.memorized_nav_limit = 0
  helper.auto_camera_override = False
  helper.camera_pending = False
  helper.button_prev = ButtonType.unknown
  helper.button_cnt = 0
  helper._cruise_cancel_state = False
  helper._activate_cruise = 0
  helper.tollgate_speed_changed = False
  helper.expected_tollgate_speed = 0
  helper.v_cruise_before_tollgate = 0
  helper.tollgate_nav_limit = 0
  helper.carrot_cruise_active = False
  helper._carrot_cruise_saved_op_speed = None
  helper.auto_speed_offset = 0.0
  helper._carrot_restore_deferred_target = None
  helper._carrot_lead_fresh = False
  helper._add_log = lambda _: None
  helper._auto_mode_prompt = lambda _: None
  return helper


def make_inputs(**overrides):
  state = dict(
    gearShifter=GearShifter.drive, cruiseState=SimpleNamespace(available=True),
    brakePressed=False, regenBraking=False, buttonEvents=[], canValid=True,
    navDataValid=True, mapSource=0, navSectionValid=False,
    navSectionActive=False, navSpeedLimit=80, navTollExist=0,
    navLinkClass=1, speedLimit=80, speedLimitDataValid=True,
    gasPressed=False, vEgo=0.0,
  )
  state.update(overrides)
  return SimpleNamespace(**state), SimpleNamespace(enabled=True)


def run_auto(helper, CS, CC, *, backed_up=100, coast=60):
  return helper._update_auto_mode(
    CS, CC, backed_up,
    physical_speed_button=False,
    manual_speed_change=False,
    restore_speed=coast,
    automatic_carrot_restore_target=backed_up,
  )


def test_automatic_exit_restores_previous_setpoint_on_valid_navigation():
  helper, (CS, CC) = make_helper(), make_inputs()
  result = run_auto(helper, CS, CC)
  assert result == pytest.approx(100.0)
  assert helper.auto_sync_speed is None
  assert helper.auto_mode_needs_sync is False
  assert helper.auto_prev_limit == 80


def test_automatic_exit_restores_previous_setpoint_when_navigation_missing():
  helper, (CS, CC) = make_helper(), make_inputs(navDataValid=False)
  result = run_auto(helper, CS, CC)
  assert result == pytest.approx(100.0)
  assert helper.auto_sync_speed is None
  assert helper.auto_baseline_pending


def test_active_camera_hold_is_not_bypassed_by_automatic_restore():
  helper, (CS, CC) = make_helper(), make_inputs(mapSource=2)
  result = run_auto(helper, CS, CC)
  assert result == pytest.approx(60.0)
  assert helper.camera_pending is True
  assert helper.auto_sync_speed == 60


def test_active_section_control_does_not_force_old_setpoint():
  helper, (CS, CC) = make_helper(), make_inputs(
    mapSource=2, navSectionValid=True, navSectionActive=True,
  )
  result = run_auto(helper, CS, CC)
  assert result < 100


def test_tollgate_hold_keeps_reduced_target():
  helper, (CS, CC) = make_helper(), make_inputs(navTollExist=1)
  result = run_auto(helper, CS, CC)
  assert result < 100
  assert helper.prev_is_tollgate is True


def test_auto_mode_without_restore_keeps_normal_limit_sync():
  helper, (CS, CC) = make_helper(), make_inputs()
  result = helper._update_auto_mode(
    CS, CC, 100,
    physical_speed_button=False,
    manual_speed_change=False,
    restore_speed=60,
  )
  assert result == pytest.approx(80.0)

def coast_frame(helper, CS, CC, speed=60):
  return helper._update_auto_mode(
    CS, CC, speed,
    physical_speed_button=False, manual_speed_change=False,
    restore_speed=None,
  )


def test_auto_mode_speed_limit_reduction_is_remembered_during_coast():
  helper, (CS, CC) = make_helper(), make_inputs()
  helper._v_cruise_kph_at_brake = 100
  helper._carrot_cruise_saved_op_speed = 100
  helper.carrot_cruise_active = True

  # At first the normal auto target remains 100, while coast is limited to 60.
  assert coast_frame(helper, CS, CC, 60) == pytest.approx(60)
  assert helper._carrot_auto_shadow_target == pytest.approx(100)

  # A verified 80->60 road-limit change would make mode 5 choose 80.
  CS.navSpeedLimit = 60
  assert coast_frame(helper, CS, CC, 55) == pytest.approx(55)
  assert helper._carrot_auto_shadow_target == pytest.approx(80)
  assert helper.auto_prev_limit == pytest.approx(60)

  # A normal automatic exit must use 80, not old 100 or current 55.
  helper.carrot_cruise_active = False
  assert run_auto(helper, CS, CC, backed_up=100, coast=55) == pytest.approx(80)
  assert helper._carrot_auto_shadow_target is None


def test_auto_mode_speed_limit_increase_is_remembered_during_coast():
  helper, (CS, CC) = make_helper(), make_inputs()
  helper._v_cruise_kph_at_brake = 100
  helper.auto_speed_offset = 10
  helper._carrot_cruise_saved_op_speed = 100
  helper.carrot_cruise_active = True
  assert coast_frame(helper, CS, CC) == pytest.approx(60)

  # For this configured +10 offset, mode 5 would raise 100->110.
  CS.navSpeedLimit = 100
  assert coast_frame(helper, CS, CC, 60) == pytest.approx(60)
  assert helper._carrot_auto_shadow_target == pytest.approx(110)

  helper.carrot_cruise_active = False
  assert run_auto(helper, CS, CC, backed_up=100, coast=60) == pytest.approx(110)


def test_camera_pending_during_coast_still_holds_lower_restore_speed():
  helper, (CS, CC) = make_helper(), make_inputs(mapSource=2)
  helper._v_cruise_kph_at_brake = 100
  helper._carrot_cruise_saved_op_speed = 100
  helper.carrot_cruise_active = True
  assert coast_frame(helper, CS, CC) == pytest.approx(60)
  assert helper.camera_pending
  assert helper._carrot_auto_shadow_target == pytest.approx(100)

  helper.carrot_cruise_active = False
  assert run_auto(helper, CS, CC, backed_up=100, coast=60) == pytest.approx(60)


def test_tollgate_transition_is_tracked_in_shadow_not_by_coasting_speed():
  helper, (CS, CC) = make_helper(), make_inputs(navTollExist=1)
  helper._v_cruise_kph_at_brake = 100
  helper._carrot_cruise_saved_op_speed = 100
  helper.carrot_cruise_active = True
  assert coast_frame(helper, CS, CC, 60) == pytest.approx(60)
  assert helper._carrot_auto_shadow_target == pytest.approx(80)
  assert helper.prev_is_tollgate

  CS.navTollExist = 0
  assert coast_frame(helper, CS, CC, 55) == pytest.approx(55)
  assert helper._carrot_auto_shadow_target == pytest.approx(100)
  assert not helper.prev_is_tollgate

  helper.carrot_cruise_active = False
  assert run_auto(helper, CS, CC, backed_up=100, coast=55) == pytest.approx(100)


def test_explicit_set_speed_discards_shadow_target():
  helper, (CS, CC) = make_helper(), make_inputs()
  helper._carrot_auto_shadow_target = 100
  helper.carrot_cruise_active = False
  result = helper._update_auto_mode(
    CS, CC, 55, physical_speed_button=True, manual_speed_change=True,
    restore_speed=None,
  )
  assert result == pytest.approx(55)
  assert helper._carrot_auto_shadow_target is None


def test_new_limit_on_exact_exit_frame_still_uses_auto_mode_calculation():
  helper, (CS, CC) = make_helper(), make_inputs(navSpeedLimit=60)
  helper._carrot_auto_shadow_target = 100
  result = run_auto(helper, CS, CC, backed_up=100, coast=55)
  assert result == pytest.approx(80)
  assert helper.auto_prev_limit == pytest.approx(60)

def test_vturn_automatic_exit_uses_tracked_auto_speed_not_initial_backup():
  helper, (CS, CC) = make_helper(), make_inputs(navSpeedLimit=60)
  helper.frame = 1
  helper.current_driving_mode = 5
  helper.carrot_cruise_active = True
  helper.prev_carrot_active = True
  helper.carrot_cruise_min_timer = 0
  helper._v_cruise_kph_at_brake = 100
  helper._carrot_cruise_saved_op_speed = 100
  helper._carrot_auto_shadow_target = 80
  helper.auto_prev_limit = 60
  helper.v_ego_kph_set = 55
  helper.desiredSource = "VTRUN"
  helper.desiredSpeed = 55
  helper.d_rel = 0
  helper.v_lead_kph = 30
  helper._paddle_mode = 0
  helper._paddle_decel_active = False
  helper._prepare_buttons = lambda _cs, speed: (speed, 0, False)
  helper._carrot_command = lambda speed, button, is_long: (speed, button, is_long)
  helper._update_cruise_state = lambda _cs, _cc, speed: speed

  result = helper._update_cruise_buttons(CS, CC, 55)
  assert not helper.carrot_cruise_active
  assert result == pytest.approx(80)
  assert helper._v_cruise_kph_at_brake == 0
  assert helper._carrot_auto_shadow_target is None


def test_gas_triggered_automatic_exit_does_not_learn_offset_or_force_old_target():
  helper, (CS, CC) = make_helper(), make_inputs(gasPressed=True, navSpeedLimit=60)
  helper._carrot_auto_shadow_target = 80
  helper.auto_prev_limit = 60
  result = helper._update_auto_mode(
    CS, CC, 100, physical_speed_button=False, manual_speed_change=True,
    restore_speed=55, automatic_carrot_restore_target=100,
  )
  assert result == pytest.approx(80)
  assert helper.auto_speed_offset == 0
  assert helper.auto_sync_speed is None


def make_carrot_exit_helper():
  helper = make_helper()
  helper.current_driving_mode = 5
  helper.frame = 1
  helper.v_cruise_kph = 95
  helper.v_ego_kph_set = 0
  helper.desiredSource = "none"
  helper.desiredSpeed = 250
  helper.d_rel = 8
  helper.v_lead_kph = 0
  helper.carrot_cruise_active = True
  helper.prev_carrot_active = False
  helper.carrot_cruise_min_timer = 0
  helper.stationary_lead_timer = 0
  helper._cruise_speed_max = 161
  helper._v_cruise_kph_at_brake = 0
  helper._paddle_mode = 0
  helper._paddle_decel_active = False
  helper._prepare_buttons = lambda _cs, speed: (speed, 0, False)
  helper._carrot_command = lambda speed, button, is_long: (speed, button, is_long)
  helper._update_cruise_state = lambda _cs, _cc, speed: speed
  return helper


def test_stationary_lead_exit_recovers_prior_openpilot_setpoint_after_zero_coast():
  helper, (CS, CC) = make_carrot_exit_helper(), make_inputs()
  # The previous OpenPilot target is 95 even though the temporary coast
  # speed is already zero at the stopped lead.
  assert helper._update_cruise_buttons(CS, CC, 0) == pytest.approx(0)
  assert helper._carrot_cruise_saved_op_speed == pytest.approx(95)
  assert helper._carrot_auto_shadow_target == pytest.approx(95)
  # The stationary-lead timer reaches its existing five-second threshold.
  helper.carrot_cruise_min_timer = 0
  helper.stationary_lead_timer = 499
  helper._carrot_lead_fresh = True
  result = helper._update_cruise_buttons(CS, CC, 0)
  assert not helper.carrot_cruise_active
  assert result == pytest.approx(95)
  assert helper._carrot_cruise_saved_op_speed is None
  assert helper.prev_carrot_active is False  # The exit is recorded immediately.
  # Re-entering on the next update must capture a fresh OpenPilot target.
  helper.carrot_cruise_active = True
  helper.v_cruise_kph = 80
  helper.d_rel = 0
  assert helper._update_cruise_buttons(CS, CC, 0) == pytest.approx(0)
  assert helper._carrot_cruise_saved_op_speed == pytest.approx(80)


def test_brake_exit_preserves_openpilot_setpoint_but_never_requests_reengagement():
  helper, (CS, CC) = make_carrot_exit_helper(), make_inputs(brakePressed=True)
  CC.enabled = False
  helper.carrot_cruise_active = False  # _prepare_brake_gas already canceled it.
  helper.prev_carrot_active = True
  helper._carrot_cruise_saved_op_speed = 95
  result = helper._update_cruise_buttons(CS, CC, 0)
  assert result == pytest.approx(95)
  assert helper._activate_cruise == 0
  assert helper._carrot_cruise_saved_op_speed is None


def test_manual_cancel_does_not_restore_or_engage_from_saved_speed():
  helper, (CS, CC) = make_carrot_exit_helper(), make_inputs()
  helper.carrot_cruise_active = False  # Explicit cancel, not a brake exit.
  helper.prev_carrot_active = True
  helper._carrot_cruise_saved_op_speed = 95
  result = helper._update_cruise_buttons(CS, CC, 0)
  assert result == pytest.approx(0)
  assert helper._activate_cruise == 0
  assert helper._carrot_cruise_saved_op_speed is None


def test_zero_coast_speed_is_not_retained_as_camera_restore_cap():
  helper, (CS, CC) = make_helper(), make_inputs(mapSource=2)
  helper._carrot_auto_shadow_target = 95
  result = run_auto(helper, CS, CC, backed_up=95, coast=0)
  # The camera remains active; restoring the setpoint is not a command to
  # accelerate and camera speed planning remains independent.
  assert result == pytest.approx(95)
  assert helper.camera_pending is True
  assert helper.auto_sync_speed is None


def test_lfa_toggle_restores_saved_op_target_without_adding_speed_increment():
  helper, (CS, CC) = make_carrot_exit_helper(), make_inputs()
  helper._lfa_button_mode = 2
  helper._carrot_cruise_saved_op_speed = 95
  helper.prev_carrot_active = True
  helper._carrot_auto_shadow_target = 95
  helper._v_cruise_kph_at_brake = 100  # Old brake backup must not win.
  helper._prepare_buttons = lambda _cs, speed: (speed, ButtonType.lfaButton, False)
  result = helper._update_cruise_buttons(CS, CC, 0)
  assert result == pytest.approx(95)
  assert not helper.carrot_cruise_active
  assert helper._carrot_cruise_saved_op_speed is None


def test_zero_op_target_on_entry_does_not_invent_vehicle_or_scc_speed():
  helper, (CS, CC) = make_carrot_exit_helper(), make_inputs()
  helper.v_cruise_kph = 0
  helper._v_cruise_kph_at_brake = 100  # Stale brake backup isn't authoritative.
  result = helper._update_cruise_buttons(CS, CC, 0)
  assert result == pytest.approx(0)
  assert helper._carrot_cruise_saved_op_speed is None
  assert helper._carrot_auto_shadow_target == pytest.approx(0)

@pytest.mark.parametrize("button", (ButtonType.accelCruise, ButtonType.lfaButton))
def test_plus_or_lfa_restore_keeps_high_4a3_selection(button):
  helper, (CS, CC) = make_carrot_exit_helper(), make_inputs(
    navLinkClass=0, navSpeedLimit=80, speedLimit=50)
  helper._lfa_button_mode = 2
  helper.v_cruise_kph = 40.
  helper.prev_carrot_active = True
  helper._carrot_cruise_saved_op_speed = 100.
  helper._carrot_auto_shadow_target = 80.
  helper.auto_prev_limit = 80.
  helper.auto_source_high = True
  helper.auto_speed_offset = 10.
  helper._prepare_buttons = lambda _cs, speed: (speed, button, False)
  assert helper._update_cruise_buttons(CS, CC, 40.) == pytest.approx(80.)
  assert helper.auto_source_high is True
  assert not helper._auto_mode_manual_speed_change
  assert not helper.carrot_cruise_active


def camera_coast():
  helper, (CS, CC) = make_carrot_exit_helper(), make_inputs(
    navLinkClass=1, navSpeedLimit=80, speedLimit=50, mapSource=2)
  helper.v_cruise_kph = 40.
  helper.v_ego_kph_set = 40.
  helper.prev_carrot_active = True
  helper._carrot_cruise_saved_op_speed = 100.
  helper._carrot_auto_shadow_target = 100.
  helper.auto_prev_limit = 80.
  helper.auto_speed_offset = 10.
  helper.carrot_cruise_min_timer = 0
  helper.desiredSource = "VTRUN"
  helper.desiredSpeed = 40
  return helper, CS, CC


def test_camera_coast_restores_saved_auto_reference_on_verified_clear():
  helper, CS, CC = camera_coast()
  assert helper._update_cruise_buttons(CS, CC, 40.) == pytest.approx(40.)
  assert not helper.carrot_cruise_active
  assert helper._carrot_restore_deferred_target == pytest.approx(100.)
  assert helper.auto_sync_speed == pytest.approx(40.)

  helper.v_cruise_kph = 40.
  helper.desiredSource = ""
  CS.mapSource = 0
  assert helper._update_cruise_buttons(CS, CC, 40.) == pytest.approx(90.)
  assert helper._carrot_restore_deferred_target is None
  assert helper.auto_source_high is True


def test_camera_coast_does_not_release_on_missing_or_zero_4a3_data():
  helper, CS, CC = camera_coast()
  assert helper._update_cruise_buttons(CS, CC, 40.) == pytest.approx(40.)
  helper.v_cruise_kph = 40.
  helper.desiredSource = ""
  CS.navDataValid = False
  CS.mapSource = 0
  assert helper._update_cruise_buttons(CS, CC, 40.) == pytest.approx(40.)
  assert helper._carrot_restore_deferred_target == pytest.approx(100.)

  CS.navDataValid = True
  CS.navSpeedLimit = 0
  assert helper._update_cruise_buttons(CS, CC, 40.) == pytest.approx(40.)
  assert helper._carrot_restore_deferred_target == pytest.approx(100.)

  CS.navSpeedLimit = 80
  assert helper._update_cruise_buttons(CS, CC, 40.) == pytest.approx(90.)
  assert helper._carrot_restore_deferred_target is None

@pytest.mark.parametrize(("ego_mps", "fresh_radar", "distance_m"), (
  (8., True, 8.),     # Moving toward a stationary obstacle
  (0., False, 8.),    # Lost radar tracking
  (0., True, 70.),    # Distant static object
))
def test_stationary_lead_auto_return_requires_confirmed_near_stop(ego_mps, fresh_radar, distance_m):
  helper, (CS, CC) = make_carrot_exit_helper(), make_inputs(vEgo=ego_mps)
  helper.prev_carrot_active = True
  helper._carrot_cruise_saved_op_speed = 95.
  helper._carrot_auto_shadow_target = 95.
  helper.carrot_cruise_min_timer = 0
  helper.stationary_lead_timer = 499
  helper._carrot_lead_fresh = fresh_radar
  helper.d_rel = distance_m
  helper.v_lead_kph = 0.
  assert helper._update_cruise_buttons(CS, CC, 0.) == pytest.approx(0.)
  assert helper.carrot_cruise_active
  assert helper.stationary_lead_timer == 0


def test_manual_speed_choice_cancels_deferred_camera_target():
  helper, (CS, CC) = make_helper(), make_inputs(mapSource=2)
  helper._carrot_restore_deferred_target = 100.
  helper.auto_sync_speed = 40.
  helper.auto_mode_needs_sync = True
  assert helper._update_auto_mode(
    CS, CC, 45., physical_speed_button=True,
    manual_speed_change=True, restore_speed=None,
  ) == pytest.approx(45.)
  assert helper._carrot_restore_deferred_target is None
  assert helper.auto_sync_speed is None


def test_camera_deferred_restore_not_released_on_missing_mixed_4be():
  helper, (CS, CC) = make_helper(), make_inputs(
    mapSource=0, navLinkClass=2, speedLimitDataValid=False,
    navSpeedLimit=80, speedLimit=0,
  )
  helper._carrot_restore_deferred_target = 100.
  helper.auto_source_high = False
  helper.auto_sync_speed = 40.
  helper.auto_mode_needs_sync = True
  assert helper._update_auto_mode(
    CS, CC, 40., physical_speed_button=False,
    manual_speed_change=False, restore_speed=None,
  ) == pytest.approx(40.)
  assert helper._carrot_restore_deferred_target == pytest.approx(100.)

