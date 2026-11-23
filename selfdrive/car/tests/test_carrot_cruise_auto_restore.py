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
  helper.auto_speed_offset = 0.0
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
    gasPressed=False,
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
