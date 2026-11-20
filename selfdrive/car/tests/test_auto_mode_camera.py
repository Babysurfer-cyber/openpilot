"""Section-start signals release auto-mode camera hold through the full cruise update."""

from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.carstate import CarState
from openpilot.selfdrive.car.tests.test_can_cruise_recovery import integrated_drive


@pytest.fixture
def auto_drive(integrated_drive, monkeypatch):
  drive = integrated_drive
  helper = drive.helper
  original_get_int = helper.params.get_int
  monkeypatch.setattr(helper.params, 'get_int',
                      lambda key: 5 if key == 'MyDrivingMode' else original_get_int(key))
  helper.update_params(True)
  helper.current_driving_mode = 5
  helper.enabled_last = True
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 110.
  helper.auto_prev_limit = 100.
  helper.auto_speed_offset = 10.
  drive.state.navSpeedLimit = 80.
  drive.state.speedLimit = 50.
  drive.state.navLinkClass = 1
  drive.state.mapSource = 2
  drive.state.navTollExist = 0
  return drive


def update(drive, now):
  drive.step(now, full_update=True)
  assert 'Auto Mode Error:' not in drive.helper.log
  return float(drive.helper.v_cruise_kph)


def test_approach_announcement_holds_speed_before_section_start(auto_drive):
  drive = auto_drive
  drive.state.navSectionValid = True
  drive.state.navSectionActive = False
  for index in range(5):
    assert update(drive, 1. + index * .01) == 110.
    assert drive.helper.camera_pending
    assert drive.helper.auto_prev_limit == 100.


def test_section_start_applies_new_limit_while_camera_flag_stays_set(auto_drive):
  drive = auto_drive
  drive.state.navSectionValid = True
  drive.state.navSectionActive = False
  assert update(drive, 1.) == 110.
  drive.state.navSectionActive = True
  assert update(drive, 1.01) == 95.
  assert drive.state.mapSource == 2
  assert not drive.helper.camera_pending
  assert drive.helper.auto_prev_limit == 80.


def test_section_active_does_not_repeat_the_same_speed_reduction(auto_drive):
  drive = auto_drive
  drive.state.navSectionValid = drive.state.navSectionActive = True
  assert update(drive, 1.) == 95.
  for index in range(1, 30):
    assert update(drive, 1. + index * .01) == 95.
    assert not drive.helper.camera_pending


def test_section_exit_waits_for_remaining_camera_announcement_to_clear(auto_drive):
  drive = auto_drive
  drive.state.navSectionValid = drive.state.navSectionActive = True
  assert update(drive, 1.) == 95.
  drive.state.navSectionActive = False
  drive.state.navSpeedLimit = 100.
  assert update(drive, 1.01) == 95.
  assert drive.helper.camera_pending
  assert drive.helper.auto_prev_limit == 80.
  drive.state.mapSource = 0
  assert update(drive, 1.02) == 110.
  assert not drive.helper.camera_pending
  assert drive.helper.auto_prev_limit == 100.


@pytest.mark.parametrize(('can_valid', 'section_valid', 'section_active'), (
  (False, True, True),
  (True, False, True),
  (True, False, False),
  (True, True, False),
))
def test_invalid_or_outside_section_cannot_release_camera_hold(
  auto_drive, can_valid, section_valid, section_active,
):
  drive = auto_drive
  drive.state.canValid = can_valid
  drive.state.navSectionValid = section_valid
  drive.state.navSectionActive = section_active
  assert update(drive, 1.) == 110.
  assert drive.helper.camera_pending
  assert drive.helper.auto_prev_limit == 100.


def test_default_section_fields_preserve_existing_camera_hold(auto_drive):
  drive = auto_drive
  assert not drive.state.navSectionValid
  assert not drive.state.navSectionActive
  assert update(drive, 1.) == 110.
  assert drive.helper.camera_pending


def test_unavailable_section_signal_reinstates_camera_hold(auto_drive):
  drive = auto_drive
  drive.state.navSectionValid = drive.state.navSectionActive = True
  assert update(drive, 1.) == 95.
  drive.state.navSectionValid = False
  drive.state.navSpeedLimit = 60.
  assert update(drive, 1.01) == 95.
  assert drive.helper.camera_pending
  assert drive.helper.auto_prev_limit == 80.


def test_section_start_retains_pure_navigation_source_above_90(auto_drive):
  drive = auto_drive
  drive.state.navSectionValid = drive.state.navSectionActive = True
  assert update(drive, 1.) == 95.
  assert drive.helper.auto_prev_limit == 80.
  assert drive.state.speedLimit == 50.


def test_missing_pure_navigation_limit_never_falls_back_to_mixed_on_mainline(auto_drive):
  drive = auto_drive
  drive.state.navSectionValid = drive.state.navSectionActive = True
  drive.state.navSpeedLimit = 0.
  assert update(drive, 1.) == 110.
  assert not drive.helper.camera_pending
  assert drive.helper.auto_prev_limit == 100.


def test_non_auto_mode_does_not_apply_section_camera_speed_change(auto_drive):
  drive = auto_drive
  drive.helper.frame = 1
  drive.helper.current_driving_mode = 3
  drive.state.navSectionValid = drive.state.navSectionActive = True
  assert update(drive, 1.) == 110.
  assert drive.helper.auto_prev_limit == 100.


@pytest.mark.parametrize(('average_speed', 'age_ns', 'valid', 'active'), (
  (0, 0, True, False),
  (80, 500_000_000, True, True),
  (80, 1_000_000_001, False, False),
))
def test_navigation_section_decode_and_freshness_control_real_cruise_hold(
  auto_drive, average_speed, age_ns, valid, active,
):
  drive = auto_drive
  now_ns = 5_000_000_000
  cp = SimpleNamespace(
    _last_update_nanos=now_ns,
    ts_nanos={'NEW_MSG_4B4': {'POS_RANGE_AVG_SPEED': now_ns - age_ns}},
    vl={'NEW_MSG_4B4': {'POS_RANGE_AVG_SPEED': average_speed}},
  )
  # Exercise the real decoder and ensure loss/invalidity clears previous flags.
  drive.state.navSectionValid = drive.state.navSectionActive = True
  decoder = CarState.__new__(CarState)
  decoder._update_vehicle_navi_section(cp, drive.state)
  assert drive.state.navSectionValid == valid
  assert drive.state.navSectionActive == active
  assert update(drive, 1.) == (95. if active else 110.)
  assert drive.helper.camera_pending != active
