"""Physical bounds of the imported CUT-IN/CUT-OUT longitudinal responses."""

from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.cutin_predecel import (
  apply_cutin_predecel_accel_limit,
  get_cutin_predecel_accel_limit,
)
from openpilot.selfdrive.controls.lib.longitudinal_cutout import cutout_obstacle_relief


def risk(**changes):
  values = dict(status=True, radar=True, score=0.9, dRel=20.0, vRel=-6.5)
  values.update(changes)
  return NS(leadCutInRisk=NS(**values))


def lead(**changes):
  values = dict(status=True, radar=True, radarTrackId=50, dRel=25.0,
                vRel=-1.0, vLead=14.0, aLeadK=-0.5,
                cutOutTime=1.0, cutOutConfidence=1.0)
  values.update(changes)
  return NS(**values)


@pytest.mark.parametrize("changes", [
  {"status": False}, {"radar": False}, {"score": 0.149},
  {"score": float("nan")}, {"score": float("inf")},
  {"dRel": 0.0}, {"dRel": -1.0}, {"dRel": float("nan")},
  {"dRel": float("inf")}, {"vRel": 0.0}, {"vRel": 2.0},
  {"vRel": float("nan")}, {"vRel": -float("inf")},
])
def test_cutin_invalid_or_nonclosing_risk_cannot_request_predeceleration(changes):
  assert get_cutin_predecel_accel_limit(risk(**changes)) is None


def test_missing_cutin_signal_is_inactive():
  assert get_cutin_predecel_accel_limit(NS()) is None
  assert get_cutin_predecel_accel_limit(NS(leadCutInRisk=None)) is None


def test_cutin_response_is_bounded_and_increases_with_collision_urgency():
  urgent = get_cutin_predecel_accel_limit(risk(dRel=8.0, vRel=-8.0))
  ordinary = get_cutin_predecel_accel_limit(risk())
  distant = get_cutin_predecel_accel_limit(risk(dRel=40.0, vRel=-4.0))
  assert urgent == pytest.approx(-0.65)
  assert ordinary == pytest.approx(-0.608, abs=0.002)
  assert distant == pytest.approx(-0.25)
  assert urgent < ordinary < distant < 0.0


def test_cutin_risk_smoothly_removes_positive_acceleration_before_decelerating():
  acceleration = 0.96
  ceiling = get_cutin_predecel_accel_limit(risk())
  previous = acceleration
  applied = []
  for _ in range(20):
    acceleration = apply_cutin_predecel_accel_limit(1.0, previous, ceiling)
    assert 0.0 <= previous - acceleration <= 0.15 + 1e-12
    assert acceleration >= ceiling
    applied.append(acceleration)
    previous = acceleration
  assert applied[0] == pytest.approx(0.81)
  assert applied[6] < 0.0
  assert applied[-1] == pytest.approx(ceiling)


@pytest.mark.parametrize("maximum,desired", [(1.0, 0.0), (-1.0, 0.5), (-2.0, -2.5)])
def test_absent_cutin_risk_preserves_existing_acceleration_slew(maximum, desired):
  assert apply_cutin_predecel_accel_limit(maximum, desired, None) == pytest.approx(max(maximum, desired - 0.05))


def test_cutout_keeps_preclearance_obstacle_and_measured_lead_unchanged():
  target = lead()
  original = vars(target).copy()
  times = np.array([0.0, 1.0, 1.3, 1.55, 1.8, 5.0])
  saved_times = times.copy()
  relief = cutout_obstacle_relief(target, 15.0, times, 1.45, 6.0)
  np.testing.assert_array_equal(relief[:3], 0.0)
  assert relief[3] == pytest.approx(3.75)
  np.testing.assert_allclose(relief[4:], 7.5)
  assert vars(target) == original
  np.testing.assert_array_equal(times, saved_times)


@pytest.mark.parametrize("ego_speed,follow_time,confidence,maximum", [
  (15.0, 1.45, 1.0, 7.5),
  (40.0, 1.45, 1.0, 8.0),
  (15.0, 0.4, 1.0, 3.0),
  (15.0, 1.45, 0.25, 1.875),
])
def test_cutout_relief_is_limited_by_headway_distance_and_confidence(ego_speed, follow_time, confidence, maximum):
  result = cutout_obstacle_relief(lead(cutOutConfidence=confidence), ego_speed,
                                 np.array([0.0, 1.3, 1.55, 2.0, 10.0]), follow_time, 6.0)
  assert result[-1] == pytest.approx(maximum)
  assert result.max() <= maximum + 1e-12
  assert result.min() == 0.0


@pytest.mark.parametrize("changes", [
  {"status": False}, {"radar": False}, {"radarTrackId": -1},
  {"cutOutTime": 0.0}, {"cutOutTime": -1.0}, {"cutOutTime": 2.51},
  {"cutOutTime": float("nan")}, {"cutOutTime": float("inf")},
  {"cutOutConfidence": 0.0}, {"cutOutConfidence": -0.1},
  {"cutOutConfidence": 1.01}, {"cutOutConfidence": float("nan")},
  {"cutOutConfidence": float("inf")},
  {"vLead": 4.0}, {"vLead": float("nan")},
  {"aLeadK": -2.51}, {"aLeadK": float("nan")},
  {"dRel": float("inf")}, {"vRel": float("nan")},
])
def test_cutout_invalid_or_braking_lead_cannot_relax_obstacle(changes):
  result = cutout_obstacle_relief(lead(**changes), 15.0, np.array([0.0, 2.0, 5.0]), 1.45, 6.0)
  np.testing.assert_array_equal(result, 0.0)


@pytest.mark.parametrize("ego_speed,follow_time,stop_distance", [
  (4.99, 1.45, 6.0), (float("nan"), 1.45, 6.0),
  (float("inf"), 1.45, 6.0), (15.0, 0.0, 6.0),
  (15.0, -1.0, 6.0), (15.0, float("nan"), 6.0),
  (15.0, 1.45, float("nan")),
])
def test_cutout_requires_valid_speed_and_following_configuration(ego_speed, follow_time, stop_distance):
  result = cutout_obstacle_relief(lead(), ego_speed, np.array([0.0, 2.0, 5.0]), follow_time, stop_distance)
  np.testing.assert_array_equal(result, 0.0)


def test_cutout_requires_enough_gap_at_predicted_clearance():
  times = np.array([0.0, 3.0, 5.0])
  # This car is predicted to cross the clearance threshold only after the
  # available range has already been consumed by its closing speed.
  closing = lead(dRel=25.0, vRel=-10.0, cutOutTime=2.0)
  assert not cutout_obstacle_relief(closing, 15.0, times, 1.45, 6.0).any()
  # A configured standstill gap larger than the remaining range also vetoes it.
  assert not cutout_obstacle_relief(lead(), 15.0, times, 1.45, 25.0).any()


def test_missing_cutout_metadata_preserves_obstacle():
  target = lead()
  del target.cutOutTime
  del target.cutOutConfidence
  np.testing.assert_array_equal(cutout_obstacle_relief(target, 15.0, np.array([0.0, 2.0, 5.0]), 1.45, 6.0), 0.0)
