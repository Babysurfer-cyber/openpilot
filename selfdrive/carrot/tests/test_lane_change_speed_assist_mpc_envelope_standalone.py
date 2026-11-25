"""MPC obstacle-envelope regression for PR #60.

Numerical proxy for source formulas in long_mpc.py; not an Acados solve,
not an acceleration, jerk, or physical vehicle safety certification.
"""
import unittest
import numpy as np

COMFORT_BRAKE = 2.5
T_FOLLOW = 1.45
STOP_DISTANCE = 6.0


def lead_obstacle(distance_m, lead_v_mps, times_s):
  return distance_m + lead_v_mps * times_s + lead_v_mps ** 2 / (2.0 * COMFORT_BRAKE)


class TestMpcLeadTwoEnvelope(unittest.TestCase):
  def test_new_second_lead_cannot_relax_obstacle_envelope(self):
    times = np.linspace(0., 8., 41)
    checks = 0
    for ego_v in (10., 20., 27., 35.):
      for first_v in (ego_v, ego_v - 1., max(2., ego_v - 5.)):
        for side_v in (max(2., ego_v - 3.), max(2., ego_v - 8.)):
          lead0 = lead_obstacle(60., first_v, times)
          side = lead_obstacle(35., side_v, times)
          cruise = lead_obstacle(200., ego_v, times)
          baseline = np.minimum(lead0, cruise)
          modified = np.minimum(baseline, side)
          self.assertTrue(np.all(modified <= baseline + 1e-12))
          checks += 1
    self.assertEqual(checks, 24)

  def test_obstacle_step_is_visible_on_side_lead_appearance_and_release(self):
    # Test DETECTS a potential discontinuity; it does NOT establish safe control.
    ego = 27.
    lead0 = float(lead_obstacle(60., 26., 0.))
    side = float(lead_obstacle(35., 24., 0.))
    desired = ego**2 / (2.*COMFORT_BRAKE) + T_FOLLOW*ego + STOP_DISTANCE
    jump_in = min(lead0, side) - lead0
    jump_out = lead0 - min(lead0, side)
    self.assertAlmostEqual(jump_in, -45., places=6)
    self.assertAlmostEqual(jump_out, +45., places=6)
    self.assertAlmostEqual(side - desired, -40.75, places=6)
    print(f'Obstacle discontinuity (not an acceleration): {jump_in:+.2f}m / {jump_out:+.2f}m')


if __name__ == '__main__':
  unittest.main()
