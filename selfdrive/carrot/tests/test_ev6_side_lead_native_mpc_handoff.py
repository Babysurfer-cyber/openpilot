"""Native Acados longitudinal MPC side-lead entry/exit safety gate.

Requires the actual compiled openpilot generated solver. Never falls back to a
mock or skips a missing native dependency. No car, Panda, or CAN transmission.
The finite jerk acceptance threshold is a test engineering gate, not proof of
on-road comfort; confirm it against production requirements and replay logs.
"""
from __future__ import annotations

import math
import os
import unittest
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  ACCEL_MIN, LongitudinalMpc, N,
)

ACCEL_UPPER = 1.5  # configured maximum acceleration for this test
JERK_GATE = float(os.environ.get("EV6_NATIVE_MPC_JERK_GATE", "5.0"))


@dataclass
class FakeCarrot:
  v_cruise: float
  mode: str = "acc"
  stop_dist: float = 1000.
  comfort_brake: float = 2.5
  stop_distance: float = 6.
  trafficStopDistanceAdjust: float = 0.
  j_lead_factor: float = 1.
  autoNaviSpeedDecelRate: float = 1.

  def get_T_FOLLOW(self, personality, v_ego, a_ego):
    return 1.45

  def dynamic_t_follow(self, t_follow, lead, desired_distance, prev_a):
    return t_follow


def lead(status=False, d_rel=0., speed=0.):
  return SimpleNamespace(status=status, dRel=d_rel, vLead=speed,
                         aLeadK=0., aLeadTau=1.5, jLead=0.,
                         modelProb=0.85)


def run_synthetic_handoff(v_ego0, front_dist, front_speed, side_dist, side_speed):
  # Start and finish with no side lead, with a stable side lead in between.
  # Propagate ego and relative distances using the MPC's first planned jerk.
  planner = LongitudinalMpc(mode="acc", dt=DT_MDL)
  planner.set_accel_limits(-2.0, ACCEL_UPPER)
  carrot = FakeCarrot(v_cruise=v_ego0)
  v_ego, a_ego = float(v_ego0), 0.
  front_d, side_d = float(front_dist), float(side_dist)
  measurements = []
  for frame in range(72):
    active = 16 <= frame < 44
    state = SimpleNamespace(
      leadOne=lead(True, max(1., front_d), front_speed),
      leadTwo=lead(active, max(1., side_d), side_speed),
    )
    carrot.v_cruise = v_ego0  # fixed speed setting, independent of side lead
    planner.set_cur_state(v_ego, a_ego)
    zeros = np.zeros(N + 1)
    planner.update(carrot, False, state, v_ego0,
                   zeros.copy(), zeros.copy(), zeros.copy(), zeros.copy())
    # A failed solver can reset itself; a non-positive solve_time reveals
    # that there was no valid native solution on this frame.
    assert planner.solve_time > 0., (frame, planner.solve_time)
    assert planner.solution_status == 0, (frame, planner.solution_status)
    assert np.all(np.isfinite(planner.a_solution)), (frame, "nonfinite acceleration")
    assert np.all(np.isfinite(planner.j_solution)), (frame, "nonfinite jerk")
    assert np.all(planner.a_solution <= ACCEL_UPPER + .03), (frame, "over accel bound", planner.a_solution)
    assert np.all(planner.a_solution >= ACCEL_MIN - .03), (frame, "over braking bound", planner.a_solution)
    j0 = float(planner.j_solution[0])
    assert abs(j0) <= JERK_GATE, (frame, "jerk exceeds review gate", j0)
    measurements.append((frame, active, float(planner.a_solution[1]), j0, planner.source))
    front_d += (front_speed - v_ego) * DT_MDL
    side_d += (side_speed - v_ego) * DT_MDL
    a_ego = float(np.clip(a_ego + j0 * DT_MDL, ACCEL_MIN, ACCEL_UPPER))
    v_ego = max(0., v_ego + a_ego * DT_MDL)
  return measurements


class EV6NativeSideLeadMpcGate(unittest.TestCase):
  def test_entry_exit_with_slower_side_lead(self):
    # 97 km/h ego; a slower side lead enters and exits.
    measured = run_synthetic_handoff(27., 60., 26., 55., 25.)
    self.assertEqual(len(measured), 72)
    self.assertTrue(any(active for _, active, *_ in measured))
    print("medium change: max|jerk|=%.3f m/s^3, min accel=%.3f m/s^2" %
          (max(abs(x[3]) for x in measured), min(x[2] for x in measured)))

  def test_close_slower_lead_is_bounded(self):
    measured = run_synthetic_handoff(27., 60., 26., 35., 24.)
    self.assertEqual(len(measured), 72)
    print("close change: max|jerk|=%.3f m/s^3, min accel=%.3f m/s^2" %
          (max(abs(x[3]) for x in measured), min(x[2] for x in measured)))

  def test_low_speed_entry_and_exit(self):
    measured = run_synthetic_handoff(20., 48., 19., 32., 17.)
    self.assertEqual(len(measured), 72)
    print("low-speed change: max|jerk|=%.3f m/s^3, min accel=%.3f m/s^2" %
          (max(abs(x[3]) for x in measured), min(x[2] for x in measured)))


if __name__ == "__main__":
  unittest.main(verbosity=2)
