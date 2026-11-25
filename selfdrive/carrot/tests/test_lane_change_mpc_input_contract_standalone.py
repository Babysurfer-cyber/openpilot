"""Production-source MPC input-contract regression for adjacent-lane leadTwo.

Extracts the real longitudinal MPC's small pure functions and lead preprocessing
with AST. Deliberately does NOT run acados, drive a car, or certify accel/jerk.
"""
from __future__ import annotations

import ast
import copy
import math
import random
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
MPC_SOURCE = ROOT / 'selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py'


def load_pure_mpc(src: Path):
  tree = ast.parse(src.read_text(encoding='utf-8'), filename=str(src))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalMpc')
  method_map = {n.name: n for n in cls.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
  top_map = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
  wanted = ('get_stopped_equivalence_factor', 'get_safe_obstacle_distance')
  assert all(k in top_map for k in wanted)
  assert all(k in method_map for k in ('process_lead', 'extrapolate_lead'))
  nodes = [copy.deepcopy(top_map[k]) for k in wanted]
  nodes += [copy.deepcopy(method_map[k]) for k in ('extrapolate_lead', 'process_lead')]
  for node in nodes:
    node.decorator_list = []
  scope = {
    'np': np, 'COMFORT_BRAKE': 2.5, 'STOP_DISTANCE': 6.,
    'ACCEL_MIN': -4.0, '_LEAD_ACCEL_TAU': 1.5,
    'get_T_FOLLOW': lambda: 1.45,
    'T_IDXS': np.linspace(0., 10., 13),
    'T_DIFFS': np.diff(np.linspace(0., 10., 13), prepend=[0.]),
  }
  exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(src), 'exec'), scope)
  return scope, tree


def processed_obstacle(mpc, scope, distance, speed, lead_speed):
  lead = SimpleNamespace(status=True, dRel=distance, vLead=lead_speed,
                         aLeadK=0., aLeadTau=1.5)
  xv, _ = scope['process_lead'](mpc, lead, 0.)
  return float(xv[0, 0] + scope['get_stopped_equivalence_factor'](xv[0, 1]))


class TestMpcProductionInputContract(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.ns, cls.tree = load_pure_mpc(MPC_SOURCE)

  def mpc(self, ego):
    return SimpleNamespace(x0=np.array([0., ego, 0.]),
                           extrapolate_lead=self.ns['extrapolate_lead'])

  def test_45m_lead_only_step_is_40_75m_with_cruise_obstacle_example(self):
    ns = self.ns
    mpc = self.mpc(27.)
    primary = processed_obstacle(mpc, ns, 60., 27., 26.)
    side = processed_obstacle(mpc, ns, 35., 27., 24.)
    cruise = float(ns['get_safe_obstacle_distance'](27., 1.45, 2.5, 6.))
    self.assertAlmostEqual(primary, 195.2)
    self.assertAlmostEqual(side, 150.2)
    self.assertAlmostEqual(cruise, 190.95)
    self.assertAlmostEqual(side - primary, -45.)
    self.assertAlmostEqual(min(primary, side, cruise) - min(primary, cruise), -40.75)

  def test_25000_distance_eligible_side_lead_obstacle_cases(self):
    rng = random.Random(20261126)
    ns = self.ns
    marked = 0
    max_change = 0.
    for _ in range(25000):
      ego = rng.uniform(8., 36.)
      first_v = rng.uniform(2., ego)
      side_v = rng.uniform(2., ego)
      first_dist = rng.uniform(10., 80.)
      # Matches LaneChangeSpeedAssist's distance gate for an active maneuver.
      lo = max(8., ego * .5) + 0.01
      hi = min(80., ego * 2.5) - 0.01
      side_dist = rng.uniform(lo, hi)
      t_follow = rng.uniform(1.0, 2.0)
      mpc = self.mpc(ego)
      primary = processed_obstacle(mpc, ns, first_dist, ego, first_v)
      side = processed_obstacle(mpc, ns, side_dist, ego, side_v)
      cruise = float(ns['get_safe_obstacle_distance'](ego, t_follow, 2.5, 6.))
      old = min(primary, cruise)
      new = min(primary, cruise, side)
      self.assertLessEqual(new, old + 1e-9)
      delta = old - new
      marked += delta >= 10.
      max_change = max(max_change, delta)
    self.assertGreaterEqual(marked, 0)  # Report risk; do not call it a safe-jerk result.
    print(f'PRESOLVER INPUT ONLY: >=10m step in {marked}/25000; max={max_change:.2f}m')

  def test_contract_does_not_edit_mpc_acceleration_limit_or_jerk(self):
    src = MPC_SOURCE.read_text(encoding='utf-8')
    # This checks wiring only; it cannot bound the actual solver's acceleration or jerk.
    cls = next(n for n in self.tree.body if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalMpc')
    update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
    code = ast.unparse(update)
    self.assertIn('self.process_lead(radarstate.leadTwo, 0.0)', code)
    self.assertIn('self.params[:, 1] = max(0.0, self.max_a if not reset_state else a_ego)', code)
    self.assertIn('self.params[:, 2] = np.min(x_obstacles, axis=1)', code)
    writes_max_a = [n for n in ast.walk(update) if isinstance(n, (ast.Assign, ast.AugAssign))
                    and 'self.max_a' in ast.unparse(n).split('=')[0]]
    self.assertFalse(writes_max_a)


if __name__ == '__main__':
  unittest.main(verbosity=2)
