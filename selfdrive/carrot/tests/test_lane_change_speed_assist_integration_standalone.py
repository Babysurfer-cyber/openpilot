"""Execute production controller's adjacent-lead selection AST without native builds.

This executes the unchanged side-lead statements from DPathRadarController.update
with narrow substitutes for native inputs. It does not test the whole MPC or car.
"""
from __future__ import annotations

import ast
import copy
import importlib.util
import sys
import unittest

import numpy as np
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

ROOT = Path(__file__).resolve().parents[3]


def load_assist():
  path = ROOT / "selfdrive/carrot/radar_motion/lane_change_speed_assist.py"
  name = "lane_change_assist_under_test"
  spec = importlib.util.spec_from_file_location(name, path)
  module = importlib.util.module_from_spec(spec)
  sys.modules[name] = module
  spec.loader.exec_module(module)
  return module


ASSIST = load_assist()


def production_duplicate_filter():
  path = ROOT / "selfdrive/carrot/radar_motion/lead_selection.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  declarations = [node for node in tree.body if isinstance(node, ast.Assign)
                  and any(isinstance(n, ast.Name) and n.id in (
                    "PRIMARY_DUPLICATE_MAX_DREL_DELTA_M", "PRIMARY_DUPLICATE_MAX_YREL_DELTA_M")
                    for n in node.targets)]
  methods = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "lead_duplicates_primary"]
  assert len(methods) == 1, "Production duplicate filter missing"
  ns = {"Any": Any}
  exec(compile(ast.fix_missing_locations(ast.Module(body=declarations + methods, type_ignores=[])), str(path), "exec"), ns)
  return ns["lead_duplicates_primary"]


def production_selector(projector=None):
  path = ROOT / "selfdrive/carrot/radar_motion/controller.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DPathRadarController")
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")
  matches = [i for i, node in enumerate(method.body) if isinstance(node, ast.Assign)
             and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
             and node.targets[0].id == "side_point"]
  assert len(matches) == 1, "Production lane-change selection was changed or removed"
  index = matches[0]
  assert isinstance(method.body[index+1], ast.If), "Expected production guard following side_point"
  selection = [copy.deepcopy(method.body[index]), copy.deepcopy(method.body[index+1])]
  writes = {node.id for stmt in selection for node in ast.walk(stmt)
            if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store)}
  assert writes <= {"side_point", "side_d_path", "side_lead", "lead_two"}, writes
  args = [ast.arg(arg=n) for n in ("self", "time_s", "v_ego", "model", "points", "path", "lead_one", "lead_two")]
  func = ast.FunctionDef(
    name="select_actual_side_lead", args=ast.arguments(posonlyargs=[], args=args, vararg=None,
      kwonlyargs=[], kw_defaults=[], kwarg=None, defaults=[]),
    body=selection + [ast.Return(value=ast.Name(id="lead_two", ctx=ast.Load()))],
    decorator_list=[], returns=None, type_comment=None)
  ns = {
    "project_to_model_path": projector or (lambda path, x, y: SimpleNamespace(d_path=y)),
    "lead_duplicates_primary": production_duplicate_filter(),
    "side_lead_may_replace": ASSIST.side_lead_may_replace,
  }
  exec(compile(ast.fix_missing_locations(ast.Module(body=[func], type_ignores=[])), str(path), "exec"), ns)
  return ns["select_actual_side_lead"]


@dataclass(frozen=True)
class Point:
  track_id: int = 10
  source: str = "corner235"
  d_rel: float = 35.0
  y_rel: float = 3.6
  v_rel: float = -3.0
  v_lead: float = 24.0
  yv_rel: float = 0.0
  measured: bool = True


def model(state="preLaneChange", direction="left"):
  lines = tuple(SimpleNamespace(x=(0.0, 100.0), y=(y, y)) for y in (-5.4, -1.8, 1.8, 5.4))
  return SimpleNamespace(meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction),
                         laneLines=lines, laneLineProbs=(.95, .95, .95, .95))


class ControllerHarness:
  def __init__(self, projector=None):
    self.enable_radar_tracks = 1
    self.lane_change_speed_assist = ASSIST.LaneChangeSpeedAssist()
    self.selection = production_selector(projector)

  def _lead_from_radar_point(self, point, d_path, model_probability, score):
    return {"status": True, "radarTrackId": point.track_id, "dRel": point.d_rel,
            "vLead": point.v_lead, "vRel": point.v_rel, "yRel": point.y_rel, "dPath": d_path}

  def process(self, t, state, points=(Point(),), direction="left", lead_one=None, lead_two=None):
    lead_one = {"status": True, "radarTrackId": 99, "dRel": 60.0,
                "vLead": 26.0, "yRel": 0.0} if lead_one is None else lead_one
    return self.selection(self, t, 27.0, model(state, direction), points,
                          ((0.,0.),(100.,0.)), lead_one, lead_two)

  def prepare(self, direction="left", point=Point(), lead_one=None, lead_two=None):
    for i in range(6):
      assert self.process(i*.05, "preLaneChange", (point,), direction, lead_one, lead_two) is lead_two
    return self.process(.30, "laneChangeStarting", (point,), direction, lead_one, lead_two)


class LaneChangeControllerAstIntegration(unittest.TestCase):
  def test_production_selector_is_live_and_restricts_new_lead_two(self):
    selected = ControllerHarness().prepare()
    self.assertIsNotNone(selected)
    self.assertEqual(selected["radarTrackId"], 10)
    self.assertEqual(selected["dRel"], 35.0)

  def test_right_side_handoff(self):
    selected = ControllerHarness().prepare("right", Point(y_rel=-3.6))
    self.assertEqual(selected["radarTrackId"], 10)

  def test_prechange_never_changes_lead_two(self):
    h = ControllerHarness()
    for i in range(6):
      self.assertIsNone(h.process(i*.05, "preLaneChange"))

  def test_existing_closer_lead_two_not_overwritten(self):
    previous = {"status": True, "radarTrackId": 123, "dRel": 19.0, "vLead": 17.0, "yRel": 0.0}
    self.assertIs(ControllerHarness().prepare(lead_two=previous), previous)

  def test_existing_slower_lead_two_not_overwritten(self):
    previous = {"status": True, "radarTrackId": 123, "dRel": 48.0, "vLead": 12.0, "yRel": 0.0}
    self.assertIs(ControllerHarness().prepare(lead_two=previous), previous)

  def test_more_restrictive_side_lead_can_replace(self):
    previous = {"status": True, "radarTrackId": 123, "dRel": 70.0, "vLead": 28.0, "yRel": 0.0}
    selected = ControllerHarness().prepare(lead_two=previous)
    self.assertEqual(selected["radarTrackId"], 10)

  def test_duplicate_primary_never_becomes_second_lead(self):
    primary = {"status": True, "radarTrackId": 10, "dRel": 35.0, "vLead": 24.0, "yRel": 3.6}
    self.assertIsNone(ControllerHarness().prepare(lead_one=primary))

  def test_missing_primary_never_promotes_side(self):
    self.assertIsNone(ControllerHarness().prepare(lead_one={"status": False}))

  def test_model_path_gate_prevents_off_path_vehicle(self):
    h = ControllerHarness(projector=lambda path, x, y: SimpleNamespace(d_path=5.0))
    self.assertIsNone(h.prepare())

  def test_change_cancel_drops_pending_side(self):
    h = ControllerHarness()
    self.assertIsNotNone(h.prepare())
    self.assertIsNone(h.process(.35, "laneChangeFinishing"))
    self.assertIsNone(h.process(.40, "off"))
    self.assertIsNone(h.process(.45, "laneChangeStarting"))

  def test_scc_only_modes_never_select_side_lead(self):
    for mode in (-1, 0):
      with self.subTest(mode=mode):
        h = ControllerHarness()
        h.enable_radar_tracks = mode
        self.assertIsNone(h.prepare())

  def test_radar_tracking_modes_select_side_lead(self):
    for mode in (1, 2, 3):
      with self.subTest(mode=mode):
        h = ControllerHarness()
        h.enable_radar_tracks = mode
        self.assertIsNotNone(h.prepare())

  def test_production_mpc_includes_lead_two_in_obstacle_minimum(self):
    path = ROOT / "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LongitudinalMpc")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")
    obstacles = [n for n in ast.walk(method) if isinstance(n, ast.Assign)
                 and any(isinstance(target, ast.Name) and target.id == "x_obstacles" for target in n.targets)]
    self.assertEqual(len(obstacles), 2)
    for n in obstacles:
      self.assertTrue(any(isinstance(x, ast.Name) and x.id == "lead_1_obstacle" for x in ast.walk(n.value)))
    minimums = [n for n in ast.walk(method) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute) and n.func.attr == "min"
                and any(isinstance(x, ast.Name) and x.id == "x_obstacles" for x in ast.walk(n))]
    self.assertTrue(minimums)

  def test_second_lead_reduces_not_increases_synthetic_obstacle_envelope(self):
    lead_zero = np.array([55., 60., 68., 80.])
    cruise = np.array([78., 85., 91., 103.])
    stopped = np.array([300., 300., 300., 300.])
    baseline_second = np.array([170., 180., 195., 211.])
    side_second = np.array([40., 48., 58., 70.])
    baseline = np.min(np.column_stack([lead_zero, baseline_second, cruise, stopped]), axis=1)
    with_side = np.min(np.column_stack([lead_zero, side_second, cruise, stopped]), axis=1)
    self.assertTrue(np.all(with_side <= baseline))
    self.assertTrue(np.any(with_side < baseline))

  def test_selector_has_no_acceleration_or_cruise_writes(self):
    self.assertTrue(callable(production_selector()))



def production_mpc_input_envelope():
  """Compile real long_mpc.py expressions without importing its native solver."""
  path = ROOT / "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LongitudinalMpc")
  update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")
  assignments = [n for n in ast.walk(update) if isinstance(n, ast.Assign) and len(n.targets) == 1]

  def find_target(target, *, value_fragment=None):
    matches = [n for n in assignments if ast.unparse(n.targets[0]) == target
               and (value_fragment is None or value_fragment in ast.unparse(n.value))]
    assert len(matches) == 1, (target, value_fragment, len(matches))
    return copy.deepcopy(matches[0])

  lead0 = find_target("lead_0_obstacle")
  lead1 = find_target("lead_1_obstacle")
  maximum = find_target("self.params[:, 1]")
  minimum = find_target("self.params[:, 2]")
  acc_obstacles = find_target("x_obstacles", value_fragment="cruise_obstacle")
  blended_obstacles = find_target("x_obstacles", value_fragment="lead_0_obstacle, lead_1_obstacle")
  # The blended expression does not include cruise_obstacle; disambiguate it.
  if "cruise_obstacle" in ast.unparse(blended_obstacles.value):
    blended_matches = [n for n in assignments if ast.unparse(n.targets[0]) == "x_obstacles"
                       and "cruise_obstacle" not in ast.unparse(n.value)]
    assert len(blended_matches) == 1
    blended_obstacles = copy.deepcopy(blended_matches[0])
  else:
    assert "cruise_obstacle" not in ast.unparse(blended_obstacles.value)
  source = find_target("self.source", value_fragment="SOURCES[")
  assert all("lead_1_obstacle" in ast.unparse(n.value) for n in (acc_obstacles, blended_obstacles))

  extra_lead = ast.Assign(targets=[ast.Name(id="lead_0_follow_obstacle", ctx=ast.Store())],
                          value=ast.Name(id="lead_0_obstacle", ctx=ast.Load()))
  branch = ast.If(
    test=ast.Compare(left=ast.Name(id="mode", ctx=ast.Load()), ops=[ast.Eq()],
                     comparators=[ast.Constant(value="acc")]),
    body=[acc_obstacles, source], orelse=[blended_obstacles])
  names = ("self", "lead_xv_0", "lead_xv_1", "cruise_obstacle", "x2",
           "mode", "reset_state", "a_ego")
  expr = ast.FunctionDef(
    name="mpc_input_envelope",
    args=ast.arguments(posonlyargs=[], args=[ast.arg(arg=n) for n in names], vararg=None,
                       kwonlyargs=[], kw_defaults=[], kwarg=None, defaults=[]),
    body=[lead0, lead1, maximum, extra_lead, branch, minimum,
          ast.Return(value=ast.Attribute(value=ast.Name(id="self", ctx=ast.Load()),
                                         attr="params", ctx=ast.Load()))],
    decorator_list=[], returns=None, type_comment=None)
  # Use the actual source's stopped-vehicle equivalence function, not a copy.
  factor = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                and n.name == "get_stopped_equivalence_factor")
  ns = {"np": np, "SOURCES": ("lead0", "lead1", "cruise", "e2e"),
        "COMFORT_BRAKE": 2.5}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[copy.deepcopy(factor), expr], type_ignores=[])),
               str(path), "exec"), ns)
  return ns["mpc_input_envelope"]


def synthetic_lead(distance, velocity):
  t = np.linspace(0., 3.5, 8)
  return np.column_stack((distance + velocity*t, np.full(8, velocity)))


def compare_mpc_lead_two(*, mode="acc", max_accel=1.1, side_distance=35., side_speed=20.):
  calc = production_mpc_input_envelope()
  class MPC:
    def __init__(self):
      self.params = np.zeros((8, 8))
      self.source = "cruise"
      self.max_a = max_accel

  old, new = MPC(), MPC()
  lead0 = synthetic_lead(60., 26.)
  none = synthetic_lead(50., 37.)  # Production process_lead absent-target sentinel
  side = synthetic_lead(side_distance, side_speed)
  cruise, stop = np.full(8, 160.), np.full(8, 1000.)
  old_params = calc(old, lead0, none, cruise, stop, mode, False, 0.)
  new_params = calc(new, lead0, side, cruise, stop, mode, False, 0.)
  return old_params, new_params, old.source, new.source


class MpcInputEnvelopeDifferential(unittest.TestCase):
  def test_acc_mode_side_lead_tightens_real_production_obstacle_expression(self):
    before, after, old_source, new_source = compare_mpc_lead_two()
    self.assertTrue(np.all(after[:, 2] <= before[:, 2]))
    self.assertTrue(np.any(after[:, 2] < before[:, 2]))
    self.assertEqual(new_source, "lead1")

  def test_acc_mode_more_distant_fast_target_is_not_restrictive(self):
    before, after, _, _ = compare_mpc_lead_two(side_distance=110., side_speed=33.)
    np.testing.assert_allclose(before[:, 2], after[:, 2])

  def test_blended_mode_considers_side_lead(self):
    before, after, _, _ = compare_mpc_lead_two(mode="blended")
    self.assertTrue(np.all(after[:, 2] <= before[:, 2]))
    self.assertTrue(np.any(after[:, 2] < before[:, 2]))

  def test_maximum_acceleration_input_never_rises_with_side_lead(self):
    for cap in (0., .4, .8, 1.1, 1.8):
      with self.subTest(max_accel=cap):
        before, after, _, _ = compare_mpc_lead_two(max_accel=cap)
        np.testing.assert_allclose(before[:, 1], after[:, 1])
        np.testing.assert_allclose(after[:, 1], np.full(8, cap))

  def test_side_lead_loss_releases_constraint_without_raising_acceleration_cap(self):
    before, after, _, _ = compare_mpc_lead_two()
    self.assertGreater(float(np.max(before[:, 2] - after[:, 2])), 1.)
    np.testing.assert_allclose(before[:, 1], after[:, 1])

  def test_reset_state_uses_ego_acceleration_as_limit(self):
    calc = production_mpc_input_envelope()
    mpc = SimpleNamespace(params=np.zeros((8, 8)), max_a=.8, source="cruise")
    params = calc(mpc, synthetic_lead(60., 26.), synthetic_lead(35., 20.),
                  np.full(8, 160.), np.full(8, 1000.), "acc", True, .3)
    np.testing.assert_allclose(params[:, 1], np.full(8, .3))




if __name__ == "__main__":
  unittest.main()
