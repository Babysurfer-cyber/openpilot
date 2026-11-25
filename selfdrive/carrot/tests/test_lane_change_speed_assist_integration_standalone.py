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

  def test_selector_has_no_acceleration_or_cruise_writes(self):
    self.assertTrue(callable(production_selector()))


if __name__ == "__main__":
  unittest.main()
