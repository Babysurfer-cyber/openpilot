"""Standalone regression tests: run without openpilot native build dependencies.

Loads the actual production method syntax from the working tree, so ATC
mode persistence and LFA press cadence are exercised without hardware.
"""
import ast
import math
import unittest
from enum import IntEnum
from pathlib import Path
from types import SimpleNamespace


REPO = Path(__file__).resolve().parents[3]


class ButtonType(IntEnum):
  unknown = 0
  accelCruise = 1
  decelCruise = 2
  gapAdjustCruise = 3
  cancel = 4
  lfaButton = 5
  paddleLeft = 6
  paddleRight = 7


class FakeParams:
  def __init__(self, mode=2, enabled=True):
    self.values = {"AutoTurnControl": mode, "AutoTurnControlEnabled": enabled}
    self.writes = []

  def get_int(self, key):
    return self.values.get(key, 0)

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get_float(self, key):
    return float(self.get_int(key))

  def get(self, key, encoding=None):
    return "30" if key == "AutoCurveSpeedLowerLimit" else None

  def put_bool_nonblocking(self, key, value):
    self.values[key] = value
    self.writes.append((key, value))


def read_ast(path):
  return ast.parse((REPO / path).read_text(encoding="utf-8"))


def load_real_methods(path, class_name, names):
  """Compile the unchanged target methods with narrow test-only globals."""
  source = read_ast(path)
  cls = next(node for node in source.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  methods = [node for node in cls.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in names]
  assert {node.name for node in methods} == set(names), (class_name, names)
  test_cls = ast.ClassDef(name="ProductionMethods", bases=[], keywords=[], body=methods, decorator_list=[])
  module = ast.fix_missing_locations(ast.Module(body=[test_cls], type_ignores=[]))
  namespace = {"math": math, "ButtonType": ButtonType, "CV": SimpleNamespace(MPH_TO_KPH=1.60934)}
  exec(compile(module, str(path), "exec"), namespace)
  return namespace["ProductionMethods"]


class LanelessAtcStandalone(unittest.TestCase):
  def test_effective_atc_off_matches_menu_zero_and_restores_saved_mode(self):
    impl = load_real_methods("selfdrive/carrot/carrot_serv.py", "CarrotServ", {"update_params"})
    for mode in range(4):
      for enabled in (False, True):
        with self.subTest(configured=mode, enabled=enabled):
          obj = impl()
          obj.params = FakeParams(mode, enabled)
          obj.update_params()
          self.assertEqual(obj.autoTurnControl, mode if enabled else 0)
          self.assertEqual(obj.params.values["AutoTurnControl"], mode)

  def test_toggle_never_writes_saved_atc_mode(self):
    impl = load_real_methods("selfdrive/car/cruise.py", "VCruiseCarrot", {"_toggle_atc_enabled"})
    for mode in range(4):
      with self.subTest(configured=mode):
        obj = impl()
        obj.params = FakeParams(mode, True)
        obj._atc_toggle_enabled = True
        logs = []
        obj._add_log = logs.append
        obj._toggle_atc_enabled()
        self.assertFalse(obj._atc_toggle_enabled)
        self.assertEqual(obj.params.writes, [("AutoTurnControlEnabled", False)])
        self.assertEqual(obj.params.values["AutoTurnControl"], mode)
        self.assertEqual(logs[-1], "ATC OFF")
        obj._toggle_atc_enabled()
        self.assertTrue(obj._atc_toggle_enabled)
        self.assertEqual(obj.params.values["AutoTurnControl"], mode)
        self.assertEqual(obj.params.writes[-1], ("AutoTurnControlEnabled", True))
        self.assertEqual(logs[-1], "ATC ON" if mode else "ATC OFF")

  def test_lfa_press_emits_one_long_event_and_no_release_short_event(self):
    impl = load_real_methods("selfdrive/car/cruise.py", "VCruiseCarrot", {"_prepare_buttons"})
    obj = impl()
    obj.button_cnt = 0
    obj.button_prev = ButtonType.unknown
    obj.long_pressed = False
    obj._cruise_button_long_delay = 40
    obj.button_long_time = 50
    obj._cruise_speed_unit_basic = 1
    obj._cruise_speed_unit = 10
    obj._cruise_button_mode = 0
    obj.is_metric = True
    press = SimpleNamespace(type=ButtonType.lfaButton, pressed=True)
    release = SimpleNamespace(type=ButtonType.lfaButton, pressed=False)

    for _ in range(2):
      emitted = []
      for events in ([press],) + ([],) * 120 + ([release],):
        speed, event, long_press = obj._prepare_buttons(SimpleNamespace(buttonEvents=events), 80)
        self.assertEqual(speed, 80)
        if event:
          emitted.append((event, long_press))
      self.assertEqual(emitted, [(ButtonType.lfaButton, True)])

  def test_short_lfa_press_remains_short(self):
    impl = load_real_methods("selfdrive/car/cruise.py", "VCruiseCarrot", {"_prepare_buttons"})
    obj = impl()
    obj.button_cnt = 0
    obj.button_prev = ButtonType.unknown
    obj.long_pressed = False
    obj._cruise_button_long_delay = 40
    obj.button_long_time = 50
    obj._cruise_speed_unit_basic = 1
    obj._cruise_speed_unit = 10
    obj._cruise_button_mode = 0
    obj.is_metric = True
    obj._prepare_buttons(SimpleNamespace(buttonEvents=[
      SimpleNamespace(type=ButtonType.lfaButton, pressed=True)]), 80)
    _, event, long_press = obj._prepare_buttons(SimpleNamespace(buttonEvents=[
      SimpleNamespace(type=ButtonType.lfaButton, pressed=False)]), 80)
    self.assertEqual(event, ButtonType.lfaButton)
    self.assertFalse(long_press)

  def test_laneless_control_cannot_reenter_lane_mode(self):
    planner = read_ast("selfdrive/controls/lib/lateral_planner.py")
    mode_assignments = []
    for node in ast.walk(planner):
      if isinstance(node, (ast.Assign, ast.AnnAssign)):
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if any(isinstance(t, ast.Attribute) and t.attr in ("useLaneLineMode", "lanefull_mode") for t in targets):
          mode_assignments.append(node.value)
    self.assertTrue(mode_assignments)
    self.assertTrue(all(isinstance(value, ast.Constant) and value.value is False for value in mode_assignments))
    text = (REPO / "selfdrive/controls/lib/lateral_planner.py").read_text()
    self.assertNotIn('get_int("UseLaneLineSpeed")', text)

    cruise = read_ast("selfdrive/car/cruise.py")
    legacy_assignments = []
    for node in ast.walk(cruise):
      if isinstance(node, ast.Assign):
        for target in node.targets:
          if isinstance(target, ast.Attribute) and target.attr in ("useLaneLineSpeed", "useLaneLineSpeedApply"):
            legacy_assignments.append(node.value)
    self.assertTrue(legacy_assignments)
    self.assertTrue(all(isinstance(value, ast.Constant) and value.value == 0 for value in legacy_assignments))

  def test_old_lane_settings_hidden_but_lane_change_safety_options_stay(self):
    settings = (REPO / "selfdrive/ui/qt/offroad/settings.cc").read_text()
    for old_key in ("UseLaneLineSpeed", "UseLaneLineCurveSpeed", "ShowPathModeLane", "ShowPathColorLane"):
      self.assertNotIn('CValueControl("' + old_key + '"', settings)
    for safety_key in ("LaneChangeNeedTorque", "LaneChangeBsd", "LaneLineCheck"):
      self.assertIn('CValueControl("' + safety_key + '"', settings)
    ui = (REPO / "selfdrive/ui/carrot.cc").read_text()
    self.assertIn('atc_enabled = params.getBool("AutoTurnControlEnabled")', ui)
    self.assertIn('nvgRGBA(169, 118, 0, 210)', ui)
    self.assertIn('nvgRGBA(255, 228, 158, 210)', ui)
    params = (REPO / "common/params_keys.h").read_text()
    self.assertIn('"AutoTurnControlEnabled", {PERSISTENT, BOOL, "1"}', params)


if __name__ == "__main__":
  unittest.main()
