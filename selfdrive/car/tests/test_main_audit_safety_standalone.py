"""Regression checks for 2026-11-25 main branch audit (F1 through F4).

Only Python source/AST simulation and Panda source guards are checked here.
The Panda C safety tests and vehicle bench tests remain mandatory before merge.
"""
from __future__ import annotations

import ast
import copy
import math
import time
import unittest
from enum import IntEnum
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]


def read_tree(path):
  return ast.parse((ROOT / path).read_text(encoding="utf-8"))


def production_method(path, cls_name, method_name, globals_dict):
  cls = next(n for n in read_tree(path).body if isinstance(n, ast.ClassDef) and n.name == cls_name)
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == method_name)
  fake = ast.ClassDef(name="Production", bases=[], keywords=[], body=[copy.deepcopy(fn)], decorator_list=[])
  env = dict(globals_dict)
  exec(compile(ast.fix_missing_locations(ast.Module(body=[fake], type_ignores=[])), path, "exec"), env)
  return env["Production"]


class ButtonType(IntEnum):
  cancel = 4
  unknown = 0


class GearShifter:
  drive = "drive"
  park = "park"


class FakeParams:
  def __init__(self, value=1):
    self.value = value
    self.writes = []

  def get_int(self, key):
    return self.value

  def put_int(self, key, value):
    self.writes.append((key, value))


class AuditSafetyRegression(unittest.TestCase):
  def test_f1_passive_spas_registration_ignores_counter(self):
    path = "opendbc_repo/opendbc/car/hyundai/carstate.py"
    cls = next(n for n in read_tree(path).body if isinstance(n, ast.ClassDef) and n.name == "CarState")
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "get_can_parsers_canfd")
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == "_add_message"]
    self.assertTrue(any(any(k.arg == "ignore_counter" and isinstance(k.value, ast.Constant)
                            and k.value.value is True for k in n.keywords) for n in calls))
    text = (ROOT / path).read_text(encoding="utf-8")
    self.assertNotIn('msgs += [("SPAS1", math.nan), ("SPAS2", math.nan)]', text)

  def test_f2_host_rejects_stale_CAN_even_if_parked(self):
    path = "opendbc_repo/opendbc/car/hyundai/carcontroller.py"
    structs = SimpleNamespace(CarState=SimpleNamespace(GearShifter=GearShifter))
    cp = SimpleNamespace(carFingerprint="ev6")
    can = SimpleNamespace(ECAN=0)
    params = FakeParams(1)
    impl = production_method(path, "CarController", "_ev6_blinker_stalk_trial", {
      "Params": lambda: params, "CAR": SimpleNamespace(KIA_EV6="ev6"),
      "structs": structs,
    })
    controller = impl()
    controller.frame = 100
    controller.CP = cp
    controller.CAN = can
    controller.ev6_last_blinker_trial_frame = -1000
    controller.packer = SimpleNamespace(make_can_msg=lambda *args: self.fail("Unsafe TX invoked"))
    out = SimpleNamespace(canValid=False, canTimeout=False, gearShifter=GearShifter.park,
                          standstill=True, vEgo=0., brakePressed=True,
                          steeringPressed=False, leftBlinker=False, rightBlinker=False)
    cs = SimpleNamespace(out=out, cp=SimpleNamespace(ts_nanos={}), gear_msg_canfd="ACCELERATOR",
                         blinker_stalks={"COUNTER_ALT": 0})
    self.assertEqual(controller._ev6_blinker_stalk_trial(None, cs, 10**9), [])
    self.assertEqual(params.writes, [("Ev6BlinkerTest", 0)])

  def test_f2_panda_requires_valid_rx_and_park(self):
    text = (ROOT / "opendbc_repo/opendbc/safety/safety/safety_hyundai_canfd.h").read_text()
    gate = text.split("if (addr == 0x3C1) {", 1)[1].split("return allowed;", 1)[0]
    for guard in ("!safety_rx_checks_invalid", "!controls_allowed",
                  "hyundai_ev6_gear_park", "hyundai_ev6_parked_inputs_valid(now)",
                  "hyundai_ev6_gear_rx_us"):
      self.assertIn(guard, gate)
    self.assertIn("GET_BYTE(to_push, 24) & 0x7U", text)
    self.assertIn("valid_checksum", text)
    self.assertIn("wrong_counters", text)
    self.assertIn("valid_quality_flag", text)
    self.assertIn("120000U", text)

  def test_f3_cancel_and_disabled_controls_block_restore(self):
    path = "selfdrive/car/cruise.py"
    impl = production_method(path, "VCruiseCarrot", "_update_auto_mode", {
      "GearShifter": GearShifter, "ButtonType": ButtonType,
    })
    for enabled, pressed in ((False, False), (True, True)):
      with self.subTest(enabled=enabled, pressed=pressed):
        obj = impl()
        obj.button_prev = ButtonType.unknown
        obj.button_cnt = 0
        obj._cruise_cancel_state = False
        obj._activate_cruise = 0
        obj._carrot_restore_deferred_target = 100.
        obj._auto_mode_active_last = True
        obj._reset_auto_mode_session = lambda: setattr(obj, "_carrot_restore_deferred_target", None)
        cs = SimpleNamespace(gearShifter=GearShifter.drive,
                             cruiseState=SimpleNamespace(available=True),
                             brakePressed=False, regenBraking=False,
                             buttonEvents=[SimpleNamespace(type=ButtonType.cancel, pressed=True)] if pressed else [])
        cc = SimpleNamespace(enabled=enabled)
        speed = obj._update_auto_mode(cs, cc, 40., physical_speed_button=False,
                                      manual_speed_change=False, restore_speed=None)
        self.assertEqual(speed, 40.)
        self.assertIsNone(obj._carrot_restore_deferred_target)

  def test_f4_carrot_man_does_not_accept_5_second_old_message(self):
    path = "selfdrive/car/cruise.py"
    cls = next(n for n in read_tree(path).body if isinstance(n, ast.ClassDef) and n.name == "VCruiseCarrot")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update_v_cruise")
    assigns = [copy.deepcopy(n) for n in method.body if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id in ("carrot_recv", "carrot_fresh") for t in n.targets)]
    self.assertEqual(len(assigns), 2)
    code = ast.Module(body=assigns, type_ignores=[])
    now = 100.
    for age in (0.1, 5.0):
      sm = SimpleNamespace(recv_time={"carrotMan": now - age},
                           alive={"carrotMan": True}, valid={"carrotMan": True})
      mock_time = SimpleNamespace(monotonic=lambda: now)
      variables = {"sm": sm, "time": mock_time}
      exec(compile(ast.fix_missing_locations(copy.deepcopy(code)), path, "exec"), variables)
      self.assertEqual(variables["carrot_fresh"], age == 0.1)


  def test_o1_navigation_loss_without_prior_camera_does_not_create_hold(self):
    path = "selfdrive/car/cruise.py"
    cls = next(n for n in read_tree(path).body if isinstance(n, ast.ClassDef) and n.name == "VCruiseCarrot")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_update_auto_mode_target")
    nav_guard = [n for n in method.body if isinstance(n, ast.If)
                 and "navDataValid" in ast.unparse(n.test)]
    self.assertEqual(len(nav_guard), 1)
    # No unconditional camera_pending=True when nav data disappears.
    self.assertFalse(any(isinstance(t, ast.Assign)
                         and any(ast.unparse(v) == "self.camera_pending" for v in t.targets)
                         for t in ast.walk(ast.Module(body=nav_guard[0].body, type_ignores=[]))))

  def test_m1_shadow_speed_change_does_not_sound_prompt(self):
    impl = production_method("selfdrive/car/cruise.py", "VCruiseCarrot", "_auto_mode_prompt", {})
    instance = impl()
    instance.carrot_cruise_active = True
    instance._auto_mode_notification_sent = False
    instance._auto_mode_prompt("shadow 100→80")
    self.assertFalse(instance._auto_mode_notification_sent)


if __name__ == "__main__":
  unittest.main()
