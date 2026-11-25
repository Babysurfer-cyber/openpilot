"""Independent tests for physical one-touch provenance in radar side-lead assistance.

Imports the production helper directly without the full openpilot runtime.
The cereal schema / modeld / DesireHelper wiring checks inspect production source.
No native MPC solver, Panda hardware, CAN TX, or vehicle control is exercised.
"""
from __future__ import annotations

import ast
import importlib.util
import sys
import unittest
from dataclasses import dataclass
from pathlib import Path
from random import Random
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "selfdrive/carrot/radar_motion/lane_change_speed_assist.py"


def load_production_helper():
  spec = importlib.util.spec_from_file_location("lane_change_origin_gate_under_test", SOURCE)
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  return module


ASSIST = load_production_helper()


@dataclass(frozen=True)
class Point:
  track_id: int = 10
  source: str = "corner235"
  d_rel: float = 35.
  y_rel: float = 3.6
  v_rel: float = -3.
  v_lead: float = 24.
  yv_rel: float = 0.
  measured: bool = True
  radar_track_state: int = 2


def model(state="off", direction="left", origin=True, *, with_field=True):
  meta = dict(laneChangeState=state, laneChangeDirection=direction)
  if with_field:
    meta["laneChangeDriverRequest"] = origin
  lines = tuple(SimpleNamespace(x=(0., 100.), y=(y, y))
                for y in (-5.4, -1.8, 1.8, 5.4))
  return SimpleNamespace(meta=SimpleNamespace(**meta),
                         laneLines=lines, laneLineProbs=(.95, .95, .95, .95))


def seed(assist, *, direction="left", point=None, origin=True, with_field=True):
  point = Point() if point is None else point
  for i in range(6):
    state = "off" if i < 3 else "preLaneChange"
    assert assist.update(i*.05, 27., model(state, direction, origin, with_field=with_field), (point,)) is None
  return assist.update(.30, 27., model("laneChangeStarting", direction, origin,
                                      with_field=with_field), (point,))


class OneTouchOriginGate(unittest.TestCase):
  def test_confirmed_physical_one_touch_left_can_start(self):
    self.assertIsNotNone(seed(ASSIST.LaneChangeSpeedAssist()))

  def test_confirmed_physical_one_touch_right_can_start(self):
    point = Point(y_rel=-3.6)
    self.assertIsNotNone(seed(ASSIST.LaneChangeSpeedAssist(), direction="right", point=point))

  def test_atc_start_cannot_create_side_lead(self):
    self.assertIsNone(seed(ASSIST.LaneChangeSpeedAssist(), origin=False))

  def test_no_provenance_from_old_cereal_fails_closed(self):
    self.assertIsNone(seed(ASSIST.LaneChangeSpeedAssist(), with_field=False))

  def test_missing_or_untrusted_provenance_values_fail_closed(self):
    for origin in (None, 0, 1, "True", "", False):
      with self.subTest(origin=origin):
        self.assertIsNone(seed(ASSIST.LaneChangeSpeedAssist(), origin=origin))

  def test_active_physical_request_drops_on_provenance_loss(self):
    assist = ASSIST.LaneChangeSpeedAssist()
    self.assertIsNotNone(seed(assist))
    self.assertIsNone(assist.update(.35, 27., model("laneChangeStarting", origin=False), (Point(),)))
    # No mid-manoeuvre reassociation even if the source becomes true again.
    self.assertIsNone(assist.update(.40, 27., model("laneChangeStarting", origin=True), (Point(),)))

  def test_atc_cannot_latch_late_physical_bit_mid_manoeuvre(self):
    assist = ASSIST.LaneChangeSpeedAssist()
    self.assertIsNone(seed(assist, origin=False))
    self.assertIsNone(assist.update(.35, 27., model("laneChangeStarting", origin=True), (Point(),)))

  def test_new_physical_request_after_reset_may_rearm(self):
    assist = ASSIST.LaneChangeSpeedAssist()
    self.assertIsNone(seed(assist, origin=False))
    self.assertIsNone(assist.update(.35, 27., model("off", origin=False), (Point(),)))
    for i in range(6):
      self.assertIsNone(assist.update(.40+i*.05, 27., model("preLaneChange"), (Point(),)))
    self.assertIsNotNone(assist.update(.70, 27., model("laneChangeStarting"), (Point(),)))

  def test_fsm_without_prechange_never_triggers_even_with_origin(self):
    assist = ASSIST.LaneChangeSpeedAssist()
    self.assertIsNone(assist.update(.30, 27., model("laneChangeStarting", origin=True), (Point(),)))

  def test_prechange_itself_never_brakes(self):
    assist = ASSIST.LaneChangeSpeedAssist()
    for i in range(10):
      self.assertIsNone(assist.update(i*.05, 27., model("preLaneChange", origin=True), (Point(),)))

  def test_randomized_atc_only_18k_frames_never_selects_side_lead(self):
    rng = Random(20261126)
    for _ in range(75):
      assist = ASSIST.LaneChangeSpeedAssist()
      for frame in range(240):
        state = rng.choice(("off", "preLaneChange", "laneChangeStarting", "laneChangeFinishing"))
        direction = rng.choice(("left", "right"))
        point = Point(y_rel=3.6 if direction == "left" else -3.6,
                      d_rel=rng.uniform(15., 75.),
                      v_lead=rng.uniform(4., 28.),
                      v_rel=rng.uniform(-12., -0.2))
        result = assist.update(frame*.05, 27., model(state, direction, origin=False), (point,))
        self.assertIsNone(result)

  def test_adversarial_nonphysical_origin_never_accepts_stable_side_track(self):
    # A coherent track, healthy lane geometry and active FSM cannot substitute
    # for the verified physical one-touch origin.
    rng = Random(12632026)
    for origin in (None, False, 0, 1, "True"):
      for trial in range(100):
        direction = "left" if trial % 2 == 0 else "right"
        point = Point(y_rel=3.6 if direction == "left" else -3.6,
                      d_rel=rng.uniform(15., 60.), v_rel=rng.uniform(-9., -0.01),
                      v_lead=rng.uniform(3., 25.))
        assist = ASSIST.LaneChangeSpeedAssist()
        self.assertIsNone(seed(assist, direction=direction, point=point, origin=origin),
                          (origin, direction, trial))

  def test_nonphysical_origin_cannot_reacquire_after_real_one_touch(self):
    assist = ASSIST.LaneChangeSpeedAssist()
    self.assertIsNotNone(seed(assist))
    self.assertIsNone(assist.update(.35, 27., model("laneChangeStarting", origin=None), (Point(),)))
    self.assertIsNone(assist.update(.40, 27., model("laneChangeStarting", origin=True), (Point(),)))
    self.assertIsNone(assist.update(.45, 27., model("laneChangeStarting", origin=True), (Point(),)))

  def test_source_contract_has_unique_bool_schema_field(self):
    schema = (ROOT / "cereal/log.capnp").read_text(encoding="utf-8")
    self.assertEqual(schema.count("laneChangeDriverRequest @21 :Bool;"), 1)
    car_schema = (ROOT / "cereal/car.capnp").read_text(encoding="utf-8")
    self.assertEqual(car_schema.count("blinkerStalkValid @96 :Bool;"), 1)
    self.assertLess(schema.index("struct ModelDataV2 {"),
                    schema.index("laneChangeDriverRequest @21 :Bool;"))
    self.assertLess(schema.index("laneChangeDriverRequest @21 :Bool;"),
                    schema.index("struct EncodeIndex {"))

  def test_modeld_publishes_desirehelper_provenance_only_after_update(self):
    text = (ROOT / "selfdrive/modeld/modeld.py").read_text(encoding="utf-8")
    origin = "modelv2_send.modelV2.meta.laneChangeDriverRequest = bool("
    self.assertEqual(text.count(origin), 1)
    self.assertIn("DH.lane_change_driver_request and sm.alive['carState'] and sm.valid['carState']", text)
    self.assertIn("and sm['carState'].canValid", text)
    self.assertIn("getattr(sm['carState'], 'blinkerStalkValid', False)", text)
    self.assertLess(text.index("DH.update(sm['carState']"), text.index(origin))

  def test_desirehelper_uses_driver_stalk_not_atc_for_provenance(self):
    text = (ROOT / "selfdrive/controls/lib/desire_helper.py").read_text(encoding="utf-8")
    self.assertIn("self.lane_change_driver_request = physical_one_touch", text)
    self.assertIn("and carstate.canValid", text)
    self.assertIn('getattr(carstate, "blinkerStalkValid", False)', text)
    self.assertIn("and self.stalk_max_level == 1", text)
    self.assertIn("carstate.blinkerLever in (0, 1)", text)
    self.assertIn("self.stalk_direction == BLINKER_LEFT", text)
    self.assertIn("self.stalk_direction == BLINKER_RIGHT", text)
    self.assertIn("if self.stalk_max_level == 2:", text)

  def test_desirehelper_origin_assignment_rejects_count_only_and_held_stalk(self):
    # Extract and execute the production condition without requiring cereal.
    path = ROOT / "selfdrive/controls/lib/desire_helper.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DesireHelper")
    update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")
    candidates = [
      n for n in ast.walk(update)
      if isinstance(n, ast.Assign) and len(n.targets) == 1
      and isinstance(n.targets[0], ast.Name) and n.targets[0].id == "physical_one_touch"
    ]
    self.assertEqual(len(candidates), 1)
    expr_code = compile(ast.Expression(body=candidates[0].value), str(path), "eval")

    def check(*, lever=0, max_level=1, triggered=True, driver=True, can_valid=True,
              stalk_valid=True, side=1, direction=1, completed=False, bsd=False):
      obj = SimpleNamespace(one_touch_triggered=triggered, stalk_max_level=max_level,
                            stalk_direction=side, lane_change_direction=direction,
                            one_touch_completed=completed, bsd_stalk_cancelled=bsd)
      context = {"self": obj, "driver_enabled": driver,
                 "carstate": SimpleNamespace(blinkerLever=lever, canValid=can_valid,
                                              blinkerStalkValid=stalk_valid),
                 "BLINKER_LEFT": 1, "BLINKER_RIGHT": 2,
                 "LaneChangeDirection": SimpleNamespace(left=1, right=2)}
      return eval(expr_code, context)

    self.assertTrue(check())
    self.assertTrue(check(lever=1))
    self.assertTrue(check(side=2, direction=2))
    for scenario in (
      {"max_level": 0},             # Counter edge, no observed depth 1
      {"max_level": 2},             # Full-lock/held lever
      {"lever": 2},
      {"triggered": False},
      {"driver": False},
      {"can_valid": False},   # main CAN receive not valid
      {"stalk_valid": False}, # optional physical lever frame is stale
      {"side": 2, "direction": 1},  # Opposite source direction
      {"side": 0},                  # Ambiguous stalk counts
      {"completed": True},
      {"bsd": True},
    ):
      with self.subTest(**scenario):
        self.assertFalse(check(**scenario))


  def test_carstate_optional_stalk_message_expires_independently_of_can_valid(self):
    # Run the real CarState parsing method on a synthetic CANParser clock.
    # A healthy main bus must not make an expired optional 0x3C1 usable.
    path = ROOT / "opendbc_repo/opendbc/car/hyundai/carstate.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CarState")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_update_blinker_stalks")
    namespace = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(path), "exec"), namespace)
    update = namespace["_update_blinker_stalks"]
    stalk = {"LEFT_BLINKER": 1, "RIGHT_BLINKER": 0,
             "LEFT_BLINKER_TAP": 1, "RIGHT_BLINKER_TAP": 0}
    for age_ns, present, expected in (
      (0, True, True), (249_000_000, True, True),
      (250_000_001, True, False), (300_000_000, True, False),
      (-1, True, False), (100_000_000, False, False),
    ):
      with self.subTest(age_ns=age_ns, present=present):
        car = SimpleNamespace(
          blinker_stalks=stalk if present else None,
          left_stalk_prev=False, right_stalk_prev=False, blinker_stalk_armed=True,
          left_blinker_stalk_count=0, right_blinker_stalk_count=0,
          _vehicle_navi_message_timestamp=lambda cp, name: max(cp.ts_nanos.get(name, {}).values(), default=0),
        )
        now = 1_000_000_000
        cp = SimpleNamespace(_last_update_nanos=now,
                             ts_nanos={"BLINKER_STALKS": {"LEFT_BLINKER": now - age_ns}} if present else {})
        result = SimpleNamespace(blinkerLever=0, blinkerStalkValid=False)
        update(car, result, cp)
        self.assertEqual(result.blinkerStalkValid, expected)
        self.assertEqual(result.blinkerLever, 1 if expected else 0)
        self.assertEqual(car.left_blinker_stalk_count, 1 if expected else 0)

  def test_stalk_rearm_rejects_pressed_first_frame_after_signal_loss(self):
    path = ROOT / "opendbc_repo/opendbc/car/hyundai/carstate.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CarState")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_update_blinker_stalks")
    scope = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(path), "exec"), scope)
    parser = scope["_update_blinker_stalks"]
    now = 1_000_000_000
    stalk = {"LEFT_BLINKER": 1, "RIGHT_BLINKER": 0, "LEFT_BLINKER_TAP": 1, "RIGHT_BLINKER_TAP": 0}
    owner = SimpleNamespace(
      blinker_stalks=stalk, blinker_stalk_armed=False,
      left_stalk_prev=False, right_stalk_prev=False,
      left_blinker_stalk_count=0, right_blinker_stalk_count=0,
      _vehicle_navi_message_timestamp=lambda cp, name: max(cp.ts_nanos.get(name, {}).values(), default=0),
    )
    def replay(age_ns, *, neutral=False):
      owner.blinker_stalks = ({name: 0 for name in stalk} if neutral else stalk)
      cp = SimpleNamespace(_last_update_nanos=now,
                           ts_nanos={"BLINKER_STALKS": {"LEFT_BLINKER": now - age_ns}})
      ret = SimpleNamespace(blinkerLever=0, blinkerStalkValid=False)
      parser(owner, ret, cp)
      return ret
    # Signal loss invalidates authorization immediately.
    self.assertFalse(replay(300_000_000).blinkerStalkValid)
    # First resumed press is not proof of a new tap.
    self.assertFalse(replay(10_000_000).blinkerStalkValid)
    self.assertEqual(owner.left_blinker_stalk_count, 0)
    # Once a new fresh neutral position was observed, allow the next tap.
    neutral = replay(10_000_000, neutral=True)
    self.assertTrue(neutral.blinkerStalkValid)
    self.assertEqual(neutral.blinkerLever, 0)
    pressed = replay(10_000_000)
    self.assertTrue(pressed.blinkerStalkValid)
    self.assertEqual((pressed.blinkerLever, owner.left_blinker_stalk_count), (1, 1))

  def test_modeld_retracts_stalk_provenance_after_message_timeout(self):
    path = ROOT / "selfdrive/modeld/modeld.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    writes = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
              and len(n.targets) == 1 and isinstance(n.targets[0], ast.Attribute)
              and n.targets[0].attr == "laneChangeDriverRequest"]
    self.assertEqual(len(writes), 1)
    expression = compile(ast.Expression(body=writes[0].value), str(path), "eval")
    def decide(*, lane=True, can_valid=True, stalk_valid=True, alive=True, valid=True):
      inputs = {"carState": SimpleNamespace(canValid=can_valid, blinkerStalkValid=stalk_valid)}
      class SubMasterStub(dict):
        def __init__(self):
          super().__init__(inputs)
          self.alive = {"carState": alive}
          self.valid = {"carState": valid}
      return eval(expression, {"DH": SimpleNamespace(lane_change_driver_request=lane),
                               "sm": SubMasterStub()})
    self.assertTrue(decide())
    self.assertFalse(decide(lane=False))
    self.assertFalse(decide(can_valid=False))
    self.assertFalse(decide(stalk_valid=False))
    self.assertFalse(decide(alive=False))
    self.assertFalse(decide(valid=False))

  def test_helper_fails_closed_for_absent_field(self):
    text = SOURCE.read_text(encoding="utf-8")
    self.assertIn('getattr(meta, "laneChangeDriverRequest", False) is not True', text)


if __name__ == "__main__":
  unittest.main()
