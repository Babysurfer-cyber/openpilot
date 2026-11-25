"""Independent tests for physical one-touch provenance in radar side-lead assistance.

Imports the production helper directly without the full openpilot runtime.
The cereal schema / modeld / DesireHelper wiring checks inspect production source.
No native MPC solver, Panda hardware, CAN TX, or vehicle control is exercised.
"""
from __future__ import annotations

import importlib.util
import sys
import unittest
from dataclasses import dataclass
from pathlib import Path
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

  def test_source_contract_has_unique_bool_schema_field(self):
    schema = (ROOT / "cereal/log.capnp").read_text(encoding="utf-8")
    self.assertEqual(schema.count("laneChangeDriverRequest @21 :Bool;"), 1)
    self.assertLess(schema.index("struct ModelDataV2 {"),
                    schema.index("laneChangeDriverRequest @21 :Bool;"))
    self.assertLess(schema.index("laneChangeDriverRequest @21 :Bool;"),
                    schema.index("struct EncodeIndex {"))

  def test_modeld_publishes_desirehelper_provenance_only_after_update(self):
    text = (ROOT / "selfdrive/modeld/modeld.py").read_text(encoding="utf-8")
    origin = "modelv2_send.modelV2.meta.laneChangeDriverRequest = bool(DH.lane_change_driver_request)"
    self.assertEqual(text.count(origin), 1)
    self.assertLess(text.index("DH.update(sm['carState']"), text.index(origin))

  def test_desirehelper_uses_driver_stalk_not_atc_for_provenance(self):
    text = (ROOT / "selfdrive/controls/lib/desire_helper.py").read_text(encoding="utf-8")
    self.assertIn("self.lane_change_driver_request = driver_enabled and self.one_touch_triggered", text)
    self.assertIn("if self.stalk_max_level == 2:", text)
    self.assertIn("self.one_touch_triggered = False", text)

  def test_helper_fails_closed_for_absent_field(self):
    text = SOURCE.read_text(encoding="utf-8")
    self.assertIn('getattr(meta, "laneChangeDriverRequest", False) is not True', text)


if __name__ == "__main__":
  unittest.main()
