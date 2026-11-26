"""Exercise the actual _prepare_buttons AST without importing the vehicle stack."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from selfdrive.carrot.atc_one_shot_skip import should_request_atc_skip


class MockButtonType:
  accelCruise = 1
  decelCruise = 2
  gapAdjustCruise = 3
  cancel = 4
  lfaButton = 5
  paddleLeft = 6
  paddleRight = 7


def prepare_function():
  path = Path(__file__).resolve().parents[2] / "car" / "cruise.py"
  tree = ast.parse(path.read_text(encoding="utf8"), filename=str(path))
  matches = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_prepare_buttons"]
  if len(matches) != 1:
    raise AssertionError("Expected exactly one real _prepare_buttons implementation")
  fn = matches[0]
  namespace = {"math": math, "ButtonType": MockButtonType,
               "CV": SimpleNamespace(MPH_TO_KPH=1.609344)}
  exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), "exec"), namespace)
  return namespace["_prepare_buttons"]


class TestPaddlePressClassifier(unittest.TestCase):
  def setUp(self):
    self.prepare = prepare_function()
    self.helper = SimpleNamespace(
      button_cnt=0, button_prev=0, long_pressed=False,
      _cruise_speed_unit_basic=1, _cruise_speed_unit=10,
      _cruise_button_mode=2, _cruise_button_long_delay=40,
      is_metric=True,
    )
    self.state = SimpleNamespace(buttonEvents=[])

  def frame(self, events=()):
    self.state.buttonEvents = [SimpleNamespace(type=kind, pressed=pressed)
                               for kind, pressed in events]
    return self.prepare(self.helper, self.state, 100)

  def request(self, event, long_pressed):
    return should_request_atc_skip(
      paddle_mode=4, button_type=event,
      left_paddle_type=MockButtonType.paddleLeft,
      long_pressed=long_pressed, cruise_enabled=True, cruise_available=True,
    )

  def test_left_short_click_only_on_release(self):
    self.assertEqual(self.frame([(MockButtonType.paddleLeft, True)])[1], 0)
    event = self.frame([(MockButtonType.paddleLeft, False)])
    self.assertEqual(event[1], MockButtonType.paddleLeft)
    self.assertFalse(event[2])
    self.assertTrue(self.request(event[1], event[2]))
    self.assertEqual(self.frame()[1], 0)

  def test_right_short_click_never_requests_skip(self):
    self.frame([(MockButtonType.paddleRight, True)])
    event = self.frame([(MockButtonType.paddleRight, False)])
    self.assertEqual(event[1], MockButtonType.paddleRight)
    self.assertFalse(self.request(event[1], event[2]))

  def test_left_hold_does_not_request_skip_on_threshold_or_release(self):
    self.frame([(MockButtonType.paddleLeft, True)])
    seen_long = []
    for _ in range(80):
      event = self.frame()
      if event[1]:
        seen_long.append(event)
    self.assertEqual(len(seen_long), 1)
    self.assertEqual(seen_long[0][1], MockButtonType.paddleLeft)
    self.assertTrue(seen_long[0][2])
    self.assertFalse(self.request(seen_long[0][1], seen_long[0][2]))
    released = self.frame([(MockButtonType.paddleLeft, False)])
    self.assertEqual(released[1], 0)
    self.assertEqual(self.frame()[1], 0)


if __name__ == "__main__":
  unittest.main()
