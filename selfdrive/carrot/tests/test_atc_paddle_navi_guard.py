"""Unit-test real CarrotServ skip guard with lightweight fake car and navigation state."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from selfdrive.carrot.atc_one_shot_skip import AtcOneShotSkip, fresh_atc_skip_request, atc_skip_early_window_m


class FakeLaneChange:
  off = 0
  preLaneChange = 1
  laneChangeStarting = 2
  laneChangeFinishing = 3


def poll_function():
  path = Path(__file__).resolve().parents[1] / "carrot_serv.py"
  tree = ast.parse(path.read_text(encoding="utf8"), filename=str(path))
  matches = [node for node in ast.walk(tree)
             if isinstance(node, ast.FunctionDef) and node.name == "_poll_atc_one_shot_request"]
  if len(matches) != 1:
    raise AssertionError("Expected exactly one CarrotServ skip polling function")
  namespace = {
    "math": math,
    "time": SimpleNamespace(monotonic_ns=lambda: 100_000_000_000),
    "log": SimpleNamespace(LaneChangeState=FakeLaneChange),
    "fresh_atc_skip_request": fresh_atc_skip_request,
    "atc_skip_early_window_m": atc_skip_early_window_m,
  }
  exec(compile(ast.Module(body=[matches[0]], type_ignores=[]), str(path), "exec"), namespace)
  return namespace["_poll_atc_one_shot_request"]


class FakeSubMaster(dict):
  def __init__(self, entries):
    super().__init__(entries)
    self.alive = dict.fromkeys(entries, True)
    self.valid = dict.fromkeys(entries, True)


class ParamStore:
  def __init__(self, mode=4):
    self.mode = mode
    self.values = {}

  def get_int(self, key):
    return self.mode if key == "PaddleMode" else 0

  def get(self, key):
    return self.values.get(key)

  def put_nonblocking(self, key, val):
    self.values[key] = val


class NavState:
  def __init__(self, *, atc_mode=2, paddle_mode=4, cruise_kph=54,
               cruise_on=True, lane_state=FakeLaneChange.off, prev_decel=False):
    self.serv = SimpleNamespace(
      params=ParamStore(paddle_mode),
      params_memory=ParamStore(),
      _atc_skip_request_seen=None,
      atc_one_shot_skip=AtcOneShotSkip(),
      autoTurnControl=atc_mode,
      xTurnInfo=2, xDistToTurn=180., nRoadLimitSpeed=50.,
      _atc_speed_reduction_last=prev_decel,
      atc_guide_start_distance=lambda limit: 200.,
      atcSpeed=20., atcDist=6 * 20 / 3.6, autoNaviSpeedDecelRate=0.8,
    )
    self.serv.atc_one_shot_skip.observe(("nav", 1), 2, 180.)
    self.car_state = SimpleNamespace(vCruiseCluster=cruise_kph,
                                     cruiseState=SimpleNamespace(available=True))
    self.sm = FakeSubMaster({
      "carControl": SimpleNamespace(enabled=cruise_on),
      "modelV2": SimpleNamespace(meta=SimpleNamespace(laneChangeState=lane_state)),
      "carState": self.car_state,
    })
    self.poll = poll_function()

  def click(self, atc_kph=65., event_ns=99_990_000_000):
    self.serv.params_memory.put_nonblocking("AtcSkipRequest", str(event_ns))
    self.poll(self.serv, self.sm, self.car_state, atc_kph, "atc right")
    return self.serv.atc_one_shot_skip.skipped


class TestCarrotServPaddleSkipGuard(unittest.TestCase):
  def test_units_kph_eligible_before_decel(self):
    self.assertTrue(NavState(cruise_kph=54).click(atc_kph=65.))
    self.assertTrue(NavState(cruise_kph=65).click(atc_kph=65.))
    self.assertFalse(NavState(cruise_kph=100).click(atc_kph=65.))

  def test_invalid_cruise_or_disabled_modes(self):
    for settings in (
      {"cruise_kph": 0}, {"cruise_on": False}, {"paddle_mode": 3},
      {"prev_decel": True}, {"lane_state": FakeLaneChange.laneChangeStarting},
      {"lane_state": FakeLaneChange.laneChangeFinishing},
    ):
      with self.subTest(settings=settings):
        self.assertFalse(NavState(**settings).click())

  def test_one_event_not_replayed(self):
    state = NavState()
    self.assertTrue(state.click())
    state.serv.atc_one_shot_skip.reset()
    state.serv.atc_one_shot_skip.observe(("nav", 1), 2, 180.)
    self.assertFalse(state.click())

  def test_invalid_or_expired_request(self):
    self.assertFalse(NavState().click(event_ns=98_000_000_000))
    self.assertFalse(NavState().click(event_ns=100_000_000_001))


if __name__ == "__main__":
  unittest.main()
