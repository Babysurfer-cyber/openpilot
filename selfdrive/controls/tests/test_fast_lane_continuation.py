"""Consecutive physical requests rearm the actual FSM without its old tail wait."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib import desire_helper as desire
from openpilot.selfdrive.controls.lib.desire_lib.next_lane import LaneCrossing
from openpilot.selfdrive.controls.tests.test_desire_continuation import MemoryParams, Rig, LCS, LCD


@pytest.fixture(autouse=True)
def no_device_params(monkeypatch):
  monkeypatch.setattr(desire, "Params", MemoryParams)


def advance_crossing(rig):
  """Stop on the first new-lane rearm frame, before the next model pulse."""
  for index, (left, right) in enumerate(rig.CROSSING):
    if rig.side == "right":
      left, right = -right, -left
    rig.geometry(left, right)
    state = rig.step()
    if state == LCS.preLaneChange:
      assert index >= 5  # No handoff during the approach to the first boundary.
      return index
    assert state == LCS.laneChangeStarting
  raise AssertionError("observed crossing did not rearm the queued change")


def model_pulse_code():
  """Use production ModelState.run's edge detector without native inference."""
  path = Path(__file__).resolve().parents[3] / "selfdrive/modeld/modeld.py"
  tree = ast.parse(path.read_text())
  model = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "ModelState")
  run = next(node for node in model.body if isinstance(node, ast.FunctionDef) and node.name == "run")
  targets = {"desire_input[0]", "new_desire", "self.prev_desire[:]"}
  nodes = [node for node in run.body if isinstance(node, ast.Assign)
           and len(node.targets) == 1 and ast.unparse(node.targets[0]) in targets]
  assert {ast.unparse(node.targets[0]) for node in nodes} == targets
  return compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(path), "exec")


@pytest.mark.parametrize("side", ["left", "right"])
def test_after_crossing_a_released_tap_hands_off_in_one_rearm_frame(side):
  rig = Rig(side)
  rig.start()
  rig.cross()
  rig.probability = .8  # The model has not declared the first action finished.
  rig.press(lever=1)
  before = rig.h.lane_change_ll_prob
  assert before < 1.
  assert not rig.h.queued_lane_change
  rig.cs.blinkerLever = 0
  assert rig.step() == LCS.preLaneChange
  assert rig.h.fast_lane_change_handoff
  assert rig.h.desire == log.Desire.none
  # Active changes use zero actual lane guidance in lateral_planner, even
  # while the FSM's internal finishing blend has not reached zero yet.
  assert rig.h.lane_change_ll_prob == 0.
  assert rig.h.queued_lane_change == rig.number
  assert rig.h.blinker_hold_direction == rig.direction
  assert rig.step() == LCS.laneChangeStarting
  assert rig.h.desire == (log.Desire.laneChangeLeft if side == "left" else log.Desire.laneChangeRight)
  assert rig.h.lane_change_ll_prob < 1.
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == rig.direction


@pytest.mark.parametrize("side", ["left", "right"])
def test_an_early_released_tap_waits_only_for_crossing_then_restarts(side):
  rig = Rig(side)
  rig.start()
  assert rig.tap() == LCS.laneChangeStarting
  assert rig.h.queued_lane_change == rig.number
  assert rig.step(8) == LCS.laneChangeStarting
  assert not rig.h.fast_lane_change_handoff
  advance_crossing(rig)
  assert rig.h.desire == log.Desire.none
  assert rig.step() == LCS.laneChangeStarting
  assert rig.h.queued_lane_change == 0


@pytest.mark.parametrize("side", ["left", "right"])
def test_consecutive_same_direction_changes_generate_two_actual_model_pulses(side):
  code = model_pulse_code()
  model = SimpleNamespace(prev_desire=np.zeros(8, dtype=np.float32))
  rig = Rig(side)
  raw_step = rig.step
  desires, pulses = [], []

  def step(frames=1):
    for _ in range(frames):
      state = raw_step()
      input_vector = np.zeros(8, dtype=np.float32)
      input_vector[int(rig.h.desire)] = 1.
      scope = {"np": np, "self": model, "desire_input": input_vector}
      exec(code, scope)
      desires.append(rig.h.desire)
      pulses.append(scope["new_desire"].copy())
    return state

  rig.step = step
  rig.start()
  rig.tap()  # A second physical request is accepted before relabeling.
  advance_crossing(rig)
  assert desires[-1] == log.Desire.none
  rig.step()
  target = int(log.Desire.laneChangeLeft if side == "left" else log.Desire.laneChangeRight)
  pulse_indices = [index for index, pulse in enumerate(pulses) if pulse[target] > .99]
  assert len(pulse_indices) == 2
  assert desires[pulse_indices[1] - 1] == log.Desire.none
  assert desires[pulse_indices[1]] == target
  assert not any(pulse[3 if target == 4 else 4] for pulse in pulses)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("gate", ["bsd", "object", "solid", "torque", "signal", "delay", "road_edge", "probability"])
def test_fast_handoff_rechecks_a_new_blocker_before_the_second_start(side, gate):
  rig = Rig(side)
  rig.queue()
  before = rig.h.lane_change_ll_prob
  if gate == "bsd":
    rig.block_bsd()
  elif gate == "object":
    obj = getattr(rig.radar, "leadLeft" if side == "left" else "leadRight")
    obj.status, obj.dRel, obj.vLead = True, 5., 0.
  elif gate == "solid":
    rig.h.laneLineCheck = 2
    setattr(rig.cs, side + "LaneLine", 21)
  elif gate == "torque":
    rig.h.laneChangeNeedTorque = 1
  elif gate == "signal":
    rig.lamps("none")
  elif gate == "delay":
    rig.h.lane_change_delay = .5
  elif gate == "road_edge":
    rig.geometry(adjacent=False)
  else:
    probabilities = list(rig.md.laneLineProbs)
    probabilities[0 if side == "left" else 3] = .1
    rig.md.laneLineProbs = probabilities
  assert rig.step() == LCS.preLaneChange
  assert rig.h.queued_lane_change == rig.number
  assert rig.h.desire == log.Desire.none
  assert rig.h.lane_change_ll_prob == pytest.approx(min(1., before + DT_MDL))


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("gate", ["bsd", "object", "solid", "torque", "signal", "delay"])
def test_a_blocked_next_lane_cannot_skip_the_current_completion_phase(side, gate):
  rig = Rig(side)
  rig.start()
  rig.cross()
  rig.press(lever=1)
  if gate == "bsd":
    rig.block_bsd()
  elif gate == "object":
    obj = getattr(rig.radar, "leadLeft" if side == "left" else "leadRight")
    obj.status, obj.dRel, obj.vLead = True, 5., 0.
  elif gate == "solid":
    rig.h.laneLineCheck = 2
    setattr(rig.cs, side + "LaneLine", 21)
  elif gate == "torque":
    rig.h.laneChangeNeedTorque = 1
  elif gate == "signal":
    rig.lamps("none")
  else:
    rig.h.lane_change_delay = .5
  rig.cs.blinkerLever = 0
  assert rig.step() == LCS.laneChangeStarting
  assert not rig.h.fast_lane_change_handoff
  assert rig.h.queued_lane_change == rig.number


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("change", ["no_adjacent", "wrong_side_outer", "wide_lane", "reversed_inner", "nan", "infinite"])
def test_fresh_geometry_rejects_a_second_lane_despite_old_available_average(side, change):
  rig = Rig(side)
  rig.start()
  rig.cross()
  outer_index, inner_index = (0, 1) if side == "left" else (3, 2)
  direction = -1. if side == "left" else 1.
  if change == "no_adjacent":
    rig.geometry(adjacent=False)
  elif change == "wrong_side_outer":
    inner = rig.md.laneLines[inner_index].y[0]
    rig.md.laneLines[outer_index].y = [inner - direction * 3.6] * 33
  elif change == "wide_lane":
    inner = rig.md.laneLines[inner_index].y[0]
    rig.md.laneLines[outer_index].y = [inner + direction * 7.] * 33
  elif change == "reversed_inner":
    rig.geometry(left=1., right=-1.)
  else:
    samples = list(rig.md.laneLines[outer_index].y)
    samples[0] = math.nan if change == "nan" else math.inf
    rig.md.laneLines[outer_index].y = samples
  assert rig.tap() == LCS.laneChangeStarting
  assert rig.h.queued_lane_change == rig.number
  assert not rig.h.fast_lane_change_handoff
  # One or two fresh samples cannot erase SideState's older width history.
  assert rig.selected.lane_available or rig.selected.edge_available


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("field", ["inner", "outer", "edge", "probabilities"])
def test_geometry_observer_rejects_missing_samples_without_assuming_a_next_lane(side, field):
  rig = Rig(side)
  if field == "probabilities":
    rig.md.laneLineProbs = []
  elif field == "edge":
    rig.md.roadEdges[0 if side == "left" else 1].y = []
  else:
    index = (1 if side == "left" else 2) if field == "inner" else (0 if side == "left" else 3)
    rig.md.laneLines[index].y = []
  assert not LaneCrossing.next_lane_geometry(rig.md, side == "left")


@pytest.mark.parametrize("side", ["left", "right"])
def test_fast_restart_does_not_finish_on_the_first_actions_old_zero_probability(side):
  rig = Rig(side)
  rig.queue()
  rig.probability = 0.
  assert rig.step() == LCS.laneChangeStarting
  for _ in range(round(desire.FAST_LANE_CHANGE_MODEL_SETTLE_S / DT_MDL) - 1):
    assert rig.step() == LCS.laneChangeStarting
  # Old zero confidence must not turn the new pulse immediately into finishing.
  assert rig.h.lane_change_timer >= desire.FAST_LANE_CHANGE_MODEL_SETTLE_S - DT_MDL


@pytest.mark.parametrize("side", ["left", "right"])
def test_a_pending_request_is_one_extra_lane_even_with_repeated_early_taps(side):
  rig = Rig(side)
  rig.start()
  rig.tap()
  rig.tap()
  assert rig.h.queued_lane_change == rig.number
  advance_crossing(rig)
  assert rig.step() == LCS.laneChangeStarting
  rig.geometry()
  rig.step()
  rig.cross()
  assert rig.finish() == LCS.off
  assert rig.h.followup_requires_press
  assert rig.step(30) == LCS.off
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none


@pytest.mark.parametrize("side", ["left", "right"])
def test_a_late_crossing_cannot_revive_an_expired_early_request(side):
  rig = Rig(side)
  rig.start()
  rig.tap()
  assert rig.h.queued_lane_change == rig.number
  rig.step(round((desire.QUEUED_LANE_WAIT_S + .2) / DT_MDL))
  assert rig.h.lane_change_state == LCS.laneChangeStarting
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none
  rig.cross()
  assert not rig.h.fast_lane_change_handoff
  assert rig.finish() == LCS.off
  assert rig.h.followup_requires_press
  assert rig.step(15) == LCS.off
  assert rig.tap() == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting


@pytest.mark.parametrize("side", ["left", "right"])
def test_opposite_press_held_and_released_during_fast_bridge_cancels_without_reversing(side):
  rig = Rig(side)
  rig.queue()
  opposite = "right" if side == "left" else "left"
  assert rig.press(opposite, lever=1) == LCS.off
  assert rig.step(10) == LCS.off
  rig.cs.blinkerLever = 0
  assert rig.step(15) == LCS.off
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none
  assert rig.h.desire == log.Desire.none
  assert rig.h.lane_change_direction == LCD.none
  assert rig.tap(opposite) == LCS.preLaneChange


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("held_direction", ["same", "opposite"])
@pytest.mark.parametrize("fault", ["lateral", "desire_disable", "timeout", "can"])
def test_a_held_press_during_fast_bridge_fault_cannot_replay_after_recovery(side, held_direction, fault):
  rig = Rig(side)
  rig.queue()
  requested = side if held_direction == "same" else ("right" if side == "left" else "left")
  if fault == "lateral":
    rig.active = False
  elif fault == "desire_disable":
    rig.h.desire_disable_count = 10
  elif fault == "timeout":
    rig.h.lane_change_timer = desire.LANE_CHANGE_TIME_MAX + .1
  else:
    rig.cs.canValid = False
  assert rig.press(requested, lever=1) == LCS.off
  assert rig.h.queued_lane_change == 0
  assert rig.h.blinker_hold_direction == LCD.none

  rig.active = rig.cs.canValid = True
  rig.h.desire_disable_count = 0
  rig.h.lane_change_timer = 0.
  assert rig.step(3) == LCS.off  # Recover while the same gesture remains held.
  rig.cs.blinkerLever = 0
  for _ in range(15):
    assert rig.step() == LCS.off
    assert rig.h.desire == log.Desire.none
    assert rig.h.blinker_hold_direction == LCD.none
    assert rig.h.queued_lane_change == 0
  # The canceled gesture was consumed; an actually new tap still works.
  assert rig.tap(requested) == LCS.preLaneChange
  assert rig.step() == LCS.laneChangeStarting
  assert rig.h.lane_change_direction == (LCD.left if requested == "left" else LCD.right)
