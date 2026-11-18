"""Exercise the actual lateral-planner branch without loading the native MPC."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cereal import log

if "laneChangeBlinkerHold" not in log.ModelDataV2.MetaData.schema.fields:
  pytest.skip(
    "Queued lane-change feature was reverted; current behavior is covered by "
    "test_rollback_one_touch and test_rollback_lateral_metadata.",
    allow_module_level=True,
  )



ROOT = Path(__file__).resolve().parents[3]
PATH = 'selfdrive/controls/lib/lateral_planner.py'
tree = ast.parse((ROOT / PATH).read_text())
planner = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'LateralPlanner')
update = next(node for node in planner.body if isinstance(node, ast.FunctionDef) and node.name == 'update')
branches = [node for node in update.body if isinstance(node, ast.If) and any(
  isinstance(child, ast.Assign) and any(ast.unparse(target) == 'self.LP.lane_change_multiplier'
                                       for target in child.targets)
  for child in ast.walk(node)
)]
assert len(branches) == 1
ASSIGN_MULTIPLIER = compile(ast.fix_missing_locations(ast.Module(body=branches, type_ignores=[])), PATH, 'exec')


def model(*, state='preLaneChange', hold='left', probability=0., desire='none'):
  md = log.ModelDataV2.new_message()
  md.meta.laneChangeState = state
  md.meta.laneChangeBlinkerHold = hold
  md.meta.laneChangeProb = probability
  md.meta.desire = desire
  return md


def multiplier(md, *, atc=False):
  planner = NS(LP=NS(lane_change_multiplier=float('nan')))
  exec(ASSIGN_MULTIPLIER, {'np': np, 'log': log, 'self': planner, 'md': md, 'carrot': NS(atc_active=atc)})
  return planner.LP.lane_change_multiplier


@pytest.mark.parametrize('hold', ['left', 'right'])
def test_neutral_model_pulse_keeps_queued_lane_guidance_faded(hold):
  md = model(hold=hold)
  with log.ModelDataV2.from_bytes(md.to_bytes()) as received:
    assert received.meta.desire == log.Desire.none
    assert multiplier(received) == 0.


@pytest.mark.parametrize('hold', ['left', 'right'])
def test_blocked_bridge_gradually_restores_lane_guidance(hold):
  md = model(hold=hold)
  fades = [0., .05, .25, .5, .75, .95, 1.]
  values = []
  for fade in fades:
    md.meta.laneChangeProb = fade
    values.append(multiplier(md))
  assert values == pytest.approx(fades)
  assert values == sorted(values)


@pytest.mark.parametrize('state', ['off', 'preLaneChange', 'laneChangeStarting', 'laneChangeFinishing'])
@pytest.mark.parametrize('probability', [0., .5, 1.])
def test_ordinary_states_without_an_explicit_hold_preserve_lane_guidance(state, probability):
  assert multiplier(model(state=state, hold='none', probability=probability)) == 1.


@pytest.mark.parametrize('state', ['off', 'laneChangeStarting', 'laneChangeFinishing'])
def test_hold_without_a_neutral_pre_handoff_does_not_change_ordinary_behavior(state):
  assert multiplier(model(state=state)) == 1.


@pytest.mark.parametrize('desire', ['laneChangeLeft', 'laneChangeRight', 'turnLeft', 'turnRight', 'keepLeft', 'keepRight'])
@pytest.mark.parametrize('hold', ['none', 'left', 'right'])
def test_existing_active_desire_still_disables_lane_guidance(desire, hold):
  assert multiplier(model(desire=desire, hold=hold, probability=.5)) == 0.


@pytest.mark.parametrize('state', ['off', 'preLaneChange', 'laneChangeStarting', 'laneChangeFinishing'])
def test_existing_navigation_override_still_disables_lane_guidance(state):
  assert multiplier(model(state=state, hold='none', probability=1.), atc=True) == 0.


@pytest.mark.parametrize('probability', [float('nan'), float('inf'), -float('inf'), 1., 1.5])
def test_invalid_or_completed_bridge_fade_restores_lane_guidance(probability):
  assert multiplier(model(probability=probability)) == 1.


def test_negative_finite_bridge_fade_is_bounded():
  assert multiplier(model(probability=-.5)) == 0.
