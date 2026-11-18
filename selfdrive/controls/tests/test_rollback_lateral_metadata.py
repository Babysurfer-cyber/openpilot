"""Run production lane guidance with the rolled-back model metadata schema."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cereal import log


PATH = Path(__file__).resolve().parents[1] / 'lib/lateral_planner.py'
tree = ast.parse(PATH.read_text())
planner = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'LateralPlanner')
update = next(node for node in planner.body if isinstance(node, ast.FunctionDef) and node.name == 'update')
branches = [node for node in update.body if isinstance(node, ast.If) and any(
  isinstance(child, ast.Assign) and any(ast.unparse(target) == 'self.LP.lane_change_multiplier'
                                       for target in child.targets)
  for child in ast.walk(node)
)]
assert len(branches) == 1
ASSIGN_MULTIPLIER = compile(ast.fix_missing_locations(ast.Module(body=branches, type_ignores=[])), str(PATH), 'exec')


def multiplier(state, desire, *, atc=False):
  md = log.ModelDataV2.new_message()
  md.meta.laneChangeState = state
  md.meta.desire = desire
  md.meta.laneChangeProb = .5
  with log.ModelDataV2.from_bytes(md.to_bytes()) as received:
    planner = NS(LP=NS(lane_change_multiplier=float('nan')))
    exec(ASSIGN_MULTIPLIER, {'np': np, 'log': log, 'self': planner,
                             'md': received, 'carrot': NS(atc_active=atc)})
  return planner.LP.lane_change_multiplier


STATES = ('off', 'preLaneChange', 'laneChangeStarting', 'laneChangeFinishing')


@pytest.mark.parametrize('state', STATES)
def test_none_desire_uses_normal_lane_guidance_without_removed_metadata(state):
  assert multiplier(state, 'none') == 1.


@pytest.mark.parametrize('state', STATES)
@pytest.mark.parametrize('desire', ('laneChangeLeft', 'laneChangeRight', 'turnLeft', 'turnRight', 'keepLeft', 'keepRight'))
def test_active_desire_still_disables_lane_guidance(state, desire):
  assert multiplier(state, desire) == 0.


@pytest.mark.parametrize('state', STATES)
def test_navigation_override_still_disables_lane_guidance(state):
  assert multiplier(state, 'none', atc=True) == 0.
