"""Run production planner/MPC methods with a recording optimizer, without acados.

The source AST is executed unchanged. Only the generated solver and process
infrastructure are replaced; obstacle construction and planner gates stay real.
This checks integration, not vehicle motion or optimizer convergence.
"""

import ast
import math
import os
import time
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cereal import car, log
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.cutin_predecel import (
  apply_cutin_predecel_accel_limit,
  get_cutin_predecel_accel_limit,
)
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longitudinal_cutout import cutout_obstacle_relief
from openpilot.selfdrive.modeld.constants import ModelConstants, index_function


LIB = Path(__file__).resolve().parents[1] / "lib"


class RecordingSolver:
  def __init__(self, _name, _solver_type, horizon):
    self.horizon = horizon
    self.reset()

  def reset(self):
    self.parameters = {}
    self.weights = {}
    self.ego_bias = 0.0

  def cost_set(self, stage, key, value):
    self.weights[stage, key] = np.array(value, copy=True)

  def set(self, stage, key, value):
    self.parameters[stage, key] = np.array(value, copy=True)

  def constraints_set(self, _stage, _key, _value):
    pass

  def solve(self):
    return 0

  def get_stats(self, _name):
    return np.array([0.0])

  def get(self, stage, key):
    if key == "u":
      return np.array([0.0])
    return np.array([self.ego_bias + 15.0 * index_function(stage, max_val=10.0, max_idx=self.horizon), 15.0, 0.0])


def production_definitions(path, namespace):
  tree = ast.parse(path.read_text(encoding="utf-8"))
  definitions = [node for node in tree.body if isinstance(node, (ast.Assign, ast.AnnAssign, ast.FunctionDef, ast.ClassDef))]
  exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), "exec"), namespace)
  return namespace


@pytest.fixture(scope="module")
def mpc_module():
  path = LIB / "longitudinal_mpc_lib/long_mpc.py"
  return production_definitions(path, {
    "__file__": str(path), "os": os, "time": time, "np": np, "log": log,
    "ACCEL_MIN": ACCEL_MIN, "DT_MDL": 0.05, "_LEAD_ACCEL_TAU": 1.5,
    "index_function": index_function, "cutout_obstacle_relief": cutout_obstacle_relief,
    "AcadosOcpSolverCython": RecordingSolver,
  })


def lead_message(distance=25.0, confidence=0.0):
  return log.RadarState.LeadData.new_message(
    status=True, radar=True, radarTrackId=50, radarSource="frontRadar",
    dRel=distance, vRel=-1.0, vLead=14.0, vLeadK=14.0,
    aLeadK=-0.5, aLeadTau=1.5, modelProb=0.99,
    cutOutTime=1.0, cutOutConfidence=confidence,
  )


def run_mpc(module, *, confidence=0.0, mode="acc", reset=False, enabled=True,
            second_distance=100.0, stop_distance=1000.0, ego_bias=0.0):
  mpc = module["LongitudinalMpc"](mode=mode)
  mpc.set_cur_state(15.0, 0.0)
  mpc.set_accel_limits(-1.2, 1.5)
  mpc.solver.ego_bias = ego_bias
  radar = log.RadarState.new_message()
  radar.leadOne = lead_message(confidence=confidence)
  radar.leadTwo = lead_message(second_distance)
  carrot = NS(get_T_FOLLOW=lambda *_: 1.45,
              dynamic_t_follow=lambda current, *_: current,
              comfort_brake=2.5, stop_distance=6.0, j_lead_factor=1.0,
              v_cruise=20.0, stop_dist=stop_distance, mode=mode,
              trafficStopDistanceAdjust=0.0, autoNaviSpeedDecelRate=1.0)
  arrays = [np.zeros(module["N"] + 1) for _ in range(4)]
  mpc.update(carrot, reset, radar, 20.0, *arrays, cutout_relief_enabled=enabled)
  return mpc


def test_cutout_changes_only_postclearance_follow_obstacle(mpc_module):
  baseline = run_mpc(mpc_module)
  changed = run_mpc(mpc_module, confidence=1.0)
  times = mpc_module["T_IDXS"]
  np.testing.assert_array_equal(changed.params[times <= 1.3], baseline.params[times <= 1.3])
  assert np.any(changed.params[:, 2] > baseline.params[:, 2])
  np.testing.assert_array_equal(changed.params[:, [0, 1, 3, 4, 5, 6, 7]], baseline.params[:, [0, 1, 3, 4, 5, 6, 7]])
  assert changed.source == baseline.source
  for stage in range(mpc_module["N"] + 1):
    np.testing.assert_array_equal(changed.solver.parameters[stage, "p"], changed.params[stage])


@pytest.mark.parametrize("changes", [
  {"mode": "blended"}, {"reset": True}, {"enabled": False},
  {"second_distance": 10.0}, {"stop_distance": 5.0},
])
def test_other_obstacles_and_disabled_modes_keep_existing_plan(mpc_module, changes):
  baseline = run_mpc(mpc_module, **changes)
  changed = run_mpc(mpc_module, confidence=1.0, **changes)
  np.testing.assert_array_equal(changed.params, baseline.params)
  assert changed.source == baseline.source


def test_cutout_does_not_relax_fcw_measurements(mpc_module):
  baseline = run_mpc(mpc_module, ego_bias=60.0)
  changed = run_mpc(mpc_module, confidence=1.0, ego_bias=60.0)
  assert baseline.crash_cnt == changed.crash_cnt == 1


class TestParams:
  def get_int(self, _key):
    return 0

  def get_float(self, key):
    return {"LongActuatorDelay": 20, "VEgoStopping": 5}.get(key, 0)


class PlannerMpc:
  def __init__(self, dt):
    self.mode = "acc"
    self.prev_a = np.zeros(13)
    self.v_solution = np.full(13, 15.0)
    self.a_solution = np.zeros(13)
    self.j_solution = np.zeros(12)
    self.crash_cnt = 0

  def set_weights(self, *_args, **_kwargs):
    pass

  def set_accel_limits(self, minimum, maximum):
    self.accel_limits = minimum, maximum

  def set_cur_state(self, speed, acceleration):
    self.current_state = speed, acceleration

  def update(self, *_args, **kwargs):
    self.cutout_enabled = kwargs["cutout_relief_enabled"]


@pytest.fixture(scope="module")
def planner_module(mpc_module):
  path = LIB / "longitudinal_planner.py"
  return production_definitions(path, {
    "__file__": str(path), "math": math, "np": np, "log": log,
    "CV": CV, "ACCEL_MIN": ACCEL_MIN, "ACCEL_MAX": ACCEL_MAX, "DT_MDL": 0.05,
    "CONTROL_N": CONTROL_N, "ModelConstants": ModelConstants,
    "FirstOrderFilter": FirstOrderFilter, "LongitudinalMpc": PlannerMpc,
    "N": mpc_module["N"], "T_IDXS_MPC": mpc_module["T_IDXS"],
    "LongCtrlState": car.CarControl.Actuators.LongControlState,
    "V_CRUISE_MAX": 145.0, "V_CRUISE_UNSET": 255.0, "Params": TestParams,
    "get_accel_from_plan": get_accel_from_plan,
    "get_cutin_predecel_accel_limit": get_cutin_predecel_accel_limit,
    "apply_cutin_predecel_accel_limit": apply_cutin_predecel_accel_limit,
  })


class PlannerInputs(dict):
  def __init__(self, *, fresh=True, gas=False, reset=False, force=False, lane_change="off", risk=True):
    model = log.ModelDataV2.new_message()
    model.meta.laneChangeState = lane_change
    model.action.desiredVelocity = 15.0
    radar = log.RadarState.new_message()
    radar.leadCutInRisk.status = risk
    radar.leadCutInRisk.radar = True
    radar.leadCutInRisk.score = 0.9
    radar.leadCutInRisk.dRel = 20.0
    radar.leadCutInRisk.vRel = -6.5
    super().__init__(
      carControl=NS(orientationNED=[]),
      carState=NS(vEgo=15.0, aEgo=0.0, vCruise=72.0, vCluRatio=1.0,
                  steeringAngleDeg=0.0, standstill=False, gasPressed=gas),
      controlsState=NS(forceDecel=force, desiredCurvature=0.0,
                       longControlState=car.CarControl.Actuators.LongControlState.off if reset else car.CarControl.Actuators.LongControlState.pid),
      selfdriveState=NS(enabled=not reset, experimentalMode=False, personality=log.LongitudinalPersonality.standard),
      liveParameters=NS(angleOffsetDeg=0.0), modelV2=model, radarState=radar,
    )
    self.fresh = fresh

  def all_checks(self, service_list):
    assert service_list == ["radarState"]
    return self.fresh


def run_planner(module, *, stop=False, carrot_lane_change=False, **inputs):
  planner = module["LongitudinalPlanner"](NS(openpilotLongitudinalControl=True), init_v=15.0)
  planner.output_should_stop = stop
  carrot = NS(mode="acc", soft_hold_active=False, jerk_factor_apply=1.0,
              aChangeCostStarting=10.0, lane_change_active=carrot_lane_change,
              update=lambda *_: 72.0, get_carrot_accel=lambda _: 1.0)
  planner.update(PlannerInputs(**inputs), carrot)
  return planner


def test_production_planner_passes_risk_ceiling_into_mpc(planner_module):
  ordinary = run_planner(planner_module, risk=False)
  changed = run_planner(planner_module)
  assert ordinary.mpc.accel_limits[1] == 1.0
  assert changed.mpc.accel_limits[1] == pytest.approx(-0.15)
  assert changed.mpc.cutout_enabled


@pytest.mark.parametrize("changes", [{"fresh": False}, {"gas": True}, {"reset": True}])
def test_production_planner_ignores_stale_or_overridden_risk(planner_module, changes):
  baseline = run_planner(planner_module, risk=False, **changes)
  changed = run_planner(planner_module, **changes)
  assert changed.mpc.accel_limits == baseline.mpc.accel_limits
  assert not changed.mpc.cutout_enabled


@pytest.mark.parametrize("changes", [
  {"force": True}, {"stop": True}, {"lane_change": "preLaneChange"},
  {"lane_change": "laneChangeStarting"}, {"lane_change": "laneChangeFinishing"},
  {"carrot_lane_change": True},
])
def test_production_planner_vetoes_cutout_during_stop_or_lane_change(planner_module, changes):
  assert not run_planner(planner_module, **changes).mpc.cutout_enabled
