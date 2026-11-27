"""Run production planner/MPC code with real association and a recording solver."""

from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cereal import car, log
from openpilot.selfdrive.controls.lib import human_like_following
from openpilot.selfdrive.controls.tests.test_human_like_following import Inputs, model_message, qualify
from openpilot.selfdrive.controls.tests.test_motion_longitudinal_integration import (
  PlannerMpc,
  TestParams as BaseParams,
  mpc_module,
  planner_module,
)


class RecordingPlannerMpc(PlannerMpc):
  def update(self, *args, **kwargs):
    super().update(*args, **kwargs)
    self.call_kwargs = kwargs


def planner_inputs():
  sm = Inputs()
  sm["carState"].vCruise = 72.
  sm["carState"].vCluRatio = 1.
  sm.update(carControl=car.CarControl.new_message(longActive=True),
            controlsState=NS(forceDecel=False, desiredCurvature=0.,
                             longControlState=car.CarControl.Actuators.LongControlState.pid),
            selfdriveState=log.SelfdriveState.new_message(enabled=True, experimentalMode=False,
                                                         personality="standard"),
            liveParameters=NS(angleOffsetDeg=0.))
  return sm


@pytest.fixture
def planner_setup(monkeypatch, planner_module):
  settings = {"enabled": True}

  class Params(BaseParams):
    def get_bool(self, key):
      assert key == "HumanLikeFollowing"
      return settings["enabled"]

  monkeypatch.setitem(planner_module, "Params", Params)
  monkeypatch.setitem(planner_module, "LongitudinalMpc", RecordingPlannerMpc)
  planner = planner_module["LongitudinalPlanner"](car.CarParams.new_message(openpilotLongitudinalControl=True), init_v=20.)
  sm = planner_inputs()
  monkeypatch.setattr(human_like_following.time, "monotonic", lambda: sm.now)
  carrot = NS(mode="acc", soft_hold_active=False, jerk_factor_apply=1., aChangeCostStarting=10.,
              lane_change_active=False, update=lambda *_: 72., get_carrot_accel=lambda _: 1.)
  return planner, sm, carrot, settings


def step_planner(planner, sm, carrot, time_s):
  sm.set_time(time_s)
  planner.update(sm, carrot)
  return planner.mpc.call_kwargs


def qualify_planner(planner, sm, carrot, *, start=10.):
  for index in range(3):
    kwargs = step_planner(planner, sm, carrot, start + .06 * index)
    assert kwargs["human_like_following_enabled"]
    assert (kwargs["human_following_prediction"] is not None) == (index == 2)
  return kwargs["human_following_prediction"]


def test_runtime_toggle_rebuilds_prediction_and_omits_disabled_mpc_kwargs(planner_setup):
  planner, sm, carrot, settings = planner_setup
  assert qualify_planner(planner, sm, carrot).radar_track_id == 11
  settings["enabled"] = False
  kwargs = step_planner(planner, sm, carrot, 10.18)
  assert "human_like_following_enabled" not in kwargs
  assert "human_following_prediction" not in kwargs
  settings["enabled"] = True
  qualify_planner(planner, sm, carrot, start=10.24)


@pytest.mark.parametrize("veto", ("blended", "reset", "force_decel", "stop", "lane_change", "inactive_long", "stock_long"))
def test_planner_eligibility_veto_discards_previous_prediction(planner_setup, veto):
  planner, sm, carrot, _ = planner_setup
  qualify_planner(planner, sm, carrot)
  if veto == "blended": carrot.mode = "blended"
  elif veto == "reset": sm["controlsState"].longControlState = car.CarControl.Actuators.LongControlState.off
  elif veto == "force_decel": sm["controlsState"].forceDecel = True
  elif veto == "stop": planner.output_should_stop = True
  elif veto == "lane_change": carrot.lane_change_active = True
  elif veto == "inactive_long": sm["carControl"].longActive = False
  elif veto == "stock_long": planner.CP.openpilotLongitudinalControl = False
  kwargs = step_planner(planner, sm, carrot, 10.18)
  assert not kwargs["human_like_following_enabled"]
  assert kwargs["human_following_prediction"] is None
  carrot.mode = "acc"
  carrot.lane_change_active = False
  sm["controlsState"].longControlState = car.CarControl.Actuators.LongControlState.pid
  sm["controlsState"].forceDecel = False
  sm["carControl"].longActive = True
  planner.CP.openpilotLongitudinalControl = True
  planner.output_should_stop = False
  qualify_planner(planner, sm, carrot, start=10.24)


def run_following_mpc(module, sm, prediction, *, enabled=True, mode="acc", reset=False, ego_bias=0.):
  mpc = module["LongitudinalMpc"](mode=mode)
  mpc.set_cur_state(sm["carState"].vEgo, 0.)
  mpc.set_accel_limits(-1.2, 1.5)
  mpc.solver.ego_bias = ego_bias
  carrot = NS(get_T_FOLLOW=lambda *_: 1.45, dynamic_t_follow=lambda current, *_: current,
              comfort_brake=2.5, stop_distance=6., j_lead_factor=1., v_cruise=40., stop_dist=1000.,
              mode=mode, trafficStopDistanceAdjust=0., autoNaviSpeedDecelRate=1.)
  arrays = [np.zeros(module["N"] + 1) for _ in range(4)]
  mpc.update(carrot, reset, sm["radarState"], 40., *arrays,
             human_like_following_enabled=enabled, human_following_prediction=prediction)
  return mpc


@pytest.mark.parametrize("speed", (10., 20., 30.))
def test_follow_obstacle_credit_has_immediate_prefix_and_distance_headway_caps(mpc_module, speed):
  sm = Inputs()
  sm["carState"].vEgo = speed
  sm["radarState"].leadOne.vLead = sm["radarState"].leadOne.vLeadK = speed
  sm["modelV2"] = model_message(velocity=speed, acceleration=1.)
  prediction = qualify(human_like_following.HumanLikeFollowing(), sm)
  baseline = run_following_mpc(mpc_module, sm, prediction, enabled=False)
  changed = run_following_mpc(mpc_module, sm, prediction)
  assert changed.human_like_following_active
  times = mpc_module["T_IDXS"]
  credit = changed.params[:, 2] - baseline.params[:, 2]
  np.testing.assert_array_equal(credit[times <= .5], 0.)
  assert np.any(credit[times > .5] > 0.)
  assert np.all(credit <= min(4., .2 * speed) + 1e-9)
  assert np.all(credit <= min(4., .2 * speed) * np.clip((times - .5) / 1.5, 0., 1.) + 1e-9)
  np.testing.assert_array_equal(changed.params[:, [0, 1, 3, 4, 5, 6, 7]],
                                baseline.params[:, [0, 1, 3, 4, 5, 6, 7]])


@pytest.mark.parametrize("changes", ({"enabled": False}, {"mode": "blended"}, {"reset": True}))
def test_disabled_or_ineligible_mpc_preserves_full_parameters(mpc_module, changes):
  sm = Inputs()
  prediction = qualify(human_like_following.HumanLikeFollowing(), sm)
  baseline = run_following_mpc(mpc_module, sm, None, **changes)
  changed = run_following_mpc(mpc_module, sm, prediction, **changes)
  np.testing.assert_array_equal(changed.params, baseline.params)
  assert not changed.human_like_following_active


def test_closer_independent_lead_two_remains_the_controlling_obstacle(mpc_module):
  sm = Inputs()
  prediction = qualify(human_like_following.HumanLikeFollowing(), sm)
  sm["radarState"].leadTwo = dict(status=True, radar=True, radarSource="corner235", radarTrackId=1205,
                                  dRel=12., vLead=15., aLeadK=0., aLeadTau=1.5)
  before = sm["radarState"].leadTwo.to_dict()
  baseline = run_following_mpc(mpc_module, sm, prediction, enabled=False)
  changed = run_following_mpc(mpc_module, sm, prediction)
  assert changed.source == baseline.source == "lead1"
  np.testing.assert_array_equal(changed.params, baseline.params)
  assert sm["radarState"].leadTwo.to_dict() == before


def test_fcw_uses_original_measured_trajectory_with_optimistic_prediction(mpc_module):
  sm = Inputs()
  sm["radarState"].leadOne.fcw = True
  prediction = qualify(human_like_following.HumanLikeFollowing(), sm)
  baseline = run_following_mpc(mpc_module, sm, prediction, enabled=False, ego_bias=60.)
  changed = run_following_mpc(mpc_module, sm, prediction, ego_bias=60.)
  assert changed.human_like_following_active
  assert baseline.crash_cnt == changed.crash_cnt == 1


@pytest.mark.parametrize("stale_field,value", (("radar_track_id", 12), ("radar_d_rel", 41.), ("radar_v_lead", 21.)))
def test_mpc_rechecks_prediction_against_current_primary(mpc_module, stale_field, value):
  sm = Inputs()
  prediction = qualify(human_like_following.HumanLikeFollowing(), sm)
  stale = replace(prediction, **{stale_field: value})
  baseline = run_following_mpc(mpc_module, sm, None)
  changed = run_following_mpc(mpc_module, sm, stale)
  np.testing.assert_array_equal(changed.params, baseline.params)
  assert not changed.human_like_following_active
