"""Run the production planner and obstacle construction with native radar messages.

The optimizer records its inputs; generated acados behavior is tested separately.
"""

from types import SimpleNamespace as NS

import numpy as np
import pytest
from cereal import log

from openpilot.selfdrive.controls.tests.test_human_like_lane_changes import Inputs, lead
from openpilot.selfdrive.controls.tests.test_motion_longitudinal_integration import (
  PlannerInputs,
  mpc_module,
  planner_module,
)


# Imported fixtures intentionally share the existing production-AST harness.
__all__ = ["mpc_module", "planner_module"]


def run_lane_mpc(module, *, enabled=True, target=None, mode="acc", reset=False,
                 primary=None, secondary=None, ego_bias=0.0, omit_optional=False):
  mpc = module["LongitudinalMpc"](mode=mode)
  mpc.set_cur_state(15.0, 0.0)
  mpc.set_accel_limits(-1.2, 1.5)
  mpc.solver.ego_bias = ego_bias
  radar = log.RadarState.new_message()
  radar.leadOne = lead(10, 70.0, 0.0, speed=15.0) if primary is None else primary
  if secondary is not None:
    radar.leadTwo = secondary
  before = (radar.leadOne.to_dict(), radar.leadTwo.to_dict())
  carrot = NS(get_T_FOLLOW=lambda *_: 1.45, dynamic_t_follow=lambda current, *_: current,
              comfort_brake=2.5, stop_distance=6.0, j_lead_factor=1.0,
              v_cruise=20.0, stop_dist=1000.0, mode=mode,
              trafficStopDistanceAdjust=0.0, autoNaviSpeedDecelRate=1.0)
  arrays = [np.zeros(module["N"] + 1) for _ in range(4)]
  optional = {} if omit_optional else dict(human_lane_changes_enabled=enabled, lane_change_lead=target)
  mpc.update(carrot, reset, radar, 20.0, *arrays, **optional)
  assert (radar.leadOne.to_dict(), radar.leadTwo.to_dict()) == before
  return mpc


def test_destination_lead_adds_elementwise_minimum_constraint_and_existing_source_enum(mpc_module):
  target = lead(41, 22.0, speed=8.0)
  baseline = run_lane_mpc(mpc_module)
  changed = run_lane_mpc(mpc_module, target=target)
  predicted, _ = changed.process_lead(target, 0.0)
  obstacle = predicted[:, 0] + mpc_module["get_stopped_equivalence_factor"](predicted[:, 1])
  np.testing.assert_allclose(changed.params[:, 2], np.minimum(baseline.params[:, 2], obstacle))
  assert np.any(changed.params[:, 2] < baseline.params[:, 2])
  np.testing.assert_array_equal(changed.params[:, [0, 1, 3, 4, 5, 6, 7]], baseline.params[:, [0, 1, 3, 4, 5, 6, 7]])
  assert changed.source == "lead1"
  assert changed.human_lane_changes_active
  assert changed.human_lane_change_track_id == 41
  wire_plan = log.LongitudinalPlan.new_message(longitudinalPlanSource=changed.source)
  with log.LongitudinalPlan.from_bytes(wire_plan.to_bytes()) as decoded:
    assert str(decoded.longitudinalPlanSource) == "lead1"


def test_closer_stopped_physical_lead_two_keeps_its_complete_constraint(mpc_module):
  secondary = lead(11, 8.0, 0.0, speed=0.0)
  baseline = run_lane_mpc(mpc_module, secondary=secondary)
  changed = run_lane_mpc(mpc_module, target=lead(41, 25.0, speed=8.0), secondary=secondary)
  np.testing.assert_array_equal(changed.params, baseline.params)
  assert changed.source == baseline.source == "lead1"


def test_destination_obstacle_can_be_more_restrictive_than_closer_fast_lead_two(mpc_module):
  secondary = lead(11, 10.0, 0.0, speed=20.0)
  baseline = run_lane_mpc(mpc_module, secondary=secondary)
  target = lead(41, 25.0, speed=0.0)
  changed = run_lane_mpc(mpc_module, target=target, secondary=secondary)
  predicted, _ = changed.process_lead(target, 0.0)
  obstacle = predicted[:, 0] + mpc_module["get_stopped_equivalence_factor"](predicted[:, 1])
  np.testing.assert_allclose(changed.params[:, 2], np.minimum(baseline.params[:, 2], obstacle))
  assert np.any(changed.params[:, 2] < baseline.params[:, 2])


@pytest.mark.parametrize("changes", [{"enabled": False}, {"reset": True}, {"mode": "blended"}])
def test_toggle_reset_and_blended_mode_leave_obstacle_plan_unchanged(mpc_module, changes):
  baseline = run_lane_mpc(mpc_module, **changes)
  changed = run_lane_mpc(mpc_module, target=lead(41, 12.0, speed=2.0), **changes)
  np.testing.assert_array_equal(changed.params, baseline.params)
  assert changed.source == baseline.source


@pytest.mark.parametrize("target", [
  lead(10, 20.0), lead(41, 20.0, status=False), lead(41, 20.0, radar=False),
  lead(41, 20.0, dRel=float("nan")), lead(41, 20.0, vLeadK=float("nan")),
])
def test_mpc_rejects_invalid_or_duplicate_target_without_changing_plan(mpc_module, target):
  baseline = run_lane_mpc(mpc_module)
  changed = run_lane_mpc(mpc_module, target=target)
  np.testing.assert_array_equal(changed.params, baseline.params)


def test_optional_arguments_default_to_existing_mpc_behavior(mpc_module):
  # A call without the new arguments must remain usable by existing callers.
  baseline = run_lane_mpc(mpc_module, omit_optional=True)
  changed = run_lane_mpc(mpc_module, enabled=False, target=lead(41, 10.0))
  np.testing.assert_array_equal(changed.params, baseline.params)


def test_destination_target_does_not_replace_primary_fcw_evidence(mpc_module):
  baseline = run_lane_mpc(mpc_module, ego_bias=30.0)
  changed = run_lane_mpc(mpc_module, target=lead(41, 5.0, speed=0.0), ego_bias=30.0)
  assert changed.crash_cnt == baseline.crash_cnt == 0


class ToggleParams:
  def __init__(self, enabled):
    self.enabled = enabled

  def get_bool(self, key):
    return self.enabled if key == "HumanLikeLaneChanges" else False

  def get_int(self, _key):
    return 0

  def get_float(self, key):
    return {"LongActuatorDelay": 20, "VEgoStopping": 5}.get(key, 0)


class LanePlannerInputs(PlannerInputs):
  def __init__(self, time_s, *, state="preLaneChange", direction="left", candidates=None, **kwargs):
    super().__init__(lane_change=state, risk=False, **kwargs)
    rich = Inputs(time_s, state=state, direction=direction,
                  desire=("laneChangeLeft" if direction == "left" else "laneChangeRight") if state == "laneChangeStarting" else "none",
                  candidates=candidates, primary=lead(10, 70.0, 0.0, speed=15.0))
    self.update(modelV2=rich["modelV2"], radarState=rich["radarState"])
    self["modelV2"].action.desiredVelocity = 15.0
    self["carState"].canValid = True
    self["carControl"].longActive = self["carControl"].latActive = True
    self["carState"].steeringPressed = False
    self["carState"].steeringTorque = 0.0
    for attr in ("logMonoTime", "recv_time", "seen", "valid", "alive", "updated"):
      setattr(self, attr, getattr(rich, attr))
    if not self.fresh:
      self.valid["radarState"] = False

  def all_checks(self, service_list=None):
    names = ("modelV2", "radarState", "carState") if service_list is None else service_list
    return self.fresh and all(self.seen[name] and self.valid[name] and self.alive[name] for name in names)


def planner_with_toggle(module, monkeypatch, enabled):
  settings = ToggleParams(enabled)
  monkeypatch.setitem(module, "Params", lambda: settings)
  planner = module["LongitudinalPlanner"](NS(openpilotLongitudinalControl=True), init_v=15.0)
  carrot = NS(mode="acc", soft_hold_active=False, jerk_factor_apply=1.0,
              aChangeCostStarting=10.0, lane_change_active=False,
              update=lambda *_: 72.0, get_carrot_accel=lambda _: 1.0)
  return settings, planner, carrot


def planner_step(planner, carrot, monkeypatch, time_s, **kwargs):
  monkeypatch.setattr("time.monotonic", lambda: time_s)
  sm = LanePlannerInputs(time_s, **kwargs)
  planner.update(sm, carrot)
  return sm


@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("direction", ["left", "right"])
def test_real_planner_reads_toggle_and_passes_native_destination_target(planner_module, monkeypatch, enabled, direction):
  _, planner, carrot = planner_with_toggle(planner_module, monkeypatch, enabled)
  candidates = [lead(lateral=3.6 if direction == "left" else -3.6)]
  for time_s in (9.90, 9.95):
    planner_step(planner, carrot, monkeypatch, time_s, direction=direction, candidates=candidates)
  sm = planner_step(planner, carrot, monkeypatch, 10.0, state="laneChangeStarting", direction=direction, candidates=candidates)
  assert planner.mpc.update_kwargs.get("human_lane_changes_enabled", False) == enabled
  selected = planner.mpc.update_kwargs.get("lane_change_lead")
  assert (selected is not None) == enabled
  if enabled:
    assert selected.radarTrackId == 41
  assert sm["radarState"].leadOne.radarTrackId == 10
  assert not sm["radarState"].leadTwo.status


def test_real_planner_live_disable_and_late_enable_wait_for_next_request(planner_module, monkeypatch):
  settings, planner, carrot = planner_with_toggle(planner_module, monkeypatch, True)
  for time_s in (9.90, 9.95):
    planner_step(planner, carrot, monkeypatch, time_s)
  planner_step(planner, carrot, monkeypatch, 10.0, state="laneChangeStarting")
  assert planner.mpc.update_kwargs.get("lane_change_lead") is not None
  settings.enabled = False
  planner_step(planner, carrot, monkeypatch, 10.05, state="laneChangeStarting")
  assert not planner.mpc.update_kwargs.get("human_lane_changes_enabled", False)
  assert planner.mpc.update_kwargs.get("lane_change_lead") is None
  settings.enabled = True
  planner_step(planner, carrot, monkeypatch, 10.10, state="laneChangeStarting")
  assert planner.mpc.update_kwargs.get("lane_change_lead") is None
  for time_s in (10.15, 10.20):
    planner_step(planner, carrot, monkeypatch, time_s)
  planner_step(planner, carrot, monkeypatch, 10.25, state="laneChangeStarting")
  assert planner.mpc.update_kwargs.get("lane_change_lead") is not None


@pytest.mark.parametrize("changes", [
  {"fresh": False}, {"gas": True}, {"reset": True}, {"force": True},
])
def test_real_planner_overrides_clear_destination_target(planner_module, monkeypatch, changes):
  _, planner, carrot = planner_with_toggle(planner_module, monkeypatch, True)
  for time_s in (9.90, 9.95):
    planner_step(planner, carrot, monkeypatch, time_s)
  planner_step(planner, carrot, monkeypatch, 10.0, state="laneChangeStarting", **changes)
  assert planner.mpc.update_kwargs.get("lane_change_lead") is None


@pytest.mark.parametrize("veto", ["lateral_off", "longitudinal_off", "disengaged", "blended", "external_longitudinal", "stopping"])
def test_real_planner_control_and_mode_gates_veto_supplemental_lead(planner_module, monkeypatch, veto):
  _, planner, carrot = planner_with_toggle(planner_module, monkeypatch, True)
  for time_s in (9.90, 9.95):
    planner_step(planner, carrot, monkeypatch, time_s)
  monkeypatch.setattr("time.monotonic", lambda: 10.0)
  sm = LanePlannerInputs(10.0, state="laneChangeStarting")
  if veto == "lateral_off":
    sm["carControl"].latActive = False
  elif veto == "longitudinal_off":
    sm["carControl"].longActive = False
  elif veto == "disengaged":
    sm["selfdriveState"].enabled = False
    sm["carControl"].longActive = sm["carControl"].latActive = False
  elif veto == "blended":
    sm["selfdriveState"].experimentalMode = True
    carrot.mode = "blended"
  elif veto == "external_longitudinal":
    planner.CP.openpilotLongitudinalControl = False
  elif veto == "stopping":
    planner.output_should_stop = True
  planner.update(sm, carrot)
  assert planner.mpc.update_kwargs.get("lane_change_lead") is None
  assert not planner.mpc.update_kwargs.get("human_lane_changes_enabled", False)
