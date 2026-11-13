"""Exercise production output methods with real car/schema messages, no optimizer."""
import math
from types import SimpleNamespace as NS

import pytest
from cereal import car, log
from openpilot.selfdrive.carrot.signal_start_guard import StartDecision
from openpilot.selfdrive.carrot.signal_stop_prepare import PrepareDecision
from openpilot.selfdrive.carrot.signal_road_policy import road_category_observation, signal_detection_suspended
from openpilot.selfdrive.controls.tests.test_motion_longitudinal_integration import mpc_module, planner_module


def outputs(module):
  planner = module['LongitudinalPlanner'](NS(openpilotLongitudinalControl=True))
  planner.output_a_target = 1.
  planner.output_v_target_now = 20.
  planner.output_j_target_now = 1.
  planner.a_desired = 1.
  state = car.CarState.new_message(vEgo=15., aEgo=0.)
  sm = {'carState': state, 'radarState': log.RadarState.new_message()}
  carrot = NS(prepare_decision=PrepareDecision(active=True), prepare_control_active=True,
              start_decision=StartDecision(), signal_green_active=False,
              signal_detection_suspended=False, signal_observation_color=1)
  return planner, sm, carrot


def test_preparation_caps_final_outputs_without_requesting_stop(planner_module):
  planner, sm, carrot = outputs(planner_module)
  planner.apply_signal_stop_prepare(sm, carrot, False, now=1.)
  assert planner.output_a_target == pytest.approx(-.0125)
  assert planner.output_v_target_now < sm['carState'].vEgo
  assert planner.output_j_target_now < 0
  assert planner.a_desired == planner.output_a_target
  assert planner.signal_prepare_active
  assert not planner.output_should_stop


def test_preparation_preserves_stronger_existing_braking(planner_module):
  planner, sm, carrot = outputs(planner_module)
  planner.output_a_target = -2.5
  planner.apply_signal_stop_prepare(sm, carrot, False, now=1.)
  assert planner.output_a_target == -2.5


@pytest.mark.parametrize('veto', ['mode1', 'inactive', 'reset', 'gas', 'brake', 'slow', 'fast',
                                  'hold', 'stopping', 'blended', 'leadOne', 'leadTwo', 'bad_lead', 'pause'])
def test_preparation_final_vetoes_restore_original_target(planner_module, veto):
  planner, sm, carrot = outputs(planner_module)
  reset = veto == 'reset'
  if veto == 'mode1': carrot.prepare_control_active = False
  if veto == 'inactive': carrot.prepare_decision = PrepareDecision()
  if veto == 'gas': sm['carState'].gasPressed = True
  if veto == 'brake': sm['carState'].brakePressed = True
  if veto == 'slow': sm['carState'].vEgo = 5 / 3.6
  if veto == 'fast': sm['carState'].vEgo = 82 / 3.6
  if veto == 'hold': sm['carState'].softHoldActive = 1
  if veto == 'stopping': planner.output_should_stop = True
  if veto == 'blended': planner.mpc.mode = 'blended'
  if veto in ('leadOne', 'leadTwo', 'bad_lead'):
    lead = getattr(sm['radarState'], 'leadTwo' if veto == 'leadTwo' else 'leadOne')
    lead.status, lead.dRel = True, math.nan if veto == 'bad_lead' else 50.
  if veto == 'pause': planner._signal_prepare_time = .7
  planner.apply_signal_stop_prepare(sm, carrot, reset, now=1.)
  assert planner.output_a_target == 1.
  assert planner.output_v_target_now == 20.
  assert not planner.signal_prepare_active


def test_preparation_ramp_is_bounded_and_resets_after_override(planner_module):
  planner, sm, carrot = outputs(planner_module)
  for index in range(60):
    planner.output_a_target = 1.
    planner.apply_signal_stop_prepare(sm, carrot, False, now=1. + index * .05)
  assert planner.output_a_target == -.5
  carrot.prepare_control_active = False
  planner.output_a_target = 1.
  planner.apply_signal_stop_prepare(sm, carrot, False, now=4.)
  assert planner.output_a_target == 1.
  carrot.prepare_control_active = True
  planner.apply_signal_stop_prepare(sm, carrot, False, now=4.05)
  # Resume from the current normal target, without reusing the previous -.5 cap.
  assert planner.output_a_target == pytest.approx(.9875)


@pytest.mark.parametrize('speed,hold', [(0., True), (.3, True), (1., True), (2., False)])
def test_start_guard_only_vetoes_stop_and_creep_phase(planner_module, speed, hold):
  planner, _, carrot = outputs(planner_module)
  carrot.start_decision = StartDecision(hold=True)
  planner.apply_signal_start_guard(carrot, speed)
  assert planner.output_should_stop == hold
  assert planner.output_v_target_now == (0. if hold else 20.)
  assert planner.output_a_target == (0. if hold else 1.)


@pytest.mark.parametrize('payload,expected', [({}, False), ({'roadcate': 0}, True),
  ({'roadcate': 1}, True), ({'roadcate': '1'}, True), ({'roadcate': 2}, False),
  ({'roadcate': 8}, False), ({'roadcate': True}, False), ({'roadcate': .5}, False),
  ({'roadcate': None}, False), ({'roadcate': 'nan'}, False)])
def test_legacy_road_category_never_infers_highway_from_default(payload, expected):
  category, valid, timestamp = road_category_observation(payload, 10_000_000_000)
  message = log.Event.new_message().init('carrotMan')
  message.activeCarrot = 2
  message.roadCategory, message.roadCategoryValid, message.roadCategoryMonoTimeNanos = category, valid, timestamp
  class SM(dict): pass
  sm = SM(carrotMan=message)
  sm.seen = sm.valid = sm.alive = {'carrotMan': True}
  sm.recv_time = {'carrotMan': 10.}
  assert signal_detection_suspended(sm, now=10.1) == expected
  assert not signal_detection_suspended(sm, now=12.1)  # stale receiver
  sm.recv_time['carrotMan'] = 20.1
  assert not signal_detection_suspended(sm, now=20.1)  # stale upstream category


def test_published_plan_reports_applied_preparation_and_schema_fields(planner_module):
  planner, _, carrot = outputs(planner_module)
  planner.mpc.solve_time = 0.
  carrot.xState = NS(value=6)
  carrot.trafficState = NS(value=1)
  carrot.myDrivingMode = NS(value=3)
  carrot.events = NS(to_msg=lambda: [])
  planner.mpc.source = 'cruise'
  planner.mpc.t_follow, planner.mpc.desired_distance = 1.45, 25.
  class SM(dict):
    def all_checks(self, service_list): return True
  sm = SM(radarState=log.RadarState.new_message())
  sm.logMonoTime = {'modelV2': 1_000_000_000}
  def message(_name):
    event = log.Event.new_message()
    event.init('longitudinalPlan')
    event.logMonoTime = 2_000_000_000
    return event
  planner_module['messaging'] = NS(new_message=message)
  sent = []
  pm = NS(send=lambda name, event: sent.append(event))
  # Candidate preparation was vetoed at the final output: do not show intervention.
  planner.publish(sm, pm, carrot)
  assert sent[-1].longitudinalPlan.signalAssistAction == 0
  planner.signal_prepare_active = True
  planner.publish(sm, pm, carrot)
  plan = sent[-1].longitudinalPlan
  assert plan.signalAssistAction == 2 and plan.signalObservationColor == 1
  assert not plan.signalDetectionSuspended
  assert plan.xState == 6 and plan.trafficState == 1
