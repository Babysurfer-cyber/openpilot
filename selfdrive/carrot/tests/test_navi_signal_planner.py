"""Mode 3 adds a reference and a bounded veto, never acceleration permission."""
import ast
from types import SimpleNamespace as NS

import pytest
from cereal import log

from openpilot.selfdrive.carrot import signal_mode as policy
from openpilot.selfdrive.carrot.navi_signal_assist import NaviSignalAssist
from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard
from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare, PrepareAccel
from openpilot.selfdrive.carrot.tests.test_navi_signal_assist import packet, candidate
from openpilot.selfdrive.carrot.tests.test_signal_start_guard import observation as start_observation
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import observation as prepare_observation
from tools.signal_analysis.replay_signal_assist import planner_class, REPO
from tools.signal_analysis.test_signal_assist_planner import inputs


def setup(monkeypatch, *, mode=3, speed=15., distance=160.):
  selected = [mode]
  monkeypatch.setattr(policy, 'signal_mode', lambda: selected[0])
  cls, namespace = planner_class((REPO / 'selfdrive/carrot/carrot_functions.py').read_text())
  planner = cls(start_guard=SignalStartGuard(), stop_prepare=SignalStopPrepare())
  return planner, inputs(speed=speed, distance=distance), selected, namespace


def update(planner, sm, now, *, nav='RED', camera=None, mode='acc', motion_valid=True,
           nav_valid=True, nav_packet=None, source_age=0., context_changes=None):
  ctx = dict(now=now, enabled=True, valid=True, drive=True)
  nctx = dict(ctx, valid=nav_valid, received_s=now, turning=False)
  if context_changes:
    nctx.update(context_changes)
  navi = packet(now - source_age, nav) if nav is not None and nav_packet is None else nav_packet
  planner.update(sm, 80., mode,
    start_observation=start_observation(now, camera) if camera else None,
    start_context=ctx,
    prepare_observation=prepare_observation(now, camera) if camera else None,
    prepare_context=dict(ctx, valid=motion_valid, turning=False, geometry=lambda _box: (30., 70.), actuate=True),
    navi_observation=navi, navi_context=nctx)


def acquire(planner, sm, *, nav='RED', camera=None, start=10., count=25, **kwargs):
  for frame in range(count):
    update(planner, sm, start + frame * .05, nav=nav, camera=camera, **kwargs)


def test_navigation_red_adds_only_bounded_preparation_when_the_camera_is_green(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch)
  acquire(planner, sm, nav='RED', camera='green')
  assert planner.navi_signal_decision.confirmed and planner.navi_signal_decision.color == 'red'
  assert planner.prepare_decision.active and planner.prepare_decision.evidence_kind == 'navigation'
  assert planner.prepare_control_active and planner.xState.name == 'e2eStopPrepare'
  assert planner.stop_dist == 1000. and not planner.start_decision.hold
  assert not planner.start_decision.green_confirmed and not planner.signal_green_active

  # Exercise the real final limiter, including preservation of stronger braking.
  path = REPO / 'selfdrive/controls/lib/longitudinal_planner.py'
  klass = next(node for node in ast.parse(path.read_text()).body if isinstance(node, ast.ClassDef) and node.name == 'LongitudinalPlanner')
  method = next(node for node in klass.body if isinstance(node, ast.FunctionDef) and node.name == 'apply_signal_stop_prepare')
  scope = {}
  exec(compile(ast.Module([method], type_ignores=[]), str(path), 'exec'), scope)
  target = NS(dt=.05, _signal_prepare_time=None, _signal_prepare_previous=None,
              _signal_prepare_accel=PrepareAccel(), output_a_target=.8, output_v_target_now=20.,
              output_j_target_now=1., output_should_stop=False, mpc=NS(mode='acc'), a_desired=.8)
  for frame in range(50):
    target.output_a_target = .8
    scope['apply_signal_stop_prepare'](target, sm, planner, False, now=20. + frame * .05)
    assert -.5 <= target.output_a_target <= .8
    assert not target.output_should_stop
  assert target.output_a_target == pytest.approx(-.5)
  target.output_a_target = -1.5
  scope['apply_signal_stop_prepare'](target, sm, planner, False, now=22.5)
  assert target.output_a_target == -1.5


def test_navigation_green_suppresses_new_visual_preparation_without_granting_go(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch)
  acquire(planner, sm, nav='GREEN', camera='red')
  assert planner.navi_signal_decision.confirmed and planner.navi_signal_decision.color == 'green'
  assert not planner.prepare_control_active and not planner.prepare_decision.active
  assert planner.xState.name == 'e2eCruise'
  assert not planner.signal_green_active and not planner.start_decision.green_confirmed


def test_navigation_green_cannot_release_an_existing_camera_red_stop(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch, speed=0.)
  acquire(planner, sm, nav=None, camera='red')
  assert planner.start_guard.hold and planner.xState.name == 'e2eStopped'
  acquire(planner, sm, nav='GREEN', camera='red', start=11.3)
  assert planner.navi_signal_decision.confirmed
  assert planner.start_decision.hold and planner.xState.name == 'e2eStopped'
  assert planner.v_cruise == 0. and not planner.signal_green_active
  assert not planner.start_decision.released and not planner.start_decision.green_confirmed


def test_navigation_green_cannot_cancel_the_models_existing_stopping_target(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch, speed=10., distance=40.)
  sm['modelV2'].velocity.x = [10.] + [0.] * 32
  acquire(planner, sm, nav='GREEN', count=40)
  assert planner.navi_signal_decision.confirmed
  assert planner.xState.name == 'e2eStop' and planner.stop_dist < 100.
  assert not planner.signal_green_active and not planner.start_decision.green_confirmed


@pytest.mark.parametrize('lever,straight,left,color', [
  (2, 'RED', 'GREEN', 'green'), (2, 'GREEN', 'RED', 'red'),
  (1, 'RED', 'GREEN', 'red'), (1, 'GREEN', 'RED', 'green'),
])
def test_only_fully_latched_left_stalk_selects_the_protected_left_reference(monkeypatch, lever, straight, left, color):
  planner, sm, _, _ = setup(monkeypatch, speed=0.)
  sm['carState'].leftBlinker, sm['carState'].blinkerLever = True, lever
  for frame in range(25):
    now = 10. + frame * .05
    navi = packet(now, candidates=[candidate('STRAIGHT', straight), candidate('LEFT', left, index=1)])
    update(planner, sm, now, nav_packet=navi)
  assert planner.navi_signal_decision.confirmed and planner.navi_signal_decision.color == color
  assert planner.navi_signal_decision.hold is (color == 'red')
  assert not planner.signal_green_active and not planner.start_decision.green_confirmed


def test_fully_latched_left_cannot_treat_straight_only_green_as_permission(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch, speed=0.)
  sm['carState'].leftBlinker, sm['carState'].blinkerLever = True, 2
  acquire(planner, sm, nav='GREEN')
  assert planner.navi_signal_decision.confirmed and planner.navi_signal_decision.color == 'red'
  assert planner.navi_signal_decision.reason == 'left_turn_not_allowed'
  assert planner.start_decision.hold and planner.xState.name == 'e2eStopped'
  assert not planner.start_decision.green_confirmed


def test_fully_latched_left_with_only_right_signal_falls_back_to_unknown(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch, speed=0.)
  sm['carState'].leftBlinker, sm['carState'].blinkerLever = True, 2
  for frame in range(25):
    now = 10. + frame * .05
    update(planner, sm, now, nav_packet=packet(now, candidates=[candidate('RIGHT', 'GREEN')]))
  assert not planner.navi_signal_decision.available and not planner.navi_signal_decision.hold
  assert not planner.signal_green_active


def test_direction_change_cannot_retire_an_independently_acquired_camera_stop(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch, speed=0.)
  acquire(planner, sm, nav=None, camera='red')
  assert planner.start_guard.hold
  sm['carState'].leftBlinker, sm['carState'].blinkerLever = True, 2
  for frame in range(25):
    now = 11.3 + frame * .05
    navi = packet(now, candidates=[candidate('STRAIGHT', 'RED'), candidate('LEFT', 'GREEN', index=1)])
    update(planner, sm, now, nav_packet=navi, camera='red')
  assert planner.navi_signal_decision.confirmed and planner.navi_signal_decision.color == 'green'
  assert planner.start_guard.hold and planner.start_decision.hold
  assert planner.xState.name == 'e2eStopped' and planner.v_cruise == 0.
  assert not planner.start_decision.green_confirmed and not planner.start_decision.released


def test_navigation_red_stopped_veto_is_visible_to_the_final_start_guard(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch, speed=0.)
  acquire(planner, sm, camera='green')
  assert planner.navi_signal_decision.hold and planner.start_decision.hold
  assert planner.xState.name == 'e2eStopped' and planner._navi_stop_owned
  assert planner.v_cruise == 0. and not planner.start_decision.green_confirmed


@pytest.mark.parametrize('next_mode', [0, 1, 2])
def test_switching_away_from_mode_three_retires_only_the_navigation_owned_stop(monkeypatch, next_mode):
  planner, sm, selected, _ = setup(monkeypatch, speed=0.)
  acquire(planner, sm)
  assert planner._navi_stop_owned and planner.xState.name == 'e2eStopped'
  selected[0] = next_mode
  update(planner, sm, 11.3, nav=None)
  assert not planner.start_decision.hold and not planner._navi_stop_owned
  assert planner.xState.name != 'e2eStopped'


@pytest.mark.parametrize('which', ['leadOne', 'leadTwo'])
@pytest.mark.parametrize('distance,blocked', [(50., True), (50.001, False)])
def test_both_existing_front_lead_boundaries_remain_in_force(monkeypatch, which, distance, blocked):
  planner, sm, _, _ = setup(monkeypatch)
  sm['radarState'].__dict__[which] = NS(status=True, dRel=distance)
  acquire(planner, sm)
  assert planner.navi_signal_decision.confirmed
  assert planner.prepare_control_active is (not blocked)


@pytest.mark.parametrize('gate', ['gas', 'brake', 'left_blinker', 'steering', 'high_mode', 'traffic_off', 'nav_motion_invalid'])
def test_mode_two_driver_turn_and_motion_gates_still_block_navigation_preparation(monkeypatch, gate):
  planner, sm, _, namespace = setup(monkeypatch)
  context_changes = {}
  if gate == 'gas': sm['carState'].gasPressed = True
  elif gate == 'brake': sm['carState'].brakePressed = True
  elif gate == 'left_blinker': sm['carState'].leftBlinker = True
  elif gate == 'steering': sm['carState'].steeringAngleDeg = 20.01
  elif gate == 'high_mode': planner.myDrivingMode = namespace['DrivingMode'].High
  elif gate == 'traffic_off': planner.trafficLightDetectMode = 0
  elif gate == 'nav_motion_invalid': context_changes['turning'] = True
  acquire(planner, sm, context_changes=context_changes)
  assert not planner.prepare_control_active


def test_camera_context_loss_does_not_disable_independently_valid_navigation_preparation(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch)
  acquire(planner, sm, camera='green', motion_valid=False)
  assert planner.navi_signal_decision.available and planner.navi_signal_decision.confirmed
  assert planner.prepare_control_active and planner.prepare_decision.evidence_kind == 'navigation'


@pytest.mark.parametrize('case,turning', [('healthy', False), ('rotation', True), ('combined_rotation', True),
  ('uncertain', True), ('stale', True), ('not_seen', True), ('invalid', True), ('degraded', True), ('nonfinite', True)])
def test_independent_navigation_motion_gate_rejects_turns_or_unhealthy_native_pose(monkeypatch, case, turning):
  planner, _, _, _ = setup(monkeypatch)
  event = log.Event.new_message()
  pose = event.init('livePose')
  pose.inputsOK = pose.posenetOK = pose.sensorsOK = True
  pose.angularVelocityDevice = dict(x=0., y=0., z=0., xStd=.001, yStd=.001, zStd=.001, valid=True)

  class PoseInputs(dict):
    seen = valid = alive = {'livePose': True}
    recv_time = {'livePose': 10.}

  sm = PoseInputs(livePose=pose.as_reader())
  if case == 'rotation': pose.angularVelocityDevice.z = .027  # Greater than 1.5 degrees/second.
  elif case == 'combined_rotation': pose.angularVelocityDevice.x = pose.angularVelocityDevice.y = .021
  elif case == 'uncertain': pose.angularVelocityDevice.zStd = .036  # Greater than 2 degrees/second.
  elif case == 'stale': sm.recv_time = {'livePose': 9.799}
  elif case == 'not_seen': sm.seen = {'livePose': False}
  elif case == 'invalid': sm.valid = {'livePose': False}
  elif case == 'degraded': pose.sensorsOK = False
  elif case == 'nonfinite': pose.angularVelocityDevice.z = float('nan')
  assert planner._navi_motion_turning(sm, 10.) is turning


@pytest.mark.parametrize('case,available', [('fresh_boundary', True), ('carState', False), ('modelV2', False),
  ('radarState', False), ('selfdriveState', False), ('carControl', False), ('can_invalid', False),
  ('disabled', False), ('long_inactive', False), ('not_drive', False)])
def test_optional_navigation_runtime_does_not_bypass_fresh_vehicle_context(monkeypatch, case, available):
  planner, values, _, _ = setup(monkeypatch)

  class VehicleInputs(dict):
    pass

  sm = VehicleInputs(values)
  sm['carControl'] = NS(longActive=True)
  sm['selfdriveState'].enabled = True
  sm['carState'].canValid = True
  services = ('carState', 'modelV2', 'radarState', 'selfdriveState', 'carControl')
  sm.seen, sm.valid, sm.alive = ({service: True for service in services} for _ in range(3))
  sm.recv_time = {service: 9.75 for service in services}
  if case in services: sm.recv_time[case] = 9.749
  elif case == 'can_invalid': sm['carState'].canValid = False
  elif case == 'disabled': sm['selfdriveState'].enabled = False
  elif case == 'long_inactive': sm['carControl'].longActive = False
  elif case == 'not_drive': sm['carState'].gearShifter = 'park'
  planner._navi_signal_assist = NaviSignalAssist()
  planner._navi_signal_runtime = NS(read=lambda _mode: (packet(10.), True, 10., 10.))
  result = planner._update_navi_signal(sm, 3, True, False, False, None, None)
  assert result.available is available


@pytest.mark.parametrize('loss', ['missing', 'invalid', 'stale', 'no_straight'])
def test_missing_or_unusable_navigation_returns_to_normal_mode_two_camera_preparation(monkeypatch, loss):
  planner, sm, _, _ = setup(monkeypatch)
  for frame in range(30):
    now = 10. + frame * .05
    nav, valid, source_age, replacement = 'GREEN', True, 0., None
    if loss == 'missing': nav = None
    elif loss == 'invalid': valid = False
    elif loss == 'stale': source_age = 2.
    elif loss == 'no_straight':
      replacement = packet(now)
      replacement.signalHud.candidates[0].direction = 'LEFT'
    update(planner, sm, now, nav=nav, camera='red', nav_valid=valid,
           source_age=source_age, nav_packet=replacement)
  assert not planner.navi_signal_decision.available
  assert planner.prepare_control_active and planner.prepare_decision.evidence_kind == 'housing'


@pytest.mark.parametrize('mode', [0, 1, 2])
def test_navigation_payload_cannot_change_modes_zero_through_two(monkeypatch, mode):
  planner, sm, _, _ = setup(monkeypatch, mode=mode)
  baseline, baseline_sm, _, _ = setup(monkeypatch, mode=mode)
  for frame in range(25):
    now = 10. + frame * .05
    update(planner, sm, now, nav='RED', camera='green')
    update(baseline, baseline_sm, now, nav=None, camera='green')
    assert planner.xState.name == baseline.xState.name
    assert planner.trafficState.value == baseline.trafficState.value
    assert planner.v_cruise == baseline.v_cruise
    assert planner.stop_dist == baseline.stop_dist
    assert planner.prepare_control_active == baseline.prepare_control_active
    assert planner.start_decision == baseline.start_decision


def test_navigation_reference_survives_real_service_and_source_cadences(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch)
  for frame in range(61):
    now = 10. + frame * .05
    source = 10. + frame // 20
    service = 10. + (frame // 10) * .5
    update(planner, sm, now, nav_packet=packet(source), context_changes={'received_s': service})
    assert planner.navi_signal_decision.available
    assert planner.prepare_control_active is (frame >= 20)
    assert not planner.start_decision.hold


@pytest.mark.parametrize('driver_hold', ['soft', 'user', 'stay'])
def test_retiring_navigation_ownership_cannot_cancel_driver_stops(monkeypatch, driver_hold):
  planner, sm, selected, _ = setup(monkeypatch, speed=0.)
  acquire(planner, sm)
  assert planner._navi_stop_owned
  if driver_hold == 'soft': sm['carState'].softHoldActive = 1
  elif driver_hold == 'user': planner.user_stop_distance = 0.
  else: planner.carrot_stay_stop = True
  selected[0] = 2
  update(planner, sm, 11.3, nav=None)
  assert planner.xState.name == 'e2eStopped'
  if driver_hold == 'soft':
    assert planner.soft_hold_active == 1  # The existing soft-hold actuator path owns braking.
  else:
    assert planner.v_cruise == 0.


def test_green_can_release_navigation_memory_without_cancelling_a_new_model_stop(monkeypatch):
  planner, sm, _, _ = setup(monkeypatch, speed=0.)
  acquire(planner, sm)
  assert planner.navi_signal_decision.hold
  sm['modelV2'].velocity.x = [0.] * 33
  sm['modelV2'].position.x = [3. * index / 32 for index in range(33)]
  acquire(planner, sm, nav='GREEN', start=11.3, count=35)
  assert not planner.navi_signal_decision.hold
  assert planner.trafficState.name == 'red' and planner.xState.name == 'e2eStopped'
  assert planner.v_cruise == 0. and not planner.start_decision.green_confirmed
