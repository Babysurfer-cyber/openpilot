"""Port regressions against the user's pre-signal planner and real schemas.

The A/B check executes both complete production CarrotPlanner implementations.
It supplies deterministic parameters, clocks and decoded cereal inputs; it does
not replace their parameter polling, stop filters, state machine or follow logic.
This is a fixed-input comparison, not a vehicle dynamics simulation.
"""
from pathlib import Path
import subprocess
import types

import numpy as np
import pytest

from cereal import car, log
from openpilot.selfdrive.carrot import carrot_functions, signal_mode
from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import observation


REPO = Path(__file__).resolve().parents[3]
BASELINE = "33b1ac32b26f427b4a77b1f5635570b48e00810b"


class Clock:
  now = 100.0

  def time(self):
    return self.now

  def monotonic(self):
    return self.now


class Params:
  def __init__(self, driving_mode, traffic_mode):
    self.values = dict(MyDrivingMode=driving_mode, MyDrivingModeAuto=0,
                       TrafficLightDetectMode=traffic_mode,
                       TFollowGap1=110, TFollowGap2=130, TFollowGap3=145, TFollowGap4=160,
                       DynamicTFollow=30, DynamicTFollowLC=60, EnableSpeedTF=-1, TFollowDecelBoost=20,
                       CruiseMaxVals0=160, CruiseMaxVals1=160, CruiseMaxVals2=120,
                       CruiseMaxVals3=100, CruiseMaxVals4=80, CruiseMaxVals5=70, CruiseMaxVals6=60,
                       StopDistanceCarrot=600, JLeadFactor3=100, AutoNaviSpeedDecelRate=150,
                       AChangeCostStarting=10, TrafficStopDistanceAdjust=400, SignalAssistMode=0)

  def get_int(self, name):
    return int(self.values.get(name, 0))

  def get_float(self, name):
    return float(self.values.get(name, 0))

  def get_bool(self, name):
    return bool(self.values.get(name, 0))


class Inputs(dict):
  def __init__(self):
    cs = car.CarState.new_message(vCluRatio=1., gearShifter="drive")
    cs.cruiseState.enabled = True
    model = log.ModelDataV2.new_message()
    model.meta.desireState = [0.] * 8
    model.meta.laneChangeState = "off"
    super().__init__(carState=cs, modelV2=model, radarState=log.RadarState.new_message(),
                     selfdriveState=log.SelfdriveState.new_message(enabled=True, personality="standard"),
                     carControl=car.CarControl.new_message(longActive=True))
    names = tuple(self) + ("carrotMan",)
    self.alive = dict.fromkeys(names, True)
    self.alive["carrotMan"] = False
    self.seen = dict.fromkeys(names, True)
    self.valid = dict.fromkeys(names, True)
    self.recv_time = dict.fromkeys(names, 100.)

  def step(self, index, now):
    cs, model, radar = self['carState'], self['modelV2'], self['radarState']
    speed = max(0., 20. - index * .16) if index < 150 else min(15., (index - 150) * .2)
    distance = max(3., 115. - index * 1.5) if index < 150 else 160.
    # Occasional closer model estimates exercise the user's approach rate limit.
    if index in (20, 45, 80):
      distance = 5.
    cs.vEgo = cs.vEgoCluster = speed
    cs.aEgo = -.8 if 5 < index < 120 else .5
    cs.standstill = speed < .1
    cs.gasPressed = index in (145, 146)
    cs.brakePressed = index == 140
    cs.softHoldActive = int(130 <= index < 138)
    cs.leftBlinker = 30 <= index < 45
    cs.steeringAngleDeg = 25. if 40 <= index < 50 else 0.
    radar.leadOne.status = 5 <= index < 35 or 160 <= index < 190
    radar.leadOne.dRel = 35. - (index % 30)
    radar.leadOne.vLead = max(0., speed - 1.)
    radar.leadOne.vRel = -1.
    radar.leadOne.aLead = -.4
    radar.leadOne.jLead = -.6
    model.position.x = np.linspace(0., distance, 33).tolist()
    model.position.y = [0.] * 33
    model.velocity.x = np.linspace(speed, 0. if index < 150 else 15., 33).tolist()
    model.meta.laneChangeState = "laneChangeStarting" if 30 <= index < 45 else "off"
    model.meta.desireState = [0., 0., 0., .95, .95, 0., 0., 0.]
    self.recv_time = dict.fromkeys(self.recv_time, now)


@pytest.fixture(scope="module")
def baseline_module():
  result = subprocess.run(['git', 'show', f'{BASELINE}:selfdrive/carrot/carrot_functions.py'],
                          cwd=REPO, capture_output=True, text=True)
  if result.returncode:
    pytest.skip("pre-port comparison commit unavailable in this source archive")
  module = types.ModuleType('pre_signal_carrot_planner')
  exec(compile(result.stdout, f'{BASELINE}/carrot_functions.py', 'exec'), module.__dict__)
  return module


def snapshot(planner):
  fields = ('v_cruise', 'stop_dist', 'xStop', 'actual_stop_distance', 'stopping_count',
            'traffic_starting_count', 'startSignCount', 'stopSignCount', 'mySafeFactor',
            'lead_alerted', 'lead_lost_count', 'lead_detect_count', 'prev_lead_dist',
            'comfort_brake', 'fakeCruiseDistance', 'trafficLightDetectMode')
  return dict({name: getattr(planner, name) for name in fields},
              xState=planner.xState.name, trafficState=planner.trafficState.name,
              drivingMode=planner.myDrivingMode.name, mode=planner.mode,
              events=tuple(planner.events.events),
              t_follow=planner.get_T_FOLLOW(v_ego=planner._comparison_speed, a_ego=-.5))


@pytest.mark.parametrize('driving_mode', (1, 2, 3, 4, 5))
@pytest.mark.parametrize('traffic_mode', (0, 1, 2))
@pytest.mark.parametrize('initial', ('cruise', 'e2ePrepare', 'e2eStopped'))
def test_mode_zero_preserves_complete_pre_port_planner(monkeypatch, baseline_module,
                                                      driving_mode, traffic_mode, initial):
  clock = Clock()
  params = Params(driving_mode, traffic_mode)
  monkeypatch.setattr(signal_mode, 'signal_mode', lambda: 0)
  for module in (baseline_module, carrot_functions):
    monkeypatch.setattr(module, 'Params', lambda: params)
    monkeypatch.setattr(module, 'time', clock)
    # The deterministic replay must not consume a real on-device prompt file.
    monkeypatch.setattr(module, 'os', types.SimpleNamespace(path=types.SimpleNamespace(isfile=lambda _path: False)))
  old, new = baseline_module.CarrotPlanner(), carrot_functions.CarrotPlanner()
  old.xState = baseline_module.XState[initial]
  new.xState = carrot_functions.XState[initial]
  sm = Inputs()
  for index in range(240):
    clock.now = 100. + index * .05
    sm.step(index, clock.now)
    for planner in (old, new):
      planner._comparison_speed = sm['carState'].vEgo
      planner.update(sm, 80., 'acc')
    assert snapshot(new) == snapshot(old), index
  # Normal mode must retain the user's explicit traffic-control disable.
  if driving_mode == 3:
    assert new.trafficLightDetectMode == 0


def test_new_signal_plan_fields_round_trip_real_cereal():
  message = log.Event.new_message()
  plan = message.init('longitudinalPlan')
  plan.signalDetectionSuspended = True
  plan.signalObservationColor = 3
  plan.signalAssistAction = 2
  with log.Event.from_bytes(message.to_bytes()) as decoded:
    assert decoded.longitudinalPlan.signalDetectionSuspended
    assert decoded.longitudinalPlan.signalObservationColor == 3
    assert decoded.longitudinalPlan.signalAssistAction == 2


@pytest.mark.parametrize('driving_mode,traffic_mode,steering,expected', [
  (2, 2, 0., True),
  (3, 2, 0., False),  # This branch deliberately disables traffic control in Normal.
  (4, 2, 0., False),
  (2, 0, 0., False),
  (2, 2, 25., False),
  (2, 2, -25., False),
])
def test_auxiliary_preparation_honors_existing_user_control_gates(
    monkeypatch, driving_mode, traffic_mode, steering, expected):
  clock = Clock()
  params = Params(driving_mode, traffic_mode)
  monkeypatch.setattr(signal_mode, 'signal_mode', lambda: 0)  # Only injected observation is enabled.
  monkeypatch.setattr(carrot_functions, 'Params', lambda: params)
  monkeypatch.setattr(carrot_functions, 'time', clock)
  monkeypatch.setattr(carrot_functions, 'os', types.SimpleNamespace(path=types.SimpleNamespace(isfile=lambda _path: False)))
  planner = carrot_functions.CarrotPlanner(stop_prepare=SignalStopPrepare())
  sm = Inputs()
  for index in range(25):
    clock.now = 100. + index * .05
    sm.step(210, clock.now)
    sm['carState'].steeringAngleDeg = steering
    planner.update(sm, 80., 'acc', prepare_observation=observation(clock.now),
                   prepare_context=dict(now=clock.now, enabled=True, valid=True, drive=True,
                                        turning=False, geometry=lambda _box: (30., 70.), actuate=True))
  assert planner.prepare_control_active is expected
  assert (planner.xState.name == 'e2eStopPrepare') is expected
