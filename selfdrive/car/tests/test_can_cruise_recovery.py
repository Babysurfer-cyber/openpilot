"""CAN-fault recovery requests use healthy data and yield to driver intent."""
import math
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.can_cruise_recovery import CanErrorCruiseRecovery


class RecoveryDrive:
  def __init__(self):
    self.recovery = CanErrorCruiseRecovery()
    self.inputs = dict(enabled=False, set_speed_kph=105., can_fault=False,
                       blocked=False, ready=True, allowed=True, driver_input=False,
                       manual_enable=False)

  def step(self, now, **changes):
    self.inputs.update(changes)
    return self.recovery.update(now_s=now, **self.inputs)


def interrupted_drive(speed=105.):
  drive = RecoveryDrive()
  assert drive.step(0., enabled=True, set_speed_kph=speed) is None
  assert drive.step(.1, enabled=False, can_fault=True, ready=False,
                    set_speed_kph=55.) is None
  return drive


def test_recovers_once_after_health_stabilizes_and_restores_previous_set_speed():
  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.499) is None
  assert drive.step(.501, set_speed_kph=60.) == 105.
  assert drive.step(.6) is None
  assert drive.step(.7) is None


def test_does_not_consume_recovery_request_while_services_still_block_entry():
  drive = interrupted_drive()
  for now in (.3, .6, .9):
    assert drive.step(now, can_fault=False, ready=False) is None
  assert drive.step(1., ready=True) is None
  assert drive.step(1.201) == 105.


def test_health_must_be_continuous_after_a_second_communication_gap():
  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.45, can_fault=True, ready=False) is None
  assert drive.step(.6, can_fault=False, ready=True) is None
  assert drive.step(.799) is None
  assert drive.step(.801) == 105.


def test_never_starts_cruise_that_was_not_previously_enabled():
  drive = RecoveryDrive()
  assert drive.step(0.) is None
  assert drive.step(.1, can_fault=True, ready=False) is None
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.6) is None


def test_normal_disengagement_is_not_treated_as_a_can_fault():
  drive = RecoveryDrive()
  assert drive.step(0., enabled=True) is None
  assert drive.step(.1, enabled=False, can_fault=False) is None
  assert drive.step(.6) is None
  assert drive.step(1.) is None


@pytest.mark.parametrize("speed", (0., -1., 255., math.nan, math.inf, -math.inf))
def test_unset_or_nonfinite_speed_cannot_become_a_recovery_target(speed):
  drive = interrupted_drive(speed)
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.6) is None


def test_unhealthy_enabled_sample_does_not_supply_a_trusted_recovery_target():
  drive = RecoveryDrive()
  assert drive.step(0., enabled=True, ready=False) is None
  assert drive.step(.1, enabled=False, can_fault=True) is None
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.6) is None


@pytest.mark.parametrize('unset_speed', (0., 255.))
def test_a_healthy_unset_target_replaces_an_older_positive_target(unset_speed):
  drive = RecoveryDrive()
  assert drive.step(0., enabled=True, set_speed_kph=105.) is None
  assert drive.step(.05, set_speed_kph=unset_speed) is None
  assert drive.step(.1, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.501) is None


@pytest.mark.parametrize("guard", ("blocked", "driver_input"))
@pytest.mark.parametrize("when", ("disengagement", "waiting", "request_boundary"))
def test_driver_intervention_or_another_fault_cancels_recovery(guard, when):
  drive = RecoveryDrive()
  assert drive.step(0., enabled=True) is None
  initial = {guard: True} if when == "disengagement" else {}
  assert drive.step(.1, enabled=False, can_fault=True, ready=False, **initial) is None
  if when == "waiting":
    assert drive.step(.2, **{guard: True}) is None
  assert drive.step(.3, can_fault=False, ready=True, **{guard: False}) is None
  boundary = {guard: True} if when == "request_boundary" else {}
  assert drive.step(.501, **boundary) is None
  assert drive.step(.8, **{guard: False}) is None


@pytest.mark.parametrize("when", ("before_fault", "pending", "request_boundary"))
def test_feature_or_vehicle_eligibility_cannot_be_bypassed(when):
  drive = RecoveryDrive()
  assert drive.step(0., enabled=True, allowed=when != "before_fault") is None
  assert drive.step(.1, enabled=False, can_fault=True, ready=False) is None
  if when == "pending":
    assert drive.step(.2, allowed=False) is None
  assert drive.step(.3, can_fault=False, ready=True, allowed=True) is None
  assert drive.step(.501, allowed=when != "request_boundary") is None
  assert drive.step(.8, allowed=True) is None


def test_a_driver_button_just_before_disengagement_prevents_an_auto_request():
  drive = RecoveryDrive()
  assert drive.step(0., enabled=True) is None
  assert drive.step(.05, driver_input=True) is None
  assert drive.step(.1, driver_input=False, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.6) is None


def test_recovery_reservation_expires_instead_of_starting_much_later():
  drive = interrupted_drive()
  assert drive.step(3.11, can_fault=False, ready=True) is None
  assert drive.step(3.4) is None


@pytest.mark.parametrize("fault_clock", (-1., math.nan, math.inf))
def test_invalid_or_reversed_clock_cancels_pending_recovery(fault_clock):
  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(fault_clock) is None
  assert drive.step(.6) is None


def test_auto_reengagement_ack_does_not_rearm_the_same_driving_session():
  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.501) == 105.
  assert drive.step(.55, enabled=True, set_speed_kph=105.) is None
  assert drive.step(.7, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(.9, can_fault=False, ready=True) is None
  assert drive.step(1.2) is None


def test_new_manual_engagement_allows_a_new_can_recovery_reservation():
  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.501) == 105.
  assert drive.step(.55, enabled=True) is None
  assert drive.step(.7, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(.9, can_fault=False, ready=True) is None
  assert drive.step(1.2) is None
  assert drive.step(1.5, driver_input=True, manual_enable=True) is None
  assert drive.step(1.6, driver_input=False, manual_enable=False, enabled=True, set_speed_kph=95.) is None
  assert drive.step(2.1, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(2.3, can_fault=False, ready=True) is None
  assert drive.step(2.501) == 95.


@pytest.mark.parametrize("manual_button", (False, True))
def test_a_non_enable_button_or_uncommanded_enable_cannot_reset_used_lock(manual_button):
  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.501) == 105.
  assert drive.step(.55, enabled=True) is None
  assert drive.step(.7, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(.9, can_fault=False, ready=True) is None
  assert drive.step(1.5, driver_input=manual_button, manual_enable=False) is None
  assert drive.step(1.6, driver_input=False, enabled=True) is None
  assert drive.step(2.1, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(2.3, can_fault=False, ready=True) is None
  assert drive.step(2.501) is None


def test_enable_button_without_successful_engagement_does_not_reset_used_lock():
  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.501) == 105.
  assert drive.step(.55, enabled=True) is None
  assert drive.step(.7, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(.9, can_fault=False, ready=True) is None
  assert drive.step(1.5, driver_input=True, manual_enable=True) is None
  assert drive.step(1.6, driver_input=False, manual_enable=False) is None
  assert drive.step(2., can_fault=True, ready=False) is None
  assert drive.step(2.3, can_fault=False, ready=True) is None
  assert drive.step(2.501) is None


def test_cancel_after_manual_enable_intent_invalidates_that_intent():
  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.501) == 105.
  assert drive.step(.55, enabled=True) is None
  assert drive.step(.7, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(.9, can_fault=False, ready=True) is None
  assert drive.step(1.5, driver_input=True, manual_enable=True) is None
  # A later Cancel overrides the earlier enable intention.
  assert drive.step(1.55, driver_input=True, manual_enable=False) is None
  assert drive.step(1.6, driver_input=False, enabled=True) is None
  assert drive.step(2.1, enabled=False, can_fault=True, ready=False) is None
  assert drive.step(2.3, can_fault=False, ready=True) is None
  assert drive.step(2.501) is None


def test_final_request_still_uses_the_normal_no_entry_gate():
  from cereal import log
  from openpilot.selfdrive.selfdrived.events import Events
  from openpilot.selfdrive.selfdrived.state import StateMachine

  drive = interrupted_drive()
  assert drive.step(.3, can_fault=False, ready=True) is None
  assert drive.step(.501) == 105.
  events = Events()
  events.add(log.OnroadEvent.EventName.buttonEnable)
  events.add(log.OnroadEvent.EventName.commIssue)
  state = StateMachine()
  enabled, _ = state.update(events)
  assert not enabled
  assert state.state == log.SelfdriveState.OpenpilotState.disabled
  # A no-entry race must not produce an unbounded series of enable requests.
  assert drive.step(.7) is None
  assert drive.step(1.) is None


class RecoverySubMaster:
  def __init__(self, control):
    from cereal import log
    self.data = {
      'carControl': control,
      'selfdriveState': log.SelfdriveState(engageable=True),
      'onroadEvents': [],
      'pandaStates': [log.PandaState(safetyModel='hyundaiCanfd', safetyParam=29)],
      'longitudinalPlan': log.LongitudinalPlan(),
      'radarState': log.RadarState(),
      'modelV2': log.ModelDataV2(),
    }
    self.alive = dict.fromkeys((*self.data, 'carrotMan', 'longitudinalPlan', 'radarState', 'drivingModelData'), False)
    self.alive.update(dict.fromkeys(self.data, True))
    self.valid = dict.fromkeys(self.data, True)
    self.recv_time = dict.fromkeys(self.data, 1.)
    self.services_healthy = True

  def __getitem__(self, name):
    return self.data[name]

  def all_checks(self, services=None):
    names = self.data if services is None else services
    return self.services_healthy and all(self.valid[name] and self.alive[name] for name in names)


@pytest.fixture
def integrated_drive(monkeypatch):
  from cereal import car, log
  from opendbc.car.hyundai.values import HyundaiFlags
  from openpilot.selfdrive.car import cruise

  class MemoryParams:
    def __init__(self, *_args, **_kwargs):
      pass

    def get_int(self, key):
      return {'AutoEngage': 1, 'CanErrorCruiseResume': 1, 'CruiseSpeedUnit': 10,
              'CruiseSpeedUnitBasic': 1, 'CruiseButtonLongDelay': 50,
              'LongitudinalPersonalityMax': 4}.get(key, 0)

    def get_float(self, _key):
      return 0.

    def get_bool(self, _key):
      return False

    def put_int_nonblocking(self, *_args):
      pass

    def put_bool_nonblocking(self, *_args):
      pass

  monkeypatch.setattr(cruise, 'Params', MemoryParams)
  cp = car.CarParams(brand='hyundai', carFingerprint='KIA_EV6',
                     flags=HyundaiFlags.CANFD.value, openpilotLongitudinalControl=True,
                     pcmCruise=False, minEnableSpeed=-1.)
  helper = cruise.VCruiseCarrot(cp)
  assert helper.autoCruiseControl == 0
  helper.can_error_cruise_resume = True
  helper._lat_enabled = True
  helper.cruise_state_available_last = True
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 105.
  control = car.CarControl(enabled=True, latActive=True)
  state = car.CarState(canValid=True, vEgo=25., vEgoCluster=25., gearShifter='drive', latEnabled=True,
                       cruiseState={'available': True})
  sm = RecoverySubMaster(control)
  clock = SimpleNamespace(now=1.)
  monkeypatch.setattr(cruise.time, 'monotonic', lambda: clock.now)

  class Drive:
    def __init__(self):
      self.helper, self.control, self.state, self.sm = helper, control, state, sm

    def step(self, now, *, enabled=None, can_valid=None, events=(), engageable=None,
             full_update=False, fresh=True, frozen_services=()):
      clock.now = now
      if enabled is not None:
        control.enabled = enabled
      if can_valid is not None:
        state.canValid = can_valid
      if engageable is not None:
        sm.data['selfdriveState'].engageable = engageable
      sm.data['onroadEvents'] = list(events)
      if fresh:
        sm.recv_time.update({name: now for name in sm.data if name not in frozen_services})
      if full_update:
        helper.update_v_cruise(state, sm, True)
      else:
        helper._activate_cruise = 0  # The ordinary update resets this pulse each cycle.
        helper._update_can_cruise_recovery(state, control, sm)
      return helper._activate_cruise

    def fault(self, *, full_update=False):
      assert self.step(1., enabled=True, full_update=full_update) == 0
      can_error = log.OnroadEvent(name='canError', noEntry=True, immediateDisable=True)
      assert self.step(1.1, enabled=False, can_valid=False, events=[can_error],
                       engageable=False, full_update=full_update) == 0

  return Drive()


def test_actual_full_cruise_update_recovers_with_auto_cruise_zero_and_preserves_set_speed(integrated_drive):
  from cereal import car, log
  from openpilot.selfdrive.car.car_specific import CarSpecificEvents
  from openpilot.selfdrive.selfdrived.state import StateMachine

  drive = integrated_drive
  drive.fault(full_update=True)
  assert drive.helper.autoCruiseControl == 0
  drive.helper.v_cruise_kph = drive.helper.v_cruise_cluster_kph = 60.
  assert drive.step(1.3, can_valid=True, engageable=True, full_update=True) == 0
  assert drive.step(1.501, full_update=True) == 1
  assert drive.helper.v_cruise_kph == drive.helper.v_cruise_cluster_kph == 105.
  assert drive.helper.button_type_pressed == car.CarState.ButtonEvent.Type.unknown
  assert list(drive.state.buttonEvents) == []

  # This pulse goes through the existing Hyundai event builder and state machine.
  previous = drive.state.as_builder() if hasattr(drive.state, 'as_builder') else car.CarState.new_message()
  previous.activateCruise = 0
  drive.state.activateCruise = drive.helper._activate_cruise
  drive.state.vCruise = drive.helper.v_cruise_kph
  events = CarSpecificEvents(drive.helper.CP).update(drive.state, previous, drive.control)
  assert log.OnroadEvent.EventName.buttonEnable in events.names
  state_machine = StateMachine()
  enabled, _ = state_machine.update(events)
  assert enabled
  assert drive.step(1.6, enabled=True, full_update=True) == 0


def test_actual_integration_waits_for_published_engageable_and_service_health(integrated_drive):
  from cereal import log
  drive = integrated_drive
  drive.fault()
  comm_issue = log.OnroadEvent(name='commIssue', noEntry=True, softDisable=True)
  assert drive.step(1.3, can_valid=True, events=[comm_issue], engageable=False) == 0
  assert drive.step(1.6, events=[comm_issue]) == 0
  drive.sm.services_healthy = False
  assert drive.step(1.8, engageable=True) == 0
  drive.sm.services_healthy = True
  assert drive.step(2.) == 0
  assert drive.step(2.201) == 1


@pytest.mark.parametrize('button', ('cancel', 'lfaButton', 'accelCruise', 'decelCruise'))
@pytest.mark.parametrize('pressed', (False, True))
def test_actual_button_on_request_boundary_is_never_replaced_by_auto_resume(integrated_drive, button, pressed):
  from cereal import car
  drive = integrated_drive
  drive.fault()
  assert drive.step(1.3, can_valid=True, engageable=True) == 0
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type=button, pressed=pressed)]
  assert drive.step(1.501) == 0
  assert str(drive.state.buttonEvents[0].type) == button
  assert drive.state.buttonEvents[0].pressed == pressed
  drive.state.buttonEvents = []
  assert drive.step(1.8) == 0


@pytest.mark.parametrize('guard', ('brakePressed', 'gasPressed', 'regenBraking', 'steeringPressed',
                                  'doorOpen', 'seatbeltUnlatched', 'canTimeout'))
def test_actual_intervention_cancels_pending_recovery(integrated_drive, guard):
  drive = integrated_drive
  drive.fault()
  assert drive.step(1.3, can_valid=True, engageable=True) == 0
  setattr(drive.state, guard, True)
  assert drive.step(1.501) == 0
  setattr(drive.state, guard, False)
  assert drive.step(1.8) == 0


@pytest.mark.parametrize('guard', ('feature_off', 'lat_disabled', 'lat_inactive', 'park', 'standstill'))
def test_actual_vehicle_eligibility_loss_cancels_recovery(integrated_drive, guard):
  drive = integrated_drive
  drive.fault()
  assert drive.step(1.3, can_valid=True, engageable=True) == 0
  if guard == 'feature_off':
    drive.helper.can_error_cruise_resume = False
  if guard == 'lat_disabled':
    drive.state.latEnabled = False
  if guard == 'lat_inactive':
    drive.control.latActive = False
  if guard == 'park':
    drive.state.gearShifter = 'park'
  if guard == 'standstill':
    drive.state.standstill = True
  assert drive.step(1.501) == 0


def test_radar_can_error_with_valid_vehicle_can_is_not_a_resume_trigger(integrated_drive):
  from cereal import log
  drive = integrated_drive
  assert drive.step(1., enabled=True) == 0
  can_error = log.OnroadEvent(name='canError', noEntry=True, immediateDisable=True)
  assert drive.step(1.1, enabled=False, can_valid=True, events=[can_error], engageable=False) == 0
  assert drive.step(1.3, engageable=True) == 0
  assert drive.step(1.501) == 0


def test_stale_onroad_status_cannot_arm_a_new_recovery(integrated_drive):
  from cereal import log
  drive = integrated_drive
  assert drive.step(1., enabled=True) == 0
  can_error = log.OnroadEvent(name='canError', noEntry=True, immediateDisable=True)
  assert drive.step(3., enabled=False, can_valid=False, events=[can_error], engageable=False, fresh=False) == 0
  assert drive.step(3.2, can_valid=True, engageable=True) == 0
  assert drive.step(3.401) == 0


def test_pre_fault_model_sample_does_not_start_the_recovery_stability_window(integrated_drive):
  from cereal import log
  drive = integrated_drive
  assert drive.step(1., enabled=True) == 0
  assert drive.step(1.09) == 0
  can_error = log.OnroadEvent(name='canError', noEntry=True, immediateDisable=True)
  assert drive.step(1.1, enabled=False, can_valid=False, events=[can_error], engageable=False,
                    frozen_services=('modelV2',)) == 0
  # This sample is only 30ms old, but it predates the CAN disengagement.
  assert drive.step(1.12, can_valid=True, engageable=True, frozen_services=('modelV2',)) == 0
  assert drive.step(1.13) == 0
  assert drive.step(1.321) == 0
  assert drive.step(1.331) == 1


@pytest.mark.parametrize('service', ('selfdriveState', 'carControl', 'longitudinalPlan', 'radarState', 'modelV2'))
def test_expired_critical_publisher_cannot_complete_a_stability_window(integrated_drive, service):
  drive = integrated_drive
  drive.fault()
  assert drive.step(1.3, can_valid=True, engageable=True) == 0
  assert drive.step(1.49, frozen_services=(service,)) == 0
  assert drive.step(1.501, frozen_services=(service,)) == 0
  assert drive.step(1.6) == 0
  assert drive.step(1.801) == 1


@pytest.mark.parametrize('when', ('fault', 'pending'))
def test_another_hard_fault_prevents_can_only_recovery(integrated_drive, when):
  from cereal import log
  drive = integrated_drive
  can_error = log.OnroadEvent(name='canError', noEntry=True, immediateDisable=True)
  mismatch = log.OnroadEvent(name='controlsMismatch', noEntry=True, immediateDisable=True)
  assert drive.step(1., enabled=True) == 0
  events = [can_error, mismatch] if when == 'fault' else [can_error]
  assert drive.step(1.1, enabled=False, can_valid=False, engageable=False, events=events) == 0
  if when == 'pending':
    assert drive.step(1.2, events=[mismatch]) == 0
  assert drive.step(1.3, can_valid=True, engageable=True) == 0
  assert drive.step(1.501) == 0


@pytest.mark.parametrize('attribute,value', (('brand', 'toyota'), ('flags', 0),
                                            ('pcmCruise', True), ('openpilotLongitudinalControl', False),
                                            ('passive', True)))
def test_unrelated_vehicle_control_modes_are_not_opted_into_recovery(integrated_drive, attribute, value):
  drive = integrated_drive
  setattr(drive.helper.CP, attribute, value)
  drive.fault()
  assert drive.step(1.3, can_valid=True, engageable=True) == 0
  assert drive.step(1.501) == 0


def test_an_already_held_res_button_blocks_recovery_without_another_edge(integrated_drive):
  from cereal import car, log
  drive = integrated_drive
  assert drive.step(1., enabled=True, full_update=True) == 0
  drive.state.buttonEvents = [car.CarState.ButtonEvent(type='accelCruise', pressed=True)]
  assert drive.step(1.01, full_update=True) == 0
  drive.state.buttonEvents = []
  # The original press edge is now older than the short input-inhibit window.
  assert drive.step(1.8, full_update=True) == 0
  assert drive.helper.button_cnt > 0
  can_error = log.OnroadEvent(name='canError', noEntry=True, immediateDisable=True)
  assert drive.step(1.9, enabled=False, can_valid=False, engageable=False,
                    events=[can_error], full_update=True) == 0
  assert drive.step(2.1, can_valid=True, engageable=True, full_update=True) == 0
  assert drive.step(2.301, full_update=True) == 0
  assert drive.helper.button_cnt > 0


def test_held_set_at_request_boundary_yields_without_a_button_edge(integrated_drive):
  from cereal import car
  drive = integrated_drive
  drive.fault()
  assert drive.step(1.3, can_valid=True, engageable=True) == 0
  drive.helper.button_prev = car.CarState.ButtonEvent.Type.decelCruise
  drive.helper.button_cnt = 10
  assert list(drive.state.buttonEvents) == []
  assert drive.step(1.501) == 0
  drive.helper.button_cnt = 0
  assert drive.step(1.8) == 0


@pytest.mark.parametrize('command,argument,target', (('CRUISE', 'STOP', 5.), ('SPEED', '45', 45.)))
def test_new_remote_speed_command_is_not_overwritten_by_recovery(integrated_drive, command, argument, target):
  from cereal import custom
  drive = integrated_drive
  drive.fault(full_update=True)
  assert drive.step(1.3, can_valid=True, engageable=True, full_update=True) == 0
  drive.sm.data['carrotMan'] = custom.CarrotMan(carrotCmdIndex=1, carrotCmd=command, carrotArg=argument,
                                               nRoadLimitSpeed=30, desiredSpeed=250.)
  drive.sm.alive['carrotMan'] = True
  assert drive.step(1.501, full_update=True) == 0
  assert drive.helper.v_cruise_kph == target
  assert drive.helper.v_cruise_cluster_kph == target
  assert drive.step(1.8, full_update=True) == 0
