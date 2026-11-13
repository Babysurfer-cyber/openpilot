"""Exercise live blinker coordination at production interface boundaries."""
from types import SimpleNamespace

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car import interfaces as base_interfaces
from opendbc.car.hyundai import carcontroller, hyundaicanfd, interface
from opendbc.car.hyundai.blinker_runtime import LiveBlinkerTest, RuntimeOutput
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags, HyundaiSafetyFlags


@pytest.fixture
def settings(monkeypatch):
  values = {"HyundaiCameraSCC": 1, "CanfdHDA2": 1, "HyundaiBlinkerTest": 0}
  writes, reads = [], []

  class MemoryParams:
    def get_int(self, key):
      reads.append(key)
      return int(values.get(key, 0))

    def get_bool(self, key):
      return bool(values.get(key, 0))

    def get(self, key):
      return values.get(key)

    def put(self, key, value):
      writes.append((key, value))
      values[key] = value

    put_nonblocking = put
    put_int = put

  for module in (base_interfaces, carcontroller, hyundaicanfd, interface):
    monkeypatch.setattr(module, "Params", MemoryParams)
  monkeypatch.setattr(carcontroller.time, "monotonic", lambda: 1.)
  return SimpleNamespace(values=values, writes=writes, reads=reads)


def parameters(candidate=CAR.KIA_EV6, fingerprint=None):
  return interface.CarInterface.get_params(candidate, fingerprint or gen_empty_fingerprint(), [], True, False, False)


@pytest.mark.parametrize("requested", [-1, 0, 1, 2, 3, 255])
def test_every_selection_preserves_all_shared_carparams_fields(settings, requested):
  baseline = parameters()
  assert baseline.openpilotLongitudinalControl and not baseline.pcmCruise
  assert not baseline.flags & HyundaiFlags.BLINKER_TEST
  assert not baseline.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.BLINKER_TEST
  settings.writes.clear()
  settings.values["HyundaiBlinkerTest"] = requested
  assert parameters().to_dict() == baseline.to_dict()
  assert not any(key == "HyundaiBlinkerTestStatus" for key, _ in settings.writes)


@pytest.mark.parametrize("candidate,camera_scc,second_panda", [
  (CAR.HYUNDAI_IONIQ_5, 1, False), (CAR.KIA_EV6_PE, 1, False),
  (CAR.KIA_OPTIMA_G4_FL, 1, False), (CAR.KIA_EV6, 0, False), (CAR.KIA_EV6, 1, True),
])
def test_unsupported_selection_preserves_existing_driving_configuration(settings, candidate, camera_scc, second_panda):
  settings.values["HyundaiCameraSCC"] = camera_scc
  fingerprint = gen_empty_fingerprint()
  if second_panda:
    fingerprint[4][0x35] = 32
  baseline = parameters(candidate, fingerprint)
  settings.values["HyundaiBlinkerTest"] = 1
  assert parameters(candidate, fingerprint).to_dict() == baseline.to_dict()


@pytest.mark.parametrize("requested", [0, 1, 2])
def test_normal_startup_retains_existing_ecu_initialization(settings, monkeypatch, requested):
  settings.values.update(HyundaiBlinkerTest=requested, HyundaiCameraSCC=0)
  cp = parameters()
  calls = []
  monkeypatch.setattr(interface, "disable_ecu", lambda *args, **kwargs: calls.append(kwargs))
  interface.CarInterface.init(cp, None, None)
  assert len(calls) == 1
  assert calls[0]["addr"] == 0x730 and calls[0]["bus"] == 1
  assert ("LongitudinalPersonalityMax", 4) in settings.writes


def test_camera_scc_startup_does_not_add_parking_ecu_diagnostics(settings, monkeypatch):
  settings.values["HyundaiBlinkerTest"] = 1
  cp = parameters()
  calls = []
  monkeypatch.setattr(interface, "disable_ecu", lambda *args, **kwargs: calls.append(kwargs))
  interface.CarInterface.init(cp, None, None)
  assert not calls


@pytest.mark.parametrize("runtime_available,blocked", [(False, False), (True, False), (True, True)])
def test_interface_observes_rx_and_blocks_cruise_only_while_runtime_blocks(settings, monkeypatch, runtime_available, blocked):
  ci = interface.CarInterface.__new__(interface.CarInterface)
  ci.CP = parameters()
  events = []
  runtime = SimpleNamespace(observe=lambda packets, now: events.append(("observe", packets, now)), blocking=blocked)
  ci.CC = SimpleNamespace(blinker_runtime=runtime if runtime_available else None)
  result = structs.CarState()
  result.canValid = True
  result.cruiseState.available = result.cruiseState.enabled = True
  packets = [(1_000_000_000, [(0x7B9, b"\x02\x50\x03\0\0\0\0\0", 0)])]

  def base_update(self, received):
    events.append(("base", received))
    return result

  monkeypatch.setattr(base_interfaces.CarInterfaceBase, "update", base_update)
  monkeypatch.setattr(interface.time, "monotonic_ns", lambda: 1_500_000_000)
  returned = ci.update(packets)
  assert returned is result and returned.canValid
  assert returned.cruiseState.available == (not blocked)
  assert returned.cruiseState.enabled == (not blocked)
  assert events == ([("observe", packets, 1.5)] if runtime_available else []) + [("base", packets)]


class NormalPathReached(Exception):
  pass


class FakeSubMaster:
  def __init__(self, services):
    self.services, self.calls = services, []
    self.valid, self.alive = {"pandaStates": True}, {"pandaStates": True}
    self.updated = {"radarState": False}
    # Publisher time can use CLOCK_BOOTTIME; local receipt uses MONOTONIC.
    self.logMonoTime, self.recv_time = {"pandaStates": 900_000_000_000}, {"pandaStates": 1.}
    self.states = [SimpleNamespace(safetyModel="hyundaiCanfd", safetyParam=29, faults=[],
                                   safetyRxChecksInvalid=False, controlsAllowed=False)]

  def update(self, timeout):
    self.calls.append(timeout)

  def __getitem__(self, service):
    if service == "pandaStates":
      return self.states
    raise NormalPathReached(service)


def make_controller(settings, monkeypatch, candidate=CAR.KIA_EV6):
  monkeypatch.setattr(carcontroller.messaging, "SubMaster", FakeSubMaster)
  return carcontroller.CarController(DBC[candidate], parameters(candidate))


@pytest.mark.parametrize("enabled,lat_active,long_active", [(False, False, False), (True, False, False),
                                                          (False, True, False), (False, False, True)])
def test_controller_blocks_driving_but_keeps_panda_and_cached_setting_poll(settings, monkeypatch,
                                                                          enabled, lat_active, long_active):
  settings.values["HyundaiBlinkerTest"] = 1
  controller = make_controller(settings, monkeypatch)
  assert isinstance(controller.blinker_runtime, LiveBlinkerTest)
  assert "pandaStates" in controller.sm.services
  sent = [(0x7B1, b"\x02\x10\x03\0\0\0\0\0", 0)]
  calls = []

  def experiment_update(request, now, observation, **conditions):
    calls.append((request, now, observation, conditions))
    return RuntimeOutput(True, sent)

  controller.blinker_runtime = SimpleNamespace(update=experiment_update, blocking=True)
  cc = structs.CarControl()
  cc.enabled, cc.latActive, cc.longActive = enabled, lat_active, long_active
  cc.actuators.accel, cc.actuators.torque = 1., 0.5
  cs = SimpleNamespace(out=structs.CarState())
  cs.out.canValid = True
  settings.reads.clear()
  for frame in range(12):
    if frame == 1:
      settings.values["HyundaiBlinkerTest"] = 0
    actuators, messages = controller.update(cc, cs, 1_000_000_000 + frame * 10_000_000)
    assert actuators.to_dict() == structs.CarControl.Actuators().to_dict()
    assert messages is sent
  assert controller.frame == 12 and len(controller.sm.calls) == 12
  assert settings.reads.count("HyundaiBlinkerTest") == 2
  assert [request for request, _, _, _ in calls] == [1] * 10 + [0] * 2
  assert all(conditions == {"enabled": enabled or lat_active or long_active, "can_valid": True}
             for _, _, _, conditions in calls)
  assert calls[0][2].received == 1.  # Unrelated publisher time 900s is unused.
  assert calls[0][2].configs == (("hyundaiCanfd", 29),)
  cs.out.canValid = False
  controller.update(cc, cs, 1_120_000_000)
  assert calls[-1][3]["can_valid"] is False


@pytest.mark.parametrize("invalid", ["not_valid", "not_alive", "fault", "rx_invalid", "controls_allowed"])
def test_controller_reports_panda_health_and_controls_to_coordinator(settings, monkeypatch, invalid):
  controller = make_controller(settings, monkeypatch)
  if invalid == "not_valid":
    controller.sm.valid["pandaStates"] = False
  elif invalid == "not_alive":
    controller.sm.alive["pandaStates"] = False
  elif invalid == "fault":
    controller.sm.states[0].faults = ["relayMalfunction"]
  elif invalid == "rx_invalid":
    controller.sm.states[0].safetyRxChecksInvalid = True
  else:
    controller.sm.states[0].controlsAllowed = True
  seen = []
  controller.blinker_runtime = SimpleNamespace(blocking=True, update=lambda selection, now, observation, **kwargs:
                                              (seen.append(observation) or RuntimeOutput(True, [])))
  controller.update(structs.CarControl(), SimpleNamespace(out=structs.CarState()), 1_000_000_000)
  assert len(seen) == 1
  assert seen[0].valid == (invalid == "controls_allowed")
  assert seen[0].controls_allowed == (invalid == "controls_allowed")


def test_normal_runtime_result_continues_original_driving_controller(settings, monkeypatch):
  controller = make_controller(settings, monkeypatch)
  controller.blinker_runtime = SimpleNamespace(blocking=False, update=lambda *args, **kwargs: RuntimeOutput(False, []))
  with pytest.raises(NormalPathReached):
    controller.update(structs.CarControl(), SimpleNamespace(out=structs.CarState()), 1_000_000_000)


def test_unsupported_selection_reports_status_and_continues_original_path(settings, monkeypatch):
  settings.values["HyundaiBlinkerTest"] = 1
  controller = make_controller(settings, monkeypatch, CAR.HYUNDAI_IONIQ_5)
  assert controller.blinker_runtime is None
  with pytest.raises(NormalPathReached):
    controller.update(structs.CarControl(), SimpleNamespace(out=structs.CarState()), 1_000_000_000)
  assert "unsupported" in settings.values["HyundaiBlinkerTestStatus"]


def test_confirmed_stop_resets_controller_history_and_keeps_this_cycle_neutral(settings, monkeypatch):
  controller = make_controller(settings, monkeypatch)
  controller.apply_torque_last, controller.accel_last, controller.angle_limit_counter = 300, 1.5, 40
  controller.corner_radar_cutin = True
  previous_jerk = controller.hyundai_jerk
  controller.blinker_runtime = SimpleNamespace(blocking=True, update=lambda *args, **kwargs: RuntimeOutput(True, [], True))
  cs = SimpleNamespace(out=structs.CarState())
  cs.out.steeringAngleDeg = 7.
  actuators, sent = controller.update(structs.CarControl(), cs, 1_000_000_000)
  assert not sent and actuators.to_dict() == structs.CarControl.Actuators().to_dict()
  assert controller.apply_torque_last == controller.accel_last == controller.angle_limit_counter == 0
  assert controller.apply_angle_last == 7. and not controller.corner_radar_cutin
  assert controller.hyundai_jerk is not previous_jerk


def test_panda_receive_after_card_timestamp_uses_nonnegative_runtime_age(settings, monkeypatch):
  controller = make_controller(settings, monkeypatch)
  # card captures now_nanos before controller's sm.update receives Panda data.
  controller.sm.recv_time["pandaStates"] = 1.001
  monkeypatch.setattr(carcontroller.time, "monotonic", lambda: 1.002)
  seen = []
  controller.blinker_runtime = SimpleNamespace(blocking=True, update=lambda selection, now, observation, **kwargs:
                                              (seen.append((now, observation)) or RuntimeOutput(True, [])))
  controller.update(structs.CarControl(), SimpleNamespace(out=structs.CarState()), 1_000_000_000)
  assert len(seen) == 1
  now, observation = seen[0]
  assert 0 <= now - observation.received <= 0.3
