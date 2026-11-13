"""Exercise the parked experiment at the production interface boundaries."""
from types import SimpleNamespace

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car import interfaces as base_interfaces
from opendbc.car.hyundai import carcontroller, hyundaicanfd, interface
from opendbc.car.hyundai.blinker_test import BlinkerHandoffTest
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags, HyundaiSafetyFlags


@pytest.fixture
def settings(monkeypatch):
  values = {"HyundaiCameraSCC": 1, "CanfdHDA2": 1, "HyundaiBlinkerTest": 0}
  writes = []

  class MemoryParams:
    def get_int(self, key):
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
  return values, writes


def parameters(candidate=CAR.KIA_EV6, fingerprint=None):
  return interface.CarInterface.get_params(candidate, fingerprint or gen_empty_fingerprint(), [], True, False, False)


@pytest.mark.parametrize("requested", [-1, 0, 3, 255])
def test_off_or_invalid_setting_preserves_the_normal_ev6_configuration(settings, requested):
  values, writes = settings
  baseline = parameters()
  assert baseline.openpilotLongitudinalControl
  assert not baseline.pcmCruise
  assert not baseline.flags & HyundaiFlags.BLINKER_TEST
  assert not baseline.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.BLINKER_TEST
  writes.clear()
  values["HyundaiBlinkerTest"] = requested
  assert parameters().to_dict() == baseline.to_dict()
  assert not any(key == "HyundaiBlinkerTestStatus" for key, _ in writes)


@pytest.mark.parametrize("requested", [1, 2])
def test_supported_ev6_arms_an_isolated_session_without_changing_vehicle_tuning(settings, requested):
  values, writes = settings
  normal = parameters()
  values["HyundaiBlinkerTest"] = requested
  test = parameters()
  assert test.flags == normal.flags | HyundaiFlags.BLINKER_TEST
  assert len(test.safetyConfigs) == 1
  assert test.safetyConfigs[-1].safetyModel == structs.CarParams.SafetyModel.hyundaiCanfd
  assert test.safetyConfigs[-1].safetyParam == normal.safetyConfigs[-1].safetyParam | HyundaiSafetyFlags.BLINKER_TEST
  assert not test.openpilotLongitudinalControl
  assert test.pcmCruise
  assert test.minEnableSpeed > 100.
  # Radar, tuning, wheelbase, fingerprints and every other unrelated field stay
  # identical to the current branch's normal configuration.
  normal_fields, test_fields = normal.to_dict(), test.to_dict()
  for key in ("flags", "safetyConfigs", "openpilotLongitudinalControl", "pcmCruise", "minEnableSpeed"):
    normal_fields.pop(key, None)
    test_fields.pop(key, None)
  assert test_fields == normal_fields
  assert any(key == "HyundaiBlinkerTestStatus" and "armed" in value for key, value in writes)


@pytest.mark.parametrize("candidate,camera_scc,second_panda", [
  (CAR.HYUNDAI_IONIQ_5, 1, False),
  (CAR.KIA_EV6_PE, 1, False),
  (CAR.KIA_OPTIMA_G4_FL, 1, False),
  (CAR.KIA_EV6, 0, False),
  (CAR.KIA_EV6, 1, True),
])
def test_unsupported_vehicle_or_bus_layout_cannot_arm(settings, candidate, camera_scc, second_panda):
  values, writes = settings
  values["HyundaiCameraSCC"] = camera_scc
  fingerprint = gen_empty_fingerprint()
  if second_panda:
    fingerprint[4][0x35] = 32
  baseline = parameters(candidate, fingerprint)
  values["HyundaiBlinkerTest"] = 1
  result = parameters(candidate, fingerprint)
  assert result.dashcamOnly
  assert not result.openpilotLongitudinalControl
  assert len(result.safetyConfigs) == 1
  assert result.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput
  baseline_fields, result_fields = baseline.to_dict(), result.to_dict()
  for key in ("dashcamOnly", "openpilotLongitudinalControl", "safetyConfigs"):
    baseline_fields.pop(key, None)
    result_fields.pop(key, None)
  assert result_fields == baseline_fields
  assert not result.flags & HyundaiFlags.BLINKER_TEST
  assert not any(config.safetyParam & HyundaiSafetyFlags.BLINKER_TEST for config in result.safetyConfigs)
  assert "unsupported" in values["HyundaiBlinkerTestStatus"]


def test_test_boot_does_not_disable_any_ecu_at_interface_startup(settings, monkeypatch):
  values, writes = settings
  values.update(HyundaiBlinkerTest=1, EnableRadarTracks=1)
  cp = parameters()
  # Even a stale legacy blinker flag must not cause a startup parking knockout.
  cp.flags |= HyundaiFlags.ENABLE_BLINKERS.value
  cp.openpilotLongitudinalControl = True
  calls = []
  monkeypatch.setattr(interface, "disable_ecu", lambda *args, **kwargs: calls.append((args, kwargs)))
  monkeypatch.setattr(interface, "enable_radar_tracks", lambda *args, **kwargs: calls.append((args, kwargs)))
  writes.clear()
  interface.CarInterface.init(cp, None, None)
  assert calls == []
  assert writes == []


def test_normal_startup_keeps_the_existing_ecu_initialization(settings, monkeypatch):
  values, _ = settings
  values["HyundaiCameraSCC"] = 0
  cp = parameters()
  cp.flags &= ~HyundaiFlags.CANFD_CAMERA_SCC.value
  calls = []
  monkeypatch.setattr(interface, "disable_ecu", lambda *args, **kwargs: calls.append(kwargs))
  interface.CarInterface.init(cp, None, None)
  assert len(calls) == 1
  assert calls[0]["addr"] == 0x730
  assert calls[0]["bus"] == 1


def test_unsupported_test_request_does_not_run_legacy_radar_diagnostics(settings, monkeypatch):
  values, _ = settings
  values.update(HyundaiBlinkerTest=1, EnableRadarTracks=1)
  cp = parameters(CAR.KIA_OPTIMA_G4_FL)
  assert cp.dashcamOnly
  calls = []
  monkeypatch.setattr(interface, "disable_ecu", lambda *args, **kwargs: calls.append("disable"))
  monkeypatch.setattr(interface, "enable_radar_tracks", lambda *args, **kwargs: calls.append("radar"))
  interface.CarInterface.init(cp, None, None)
  assert not calls


@pytest.mark.parametrize("test_boot", [False, True])
def test_interface_observes_physical_rx_and_blocks_cruise_only_in_test_mode(settings, monkeypatch, test_boot):
  values, _ = settings
  values["HyundaiBlinkerTest"] = int(test_boot)
  ci = interface.CarInterface.__new__(interface.CarInterface)
  ci.CP = parameters()
  events = []
  observer = SimpleNamespace(observe=lambda packets, now: events.append(("observe", packets, now)))
  ci.CC = SimpleNamespace(blinker_test=observer if test_boot else None)
  result = structs.CarState()
  result.canValid = True
  result.cruiseState.available = True
  result.cruiseState.enabled = True
  packets = [(1_000_000_000, [(0x7B9, b"\x02\x50\x03\0\0\0\0\0", 0)])]

  def base_update(self, received):
    events.append(("base", received))
    return result

  monkeypatch.setattr(base_interfaces.CarInterfaceBase, "update", base_update)
  monkeypatch.setattr(interface.time, "monotonic_ns", lambda: 1_500_000_000)
  returned = ci.update(packets)
  assert returned is result
  assert returned.canValid
  assert returned.cruiseState.available == (not test_boot)
  assert returned.cruiseState.enabled == (not test_boot)
  if test_boot:
    assert events == [("observe", packets, 1.5), ("base", packets)]
  else:
    assert events == [("base", packets)]


@pytest.mark.parametrize("enabled,lat_active,long_active", [(False, False, False), (True, False, False),
                                                          (False, True, False), (False, False, True)])
def test_controller_never_runs_driving_transmit_path_during_a_test_boot(settings, monkeypatch, enabled, lat_active, long_active):
  values, writes = settings
  values["HyundaiBlinkerTest"] = 1
  cp = parameters()

  class NoDrivingSubscriptions:
    def __init__(self, *args, **kwargs):
      pass

    def update(self, *args, **kwargs):
      pytest.fail("the parked test entered the normal driving controller")

  monkeypatch.setattr(carcontroller.messaging, "SubMaster", NoDrivingSubscriptions)
  controller = carcontroller.CarController(DBC[CAR.KIA_EV6], cp)
  assert isinstance(controller.blinker_test, BlinkerHandoffTest)
  sent = [(0x7B1, b"\x02\x10\x03\0\0\0\0\0", 0)]
  calls = []

  def experiment_update(request, now, **conditions):
    calls.append((request, now, conditions))
    return sent

  experiment = SimpleNamespace(update=experiment_update, status="waiting for parked conditions")
  controller.blinker_test = experiment
  cc = structs.CarControl()
  cc.enabled, cc.latActive, cc.longActive = enabled, lat_active, long_active
  cc.actuators.accel = 1.
  cc.actuators.torque = 0.5
  cs = SimpleNamespace(out=structs.CarState())
  cs.out.canValid = True
  writes.clear()
  for frame in range(12):
    if frame == 1:
      values["HyundaiBlinkerTest"] = 0
    actuators, messages = controller.update(cc, cs, 1_000_000_000 + frame * 10_000_000)
    assert actuators.to_dict() == structs.CarControl.Actuators().to_dict()
    assert messages is sent
  assert controller.frame == 12
  assert [request for request, _, _ in calls] == [1] * 10 + [0] * 2
  assert all(conditions == {"enabled": enabled or lat_active or long_active, "can_valid": True}
             for _, _, conditions in calls)
  assert calls[0][1] == 1.
  assert writes == [("HyundaiBlinkerTestStatus", experiment.status)]
  # Changing the status is published even between parameter polls.
  experiment.status = "restoring parking ECU"
  cs.out.canValid = False
  controller.update(cc, cs, 1_120_000_000)
  assert calls[-1][2]["can_valid"] is False
  assert writes[-1] == ("HyundaiBlinkerTestStatus", experiment.status)
