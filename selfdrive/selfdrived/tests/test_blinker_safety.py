from copy import deepcopy
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.selfdrived.blinker_safety import BlinkerSafetyExpectation, correlated_blinker_request


def setup_state():
  cp = SimpleNamespace(brand="hyundai", carFingerprint="KIA_EV6", passive=False, dashcamOnly=False,
                       flags=(1 << 13) | 8, alternativeExperience=1,
                       safetyConfigs=[SimpleNamespace(safetyModel="hyundaiCanfd", safetyParam=61)])
  panda = SimpleNamespace(safetyModel="hyundaiCanfd", safetyParam=61 | 1024,
                          alternativeExperience=1, controlsAllowed=False, safetyRxChecksInvalid=False, faults=[])
  return cp, panda


class MemoryParams:
  def __init__(self, mode_request="test:session-1", ack="test:session-1"):
    self.values = {"HyundaiBlinkerTestSafetyRequest": mode_request, "HyundaiBlinkerTestSafetyAck": ack}
    self.reads = []

  def get(self, key):
    self.reads.append(key)
    return self.values[key]


@pytest.mark.parametrize("mode_request,ack", [
  ("test:session-1", "test:session-1"),
  ("normal:session-1", "test:session-1"),
  ("normal:session-1", "normal:session-1"),
  (b"test:session-1", b"test:session-1"),
])
def test_only_correlated_disabled_test_config_is_expected(mode_request, ack):
  cp, panda = setup_state()
  original_cp, original_panda = deepcopy(cp), deepcopy(panda)
  expectation = BlinkerSafetyExpectation()
  assert expectation.matches(cp, panda, False, MemoryParams(mode_request, ack), 10.)
  assert vars(cp) == vars(original_cp)
  assert vars(panda) == vars(original_panda)


@pytest.mark.parametrize("mode_request,ack", [
  (None, None), ("", ""), ("test:a", "test:b"), ("normal:a", "test:b"),
  ("test:a", "normal:a"), ("test:a", "error:test:a"),
  ("test:a:b", "test:a:b"), ("test:", "test:"), ("test:a b", "test:a b"),
  ("test:" + "a" * 65, "test:" + "a" * 65), ("test:한글", "test:한글"),
  (b"test:\xff", b"test:\xff"), ("normal:a", None),
])
def test_unconfirmed_or_unrelated_requests_do_not_mask_mismatch(mode_request, ack):
  assert not correlated_blinker_request(mode_request, ack)
  cp, panda = setup_state()
  assert not BlinkerSafetyExpectation().matches(cp, panda, False, MemoryParams(mode_request, ack), 10.)


@pytest.mark.parametrize("target,key,value", [
  ("cp", "brand", "toyota"), ("cp", "carFingerprint", "KIA_EV6_PE"),
  ("cp", "flags", 8), ("cp", "flags", 1 << 13),
  ("cp", "flags", (1 << 13) | 8 | (1 << 27)),
  ("cp", "passive", True), ("cp", "dashcamOnly", True),
  ("config", "safetyModel", "hyundai"), ("config", "safetyParam", 60),
  ("config", "safetyParam", 53), ("config", "safetyParam", 61 | 1024),
  ("panda", "safetyModel", "allOutput"), ("panda", "safetyParam", 61),
  ("panda", "safetyParam", 61 | 1024 | 512),
  ("panda", "alternativeExperience", 0), ("panda", "controlsAllowed", True),
])
def test_other_vehicle_or_safety_differences_remain_errors_without_param_reads(target, key, value):
  cp, panda = setup_state()
  obj = {"cp": cp, "panda": panda, "config": cp.safetyConfigs[0]}[target]
  setattr(obj, key, value)
  params = MemoryParams()
  assert not BlinkerSafetyExpectation().matches(cp, panda, False, params, 10.)
  assert not params.reads


def test_enabled_controller_is_never_exempt_and_normal_driving_has_no_file_reads():
  cp, panda = setup_state()
  params = MemoryParams()
  expectation = BlinkerSafetyExpectation()
  assert not expectation.matches(cp, panda, True, params, 10.)
  panda.safetyParam = 61
  assert not expectation.matches(cp, panda, False, params, 10.1)
  assert not params.reads


def test_multiple_safety_configs_are_not_supported():
  cp, panda = setup_state()
  cp.safetyConfigs.append(deepcopy(cp.safetyConfigs[0]))
  params = MemoryParams()
  assert not BlinkerSafetyExpectation().matches(cp, panda, False, params, 10.)
  assert not params.reads


def test_requests_are_read_at_most_ten_times_per_second_while_test_is_present():
  cp, panda = setup_state()
  params = MemoryParams()
  expectation = BlinkerSafetyExpectation()
  for tick in range(100):
    assert expectation.matches(cp, panda, False, params, 10. + tick / 100.)
  assert len(params.reads) <= 20
  params.values["HyundaiBlinkerTestSafetyAck"] = "test:wrong-session"
  assert not expectation.matches(cp, panda, False, params, 11.1)


@pytest.fixture
def actual_panda_event_checks():
  # Execute the actual event-check loop without importing unrelated native
  # vision/location services that the full daemon needs on the device.
  source = Path(__file__).parents[1] / "selfdrived.py"
  tree = ast.parse(source.read_text())
  daemon = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "SelfdriveD")
  update = next(node for node in daemon.body if isinstance(node, ast.FunctionDef) and node.name == "update_events")
  loop = next(node for node in update.body if isinstance(node, ast.For) and
              isinstance(node.target, ast.Tuple) and any(isinstance(name, ast.Name) and name.id == "pandaState" for name in node.target.elts))
  code = compile(ast.fix_missing_locations(ast.Module(body=[loop], type_ignores=[])), str(source), "exec")

  def check(cp, panda, *, enabled=False, invalid=False, mismatch_count=0, faults=(), ack="test:session-1"):
    panda.safetyRxChecksInvalid, panda.faults = invalid, faults
    class SubMaster(dict):
      frame = 2000
    events = set()
    daemon = SimpleNamespace(CP=cp, sm=SubMaster(pandaStates=[panda]), enabled=enabled,
                             params=MemoryParams(ack=ack), blinker_safety_expectation=BlinkerSafetyExpectation(),
                             mismatch_counter=mismatch_count, events=SimpleNamespace(add=events.add))
    scope = {"self": daemon, "DT_CTRL": 0.01, "IGNORED_SAFETY_MODES": ("silent", "noOutput"),
             "time": SimpleNamespace(monotonic=lambda: 10.),
             "EventName": SimpleNamespace(controlsMismatch="controlsMismatch", relayMalfunction="relayMalfunction"),
             "log": SimpleNamespace(PandaState=SimpleNamespace(FaultType=SimpleNamespace(relayMalfunction="relay")))}
    exec(code, scope)
    return events
  return check


@pytest.mark.parametrize("kwargs,expected", [
  ({}, set()),
  ({"invalid": True}, {"controlsMismatch"}),
  ({"mismatch_count": 200}, {"controlsMismatch"}),
  ({"faults": ("relay",)}, {"relayMalfunction"}),
  ({"ack": "test:wrong-session"}, {"controlsMismatch"}),
  ({"enabled": True}, {"controlsMismatch"}),
])
def test_actual_event_checks_preserve_health_control_and_fault_errors(actual_panda_event_checks, kwargs, expected):
  cp, panda = setup_state()
  assert actual_panda_event_checks(cp, panda, **kwargs) == expected


def test_real_capnp_enum_matches_the_same_narrow_configuration():
  from cereal import car, log
  cp = car.CarParams.new_message()
  cp.brand, cp.carFingerprint, cp.flags = "hyundai", "KIA_EV6", (1 << 13) | 8
  cp.safetyConfigs = [{"safetyModel": "hyundaiCanfd", "safetyParam": 61}]
  panda = log.PandaState.new_message()
  panda.safetyModel, panda.safetyParam = "hyundaiCanfd", 61 | 1024
  assert BlinkerSafetyExpectation().matches(cp, panda, False, MemoryParams(), 10.)
