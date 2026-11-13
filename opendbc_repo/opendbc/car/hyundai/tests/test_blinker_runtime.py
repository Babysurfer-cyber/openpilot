"""Live-mode coordination simulations using real DBC CAN input."""
import pytest

from opendbc.car import structs
from opendbc.car.hyundai.blinker_runtime import ACK, REQUEST, LiveBlinkerTest, PandaObservation, supports_blinker_runtime
from opendbc.car.hyundai.tests.test_blinker_test import HandoffRig
from opendbc.car.hyundai.values import CAR, HyundaiFlags


class MemoryParams:
  def __init__(self):
    self.values = {}
    self.writes = []

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value):
    self.values[key] = value
    self.writes.append((key, value))

  put_nonblocking = put


def cp_for_runtime():
  cp = structs.CarParams()
  cp.brand = "hyundai"
  cp.carFingerprint = CAR.KIA_EV6
  cp.flags = int(HyundaiFlags.CANFD | HyundaiFlags.CAMERA_SCC | HyundaiFlags.EV)
  cp.safetyConfigs = [{"safetyModel": "hyundaiCanfd", "safetyParam": 1 | 4 | 8 | 16}]
  return cp


class RuntimeRig(HandoffRig):
  def __init__(self, direction=1):
    super().__init__(direction)
    self.params = MemoryParams()
    self.runtime = LiveBlinkerTest(cp_for_runtime(), self.params)
    self.panda_mode = "normal"
    self.panda_valid = True
    self.panda_controls = False
    self.panda_age = 0.
    self.panda_received = None
    self.last_output = None
    self.outputs = []
    rig = self

    class RuntimeProbe:
      def observe(self, packets, now):
        rig.runtime.observe(packets, now)

      def update(self, selection, now, **conditions):
        configs = rig.runtime.normal_configs if rig.panda_mode == "normal" else rig.runtime.test_configs
        observation = PandaObservation(configs, now - rig.panda_age if rig.panda_received is None else rig.panda_received,
                                       rig.panda_valid, rig.panda_controls)
        rig.last_output = rig.runtime.update(selection, now, observation, **conditions)
        rig.outputs.append(rig.last_output)
        return rig.last_output.messages

      def __getattr__(self, name):
        return getattr(rig.runtime.test, name)

    self.test = RuntimeProbe()

  def reach_entering(self):
    before = len(self.sent)
    for _ in range(50):
      self.tick()
      if self.runtime.phase == "entering":
        assert self.last_output.blocked
        assert len(self.sent) == before
        return
    raise AssertionError(self.runtime.status)


  def acknowledge_test(self):
    self.params.values[ACK] = self.runtime.request
    self.panda_mode = "test"
    out = self.tick()
    assert self.runtime.phase == "running"
    assert not out

  def reach_session(self):
    if self.runtime.phase == "normal":
      self.reach_entering()
      self.acknowledge_test()
    super().reach_session()

  def request_normal(self):
    self.request = 0
    self.tick()
    assert self.runtime.phase == "restoring"
    assert self.runtime.request == f"normal:{self.runtime.session}"

  def confirm_normal(self):
    self.params.values[ACK] = self.runtime.request
    self.panda_mode = "normal"
    for _ in range(25):
      self.tick()
      if self.runtime.phase == "normal":
        assert self.last_output.blocked and self.last_output.reset_controller
        self.tick()
        assert not self.last_output.blocked
        return
    raise AssertionError(self.runtime.status)


class CheckedRuntimeRig(RuntimeRig):
  """Apply requested Panda mode edges, then run the real C RX/TX hooks."""
  def __init__(self, direction=1):
    super().__init__(direction)
    from opendbc.safety.tests.libsafety import libsafety_py
    self.safety_module = libsafety_py
    self.safety = libsafety_py.libsafety
    self.hardware_test = False
    self.mode_edges = []
    self.checked_tx = []
    self._configure(False)
    probe_observe, probe_update = self.test.observe, self.test.update

    def observe(packets, now):
      if self.hardware_test:
        self.safety.set_timer(round(now * 1e6) & 0xFFFFFFFF)
        for _, messages in packets:
          for addr, dat, bus in messages:
            assert self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat)), (hex(addr), bus)
      probe_observe(packets, now)

    def update(selection, now, **conditions):
      messages = probe_update(selection, now, **conditions)
      self.safety.set_timer(round(now * 1e6) & 0xFFFFFFFF)
      for addr, dat, bus in messages:
        assert self.hardware_test  # No test TX while Panda remains normal.
        assert self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)), (
          self.runtime.phase, self.runtime.test.state, hex(addr), dat.hex())
        self.checked_tx.append((addr, dat, bus))
      return messages

    self.test.observe, self.test.update = observe, update

  def _configure(self, test):
    config = self.runtime.test_configs if test else self.runtime.normal_configs
    assert self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, config[0][1]) == 0
    self.safety.init_tests()
    self.hardware_test = test

  def acknowledge_test(self):
    assert self.runtime.request == f"test:{self.runtime.session}"
    self._configure(True)
    self.mode_edges.append("test")
    super().acknowledge_test()

  def confirm_normal(self):
    assert self.runtime.request == f"normal:{self.runtime.session}"
    self._configure(False)
    self.mode_edges.append("normal")
    super().confirm_normal()


@pytest.mark.parametrize("failure", ["wrong_ack", "old_hardware", "wrong_mode", "unhealthy", "controls_allowed"])
def test_no_diagnostics_before_matching_ack_and_new_hardware(failure):
  rig = RuntimeRig()
  rig.reach_entering()
  rig.params.values[ACK] = rig.runtime.request
  rig.panda_mode = "test"
  if failure == "wrong_ack":
    rig.params.values[ACK] = "test:another-session"
  elif failure == "old_hardware":
    rig.panda_received = rig.runtime.requested_at
  elif failure == "wrong_mode":
    rig.panda_mode = "normal"
  elif failure == "unhealthy":
    rig.panda_valid = False
  else:
    rig.panda_controls = True
  for _ in range(30):
    assert not rig.tick()
    assert rig.last_output.blocked
  assert rig.runtime.phase == "entering"
  assert not rig.runtime.session_requested and not rig.runtime.pause_requested


def test_selecting_a_direction_while_moving_does_not_block_normal_control():
  rig = RuntimeRig()
  rig.changes["WHEEL_SPEEDS"] = {"WHEEL_SPEED_1": 25.}
  for _ in range(100):
    assert not rig.tick()
    assert not rig.last_output.blocked
  assert rig.runtime.phase == "normal"
  assert not any(key == REQUEST for key, _ in rig.params.writes)


def test_zero_can_cancel_before_test_mode_ack_and_waits_for_normal_readback():
  rig = RuntimeRig()
  rig.reach_entering()
  nonce = rig.runtime.session
  rig.request_normal()
  assert not rig.sent
  assert rig.runtime.request == f"normal:{nonce}"
  for _ in range(20):
    rig.tick()
    assert rig.last_output.blocked
  rig.confirm_normal()
  assert not rig.runtime.blocking


@pytest.mark.parametrize("side", [1, 2])
def test_both_directions_can_run_stop_restore_and_rearm_without_reboot(side):
  rig = RuntimeRig(side)
  for direction in (side, 3 - side):
    rig.request = direction
    rig.lamps = (False, False)
    rig.reach_hold()
    assert rig.runtime.phase == "running"
    assert rig.runtime.pause_requested and rig.runtime.session_requested
    nonce = rig.runtime.session
    for i in range(60):
      lit = (i // 15) % 2 == 0
      rig.lamps = (lit and direction == 1, lit and direction == 2)
      rig.tick()
      assert rig.last_output.blocked
    rig.request = 0
    rig.tick()
    rig.lamps = (False, False)
    rig.recover()
    assert rig.runtime.phase == "restoring"
    assert rig.runtime.request == f"normal:{nonce}"
    rig.confirm_normal()
    assert rig.runtime.test.state == "waiting"
    assert not rig.runtime.pause_requested and not rig.runtime.session_requested
  tokens = [value for key, value in rig.params.writes if key == REQUEST]
  assert [value.split(":")[0] for value in tokens] == ["test", "normal", "test", "normal"]
  assert tokens[0].split(":")[1] != tokens[2].split(":")[1]


def test_unconfirmed_paused_ecu_never_requests_normal_safety():
  rig = RuntimeRig()
  rig.reach_pulse()
  rig.request = 0
  rig.tick()
  rig.stock = True
  for _ in range(300):
    rig.tick()
    assert rig.runtime.phase == "running"
    assert rig.last_output.blocked
  assert rig.test.state == "failed"
  assert rig.runtime.pause_requested
  assert rig.runtime.request.startswith("test:")
  assert not any(value.startswith("normal:") for key, value in rig.params.writes if key == REQUEST)


def test_no_pause_and_no_diagnostic_ack_can_recover_only_after_terminal_neutral_stock():
  rig = RuntimeRig()
  rig.reach_session()
  rig.request = 0
  rig.tick()
  for _ in range(15):
    rig.tick()
    assert rig.runtime.phase == "running"  # cleanup has not reached terminal
  assert rig.runtime.session_requested and not rig.runtime.pause_requested
  for _ in range(300):
    rig.tick()
    if rig.runtime.phase == "restoring":
      break
  assert rig.runtime.phase == "restoring"
  assert rig.test.state == "failed"
  assert not rig.test.restore_enable_ok and not rig.test.restore_default_ok
  rig.confirm_normal()


@pytest.mark.parametrize("failure", ["stale", "unhealthy", "wrong_param", "wrong_ack", "old_readback", "moving", "enabled"])
def test_no_normal_output_until_exact_fresh_normal_panda_and_parked_settle(failure):
  rig = RuntimeRig()
  rig.reach_entering()
  rig.request_normal()
  rig.params.values[ACK] = rig.runtime.request
  rig.panda_mode = "normal"
  extra = {}
  if failure == "stale":
    rig.panda_age = 0.4
  elif failure == "unhealthy":
    rig.panda_valid = False
  elif failure == "wrong_param":
    rig.panda_mode = "test"
  elif failure == "wrong_ack":
    rig.params.values[ACK] = "normal:wrong-token"
  elif failure == "old_readback":
    rig.panda_received = rig.runtime.requested_at
  elif failure == "moving":
    rig.changes["WHEEL_SPEEDS"] = {"WHEEL_SPEED_1": 0.03125}
  else:
    extra["enabled"] = True
  for _ in range(25):
    rig.tick(**extra)
    assert rig.runtime.phase == "restoring"
    assert rig.last_output.blocked


def test_restore_settle_timer_restarts_after_parked_input_loss():
  rig = RuntimeRig()
  rig.reach_entering()
  rig.request_normal()
  rig.params.values[ACK] = rig.runtime.request
  rig.panda_mode = "normal"
  for _ in range(10):
    rig.tick()
    assert rig.runtime.phase == "restoring"
  rig.changes["TCS"] = {"DriverBraking": 0}
  rig.tick()
  rig.changes = {}
  for _ in range(10):
    rig.tick()
    assert rig.runtime.phase == "restoring"
  rig.confirm_normal()


def test_losing_panda_health_during_hold_stops_requests_and_cannot_resume_on_old_state():
  rig = RuntimeRig()
  rig.reach_hold()
  rig.panda_valid = False
  rig.tick()
  assert rig.test.state == "restore_enable"
  assert rig.last_output.blocked
  rig.request = 0
  rig.recover()
  assert rig.runtime.phase == "restoring"
  rig.params.values[ACK] = rig.runtime.request
  rig.panda_mode = "normal"
  for _ in range(25):
    rig.tick()
    assert rig.last_output.blocked
  rig.panda_valid = True
  rig.confirm_normal()


@pytest.mark.parametrize("first_side", [1, 2])
def test_live_coordinator_repeated_mode_edges_are_compatible_with_actual_c_safety(first_side):
  rig = CheckedRuntimeRig(first_side)
  for side in (first_side, 3 - first_side):
    rig.request = side
    rig.lamps = (False, False)
    rig.reach_hold()
    assert rig.hardware_test and rig.runtime.phase == "running"
    assert rig.safety.get_current_safety_param() == rig.runtime.test_configs[0][1]
    for i in range(50):
      lit = (i // 15) % 2 == 0
      rig.lamps = (lit and side == 1, lit and side == 2)
      rig.tick()
    rig.request = 0
    rig.tick()
    rig.lamps = (False, False)
    rig.recover()
    assert rig.runtime.phase == "restoring" and rig.hardware_test
    rig.confirm_normal()
    assert not rig.hardware_test and not rig.runtime.blocking
    assert rig.safety.get_current_safety_param() == rig.runtime.normal_configs[0][1]
  assert rig.mode_edges == ["test", "normal", "test", "normal"]
  assert len(rig.checked_tx) > 25


@pytest.mark.parametrize("attribute,value", [("passive", True), ("dashcamOnly", True),
                                             ("carFingerprint", CAR.HYUNDAI_IONIQ_5)])
def test_unsupported_vehicle_or_passive_mode_is_not_selected(attribute, value):
  cp = cp_for_runtime()
  setattr(cp, attribute, value)
  assert not supports_blinker_runtime(cp, 0)


def test_unsupported_wiring_or_config_does_not_create_runtime():
  cp = cp_for_runtime()
  assert supports_blinker_runtime(cp, 0)
  assert not supports_blinker_runtime(cp, 1)
  cp.safetyConfigs = [{"safetyModel": "noOutput", "safetyParam": 0}]
  assert not supports_blinker_runtime(cp, 0)


@pytest.mark.parametrize("reason", ["wrong_brand", "old_boot_flag", "already_test_safety"])
def test_runtime_eligibility_rejects_configuration_that_cpp_coordinator_rejects(reason):
  cp = cp_for_runtime()
  if reason == "wrong_brand":
    cp.brand = "toyota"
  elif reason == "old_boot_flag":
    cp.flags |= HyundaiFlags.BLINKER_TEST.value
  else:
    cp.safetyConfigs[0].safetyParam |= 1024
  assert not supports_blinker_runtime(cp, 0)
