"""Camera-SCC steering takeover waits for real command and feedback samples."""

from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carstate, hyundaicanfd
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, HyundaiFlags


DBC = "hyundai_canfd_generated"
START = 2_000_000_000
BUSES = SimpleNamespace(ECAN=0, CAM=2, ACAN=1)
SAMPLES = {
  "MDPS": {"LKA_ACTIVE": 1, "LFA2_ACTIVE": 2, "STEERING_COL_TORQUE": 25},
  "LFA": {"LKA_MODE": 1, "LKA_ACTIVE": 3, "STEER_REQ": 1, "TORQUE_REQUEST": 25},
  "LFA_ALT": {"LKAS_ANGLE_ACTIVE": 2, "LKAS_ANGLE_CMD": 6.4, "LKAS_ANGLE_MAX_TORQUE": 51},
}


class Drive:
  def __init__(self, *, angle=False, registered=True, camera=True):
    flags = HyundaiFlags.CANFD | HyundaiFlags.CANFD_HDA2
    if camera:
      flags |= HyundaiFlags.CAMERA_SCC
    if angle:
      flags |= HyundaiFlags.ANGLE_CONTROL
    self.params = structs.CarParams.new_message(brand="hyundai", carFingerprint=CAR.KIA_EV6, flags=int(flags))
    self.params.init("safetyConfigs", 1)
    self.state = CarState.__new__(CarState)
    self.state.CP = self.params
    self.state.camera_scc_steering_ready = False
    self.state.mdps = self.state.lfa = self.state.lfa_alt = None
    self.state.adrv_0x161 = self.state.steer_touch_2af = None
    self.state.controls_ready_count = 201
    self.pt = CANParser(DBC, [("MDPS", 100)] if registered else [], BUSES.ECAN)
    commands = [("LFA_ALT", 100)] if angle else [("LFA", 100)]
    self.cam = CANParser(DBC, commands if registered else [], BUSES.CAM)
    self.pt.controls_ready = self.cam.controls_ready = True
    self.source_packer = CANPacker(DBC)
    self.output_packer = CANPacker(DBC)
    self.angle = angle

  def receive(self, name, *, timestamp=START, **values):
    parser = self.pt if name in ("MDPS", "STEER_TOUCH_2AF") else self.cam
    frame = self.source_packer.make_can_msg(name, parser.bus, {**SAMPLES.get(name, {}), **values})
    parser.update([(timestamp, [frame])])

  def admit(self):
    self.state._update_camera_scc_steering(self.pt, self.cam)

  def output(self, *, active=False, frame=50):
    control = structs.CarControl.new_message(enabled=active, latActive=active, hudControl={"leadVisible": True})
    return hyundaicanfd.create_steering_messages_camera_scc(
      frame, self.output_packer, self.params, BUSES, control, active, 40 if active else 0,
      self.state, 5.2, 60, self.angle,
    )

  def ready(self):
    self.receive("MDPS")
    self.receive("LFA_ALT" if self.angle else "LFA")
    self.admit()


def decode(messages, name, bus):
  parser = CANParser(DBC, [(name, 100)], bus)
  parser.update([(START, messages)])
  assert parser.ts_nanos[name]["COUNTER"] == START
  return parser.vl[name]


@pytest.mark.parametrize("angle", (False, True))
@pytest.mark.parametrize("present", ("MDPS", "command", "neither"))
def test_a_single_input_cannot_start_a_partial_takeover(angle, present):
  drive = Drive(angle=angle)
  if present == "MDPS":
    drive.receive("MDPS")
  elif present == "command":
    drive.receive("LFA_ALT" if angle else "LFA")
  drive.admit()
  assert not drive.state.camera_scc_steering_ready
  assert drive.output(frame=20) == []


@pytest.mark.parametrize("angle", (False, True))
def test_registered_default_dictionaries_are_not_received_samples(angle):
  drive = Drive(angle=angle)
  assert "MDPS" in drive.pt.vl
  assert ("LFA_ALT" if angle else "LFA") in drive.cam.vl
  drive.pt.update([(START, [])])
  drive.cam.update([(START, [])])
  drive.admit()
  assert drive.state.mdps is None
  command = drive.state.lfa_alt if angle else drive.state.lfa
  assert command is None
  assert not drive.state.camera_scc_steering_ready
  assert drive.output(active=True) == []


@pytest.mark.parametrize("age,ready", ((0, True), (100_000_000, True), (100_000_001, False), (-1, False)))
def test_initial_admission_requires_recent_nonfuture_samples(age, ready):
  drive = Drive()
  drive.receive("MDPS")
  drive.receive("LFA")
  drive.pt.update([(START + age, [])])
  drive.cam.update([(START + age, [])])
  drive.admit()
  assert drive.state.camera_scc_steering_ready == ready
  assert bool(drive.output()) == ready


@pytest.mark.parametrize("old_source", ("MDPS", "LFA"))
def test_a_fresh_source_cannot_hide_the_other_sources_startup_gap(old_source):
  drive = Drive()
  drive.receive("MDPS")
  drive.receive("LFA")
  drive.receive("LFA" if old_source == "MDPS" else "MDPS", timestamp=START + 100_000_001)
  drive.admit()
  assert not drive.state.camera_scc_steering_ready
  assert drive.output() == []


@pytest.mark.parametrize("angle", (False, True))
@pytest.mark.parametrize("active", (False, True))
def test_first_eligible_batch_contains_command_and_matching_feedback(angle, active):
  drive = Drive(angle=angle)
  drive.ready()
  messages = drive.output(active=active)
  addresses = {(message[0], message[2]) for message in messages}
  assert (0xEA, BUSES.CAM) in addresses
  command = "LFA_ALT" if angle else "LFA"
  assert ((0xCB if angle else 0x12A), BUSES.ECAN) in addresses
  mdps = decode(messages, "MDPS", BUSES.CAM)
  output = decode(messages, command, BUSES.ECAN)
  if angle:
    assert mdps["LFA2_ACTIVE"] == 2
    assert output["LKAS_ANGLE_ACTIVE"] == (2 if active else 1)
    assert output["LKAS_ANGLE_CMD"] == pytest.approx(-5.2)
    assert output["LKAS_ANGLE_MAX_TORQUE"] == (60 if active else 0)
  else:
    assert mdps["LKA_ACTIVE"] == 1
    assert output["LKA_MODE"] == 2
    assert output["STEER_REQ"] == int(active)
    assert output["TORQUE_REQUEST"] == (40 if active else 0)


def test_angle_control_does_not_admit_a_torque_only_camera_sample():
  drive = Drive(angle=True)
  drive.cam.vl["LFA"]
  drive.receive("MDPS")
  drive.receive("LFA")
  drive.admit()
  assert not drive.state.camera_scc_steering_ready
  assert drive.output(active=True) == []


def test_angle_control_preserves_optional_legacy_lfa_and_emergency_commands():
  drive = Drive(angle=True)
  drive.cam.vl["LFA"]
  drive.receive("LFA")
  drive.ready()
  drive.state.adrv_0x161 = {"ALERTS_1": 11}
  messages = drive.output(active=True)
  angle = decode(messages, "LFA_ALT", BUSES.ECAN)
  torque = decode(messages, "LFA", BUSES.ECAN)
  assert angle["LKAS_ANGLE_ACTIVE"] == 2
  assert angle["LKAS_ANGLE_CMD"] == pytest.approx(6.4)
  assert angle["LKAS_ANGLE_MAX_TORQUE"] == 51
  assert torque["LKA_MODE"] == 1
  assert torque["STEER_REQ"] == 1
  assert torque["TORQUE_REQUEST"] == 25


def test_touch_forwarding_starts_with_the_pair_and_keeps_its_existing_checksum():
  drive = Drive()
  drive.pt.vl["STEER_TOUCH_2AF"]
  drive.receive("STEER_TOUCH_2AF", TOUCH_DETECT=1, TOUCH1=10, TOUCH2=20)
  drive.state.steer_touch_2af = drive.pt.vl["STEER_TOUCH_2AF"]
  assert drive.output(frame=20) == []
  drive.ready()
  messages = drive.output(frame=20)
  touch = next(message[1] for message in messages if message[0] == 0x2AF)
  assert touch[0] == hyundaicanfd.hyundai_crc8(touch[1:8])
  parser = CANParser(DBC, [("STEER_TOUCH_2AF", 10)], BUSES.CAM)
  parser.update([(START, messages)])
  assert parser.vl["STEER_TOUCH_2AF"]["TOUCH_DETECT"] == 3
  assert parser.vl["STEER_TOUCH_2AF"]["TOUCH1"] == 50
  assert parser.vl["STEER_TOUCH_2AF"]["TOUCH2"] == 50


def test_source_gap_after_admission_does_not_silently_hand_back_steering():
  drive = Drive()
  drive.ready()
  drive.pt.update([(START + 1_000_000_000, [])])
  drive.cam.update([(START + 1_000_000_000, [])])
  drive.admit()
  assert drive.state.camera_scc_steering_ready
  messages = drive.output(active=True)
  assert decode(messages, "LFA", BUSES.ECAN)["STEER_REQ"] == 1
  assert decode(messages, "MDPS", BUSES.CAM)["LKA_ACTIVE"] == 1


def test_existing_hda_warning_does_not_deadlock_the_coordinated_feedback_start():
  drive = Drive()
  drive.state.lfahda_cluster = {"HDA_OptUsmSta": 7, "HDA_InfoPUDis": 4}
  drive.state.ccnc_0x162 = {"FAULT_HDA": 1}
  drive.ready()
  messages = drive.output(active=False)
  assert decode(messages, "LFA", BUSES.ECAN)["STEER_REQ"] == 0
  assert decode(messages, "MDPS", BUSES.CAM)["LKA_ACTIVE"] == 1
  assert drive.state.lfahda_cluster == {"HDA_OptUsmSta": 7, "HDA_InfoPUDis": 4}
  assert drive.state.ccnc_0x162["FAULT_HDA"] == 1


def test_fresh_pair_is_bound_before_the_old_ready_count_121(monkeypatch):
  drive = Drive(registered=False)
  drive.state.controls_ready_count = 49
  monkeypatch.setattr(carstate, "Params", lambda: SimpleNamespace(get_bool=lambda _key: True))
  drive.receive("MDPS")
  drive.receive("LFA")
  drive.state.monitor_fingerprint({Bus.pt: drive.pt, Bus.cam: drive.cam}, canfd=True)
  assert drive.state.controls_ready_count == 50
  drive.admit()
  assert drive.output() == []
  drive.receive("MDPS", timestamp=START + 10_000_000)
  drive.receive("LFA", timestamp=START + 10_000_000)
  drive.admit()
  assert drive.state.controls_ready_count < 121
  assert drive.state.camera_scc_steering_ready
  assert decode(drive.output(), "MDPS", BUSES.CAM)["LKA_ACTIVE"] == 1


def test_late_camera_arrival_after_discovery_still_completes_the_pair():
  drive = Drive(registered=False)
  drive.state.controls_ready_count = 201
  drive.receive("MDPS")
  drive.admit()
  assert not drive.state.camera_scc_steering_ready
  assert drive.state.mdps is None
  drive.receive("MDPS", timestamp=START + 10_000_000)
  drive.receive("LFA", timestamp=START + 10_000_000)
  drive.admit()
  assert not drive.state.camera_scc_steering_ready
  drive.receive("MDPS", timestamp=START + 20_000_000)
  drive.receive("LFA", timestamp=START + 20_000_000)
  drive.admit()
  assert drive.state.camera_scc_steering_ready


@pytest.mark.parametrize("active", (False, True))
def test_non_camera_scc_command_generation_keeps_its_existing_behavior(active):
  drive = Drive(camera=False)
  drive.ready()
  assert not drive.state.camera_scc_steering_ready
  messages = hyundaicanfd.create_steering_messages(
    drive.output_packer, drive.params, BUSES, active, active, 40 if active else 0, 0, 0, False,
  )
  # On HDA2 without OP longitudinal control, this is the legacy LKAS path.
  parser = CANParser(DBC, [("LKAS", 100)], BUSES.ACAN)
  parser.update([(START, messages)])
  assert parser.vl["LKAS"]["STEER_REQ"] == int(active)
  assert parser.vl["LKAS"]["TORQUE_REQUEST"] == (40 if active else 0)
