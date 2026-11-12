"""Production messages exercise the imported motion controller and its gate."""

from types import SimpleNamespace as NS

import pytest

from cereal import car, log
from opendbc.car.hyundai.values import HyundaiExtFlags, HyundaiFlags
from openpilot.selfdrive.carrot.radar import radard_dpath as runtime
from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarOutput
from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import model_with_lead


class FakeSm(dict):
  def __init__(self, model, *, time_s=10.0, v_ego=10.0, device_yaw=0.0):
    model.timestampEof = int(time_s * 1e9)
    model.velocity = NS(x=(v_ego,))
    pose = log.LivePose.new_message(inputsOK=True, sensorsOK=True)
    pose.angularVelocityDevice.valid = True
    pose.angularVelocityDevice.z = device_yaw
    super().__init__(modelV2=model, carState=car.CarState.new_message(vEgo=v_ego), livePose=pose)
    self.logMonoTime = {name: int(time_s * 1e9) for name in ("modelV2", "carState", "liveTracks", "livePose")}
    self.checks = True

  def all_checks(self):
    return self.checks


def car_params(source="corner235"):
  cp = car.CarParams.new_message(brand="hyundai", flags=int(HyundaiFlags.CANFD), radarDelay=0.0)
  cp.extFlags = int(HyundaiExtFlags.CORNER_RADAR_OBJECTS_235 if source == "corner235"
                    else HyundaiExtFlags.CORNER_RADAR_OBJECTS_180)
  return cp


def radar_point(track_id, distance, lateral, *, source="frontRadar", speed=-2.0, lateral_speed=0.0, state=2, v_ego=10.0):
  return dict(trackId=track_id, radarSource=source, dRel=distance, yRel=lateral,
              vRel=speed, vLead=v_ego + speed, yvRel=lateral_speed, measured=True, trackState=state)


@pytest.fixture
def params(monkeypatch):
  settings = {"EnableCornerRadar": 1, "EnableRadarTracks": 1}

  class Params:
    def get_int(self, key):
      return settings.get(key, 0)

  monkeypatch.setattr(runtime, "Params", Params)
  return settings


@pytest.mark.parametrize("source", ("corner180", "corner235"))
@pytest.mark.parametrize("mode", (0, 1, 2))
def test_corner_pipeline_requires_complete_vehicle_feature_gate(source, mode):
  cp = car_params(source)
  assert runtime.corner_radar_enabled(cp, mode) == (mode > 0)
  cp.extFlags = 0
  assert not runtime.corner_radar_enabled(cp, mode)
  cp = car_params(source)
  cp.flags = 0
  assert not runtime.corner_radar_enabled(cp, mode)
  cp = car_params(source)
  cp.brand = "toyota"
  assert not runtime.corner_radar_enabled(cp, mode)


def test_main_selects_one_pipeline_and_keeps_legacy_subscriptions_when_disabled(monkeypatch):
  from openpilot.selfdrive.controls import radard

  class EndLoop(Exception):
    pass

  for enabled in (0, 1, 2):
    cp = car_params()
    selected, subscriptions = [], []

    class Params:
      def get(self, *args, **kwargs):
        return b"car-params"

      def get_int(self, key):
        return enabled

    class NewRadar:
      def __init__(self, actual_cp):
        assert actual_cp is cp
        selected.append("trajectory")

    class LegacyRadar:
      def __init__(self, delay):
        selected.append("legacy")

    class Sm:
      def __init__(self, services, **kwargs):
        subscriptions.extend(services)
        assert "ignore_alive" not in kwargs and "ignore_valid" not in kwargs

      def update(self):
        raise EndLoop

    monkeypatch.setattr(radard, "Params", Params)
    monkeypatch.setattr(radard, "RadarD", LegacyRadar)
    monkeypatch.setattr(runtime, "DPathRadarD", NewRadar)
    monkeypatch.setattr(radard, "config_realtime_process", lambda *args: None)
    monkeypatch.setattr(radard, "messaging", NS(
      log_from_bytes=lambda *args: cp, SubMaster=Sm, PubMaster=lambda *args: None,
    ))
    with pytest.raises(EndLoop):
      radard.main()
    assert selected == (["trajectory"] if enabled else ["legacy"])
    assert ("livePose" in subscriptions) == bool(enabled)


@pytest.mark.parametrize("source", ("corner180", "corner235"))
def test_original_cut_in_reaches_low_speed_auxiliary_lead_and_source_message(params, source):
  radar = runtime.DPathRadarD(car_params(source))
  saw_risk = False
  for index in range(32):
    distance, lateral = 12.0 - 0.10 * index, -3.0 + 0.05 * index
    sm = FakeSm(model_with_lead(30.0, 0.1, 12.0), time_s=10.0 + index * 0.05)
    rr = car.RadarData.new_message(points=[
      radar_point(10, 30.0, 0.1, speed=2.0),
      radar_point(49, distance, lateral),
      radar_point(1005, distance - 0.1, lateral - 0.1, source=source, lateral_speed=1.0),
    ])
    radar.update(sm, rr)
    saw_risk |= radar.radar_state.leadCutInRisk.status
  result = radar.radar_state
  assert radar.radar_state_valid
  assert result.leadOne.radarTrackId == 10
  assert result.leadTwo.status and result.leadTwo.radarTrackId == 1005
  assert result.leadTwo.dRel < result.leadOne.dRel
  assert str(result.leadTwo.radarSource) == source
  assert saw_risk
  with log.RadarState.from_bytes(result.to_bytes()) as decoded:
    assert str(decoded.leadTwo.radarSource) == source


def test_native_front_state_supports_stationary_position_evidence_in_production(params):
  params["EnableRadarTracks"] = 2
  radar = runtime.DPathRadarD(car_params())
  for index in range(18):
    t = index * 0.05
    distance = 120.0 - 23.0 * t
    model = model_with_lead(distance - 1.0, 0.0, 19.0, probability=0.65)
    model.leadsV3[0].xStd = (12.0,)
    model.leadsV3[0].vStd = (5.0,)
    sm = FakeSm(model, time_s=10.0 + t, v_ego=24.0, device_yaw=-0.04)
    rr = car.RadarData.new_message(points=[radar_point(53, distance, 0.2, speed=-23.0, v_ego=24.0)])
    radar.update(sm, rr)
  assert radar.radar_state_valid
  assert radar.radar_state.leadOne.radarTrackId == 53
  assert radar.radar_state.leadOne.radar
  assert radar.radar_state.leadOne.dRel > 100.0
  assert radar.radar_state.leadOne.vLead == pytest.approx(1.0)


def test_model_exposure_timestamp_yaw_sign_and_cutout_fields_survive_wrapper(params):
  radar = runtime.DPathRadarD(car_params())
  sm = FakeSm(model_with_lead(25.0, 0.0, 8.0), device_yaw=0.12)
  sm.logMonoTime["modelV2"] += 80_000_000
  sm.logMonoTime["livePose"] += 80_000_000
  sm.logMonoTime["liveTracks"] -= 20_000_000
  rr = car.RadarData.new_message(points=[radar_point(50, 25.0, 0.0)])
  captured = {}

  class Controller:
    def update(self, **kwargs):
      captured.update(kwargs)
      lead = dict(status=True, radar=True, radarTrackId=50, dRel=25.0,
                  cutOutTime=1.25, cutOutConfidence=0.5)
      return DPathRadarOutput(lead, None, None, None, (), (), (), (), (), (), None)

  radar.controller = Controller()
  radar.update(sm, rr)
  assert captured["time_s"] == 10.0
  assert captured["radar_to_model_time_s"] == pytest.approx(0.02)
  assert captured["yaw_rate_rad_s"] == pytest.approx(-0.12)
  with log.RadarState.from_bytes(radar.radar_state.to_bytes()) as decoded:
    assert str(decoded.leadOne.radarSource) == "frontRadar"
    assert decoded.leadOne.cutOutTime == 1.25
    assert decoded.leadOne.cutOutConfidence == 0.5


@pytest.mark.parametrize("failure", (
  "required_feed", "pose_flags", "pose_angular", "pose_nan", "pose_missing", "pose_old", "pose_future",
  "radar_old", "radar_future", "ego_nan",
))
def test_invalid_inputs_clear_outputs_and_motion_history_then_recover(params, failure):
  radar = runtime.DPathRadarD(car_params())
  sm = FakeSm(model_with_lead(25.0, 0.0, 8.0))
  rr = car.RadarData.new_message(points=[radar_point(50, 25.0, 0.0)])
  radar.update(sm, rr)
  previous = radar.controller
  radar.radar_state.leadOne.status = True
  radar.radar_state.leadOne.cutOutConfidence = 1.0
  radar.radar_state.leadCutInRisk.status = True
  if failure == "required_feed":
    sm.checks = False
  elif failure == "pose_flags":
    sm["livePose"].sensorsOK = False
  elif failure == "pose_angular":
    sm["livePose"].angularVelocityDevice.valid = False
  elif failure == "pose_nan":
    sm["livePose"].angularVelocityDevice.z = float("nan")
  elif failure == "pose_missing":
    sm.logMonoTime["livePose"] = 0
  elif failure == "pose_old":
    sm.logMonoTime["livePose"] -= 201_000_000
  elif failure == "pose_future":
    sm.logMonoTime["livePose"] += 51_000_000
  elif failure == "radar_old":
    sm.logMonoTime["liveTracks"] -= 151_000_000
  elif failure == "radar_future":
    sm.logMonoTime["liveTracks"] += 151_000_000
  elif failure == "ego_nan":
    sm["carState"].vEgo = float("nan")
  radar.update(sm, rr)
  assert not radar.radar_state_valid
  assert radar.controller is not previous
  assert not radar.radar_state.leadOne.status
  assert radar.radar_state.leadOne.cutOutConfidence == 0.0
  assert not radar.radar_state.leadCutInRisk.status
  assert not radar.radar_state.leadTwo.status
  assert not radar.radar_state.leadsCutIn
  sm = FakeSm(model_with_lead(25.0, 0.0, 8.0), time_s=10.1)
  radar.update(sm, rr)
  assert radar.radar_state_valid
  assert radar.radar_state.leadOne.cutOutConfidence == 0.0
  assert not radar.radar_state.leadCutInRisk.status
