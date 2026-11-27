"""Exercise lead association and fail-closed state with native cereal messages."""

from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from cereal import car, log
from openpilot.selfdrive.controls.lib.human_like_following import HumanLikeFollowing, LeadPrediction


LEAD_TIMES = (0., 2., 4., 6., 8., 10.)
CAMERA_OFFSET = 1.52


def model_message(*, distance=40., velocity=20., acceleration=.2):
  model = log.ModelDataV2.new_message()
  model.init("leadsV3", 1)
  lead = model.leadsV3[0]
  lead.prob = .99
  lead.t = LEAD_TIMES
  lead.x = [distance + CAMERA_OFFSET + velocity * t + .5 * acceleration * t * t for t in LEAD_TIMES]
  lead.y = [-.25] * 6
  lead.v = [velocity + acceleration * t for t in LEAD_TIMES]
  lead.a = [acceleration] * 6
  lead.xStd = [.25] * 6
  lead.yStd = [.1] * 6
  lead.vStd = [.2] * 6
  lead.aStd = [.2] * 6
  return model


class Inputs(dict):
  """Only SubMaster's transport metadata is synthetic; payloads use cereal."""
  def __init__(self):
    radar = log.RadarState.new_message()
    radar.leadOne = dict(status=True, radar=True, radarSource="frontRadar", radarTrackId=11,
                         dRel=40., yRel=.25, vRel=0., vLead=20., vLeadK=20., aLeadK=0.,
                         aLead=0., aLeadTau=1.5, modelProb=.99)
    super().__init__(carState=car.CarState.new_message(vEgo=20.), radarState=radar, modelV2=model_message())
    self.alive = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)
    self.seen = dict.fromkeys(self, True)
    self.updated = dict.fromkeys(self, True)
    self.logMonoTime = {}
    self.recv_time = {}
    self.frame = 0
    self.set_time(10.)

  def all_checks(self, service_list=None):
    return all(self.alive[name] and self.valid[name] for name in (service_list or self))

  def set_time(self, time_s, *, model_time=None, radar_model_time=None):
    model_time = time_s if model_time is None else model_time
    self.frame += 1
    self.now = time_s
    self.logMonoTime.update(carState=int(round(time_s * 1e9)), radarState=int(round(time_s * 1e9)),
                            modelV2=int(round(model_time * 1e9)))
    self.recv_time.update(dict.fromkeys(self, time_s))
    self["modelV2"].timestampEof = self.logMonoTime["modelV2"]
    self["radarState"].mdMonoTime = int(round((model_time if radar_model_time is None else radar_model_time) * 1e9))


def observe(helper, sm, enabled=True, *, now=None):
  return helper.update(sm, enabled, now=sm.now if now is None else now)


def qualify(helper, sm, *, start=10.):
  outputs = []
  for index in range(3):
    sm.set_time(start + .06 * index)
    outputs.append(observe(helper, sm))
  assert outputs[:2] == [None, None]
  assert outputs[2] is not None
  return outputs[2]


def test_requires_three_distinct_frames_and_preserves_radar_anchor():
  sm = Inputs()
  prediction = qualify(HumanLikeFollowing(), sm)
  assert isinstance(prediction, LeadPrediction)
  assert prediction.radar_track_id == 11
  assert prediction.radar_d_rel == pytest.approx(40.)
  assert prediction.radar_v_lead == pytest.approx(20.)
  assert prediction.model_mono_time == sm["radarState"].mdMonoTime
  np.testing.assert_allclose(prediction.t, LEAD_TIMES)
  assert prediction.x[0] == pytest.approx(40.)
  assert prediction.v[0] == pytest.approx(20.)
  assert prediction.x[-1] > prediction.x[0] + 20. * LEAD_TIMES[-1]
  assert prediction.v[-1] > prediction.v[0]
  with pytest.raises(FrozenInstanceError):
    prediction.radar_track_id = 12


def test_repeated_frame_cannot_build_confirmation():
  helper, sm = HumanLikeFollowing(), Inputs()
  for _ in range(12):
    assert observe(helper, sm) is None
  sm.set_time(10.06)
  assert observe(helper, sm) is None
  sm.set_time(10.12)
  assert observe(helper, sm) is not None


def test_three_frames_also_require_a_tenth_second_of_evidence():
  helper, sm = HumanLikeFollowing(), Inputs()
  for time_s in (10., 10.02, 10.04):
    sm.set_time(time_s)
    assert observe(helper, sm) is None
  sm.set_time(10.12)
  assert observe(helper, sm) is not None


def test_track_switch_restarts_confirmation():
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  sm["radarState"].leadOne.radarTrackId = 12
  qualify(helper, sm, start=10.18)


def test_disable_discards_confirmation_and_requires_new_frames_after_enable():
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  sm.set_time(10.18)
  assert observe(helper, sm, False) is None
  qualify(helper, sm, start=10.24)


def test_prediction_does_not_mutate_native_inputs():
  helper, sm = HumanLikeFollowing(), Inputs()
  before = sm["radarState"].leadOne.to_dict(), sm["modelV2"].leadsV3[0].to_dict()
  qualify(helper, sm)
  assert (sm["radarState"].leadOne.to_dict(), sm["modelV2"].leadsV3[0].to_dict()) == before


def test_radar_uses_exact_cached_model_instead_of_latest_model():
  helper, sm = HumanLikeFollowing(), Inputs()
  assert observe(helper, sm) is None
  for time_s in (10.06, 10.12, 10.18):
    # The newest model is valid but has a distinguishable future acceleration.
    sm["modelV2"] = model_message(acceleration=.2 + (time_s - 10.) * 2.)
    sm.set_time(time_s, radar_model_time=time_s - .06)
    prediction = observe(helper, sm)
  assert prediction is not None
  assert prediction.model_mono_time == 10_120_000_000
  assert prediction.model_mono_time != sm.logMonoTime["modelV2"]
  # The 10.12 s frame predicts .44 m/s^2, while the latest predicts .56.
  assert prediction.v[-1] == pytest.approx(24.4, abs=1e-4)


def test_unseen_model_timestamp_cannot_be_substituted_with_nearby_frame():
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  sm.set_time(10.18, radar_model_time=10.15)
  assert observe(helper, sm) is None


@pytest.mark.parametrize("feed", ("modelV2", "radarState", "carState"))
@pytest.mark.parametrize("bad_metadata", ("alive", "valid"))
def test_unhealthy_services_clear_confirmation(feed, bad_metadata):
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  getattr(sm, bad_metadata)[feed] = False
  sm.set_time(10.18)
  assert observe(helper, sm) is None
  getattr(sm, bad_metadata)[feed] = True
  qualify(helper, sm, start=10.24)


@pytest.mark.parametrize("feed", ("modelV2", "radarState", "carState"))
@pytest.mark.parametrize("receipt_age", (.251, -.051))
def test_stale_or_future_transport_receipts_reject_even_with_matching_timestamps(feed, receipt_age):
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  sm.set_time(10.18)
  sm.recv_time[feed] = sm.now - receipt_age
  assert observe(helper, sm) is None


@pytest.mark.parametrize("stale_source", ("radar_publication", "model_exposure", "association"))
def test_stale_message_timestamps_reject(stale_source):
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  sm.set_time(10.18)
  if stale_source == "radar_publication":
    sm.logMonoTime["radarState"] -= 251_000_000
  elif stale_source == "model_exposure":
    sm["modelV2"].timestampEof -= 251_000_000
  else:
    sm["radarState"].mdMonoTime -= 251_000_000
  assert observe(helper, sm) is None


def test_clock_regression_and_long_gap_restart_confirmation():
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  sm.set_time(10.06)
  assert observe(helper, sm) is None
  qualify(helper, sm, start=10.3)
  sm.set_time(10.7)
  assert observe(helper, sm) is None
  sm.set_time(10.76)
  assert observe(helper, sm) is None
  sm.set_time(10.82)
  assert observe(helper, sm) is not None


@pytest.mark.parametrize("bad_primary", (
  "status", "vision_only", "corner180", "corner235", "missing_id", "low_probability",
  "braking", "stopped", "close", "cutout_time", "cutout_confidence",
))
def test_unsafe_or_unmatched_primary_rejects(bad_primary):
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  lead = sm["radarState"].leadOne
  if bad_primary == "status": lead.status = False
  elif bad_primary == "vision_only": lead.radar = False
  elif bad_primary in ("corner180", "corner235"): lead.radarSource = bad_primary
  elif bad_primary == "missing_id": lead.radarTrackId = -1
  elif bad_primary == "low_probability": lead.modelProb = .5
  elif bad_primary == "braking": lead.aLeadK = -2.
  elif bad_primary == "stopped": lead.vLead = lead.vLeadK = 0.
  elif bad_primary == "close": lead.dRel = 5.
  elif bad_primary == "cutout_time": lead.cutOutTime = 1.5
  elif bad_primary == "cutout_confidence": lead.cutOutConfidence = .8
  sm.set_time(10.18)
  assert observe(helper, sm) is None


@pytest.mark.parametrize("maneuver", ("brake", "gas", "standstill", "lane_change", "cutin_risk", "primary_cutin"))
def test_driver_and_maneuver_vetoes(maneuver):
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  if maneuver == "brake": sm["carState"].brakePressed = True
  elif maneuver == "gas": sm["carState"].gasPressed = True
  elif maneuver == "standstill": sm["carState"].standstill = True
  elif maneuver == "lane_change": sm["modelV2"].meta.laneChangeState = "laneChangeStarting"
  elif maneuver == "cutin_risk": sm["radarState"].leadCutInRisk.status = True
  elif maneuver == "primary_cutin":
    sm["radarState"].leadsCutIn = [dict(status=True, radarTrackId=11, radarSource="frontRadar")]
  sm.set_time(10.18)
  assert observe(helper, sm) is None


@pytest.mark.parametrize("field,bad_value", (
  ("x", 20.), ("y", 2.), ("v", 5.), ("prob", .5),
  ("xStd", 20.), ("yStd", 3.), ("vStd", 10.),
))
def test_model_match_requires_confident_position_and_velocity(field, bad_value):
  helper, sm = HumanLikeFollowing(), Inputs()
  lead = sm["modelV2"].leadsV3[0]
  if field == "prob":
    lead.prob = bad_value
  else:
    values = list(getattr(lead, field))
    values[0] = bad_value
    setattr(lead, field, values)
  for time_s in (10., 10.06, 10.12, 10.18):
    sm.set_time(time_s)
    assert observe(helper, sm) is None


@pytest.mark.parametrize("field", ("t", "x", "y", "v", "xStd", "yStd", "vStd"))
@pytest.mark.parametrize("bad_value", (float("nan"), float("inf"), float("-inf")))
def test_nonfinite_trajectory_or_uncertainty_is_rejected(field, bad_value):
  helper, sm = HumanLikeFollowing(), Inputs()
  values = list(getattr(sm["modelV2"].leadsV3[0], field))
  values[-1] = bad_value
  setattr(sm["modelV2"].leadsV3[0], field, values)
  for time_s in (10., 10.06, 10.12):
    sm.set_time(time_s)
    assert observe(helper, sm) is None


@pytest.mark.parametrize("field", ("dRel", "yRel", "vLead", "vRel", "aLeadK", "jLead", "modelProb", "cutOutTime"))
@pytest.mark.parametrize("bad_value", (float("nan"), float("inf")))
def test_nonfinite_radar_is_rejected(field, bad_value):
  helper, sm = HumanLikeFollowing(), Inputs()
  setattr(sm["radarState"].leadOne, field, bad_value)
  for time_s in (10., 10.06, 10.12):
    sm.set_time(time_s)
    assert observe(helper, sm) is None


@pytest.mark.parametrize("field", ("t", "x", "y", "v", "xStd", "yStd", "vStd"))
@pytest.mark.parametrize("size", (0, 5, 7))
def test_malformed_trajectory_lengths_reject(field, size):
  helper, sm = HumanLikeFollowing(), Inputs()
  values = list(getattr(sm["modelV2"].leadsV3[0], field))
  setattr(sm["modelV2"].leadsV3[0], field, (values + [values[-1]])[:size])
  for time_s in (10., 10.06, 10.12):
    sm.set_time(time_s)
    assert observe(helper, sm) is None


@pytest.mark.parametrize("times", ((1., 2., 4., 6., 8., 10.), (0., 2., 2., 6., 8., 10.), (0., 4., 2., 6., 8., 10.)))
def test_nonmonotonic_or_nonzero_origin_time_rejects(times):
  helper, sm = HumanLikeFollowing(), Inputs()
  sm["modelV2"].leadsV3[0].t = times
  assert observe(helper, sm) is None


def test_small_model_offsets_are_removed_by_radar_anchoring():
  helper, sm = HumanLikeFollowing(), Inputs()
  sm["modelV2"] = model_message(distance=42.9, velocity=21.)
  prediction = qualify(helper, sm)
  assert prediction.x[0] == pytest.approx(40.)
  assert prediction.v[0] == pytest.approx(20.)
  assert prediction.v[-1] == pytest.approx(22.)
  assert prediction.x[-1] <= 250.001


def test_scaled_position_cannot_grant_credit_without_velocity_support():
  helper, sm = HumanLikeFollowing(), Inputs()
  lead = sm["modelV2"].leadsV3[0]
  lead.x = [lead.x[0] + (x - lead.x[0]) * 2. for x in lead.x]
  for time_s in (10., 10.06, 10.12):
    sm.set_time(time_s)
    assert observe(helper, sm) is None


def test_independent_lead_two_is_unchanged_and_does_not_supply_primary_identity():
  helper, sm = HumanLikeFollowing(), Inputs()
  sm["radarState"].leadTwo = dict(status=True, radar=True, radarSource="corner235", radarTrackId=1205,
                                  dRel=25., vLead=16., aLeadK=-1.)
  lead_two_before = sm["radarState"].leadTwo.to_dict()
  assert qualify(helper, sm).radar_track_id == 11
  assert sm["radarState"].leadTwo.to_dict() == lead_two_before
  sm["radarState"].leadOne.status = False
  sm.set_time(10.18)
  assert observe(helper, sm) is None
  assert sm["radarState"].leadTwo.to_dict() == lead_two_before


@pytest.mark.parametrize("missing", ("all", "modelV2", "radarState", "carState", "all_checks", "logMonoTime", "recv_time"))
def test_missing_submaster_inputs_fail_closed(missing):
  helper, sm = HumanLikeFollowing(), Inputs()
  qualify(helper, sm)
  if missing == "all":
    malformed = None
  elif missing in ("modelV2", "radarState", "carState"):
    del sm[missing]
    malformed = sm
  elif missing == "all_checks":
    malformed = dict(sm)
  else:
    delattr(sm, missing)
    malformed = sm
  assert helper.update(malformed, True, now=10.18) is None
