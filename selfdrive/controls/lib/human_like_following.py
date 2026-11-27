"""Associate a validated model future with the measured front-radar primary.

This adapts radar-anchored trajectory following to this branch's independent
leadTwo selection. Model compatibility is checked from the published motion,
without assuming a particular driving-model version. Rejected inputs retain
the existing radar prediction in the MPC.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
import time
from typing import Any


RADAR_TO_CAMERA_M = 1.52
MAX_AGE_NS = 250_000_000
MAX_FUTURE_NS = 50_000_000
MAX_CACHE_FRAMES = 8
MIN_STABLE_NS = 100_000_000
MAX_FRAME_GAP_NS = 150_000_000
MIN_PROBABILITY = 0.90
TRAJECTORY_POINTS = 6


@dataclass(frozen=True)
class LeadPrediction:
  t: tuple[float, ...]
  x: tuple[float, ...]
  v: tuple[float, ...]
  radar_track_id: int
  radar_d_rel: float
  radar_v_lead: float
  model_mono_time: int


@dataclass(frozen=True)
class _ModelLead:
  t: tuple[float, ...]
  x: tuple[float, ...]
  y: tuple[float, ...]
  v: tuple[float, ...]
  probability: float
  exposure_time: int


@dataclass(frozen=True)
class _Observation:
  track_id: int
  radar_time: int
  model_time: int
  d_rel: float
  y_rel: float
  v_lead: float
  v_rel: float
  v_lat: float


def _name(value: Any) -> str:
  return str(value).rsplit(".", 1)[-1]


def _reader(message: Any) -> Any:
  as_reader = getattr(message, "as_reader", None)
  return as_reader() if callable(as_reader) else message


def _finite_values(values: Any) -> tuple[float, ...]:
  if len(values) != TRAJECTORY_POINTS:
    raise ValueError("invalid trajectory length")
  result = tuple(float(value) for value in values)
  if not all(math.isfinite(value) for value in result):
    raise ValueError("invalid trajectory")
  return result


def _snapshot(model: Any, publication_time: int) -> _ModelLead | None:
  try:
    exposure_time = int(model.timestampEof)
    if exposure_time <= 0 or not -MAX_FUTURE_NS <= publication_time - exposure_time <= MAX_AGE_NS:
      return None
    # These are hypotheses for the lead at different future times, not IDs
    # of separate physical vehicles. Only hypothesis zero identifies L1 here.
    leads = model.leadsV3
    if not leads:
      return None
    lead = leads[0]
    t = _finite_values(lead.t)
    x = _finite_values(lead.x)
    y = _finite_values(lead.y)
    v = _finite_values(lead.v)
    x_std = _finite_values(lead.xStd)
    y_std = _finite_values(lead.yStd)
    v_std = _finite_values(lead.vStd)
    probability = float(lead.prob)
    probability_time = float(lead.probTime)
    if not math.isfinite(probability) or not MIN_PROBABILITY <= probability <= 1.0:
      return None
    if not math.isfinite(probability_time) or abs(probability_time) > 1e-6:
      return None
    for uncertainty, current_limit, future_limit in ((x_std, 3.0, 15.0), (y_std, 0.75, 3.0), (v_std, 2.0, 5.0)):
      if uncertainty[0] > current_limit or any(value < 0.0 or value > future_limit for value in uncertainty):
        return None
    if abs(t[0]) > 1e-6 or t[-1] < 10.0 or any(b <= a for a, b in zip(t, t[1:])):
      return None
    if any(speed < 0.0 or speed > 60.0 for speed in v):
      return None
    for index in range(1, TRAJECTORY_POINTS):
      displacement = x[index] - x[index - 1]
      speed_integral = 0.5 * (v[index - 1] + v[index]) * (t[index] - t[index - 1])
      if displacement < 0.0 or abs(displacement - speed_integral) > max(2.0, 0.15 * speed_integral):
        return None
    return _ModelLead(t, x, y, v, probability, exposure_time)
  except (AttributeError, TypeError, ValueError, OverflowError):
    return None


class HumanLikeFollowing:
  def __init__(self) -> None:
    self._models: OrderedDict[int, _ModelLead] = OrderedDict()
    self._latest_model_time = 0
    self._previous: _Observation | None = None
    self._stable_frames = 0
    self._stable_since = 0
    self.reason = "disabled"

  def _reject(self, reason: str, *, clear_cache: bool = False) -> None:
    self.reason = reason
    self._previous = None
    self._stable_frames = 0
    self._stable_since = 0
    if clear_cache:
      self._models.clear()
      self._latest_model_time = 0

  def update(self, sm: Any, enabled: bool, *, now: float | None = None) -> LeadPrediction | None:
    """Return a radar-anchored L1 future, or None to preserve radar behavior.

    The caller also gates engagement/reset state. RadarState has no per-lead
    measured flag; frontRadar identity and consecutive physical observations
    use the branch's front-radar publication contract. No radar schema or
    lead-selection policy is changed here.
    """
    try:
      return self._update(sm, enabled, now)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      self._reject("invalid_input", clear_cache=True)
      return None

  def _update(self, sm: Any, enabled: bool, now: float | None) -> LeadPrediction | None:
    if not enabled:
      self._reject("disabled", clear_cache=True)
      return None
    if not sm.all_checks(service_list=["radarState", "modelV2", "carState"]):
      self._reject("invalid_services", clear_cache=True)
      return None
    current_time = time.monotonic() if now is None else float(now)
    receipt_ages = tuple(current_time - float(sm.recv_time[service]) for service in ("radarState", "modelV2", "carState"))
    if not math.isfinite(current_time) or any(not math.isfinite(age) or not 0.0 <= age <= 0.25 for age in receipt_ages):
      self._reject("stale_services", clear_cache=True)
      return None

    try:
      model_time = int(sm.logMonoTime["modelV2"])
      radar_time = int(sm.logMonoTime["radarState"])
      radar = _reader(sm["radarState"])
      associated_time = int(radar.mdMonoTime)
      if model_time <= 0 or radar_time <= 0 or associated_time <= 0:
        raise ValueError("missing timestamps")
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      self._reject("invalid_timestamps", clear_cache=True)
      return None

    if model_time < self._latest_model_time:
      self._reject("time_regression", clear_cache=True)
      return None
    self._latest_model_time = model_time
    model = _reader(sm["modelV2"])
    captured = _snapshot(model, model_time)
    if captured is None:
      self._reject("invalid_model")
      return None
    self._models[model_time] = captured
    for timestamp in tuple(self._models):
      if not 0 <= model_time - timestamp <= MAX_AGE_NS:
        del self._models[timestamp]
    while len(self._models) > MAX_CACHE_FRAMES:
      self._models.popitem(last=False)

    if (not 0 <= model_time - associated_time <= MAX_AGE_NS
        or not -MAX_FUTURE_NS <= model_time - radar_time <= MAX_AGE_NS
        or not -MAX_FUTURE_NS <= radar_time - associated_time <= MAX_AGE_NS):
      self._reject("stale_radar")
      return None
    matched_model = self._models.get(associated_time)
    if matched_model is None:
      self._reject("missing_model_frame")
      return None
    if not -MAX_FUTURE_NS <= model_time - matched_model.exposure_time <= MAX_AGE_NS:
      self._reject("stale_model_exposure")
      return None

    cs = _reader(sm["carState"])
    if (cs.gasPressed or cs.brakePressed or cs.standstill
        or _name(model.meta.laneChangeState) != "off"):
      self._reject("driver_or_lane_change")
      return None
    lead = radar.leadOne
    try:
      track_id = int(lead.radarTrackId)
      values = tuple(float(value) for value in (
        lead.dRel, lead.yRel, lead.dPath, lead.vLead, lead.vRel,
        lead.vLat, lead.aLeadK, lead.jLead, lead.modelProb, cs.vEgo,
        lead.cutOutTime, lead.cutOutConfidence,
      ))
      if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite radar")
    except (AttributeError, TypeError, ValueError, OverflowError):
      self._reject("invalid_radar")
      return None
    d_rel, y_rel, d_path, v_lead, v_rel, v_lat, a_lead, j_lead, probability, v_ego, exit_time, exit_confidence = values
    if (not lead.status or not lead.radar or _name(lead.radarSource) != "frontRadar"
        or track_id < 0 or not MIN_PROBABILITY <= probability <= 1.0):
      self._reject("unmatched_primary")
      return None
    if not (10.0 <= d_rel <= 150.0 and 5.0 < v_lead <= 60.0 and 5.0 < v_ego <= 60.0):
      self._reject("stopped_or_close_primary")
      return None
    if a_lead < -0.5 or a_lead > 3.0 or abs(j_lead) > 0.5:
      self._reject("radar_braking_or_jerk")
      return None
    if (exit_time != 0.0 or exit_confidence != 0.0
        or bool(getattr(getattr(radar, "leadCutInRisk", None), "status", False))
        or any(candidate.status and int(candidate.radarTrackId) == track_id
               and _name(candidate.radarSource) == "frontRadar" for candidate in radar.leadsCutIn)):
      self._reject("active_maneuver")
      return None
    if (abs(d_rel - (matched_model.x[0] - RADAR_TO_CAMERA_M)) > 3.0
        or abs(y_rel + matched_model.y[0]) > 0.75 or abs(d_path) > 0.75
        or abs(v_lead - matched_model.v[0]) > 2.0):
      self._reject("position_or_speed_mismatch")
      return None

    observation = _Observation(track_id, radar_time, associated_time, d_rel, y_rel, v_lead, v_rel, v_lat)
    previous = self._previous
    if previous is not None and radar_time == previous.radar_time and associated_time != previous.model_time:
      self._reject("reused_radar_timestamp")
      return None
    if previous is None or radar_time != previous.radar_time:
      delta_ns = associated_time - previous.model_time if previous is not None else 0
      dt = delta_ns * 1e-9
      continuous = (
        previous is not None and track_id == previous.track_id
        and radar_time > previous.radar_time and 0 < delta_ns <= MAX_FRAME_GAP_NS
        and abs(d_rel - (previous.d_rel + previous.v_rel * dt)) <= 1.5
        and abs(y_rel - (previous.y_rel + previous.v_lat * dt)) <= 0.5
        and abs(v_lead - previous.v_lead) <= 1.5
      )
      self._stable_frames = self._stable_frames + 1 if continuous else 1
      if not continuous:
        self._stable_since = associated_time
      self._previous = observation
    if self._stable_frames < 3 or associated_time - self._stable_since < MIN_STABLE_NS:
      self.reason = "confirming_primary"
      return None

    speeds = tuple(v_lead + speed - matched_model.v[0] for speed in matched_model.v)
    if any(speed < 0.0 or speed > 60.0 for speed in speeds):
      self._reject("invalid_anchored_speed")
      return None
    positions = [d_rel]
    speed_integral = d_rel
    for index in range(1, TRAJECTORY_POINTS):
      speed_integral += 0.5 * (speeds[index - 1] + speeds[index]) * (matched_model.t[index] - matched_model.t[index - 1])
      positions.append(min(d_rel + matched_model.x[index] - matched_model.x[0], speed_integral))
    if any(b < a for a, b in zip(positions, positions[1:])):
      self._reject("invalid_anchored_position")
      return None
    self.reason = "active"
    return LeadPrediction(matched_model.t, tuple(positions), speeds, track_id, d_rel, v_lead, associated_time)
