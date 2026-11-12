#!/usr/bin/env python3
"""Independent RadarD using vision primary matching and physical dPath leadTwo."""

from __future__ import annotations

import math
from typing import Any

from cereal import car, log, messaging
from openpilot.common.params import Params
from opendbc.car.hyundai.values import HyundaiExtFlags, HyundaiFlags
from openpilot.selfdrive.carrot.radar import effective_radar_track_mode
from openpilot.selfdrive.carrot.radar_motion.coordinates import device_yaw_to_radar
from openpilot.selfdrive.carrot.radar_motion.timing import front_radar_distance_delay_s
from openpilot.selfdrive.carrot.radar_motion import (
  DPathRadarController,
  RADAR_MOTION_MAX_TIME_SKEW_S,
)


CORNER_RADAR_FLAGS = int(
  HyundaiExtFlags.CORNER_RADAR_OBJECTS_235
  | HyundaiExtFlags.CORNER_RADAR_OBJECTS_180
)
PRODUCTION_CUT_IN_SENSITIVITY = 3
MAX_POSE_AGE_S = 0.20
MAX_POSE_FUTURE_S = 0.05
EMPTY_LEAD = {
  "dRel": 0.0,
  "yRel": 0.0,
  "vRel": 0.0,
  "aRel": 0.0,
  "vLead": 0.0,
  "aLead": 0.0,
  "dPath": 0.0,
  "vLat": 0.0,
  "vLeadK": 0.0,
  "aLeadK": 0.0,
  "fcw": False,
  "status": False,
  "aLeadTau": 0.0,
  "modelProb": 0.0,
  "radar": False,
  "radarTrackId": -1,
  "jLead": 0.0,
  "score": 0.0,
  "cutOutTime": 0.0,
  "cutOutConfidence": 0.0,
}


def empty_lead() -> dict[str, Any]:
  return EMPTY_LEAD.copy()


def corner_radar_enabled(CP: car.CarParams, enable_corner_radar: int) -> bool:
  return (
    CP.brand == "hyundai"
    and bool(int(CP.flags) & int(HyundaiFlags.CANFD))
    and enable_corner_radar > 0
    and bool(int(CP.extFlags) & CORNER_RADAR_FLAGS)
  )


def _yaw_rate(live_pose: Any) -> float | None:
  angular_velocity = getattr(live_pose, "angularVelocityDevice", None)
  if (
    angular_velocity is not None
    and bool(getattr(angular_velocity, "valid", False))
    and bool(getattr(live_pose, "inputsOK", False))
    and bool(getattr(live_pose, "sensorsOK", False))
  ):
    value = float(getattr(angular_velocity, "z", 0.0))
    if math.isfinite(value):
      return device_yaw_to_radar(value)
  return None


def _model_measurement_time_s(sm: messaging.SubMaster) -> float:
  """Use the camera exposure time, not model publication time."""
  timestamp_eof_ns = int(getattr(sm["modelV2"], "timestampEof", 0))
  model_log_ns = int(sm.logMonoTime["modelV2"])
  return float(timestamp_eof_ns if timestamp_eof_ns > 0 else model_log_ns) * 1e-9


class DPathRadarD:
  """Own and publish radarState without importing controls.radard."""

  def __init__(self, CP: car.CarParams) -> None:
    params = Params()
    enable_radar_tracks = effective_radar_track_mode(
      CP.brand,
      CP.radarUnavailable,
      params.get_int("EnableRadarTracks"),
    )
    self._controller_config = dict(
      prefer_corner_radar=corner_radar_enabled(
        CP,
        params.get_int("EnableCornerRadar"),
      ),
      enable_radar_tracks=enable_radar_tracks,
      cut_in_sensitivity=PRODUCTION_CUT_IN_SENSITIVITY,
      front_radar_measurement_delay_s=front_radar_distance_delay_s(CP),
      production_live_tracks=True,
    )
    self.controller = DPathRadarController(**self._controller_config)
    self._inputs_ready = True
    self.radar_state = log.RadarState.new_message()
    self.radar_state_valid = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData) -> None:
    self.radar_state_valid = sm.all_checks()
    # Cap'n Proto arenas cannot reclaim replaced lists/structs. Keeping this
    # builder across frames retains old allocations for the whole drive, even
    # though publish() copies only the currently reachable data.
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime["modelV2"]
    self.radar_state.carStateMonoTime = sm.logMonoTime["carState"]
    self.radar_state.radarErrors = rr.errors

    model_time_s = _model_measurement_time_s(sm)
    radar_time_s = float(sm.logMonoTime["liveTracks"]) * 1e-9
    model_publication_s = float(sm.logMonoTime["modelV2"]) * 1e-9
    pose_time_s = float(sm.logMonoTime["livePose"]) * 1e-9
    yaw_rate = _yaw_rate(sm["livePose"])
    v_ego = float(sm["carState"].vEgo)
    # Unknown yaw must not look like a straight road. Discard the old motion
    # history on feed loss, so a reconnect cannot complete a stale maneuver.
    self.radar_state_valid = bool(
      self.radar_state_valid
      and yaw_rate is not None
      and math.isfinite(v_ego)
      and model_time_s > 0.0
      and radar_time_s > 0.0
      and pose_time_s > 0.0
      and -MAX_POSE_FUTURE_S <= model_publication_s - pose_time_s <= MAX_POSE_AGE_S
      and abs(model_time_s - radar_time_s) <= RADAR_MOTION_MAX_TIME_SKEW_S
    )
    if not self.radar_state_valid:
      if self._inputs_ready:
        self.controller = DPathRadarController(**self._controller_config)
      self._inputs_ready = False
      return
    self._inputs_ready = True
    time_s = (
      model_time_s
      if model_time_s > 0.0
      else 1e-9 * max(sm.logMonoTime.values())
    )
    output = self.controller.update(
      time_s=time_s,
      v_ego=v_ego,
      radar_points=rr.points,
      model=sm["modelV2"],
      yaw_rate_rad_s=yaw_rate,
      radar_to_model_time_s=model_time_s - radar_time_s,
    )
    sources = {int(point.trackId): str(point.radarSource) for point in rr.points}

    def with_source(lead):
      value = dict(lead) if lead else empty_lead()
      if value.get("radar"):
        value.setdefault("radarSource", sources.get(int(value.get("radarTrackId", -1)), "frontRadar"))
      return value

    self.radar_state.leadOne = with_source(output.lead_one)
    self.radar_state.leadTwo = with_source(output.lead_two)
    self.radar_state.leadLeft = with_source(output.lead_left)
    self.radar_state.leadRight = with_source(output.lead_right)
    self.radar_state.leadsLeft = [with_source(lead) for lead in output.leads_left]
    self.radar_state.leadsCenter = [with_source(lead) for lead in output.leads_center]
    self.radar_state.leadsRight = [with_source(lead) for lead in output.leads_right]
    self.radar_state.leadsCutIn = [with_source(lead) for lead in output.leads_cutin]
    self.radar_state.leadCutInRisk = with_source(output.lead_cutin_risk)
    self.radar_state.leadsLeft2 = [with_source(lead) for lead in output.leads_left2]
    self.radar_state.leadsRight2 = [with_source(lead) for lead in output.leads_right2]

  def publish(self, pm: messaging.PubMaster) -> None:
    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)
