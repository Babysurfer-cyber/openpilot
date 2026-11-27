"""Exercise destination lane capture with production cereal messages and clocks."""

from cereal import car, log
import pytest

from openpilot.selfdrive.controls.lib.human_like_lane_changes import HumanLikeLaneChanges


NS_PER_SECOND = 1_000_000_000


def lead(track_id=41, distance=35.0, lateral=3.6, *, source="frontRadar", speed=12.0, **changes):
  fields = dict(status=True, radar=True, radarTrackId=track_id, radarSource=source,
                dRel=distance, yRel=lateral, vRel=speed - 15.0, vLead=speed,
                vLeadK=speed, aLead=0.0, aLeadK=0.0, aLeadTau=1.5, jLead=0.0,
                vLat=0.0, modelProb=0.99)
  fields.update(changes)
  return log.RadarState.LeadData.new_message(**fields)


def model(time_s, *, state="preLaneChange", direction="left", desire="none", centers=None):
  result = log.ModelDataV2.new_message(timestampEof=round(time_s * NS_PER_SECOND))
  result.meta.laneChangeState = state
  result.meta.laneChangeDirection = direction
  result.meta.desire = desire
  result.position.x = [0.0, 20.0, 40.0, 80.0, 140.0]
  result.laneLineProbs = [0.99] * 4
  result.laneLines = [dict(x=[0.0, 20.0, 40.0, 80.0, 140.0], y=[value] * 5)
                      for value in (centers or [-5.4, -1.8, 1.8, 5.4])]
  return result


class Inputs(dict):
  """A SubMaster shaped object containing native messages and receipt metadata."""

  def __init__(self, time_s=10.0, *, state="preLaneChange", direction="left", desire="none",
               candidates=None, primary=None, secondary=None, centers=None):
    md = model(time_s, state=state, direction=direction, desire=desire, centers=centers)
    radar = log.RadarState.new_message(mdMonoTime=round(time_s * NS_PER_SECOND))
    radar.leadsLeft = [lead()] if candidates is None else candidates
    radar.leadOne = lead(10, 70.0, 0.0, speed=15.0) if primary is None else primary
    if secondary is not None:
      radar.leadTwo = secondary
    cs = car.CarState.new_message(canValid=True, vEgo=15.0)
    super().__init__(modelV2=md, radarState=radar, carState=cs)
    self.logMonoTime = {name: round(time_s * NS_PER_SECOND) for name in self}
    self.recv_time = {name: time_s for name in self}
    self.seen = {name: True for name in self}
    self.valid = {name: True for name in self}
    self.alive = {name: True for name in self}
    self.updated = {name: True for name in self}
    self.fresh = True

  def all_checks(self, service_list=None):
    names = self.keys() if service_list is None else service_list
    return self.fresh and all(self.seen[name] and self.valid[name] and self.alive[name] for name in names)


def start(helper=None, *, direction="left", candidates=None, primary=None, secondary=None, time_s=10.0):
  helper = HumanLikeLaneChanges() if helper is None else helper
  if candidates is None:
    candidates = [lead(lateral=3.6 if direction == "left" else -3.6)]
  for before_s in (time_s - .10, time_s - .05):
    sm = Inputs(before_s, direction=direction, candidates=candidates, primary=primary, secondary=secondary)
    assert helper.update(sm, True, now=before_s) is None
  sm = Inputs(time_s, state="laneChangeStarting", direction=direction,
              desire="laneChangeLeft" if direction == "left" else "laneChangeRight",
              candidates=candidates, primary=primary, secondary=secondary)
  return helper, sm, helper.update(sm, True, now=time_s)


def advance(helper, time_s, *, direction="left", desire=None, candidates=None, **kwargs):
  desire = ("laneChangeLeft" if direction == "left" else "laneChangeRight") if desire is None else desire
  sm = Inputs(time_s, state="laneChangeStarting", direction=direction, desire=desire,
              candidates=candidates, **kwargs)
  return sm, helper.update(sm, True, now=time_s)


@pytest.mark.parametrize("direction,lateral", [("left", 3.6), ("right", -3.6)])
def test_starting_returns_native_destination_lead_without_mutating_physical_leads(direction, lateral):
  target = lead(lateral=lateral)
  primary, secondary = lead(10, 50.0, 0.0), lead(11, 9.0, 0.0)
  helper, sm, selected = start(direction=direction, candidates=[target], primary=primary, secondary=secondary)
  before = (sm["radarState"].leadOne.to_dict(), sm["radarState"].leadTwo.to_dict())
  assert selected is not None and selected.radarTrackId == 41
  assert selected.dRel == pytest.approx(35.0)
  assert str(selected.radarSource) == "frontRadar"
  assert selected.schema == log.RadarState.LeadData.schema
  assert (sm["radarState"].leadOne.to_dict(), sm["radarState"].leadTwo.to_dict()) == before
  assert sm["radarState"].leadOne.radarTrackId == 10
  assert sm["radarState"].leadTwo.radarTrackId == 11
  assert helper.update(sm, True, now=10.0) is not None


@pytest.mark.parametrize("source", ["frontRadar", "corner180", "corner235", "corner430"])
def test_track_identity_includes_source_and_accepts_legacy_default_front(source):
  target = lead(source=source)
  if source == "frontRadar":
    # The legacy producer omits the source field, whose wire default is frontRadar.
    fields = target.to_dict()
    fields.pop("radarSource")
    target = log.RadarState.LeadData.new_message(**fields)
  _, _, selected = start(candidates=[target], primary=lead(41, 50.0, 0.0, source="scc"))
  assert selected is not None and str(selected.radarSource) == source


@pytest.mark.parametrize("physical", ["primary", "secondary"])
def test_existing_physical_identity_is_not_added_twice(physical):
  kwargs = {physical: lead()}
  _, _, selected = start(**kwargs)
  assert selected is None


def test_same_numeric_id_in_another_source_does_not_replace_frozen_target():
  helper, _, selected = start(candidates=[lead(source="corner235")])
  assert selected is not None
  _, selected = advance(helper, 10.05, candidates=[lead(source="corner180")])
  assert selected is None
  _, selected = advance(helper, 10.10, candidates=[lead(source="corner235")])
  assert selected is None


@pytest.mark.parametrize("speed,acquired", [(0.0, False), (0.99, False), (1.0, True)])
def test_initial_capture_requires_moving_vehicle(speed, acquired):
  _, _, selected = start(candidates=[lead(speed=speed)])
  assert (selected is not None) == acquired


def test_captured_target_can_decelerate_to_a_stop():
  helper, _, selected = start(candidates=[lead(speed=4.0)])
  assert selected is not None
  for time_s, speed, distance in [(10.05, 2.0, 34.5), (10.10, 0.0, 33.8)]:
    _, selected = advance(helper, time_s, candidates=[lead(distance=distance, speed=speed)])
    assert selected is not None and selected.vLeadK == pytest.approx(speed)


def test_current_positions_select_closest_of_prechange_members_and_ignore_new_members():
  initial = [lead(41, 35.0), lead(42, 36.0), lead(43, 15.0, lateral=7.2)]
  helper, _, selected = start(candidates=initial)
  assert selected is not None and selected.radarTrackId == 41
  for index in range(1, 7):
    shift = .6 * index
    current = [lead(41, 35.0 - .15 * index, lateral=3.6 - shift),
               lead(42, 34.5 - .15 * index, lateral=3.6 - shift),
               lead(43, 5.0, lateral=7.2 - shift)]
    _, selected = advance(helper, 10.0 + .05 * index, candidates=current)
    assert selected is not None and selected.radarTrackId == 42
    assert selected.dRel == pytest.approx(34.5 - .15 * index)


def test_missing_target_cannot_reappear_as_a_reused_id_during_same_request():
  helper, _, selected = start()
  assert selected is not None
  _, selected = advance(helper, 10.05, candidates=[])
  assert selected is None
  _, selected = advance(helper, 10.10, candidates=[lead(distance=34.0)])
  assert selected is None


@pytest.mark.parametrize("changes", [
  {"status": False}, {"radar": False}, {"radarTrackId": -1}, {"dRel": 0.0},
  {"dRel": float("nan")}, {"yRel": float("nan")}, {"vRel": float("nan")},
  {"vLeadK": float("inf")}, {"aLeadK": float("nan")}, {"aLeadTau": float("nan")},
])
def test_invalid_candidates_never_arm(changes):
  _, _, selected = start(candidates=[lead(**changes)])
  assert selected is None


@pytest.mark.parametrize("fault", [
  "missing_lines", "short_lines", "unequal_arrays", "nonmonotonic_x", "nan_line",
  "low_confidence", "missing_confidence", "narrow_lane", "wide_lane", "outside_horizon",
])
def test_invalid_model_geometry_fails_closed_for_entire_request(fault):
  helper = HumanLikeLaneChanges()
  for time_s in (9.90, 9.95, 10.0):
    sm = Inputs(time_s, state="preLaneChange" if time_s < 10 else "laneChangeStarting", desire="laneChangeLeft")
    md = sm["modelV2"]
    if fault == "missing_lines":
      md.laneLines = []
    elif fault == "short_lines":
      md.laneLines[0].x, md.laneLines[0].y = [0.0], [-5.4]
    elif fault == "unequal_arrays":
      md.laneLines[0].y = [-5.4]
    elif fault == "nonmonotonic_x":
      md.laneLines[0].x = [0.0, 20.0, 19.0, 80.0, 140.0]
    elif fault == "nan_line":
      md.laneLines[0].y = [-5.4, float("nan"), -5.4, -5.4, -5.4]
    elif fault == "low_confidence":
      md.laneLineProbs = [0.49, 0.99, 0.99, 0.99]
    elif fault == "missing_confidence":
      md.laneLineProbs = []
    elif fault == "narrow_lane":
      md.laneLines[0].y = [-4.0] * 5
    elif fault == "wide_lane":
      md.laneLines[0].y = [-7.0] * 5
    elif fault == "outside_horizon":
      md.laneLines[0].x = [0.0, 5.0, 10.0, 15.0, 20.0]
    assert helper.update(sm, True, now=time_s) is None
  _, selected = advance(helper, 10.05)
  assert selected is None


def test_geometry_uses_model_coordinate_sign_and_radar_to_camera_offset():
  # At radar dRel=20, the outer line crosses y=-3.6 only after camera x=21.52.
  for outer_end, acquired in [(-3.5, False), (-3.7, True)]:
    helper = HumanLikeLaneChanges()
    for time_s in (9.90, 9.95, 10.0):
      sm = Inputs(time_s, state="preLaneChange" if time_s < 10 else "laneChangeStarting",
                  desire="laneChangeLeft", candidates=[lead(distance=20.0)])
      sm["modelV2"].laneLines[1].y = [-1.0] * 5
      sm["modelV2"].laneLines[0].x = [0.0, 20.0, 21.52, 80.0, 140.0]
      sm["modelV2"].laneLines[0].y = [-5.4, -3.7, outer_end, outer_end, outer_end]
      selected = helper.update(sm, True, now=time_s)
    assert (selected is not None) == acquired


@pytest.mark.parametrize("fault", [
  "model_stale", "radar_stale", "car_stale", "model_future", "radar_future",
  "model_invalid", "radar_invalid", "model_unseen", "radar_dead", "missing_model_reference",
  "unknown_model_reference", "missing_exposure", "future_exposure", "can_invalid", "ego_nan",
])
def test_invalid_timing_or_feed_discards_acquired_request(fault):
  helper, _, selected = start()
  assert selected is not None
  sm = Inputs(10.05, state="laneChangeStarting", desire="laneChangeLeft")
  if fault.endswith("_stale"):
    name = {"model_stale": "modelV2", "radar_stale": "radarState", "car_stale": "carState"}[fault]
    sm.recv_time[name] = 9.799
  elif fault in ("model_future", "radar_future"):
    name = "modelV2" if fault == "model_future" else "radarState"
    sm.logMonoTime[name] = round(10.11 * NS_PER_SECOND)
  elif fault == "model_invalid":
    sm.valid["modelV2"] = False
  elif fault == "radar_invalid":
    sm.valid["radarState"] = False
  elif fault == "model_unseen":
    sm.seen["modelV2"] = False
  elif fault == "radar_dead":
    sm.alive["radarState"] = False
  elif fault == "missing_model_reference":
    sm["radarState"].mdMonoTime = 0
  elif fault == "unknown_model_reference":
    sm["radarState"].mdMonoTime += 1
  elif fault == "missing_exposure":
    sm["modelV2"].timestampEof = 0
  elif fault == "future_exposure":
    sm["modelV2"].timestampEof = round(10.11 * NS_PER_SECOND)
  elif fault == "can_invalid":
    sm["carState"].canValid = False
  elif fault == "ego_nan":
    sm["carState"].vEgo = float("nan")
  assert helper.update(sm, True, now=10.05) is None
  _, selected = advance(helper, 10.10)
  assert selected is None


def test_exact_cached_model_reference_controls_membership():
  helper = HumanLikeLaneChanges()
  for time_s in (9.90, 9.95):
    assert helper.update(Inputs(time_s), True, now=time_s) is None
  # Latest geometry changed; the radar is still paired to the preceding model.
  latest = Inputs(9.975, centers=[-1.8, 1.8, 5.4, 9.0])
  latest["radarState"].mdMonoTime = round(9.95 * NS_PER_SECOND)
  assert helper.update(latest, True, now=9.975) is None
  starting = Inputs(10.025, state="laneChangeStarting", desire="laneChangeLeft", centers=[-1.8, 1.8, 5.4, 9.0])
  selected = helper.update(starting, True, now=10.025)
  assert selected is not None and selected.radarTrackId == 41


def test_cached_model_reference_expires_even_with_fresh_receipts():
  helper, _, selected = start()
  assert selected is not None
  sm = Inputs(10.05, state="laneChangeStarting", desire="laneChangeLeft")
  sm["radarState"].mdMonoTime = round(9.90 * NS_PER_SECOND)
  assert helper.update(sm, True, now=10.20) is None


@pytest.mark.parametrize("override", ["gasPressed", "brakePressed"])
def test_driver_speed_override_cancels_request(override):
  helper, _, _ = start()
  sm = Inputs(10.05, state="laneChangeStarting", desire="laneChangeLeft")
  setattr(sm["carState"], override, True)
  assert helper.update(sm, True, now=10.05) is None
  _, selected = advance(helper, 10.10)
  assert selected is None


def test_new_enable_during_starting_waits_for_next_request():
  helper, _, selected = start()
  assert selected is not None
  sm = Inputs(10.05, state="laneChangeStarting", desire="laneChangeLeft")
  assert helper.update(sm, False, now=10.05) is None
  _, selected = advance(helper, 10.10)
  assert selected is None
  _, _, selected = start(helper, time_s=10.25)
  assert selected is not None


def test_starting_with_no_prechange_evidence_cannot_arm_late():
  helper = HumanLikeLaneChanges()
  for time_s in (10.0, 10.05, 10.10):
    _, selected = advance(helper, time_s)
    assert selected is None


def test_finishing_and_off_release_supplemental_lead():
  helper, _, selected = start()
  assert selected is not None
  for time_s, state in [(10.05, "laneChangeFinishing"), (10.10, "off")]:
    sm = Inputs(time_s, state=state)
    assert helper.update(sm, True, now=time_s) is None


def test_held_starting_desire_pulse_refreshes_membership_after_boundary():
  helper, _, selected = start(candidates=[lead(41, 35.0), lead(42, 45.0, lateral=7.2)])
  assert selected is not None and selected.radarTrackId == 41
  for index in range(1, 7):
    shift = .6 * index
    current = [lead(41, 35.0 - .15 * index, lateral=3.6 - shift),
               lead(42, 45.0 - .15 * index, lateral=7.2 - shift)]
    _, selected = advance(helper, 10.0 + .05 * index, candidates=current)
    assert selected is not None and selected.radarTrackId == 41
  _, selected = advance(helper, 10.35, desire="none", candidates=current)
  assert selected is None
  _, selected = advance(helper, 10.40, candidates=current)
  assert selected is not None and selected.radarTrackId == 42


def test_held_pulse_with_ambiguous_boundary_does_not_keep_old_or_acquire_late():
  helper, _, _ = start()
  _, selected = advance(helper, 10.05, desire="none")
  assert selected is None
  sm = Inputs(10.10, state="laneChangeStarting", desire="laneChangeLeft")
  sm["modelV2"].laneLineProbs = [0.0] * 4
  assert helper.update(sm, True, now=10.10) is None
  for time_s in (10.15, 10.20, 10.25):
    _, selected = advance(helper, time_s)
    assert selected is None


def test_queued_next_request_captures_new_direction_and_membership():
  helper, _, selected = start()
  assert selected is not None
  assert helper.update(Inputs(10.05, state="laneChangeFinishing"), True, now=10.05) is None
  for time_s in (10.10, 10.15):
    sm = Inputs(time_s, direction="right", candidates=[lead(70, 30.0, lateral=-3.6)])
    assert helper.update(sm, True, now=time_s) is None
  _, selected = advance(helper, 10.20, direction="right", candidates=[lead(70, 29.5, lateral=-3.6)])
  assert selected is not None and selected.radarTrackId == 70


@pytest.mark.parametrize("field", ["leadsLeft", "leadsCenter", "leadsRight", "leadsLeft2", "leadsRight2", "leadLeft", "leadRight"])
def test_published_candidate_roles_are_classified_by_captured_geometry(field):
  helper = HumanLikeLaneChanges()
  for time_s in (9.90, 9.95, 10.0):
    sm = Inputs(time_s, state="preLaneChange" if time_s < 10 else "laneChangeStarting",
                desire="laneChangeLeft", candidates=[])
    setattr(sm["radarState"], field, [lead()] if field.startswith("leads") else lead())
    selected = helper.update(sm, True, now=time_s)
  assert selected is not None and selected.radarTrackId == 41


def test_replayed_prechange_frame_cannot_supply_two_confirmations():
  helper = HumanLikeLaneChanges()
  sm = Inputs(9.95)
  assert helper.update(sm, True, now=9.95) is None
  assert helper.update(sm, True, now=9.96) is None
  _, selected = advance(helper, 10.0)
  assert selected is None


@pytest.mark.parametrize("failure", ["exposure_gap", "receipt_silence"])
def test_starting_model_gap_permanently_cancels_this_request(failure):
  helper, sm, selected = start()
  assert selected is not None
  if failure == "exposure_gap":
    _, selected = advance(helper, 10.10)
  else:
    selected = helper.update(sm, True, now=10.076)
  assert selected is None
  _, selected = advance(helper, 10.15)
  assert selected is None


@pytest.mark.parametrize("changes", [
  {"distance": 50.0}, {"lateral": 5.0}, {"speed": 22.0},
  {"vLeadK": float("nan")}, {"status": False}, {"radar": False},
])
def test_unphysical_reused_or_invalid_acquired_id_is_permanently_dropped(changes):
  helper, _, selected = start()
  assert selected is not None
  _, selected = advance(helper, 10.05, candidates=[lead(**changes)])
  assert selected is None
  _, selected = advance(helper, 10.10)
  assert selected is None


def test_identity_swap_does_not_select_new_vehicle_during_same_request():
  helper, _, _ = start()
  _, selected = advance(helper, 10.05, candidates=[lead(42, 34.5)])
  assert selected is None
  _, selected = advance(helper, 10.10, candidates=[lead(41, 34.0), lead(42, 34.0)])
  assert selected is None


def test_conflicting_duplicate_identity_is_ambiguous_but_matching_copy_is_safe():
  helper, _, selected = start(candidates=[lead(), lead()])
  assert selected is not None
  sm = Inputs(10.05, state="laneChangeStarting", desire="laneChangeLeft")
  sm["radarState"].leadsRight = [lead(distance=37.0)]
  assert helper.update(sm, True, now=10.05) is None


@pytest.mark.parametrize("direction,torque", [("left", -1.0), ("right", 1.0)])
def test_opposite_driver_steering_cancels_supplemental_target(direction, torque):
  helper, _, _ = start(direction=direction)
  sm = Inputs(10.05, state="laneChangeStarting", direction=direction,
              desire="laneChangeLeft" if direction == "left" else "laneChangeRight",
              candidates=[lead(lateral=3.6 if direction == "left" else -3.6)])
  sm["carState"].steeringPressed = True
  sm["carState"].steeringTorque = torque
  assert helper.update(sm, True, now=10.05) is None


def test_existing_primary_is_required_for_supplemental_lane_change_constraint():
  _, _, selected = start(primary=log.RadarState.LeadData.new_message(status=False))
  assert selected is None


def test_radar_fault_cancels_supplemental_target():
  helper, sm, selected = start()
  assert selected is not None
  sm = Inputs(10.05, state="laneChangeStarting", desire="laneChangeLeft")
  sm["radarState"].radarErrors.canError = True
  assert helper.update(sm, True, now=10.05) is None


@pytest.mark.parametrize("exposure_offset", [-0.251, 0.050])
def test_stale_or_future_exposure_cannot_arm_with_fresh_envelope_and_receipts(exposure_offset):
  helper = HumanLikeLaneChanges()
  for time_s in (9.90, 9.95, 10.0):
    sm = Inputs(time_s, state="preLaneChange" if time_s < 10 else "laneChangeStarting", desire="laneChangeLeft")
    sm["modelV2"].timestampEof = round((time_s + exposure_offset) * NS_PER_SECOND)
    assert helper.update(sm, True, now=time_s) is None


def test_cached_geometry_is_immutable_and_repeated_model_can_pair_distinct_radar_frames():
  helper = HumanLikeLaneChanges()
  first = Inputs(9.90)
  assert helper.update(first, True, now=9.90) is None
  first["modelV2"].laneLines[0].y = [-1.8] * 5
  second = Inputs(9.95, centers=[-1.8, 1.8, 5.4, 9.0])
  second["radarState"].mdMonoTime = first.logMonoTime["modelV2"]
  assert helper.update(second, True, now=9.95) is None
  _, selected = advance(helper, 10.0)
  assert selected is not None and selected.radarTrackId == 41


def test_feature_first_enabled_in_starting_cannot_arm_without_new_request():
  helper = HumanLikeLaneChanges()
  for time_s in (9.90, 9.95):
    assert helper.update(Inputs(time_s), False, now=time_s) is None
  for time_s in (10.0, 10.05, 10.10):
    _, selected = advance(helper, time_s)
    assert selected is None
  _, _, selected = start(helper, time_s=10.25)
  assert selected is not None
