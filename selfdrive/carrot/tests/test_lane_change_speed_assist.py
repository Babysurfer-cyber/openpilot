"""Lane-change side-lead assist must not react to a mere blinker or false tracks."""

from dataclasses import dataclass, replace
from math import nan, inf, isfinite
from random import Random
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar_motion.lane_change_speed_assist import (
  LaneChangeSpeedAssist,
  side_lead_may_replace,
)


@dataclass
class Point:
  track_id: int = 10
  source: str = "corner235"
  d_rel: float = 35.0
  y_rel: float = 3.6
  v_rel: float = -3.0
  v_lead: float = 24.0
  yv_rel: float = 0.0
  measured: bool = True


def model(state="off", direction="none", probabilities=(0.9, 0.9, 0.9, 0.9)):
  lines = tuple(SimpleNamespace(x=(0.0, 100.0), y=(y, y)) for y in (-5.4, -1.8, 1.8, 5.4))
  return SimpleNamespace(
    meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction),
    laneLines=lines, laneLineProbs=probabilities,
  )


def seed(tracker, point=Point(), direction="left"):
  for i in range(6):
    t = i * 0.05
    tracker.update(t, 27.0, model("off" if i < 3 else "preLaneChange", direction), (point,))
  return tracker.update(0.30, 27.0, model("laneChangeStarting", direction), (point,))


def test_latches_preobserved_left_lead_only_at_start():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(0.35, 27.0, model("laneChangeStarting", "left"), (Point(y_rel=3.5),)) is not None
  assert tracker.update(0.40, 27.0, model("laneChangeFinishing", "left"), (Point(),)) is None


def test_rejects_wrong_side_and_faster_leads():
  assert seed(LaneChangeSpeedAssist(), Point(y_rel=-3.6)) is None
  assert seed(LaneChangeSpeedAssist(), Point(v_rel=2.0, v_lead=29.0)) is None


def test_rejects_scc_untrusted_lateral():
  assert seed(LaneChangeSpeedAssist(), Point(source="scc")) is None


def test_rejects_bad_lane_confidence():
  tracker = LaneChangeSpeedAssist()
  for i in range(6):
    tracker.update(i * .05, 27.0, model("preLaneChange", "left", probabilities=(0.2, 0.2, 0.9, 0.9)), (Point(),))
  assert tracker.update(.30, 27.0, model("laneChangeStarting", "left"), (Point(),)) is None


def test_rejects_blinker_only_and_late_new_track():
  tracker = LaneChangeSpeedAssist()
  for i in range(6):
    tracker.update(i * .05, 27.0, model("off"), (Point(),))
  assert tracker.update(.30, 27.0, model("laneChangeStarting", "left"), (Point(),)) is None
  assert tracker.update(.35, 27.0, model("laneChangeStarting", "left"), (Point(),)) is None


def test_rejects_track_jump_and_lost_track():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.35, 27.0, model("laneChangeStarting", "left"), (Point(d_rel=25.0),)) is None
  assert tracker.update(.40, 27.0, model("laneChangeStarting", "left"), (Point(),)) is None
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.35, 27.0, model("laneChangeStarting", "left"), ()) is None


def test_rejects_no_closing_and_distant_targets():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.35, 27.0, model("laneChangeStarting", "left"), (Point(v_rel=0.2),)) is None
  assert seed(LaneChangeSpeedAssist(), Point(d_rel=75.0)) is None


def test_reset_on_finish_requires_fresh_prechange():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.35, 27.0, model("laneChangeFinishing", "left"), (Point(),)) is None
  assert tracker.update(.40, 27.0, model("laneChangeStarting", "left"), (Point(),)) is None


def test_does_not_overwrite_more_restrictive_existing_lead_two():
  candidate = {"dRel": 26.0, "vLead": 15.0, "status": True}
  assert side_lead_may_replace(candidate, None)
  assert not side_lead_may_replace(candidate, {"dRel": 20.0, "vLead": 20.0, "status": True})
  assert not side_lead_may_replace(candidate, {"dRel": 30.0, "vLead": 10.0, "status": True})
  assert side_lead_may_replace(candidate, {"dRel": 40.0, "vLead": 20.0, "status": True})

def test_right_lane_assist_and_wrong_direction():
  assert seed(LaneChangeSpeedAssist(), Point(y_rel=-3.6), direction="right") is not None
  assert seed(LaneChangeSpeedAssist(), Point(y_rel=-3.6), direction="left") is None


def test_no_pre_lane_change_always_rejects():
  tracker = LaneChangeSpeedAssist()
  assert tracker.update(.3, 27., model("laneChangeStarting", "left"), (Point(),)) is None


def test_pre_lane_change_never_commands_before_start():
  tracker = LaneChangeSpeedAssist()
  for i in range(15):
    assert tracker.update(i * .05, 27., model("preLaneChange", "left"), (Point(),)) is None
  assert tracker.update(.75, 27., model("laneChangeStarting", "left"), (Point(),)) is not None


def test_no_rapid_target_switch_mid_change():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.35, 27., model("laneChangeStarting", "left"), (Point(track_id=11),)) is None
  assert tracker.update(.40, 27., model("laneChangeStarting", "left"), (Point(track_id=10),)) is None


def test_time_out_of_order_rejects():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.25, 27., model("laneChangeStarting", "left"), (Point(),)) is None


def test_missing_lane_boundaries_rejects():
  tracker = LaneChangeSpeedAssist()
  for i in range(10):
    frame = model("preLaneChange", "left")
    frame.laneLines = ()
    tracker.update(i * .05, 27., frame, (Point(),))
  assert tracker.update(.50, 27., model("laneChangeStarting", "left"), (Point(),)) is None


def test_unmeasured_and_invalid_radar_source_rejects():
  assert seed(LaneChangeSpeedAssist(), Point(measured=False)) is None
  assert seed(LaneChangeSpeedAssist(), Point(source="bogus")) is None


def test_no_positive_accel_when_side_vehicle_pulls_away():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.35, 27., model("laneChangeStarting", "left"), (Point(v_rel=1.0, v_lead=28.0),)) is None


def test_below_minimal_ego_speed_rejects():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.35, 3., model("laneChangeStarting", "left"), (Point(),)) is None


def test_multi_track_prefers_closest_side_vehicle():
  tracker = LaneChangeSpeedAssist()
  a, b = Point(track_id=3, d_rel=38.), Point(track_id=4, d_rel=30.)
  for i in range(6):
    tracker.update(i * .05, 27., model("off" if i < 3 else "preLaneChange", "left"), (a, b))
  chosen = tracker.update(.30, 27., model("laneChangeStarting", "left"), (a, b))
  assert chosen.track_id == 4


def test_strict_preexisting_lead_two_priority():
  candidate = {"dRel": 25., "vLead": 20., "status": True}
  assert not side_lead_may_replace(candidate, {"dRel": 24.6, "vLead": 21., "status": True})
  assert not side_lead_may_replace(candidate, {"dRel": 34., "vLead": 18., "status": True})


def test_after_finish_new_prechange_can_rearm():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker) is not None
  assert tracker.update(.35, 27., model("laneChangeFinishing", "left"), (Point(),)) is None
  for i in range(6):
    assert tracker.update(.4 + i * .05, 27., model("preLaneChange", "left"), (Point(),)) is None
  assert tracker.update(.7, 27., model("laneChangeStarting", "left"), (Point(),)) is not None


def test_direction_flip_after_start_disables_original_side():
  tracker = LaneChangeSpeedAssist()
  assert seed(tracker, Point(y_rel=3.6), direction="left") is not None
  assert tracker.update(.35, 27., model("laneChangeStarting", "right"), (Point(y_rel=3.6),)) is None


def test_direction_change_in_prechange_requires_new_observation():
  tracker = LaneChangeSpeedAssist()
  for i in range(6):
    assert tracker.update(i * .05, 27., model("preLaneChange", "left"), (Point(y_rel=-3.6),)) is None
  assert tracker.update(.30, 27., model("preLaneChange", "right"), (Point(y_rel=-3.6),)) is None
  assert tracker.update(.35, 27., model("laneChangeStarting", "right"), (Point(y_rel=-3.6),)) is None


def test_nonfinite_lane_line_probability_rejected():
  tracker = LaneChangeSpeedAssist()
  for i in range(6):
    frame = model("preLaneChange", "left", probabilities=(.9, nan, .9, .9))
    assert tracker.update(i * .05, 27., frame, (Point(),)) is None
  assert tracker.update(.30, 27., model("laneChangeStarting", "left"), (Point(),)) is None


def test_nonfinite_target_velocity_rejected():
  assert seed(LaneChangeSpeedAssist(), Point(v_lead=float("inf"))) is None


def test_synthetic_state_sequences_fail_closed():
  rng = Random(6060)
  values = (-inf, -100.0, -5.4, -3.6, -1.8, -0.1, 0, 0.1, 1.8, 3.6, 5.4, 100, inf, nan)
  probs = (0.0, 0.2, 0.49, 0.5, 0.9, nan, inf)
  states = ("off", "preLaneChange", "laneChangeStarting", "laneChangeFinishing", "unexpected")
  directions = ("none", "left", "right", "unexpected")
  for _ in range(250):
    tracker = LaneChangeSpeedAssist()
    for frame in range(35):
      time_s = frame * 0.05
      if rng.random() < 0.01:
        time_s -= 0.1
      state = rng.choice(states)
      direction = rng.choice(directions)
      m = model(state, direction, probabilities=tuple(rng.choice(probs) for _ in range(4)))
      point = Point(
        track_id=rng.choice((0, 1, 10)),
        source=rng.choice(("frontRadar", "corner180", "corner235", "scc", "other")),
        d_rel=rng.choice((0, 1, 4, 10, 35, 78, 100, nan, inf)),
        y_rel=rng.choice(values),
        v_rel=rng.choice((-30, -3, 0, 1, 30, nan, inf)),
        v_lead=rng.choice((0, 1, 2, 10, 25, nan, inf)),
        measured=rng.choice((True, False)),
      )
      result = tracker.update(time_s, rng.choice((0, 3, 8, 27, nan, inf)), m, (point,))
      if result is not None:
        assert state == "laneChangeStarting"
        assert direction in ("left", "right")
        assert result is point
        assert result.v_rel <= 0
        assert result.d_rel > 0
        assert all(isfinite(v) for v in (result.d_rel, result.y_rel, result.v_rel, result.v_lead))


def test_stable_side_track_assist_expires_after_four_seconds():
  tracker = LaneChangeSpeedAssist()
  point = Point(v_rel=0.0, v_lead=27.0)
  for i in range(6):
    tracker.update(i * .05, 27.0, model("preLaneChange", "left"), (point,))
  assert tracker.update(.3, 27.0, model("laneChangeStarting", "left"), (point,)) is not None
  for i in range(1, 86):
    time_s = .3 + i * .05
    point = replace(point, d_rel=35.0)
    selected = tracker.update(time_s, 27.0, model("laneChangeStarting", "left"), (point,))
    if time_s > 4.3:
      assert selected is None


def test_realistic_time_series_side_lead_only_during_active_change():
  """Four time series with changing range: two intended leads and two opposite-lane controls."""
  from dataclasses import replace

  for direction, side_y, expected in (
    ("left", 3.6, True),
    ("left", -3.6, False),
    ("right", 3.6, False),
    ("right", -3.6, True),
  ):
    tracker = LaneChangeSpeedAssist()
    active_frames = []
    for frame in range(120):
      time_s = frame * .05
      state = ("off" if frame < 20 else
               "preLaneChange" if frame < 35 else
               "laneChangeStarting" if frame < 85 else
               "laneChangeFinishing")
      p = replace(Point(), d_rel=45.0 - 2.0 * time_s, v_rel=-2.0,
                  v_lead=25.0, y_rel=side_y)
      candidate = tracker.update(time_s, 27.0, model(state, direction), (p,))
      active_frames.append(candidate is not None)

    assert sum(active_frames) == (50 if expected else 0), (direction, side_y)
    assert not any(active_frames[:35]), (direction, side_y)
    assert not any(active_frames[85:]), (direction, side_y)


def _lane_geometry_candidate(bounds, probabilities=(0.99, 0.99, 0.99, 0.99)):
  """Build a stable target; only intentionally invalid lane geometry may reject it."""
  from types import SimpleNamespace

  tracker = LaneChangeSpeedAssist()
  point = Point(y_rel=3.6)

  def malformed(state):
    frame = model(state, "left", probabilities=probabilities)
    frame.laneLines = tuple(
      SimpleNamespace(x=(0.0, 100.0), y=(float(y), float(y))) for y in bounds
    )
    return frame

  for i in range(6):
    tracker.update(i * .05, 27.0, malformed("preLaneChange"), (point,))
  return tracker.update(.30, 27.0, malformed("laneChangeStarting"), (point,))


def test_implausibly_wide_adjacent_lane_does_not_trigger_braking():
  assert _lane_geometry_candidate((-12.0, -1.8, 1.8, 5.4)) is None


def test_implausibly_wide_current_lane_does_not_trigger_braking():
  assert _lane_geometry_candidate((-5.4, -1.8, 6.0, 9.6)) is None


def test_normal_lane_geometry_remains_eligible():
  assert _lane_geometry_candidate((-5.4, -1.8, 1.8, 5.4)) is not None


def test_out_of_range_confidence_does_not_trigger_braking():
  assert _lane_geometry_candidate((-5.4, -1.8, 1.8, 5.4), (2.0, .99, .99, .99)) is None
