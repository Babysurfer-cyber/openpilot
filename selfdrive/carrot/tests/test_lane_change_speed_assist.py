"""Lane-change side-lead assist must not react to a mere blinker or false tracks."""

from dataclasses import dataclass
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
