from types import SimpleNamespace
import unittest

from cereal import car, log
from openpilot.selfdrive.controls.radard import (
  CUTIN_CONFIRM_FRAMES, RADAR_TO_CAMERA, RadarD, Track, match_vision_to_track,
)


def make_track(source="corner180", d_rel=15.0, y_rel=0.5):
  track = Track(1000 if source.startswith("corner") else 32)
  track.radar_source = source
  track.measured = True
  track.cnt = 30
  track.dRel = d_rel
  track.yRel = y_rel
  track.vLead = track.vLeadK = 12.0
  track.vRel = -3.0
  track.in_lane_prob = 0.8
  track.in_lane_prob_future = 0.8
  track.dPath = track.dPath_future = y_rel
  return track


def make_radar():
  radar = RadarD.__new__(RadarD)
  radar.radar_state = log.RadarState.new_message()
  radar.detect_cut_in = True
  radar.lane_line_available = True
  radar.radar_detected = False
  return radar


def make_model():
  return SimpleNamespace(position=SimpleNamespace(x=list(range(33))),
                         leadsV3=[SimpleNamespace(y=[0.0])])


class TestCornerRadarTracks(unittest.TestCase):
  def test_source_survives_track_and_message(self):
    point = car.RadarData.RadarPoint.new_message(
      trackId=1000, dRel=15, yRel=2, vRel=-3, vLead=12,
      measured=True, radarSource="corner180",
    )
    track = Track(1000)
    track.update(None, point, False, 1.0, 0.0)
    self.assertTrue(track.is_corner)
    lead = log.RadarState.LeadData.new_message(**track.get_RadarState())
    self.assertEqual(str(lead.radarSource), "corner180")
    self.assertEqual(lead.radarTrackId, 1000)

  def test_corner_cannot_replace_primary_match_or_sticky_lead(self):
    corner = make_track()
    front = make_track("frontRadar")
    lead = SimpleNamespace(x=[15 + RADAR_TO_CAMERA], y=[-0.5], v=[12.0],
                           xStd=[4.0], yStd=[1.0], vStd=[3.0])
    self.assertIsNone(match_vision_to_track(15.0, lead, 0.9, {1000: corner}))
    self.assertIs(match_vision_to_track(15.0, lead, 0.9, {1000: corner, 32: front}), front)
    corner.selected_count = 10
    front.selected_count = 5
    radar = make_radar()
    self.assertIsNone(radar.get_sticky_track({1000: corner}))
    self.assertIs(radar.get_sticky_track({1000: corner, 32: front}), front)
    self.assertFalse(corner.potential_low_speed_lead(0.0))
    self.assertTrue(front.potential_low_speed_lead(0.0))

  def test_corner_cannot_be_radar_only_center_lead(self):
    radar = make_radar()
    corner = make_track()
    self.assertFalse(radar._is_center_lead_candidate(corner))
    self.assertFalse(radar._radar_only_center_ok(corner.get_RadarState()))
    front = make_track("frontRadar")
    self.assertTrue(radar._is_center_lead_candidate(front))
    self.assertTrue(radar._radar_only_center_ok(front.get_RadarState()))

  def test_side_and_behind_objects_are_display_only(self):
    for distance in (0.0, 0.2, -5.0):
      with self.subTest(distance=distance):
        radar = make_radar()
        corner = make_track(d_rel=distance, y_rel=2.0)
        corner.cut_in_count = CUTIN_CONFIRM_FRAMES + 1
        radar.compute_leads(15.0, {1000: corner}, make_model(), 0.9)
        self.assertEqual(len(radar.radar_state.leadsLeft), 1)
        self.assertEqual(str(radar.radar_state.leadsLeft[0].radarSource), "corner180")
        self.assertEqual(len(radar.radar_state.leadsCenter), 0)
        self.assertEqual(len(radar.radar_state.leadsCutIn), 0)
        self.assertFalse(radar.leadCutIn["status"])
        self.assertIsNone(radar.leadCenter)
        self.assertIsNone(radar.leadTwo)
        self.assertEqual(corner.cut_in_count, 0)

  def test_forward_corner_requires_sustained_inward_motion(self):
    radar = make_radar()
    corner = make_track(y_rel=2.5)
    corner.in_lane_prob = 0.2
    corner.in_lane_prob_future = 0.7
    corner.dPath = 2.5
    corner.dPath_future = 1.0
    for _ in range(CUTIN_CONFIRM_FRAMES - 1):
      radar.compute_leads(15.0, {1000: corner}, make_model(), 0.9)
      self.assertFalse(radar.leadCutIn["status"])
    radar.compute_leads(15.0, {1000: corner}, make_model(), 0.9)
    self.assertTrue(radar.leadCutIn["status"])
    self.assertEqual(radar.leadCutIn["radarSource"], "corner180")
    self.assertEqual(len(radar.radar_state.leadsCenter), 0)
    radar._pick_lead_one_from_state()
    self.assertEqual(str(radar.radar_state.leadOne.radarSource), "corner180")

  def test_parallel_corner_does_not_trigger_cut_in(self):
    radar = make_radar()
    corner = make_track(y_rel=2.5)
    for _ in range(CUTIN_CONFIRM_FRAMES + 2):
      radar.compute_leads(15.0, {1000: corner}, make_model(), 0.9)
    self.assertFalse(radar.leadCutIn["status"])
    self.assertEqual(len(radar.radar_state.leadsCutIn), 0)


if __name__ == "__main__":
  unittest.main()
