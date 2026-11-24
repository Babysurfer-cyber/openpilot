"""Checks for the privacy-safe CarrotNavi traffic diagnostic endpoint logic."""
from openpilot.selfdrive.carrot.traffic_diagnostic import summarize_traffic


def snapshot(sections=None, *, connected=True, route_present=True, guidance=True):
  value = {"total_distance_m": 20000, "remain_distance_m": 12000}
  if sections is not None:
    value["traffic_sections"] = sections
  return {
    "connected": connected, "cereal_publish_count": 4, "cereal_error": None,
    "items": {
      "route": {"present": route_present, "value": value if route_present else None},
      "navigation_status": {"present": True, "value": {
        "guidance_active": guidance, "off_route": False, "route_present": True,
      }},
    },
  }


def test_colored_route_when_status_distance_and_sections_valid():
  s = snapshot([{"start_distance_m": 0, "length_m": 20000, "congestion": "good"}])
  r = summarize_traffic(s)
  assert r["bar_position_expected"] and r["bar_traffic_colors_expected"]
  assert r["traffic_sections_received"] == 1
  assert r["reasons"] == []


def test_one_metre_route_rounding_does_not_hide_real_traffic():
  s = snapshot([{"start_distance_m": 0, "length_m": 6329, "congestion": "good"}])
  route = s["items"]["route"]["value"]
  route["total_distance_m"] = 6329
  route["remain_distance_m"] = 6330
  r = summarize_traffic(s)
  assert r["bar_position_expected"] and r["bar_traffic_colors_expected"]
  assert r["traffic_covered_percent"] == 100.0
  assert r["reasons"] == []
  # Excessively inconsistent route distances still block the HUD.
  route["remain_distance_m"] = 6335
  rejected = summarize_traffic(s)
  assert rejected["bar_position_expected"] is False
  assert "route_distance_invalid_or_missing" in rejected["reasons"]


def test_valid_route_without_sections_shows_neutral_progress():
  r = summarize_traffic(snapshot())
  assert r["bar_position_expected"] and not r["bar_traffic_colors_expected"]
  assert "traffic_sections_missing_or_invalid" in r["reasons"]


def test_legacy_tbt_does_not_imply_carrot_navi_connected():
  r = summarize_traffic(snapshot(connected=False))
  assert not r["bar_position_expected"]
  assert "websocket_control_not_connected" in r["reasons"]


def test_stale_route_does_not_show_marker_or_old_colors():
  r = summarize_traffic(snapshot(route_present=False))
  assert not r["bar_position_expected"] and not r["bar_traffic_colors_expected"]
  assert "route_item_missing_or_stale" in r["reasons"]


def test_invalid_lengths_are_not_displayed_as_real_traffic():
  s = snapshot([{"start_distance_m": 19000, "length_m": 5000, "congestion": "bad"}])
  r = summarize_traffic(s)
  assert r["bar_position_expected"] and not r["bar_traffic_colors_expected"]
  assert r["traffic_sections_received"] == 1
  assert r["traffic_sections_valid"] == 0

def test_app_sdi_camera_fields_exposed_without_location_data():
  s = snapshot()
  s["items"]["speed"] = {"present": True, "value": {
    "road_limit_kph": 60,
    "sdi": {"type": 1, "distance_m": 800, "speed_limit_kph": 60},
    "sdi_secondary": None,
  }}
  result = summarize_traffic(s)
  assert result["speed_item_present"] is True
  assert result["sdi_present"] is True
  assert result["sdi_type"] == 1
  assert result["sdi_distance_m"] == 800
  assert result["sdi_speed_limit_kph"] == 60
  assert result["road_limit_kph"] == 60
  assert result["secondary_sdi_present"] is False
  assert "latitude" not in result and "longitude" not in result


def test_absent_sdi_is_reported_without_inventing_camera_warning():
  result = summarize_traffic(snapshot())
  assert result["speed_item_present"] is False
  assert result["sdi_present"] is False
  assert result["sdi_type"] is None
  assert result["sdi_distance_m"] is None


def test_traffic_coverage_identifies_unreported_gray_gap():
  s = snapshot([
    {"start_distance_m": 0, "length_m": 14000, "congestion": "good"},
    {"start_distance_m": 14000, "length_m": 1000, "congestion": "bad"},
    {"start_distance_m": 15000, "length_m": 500, "congestion": "very_bad"},
  ])
  r = summarize_traffic(s)
  assert r["bar_traffic_colors_expected"]
  assert r["traffic_covered_m"] == 15500
  assert r["traffic_uncovered_m"] == 4500
  assert r["traffic_covered_percent"] == 77.5
  assert r["traffic_first_section_start_m"] == 0
  assert r["traffic_last_section_end_m"] == 15500
  assert r["traffic_lengths_m"] == {
    "good": 14000, "bad": 1000, "very_bad": 500, "none": 0,
  }


def test_traffic_coverage_counts_unknown_gaps_even_between_valid_sections():
  s = snapshot([
    {"start_distance_m": 1000, "length_m": 3000, "congestion": "none"},
    {"start_distance_m": 5000, "length_m": 2000, "congestion": "good"},
  ])
  r = summarize_traffic(s)
  assert r["traffic_uncovered_m"] == 15000
  assert r["traffic_covered_percent"] == 25
  assert r["traffic_first_section_start_m"] == 1000
  assert r["traffic_last_section_end_m"] == 7000
  assert r["traffic_lengths_m"]["none"] == 3000


def test_no_sections_do_not_claim_colored_coverage():
  r = summarize_traffic(snapshot())
  assert r["traffic_covered_percent"] is None
  assert r["traffic_uncovered_m"] is None
  assert r["traffic_lengths_m"] is None

def test_v5_traffic_candidate_coverage_visible_without_route_geometry():
  s = snapshot([{"start_distance_m": 0, "length_m": 12000, "congestion": "good"}])
  route = s["items"]["route"]["value"]
  route.update({
    "traffic_source": "naver_route_traffics",
    "traffic_guidance_covered_m": 8000,
    "traffic_route_sections_covered_m": 9000,
    "traffic_route_traffics_covered_m": 12000,
    "traffic_selected_covered_m": 12000,
  })
  result = summarize_traffic(s)
  assert result["traffic_source"] == "naver_route_traffics"
  assert result["traffic_guidance_covered_m"] == 8000
  assert result["traffic_route_sections_covered_m"] == 9000
  assert result["traffic_route_traffics_covered_m"] == 12000
  assert result["traffic_selected_covered_m"] == 12000
  assert "latitude" not in result and "longitude" not in result
