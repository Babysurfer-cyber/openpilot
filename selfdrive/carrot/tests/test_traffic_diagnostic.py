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
