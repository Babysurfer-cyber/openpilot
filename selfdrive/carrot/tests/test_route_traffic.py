"""Unit tests for optional CarrotNM congestion, without altering the APK."""
import pytest

from openpilot.selfdrive.carrot.route_traffic import extract_route_traffic


def route(sections, total=20_000):
  return {"total_distance_m": total, "traffic_sections": sections}


def s(start, length, state):
  return {"start_distance_m": start, "length_m": length, "congestion": state}


def test_valid_sections_and_unknown_gaps():
  ok, parts = extract_route_traffic(route([s(0, 5_000, "Good"), s(6_000, 2_000, "BAD"),
                                            s(9_000, 4_000, "VeryBad"), s(15_000, 5_000, "none")]))
  assert ok
  assert [(x["startDistanceM"], x["lengthM"], x["congestionCode"]) for x in parts] == [
    (0, 5000, 1), (6000, 2000, 2), (9000, 4000, 3), (15000, 5000, 0)]


def test_no_sender_field_is_not_fake_congestion():
  assert extract_route_traffic({"total_distance_m": 20_000}) == (False, [])
  assert extract_route_traffic(route([])) == (False, [])
  assert extract_route_traffic(None) == (False, [])


@pytest.mark.parametrize("sections", [
  [s(4000, 1000, "good"), s(4500, 1000, "bad")],  # overlap
  [s(21000, 1000, "good")],  # beyond route
  [s(-1, 500, "good")],
  [s(0, 0, "good")],
  [s(0, 500, "not_known")],
  [s(True, 500, "good")],
  [s(1.5, 500, "good")],
  [s(0, float("nan"), "good")],
  [s(0, 500, None)],
  [s(0, 1, "good")] * 129,
])
def test_malformed_sections_are_fully_rejected(sections):
  assert extract_route_traffic(route(sections)) == (False, [])


def test_reroute_does_not_retain_previous_congestion():
  assert extract_route_traffic(route([s(0, 5_000, "very_bad")]))[0]
  assert extract_route_traffic({"total_distance_m": 25_000}) == (False, [])
