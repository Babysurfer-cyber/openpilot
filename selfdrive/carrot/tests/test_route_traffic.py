"""Tests for patched CarrotNM congestion fields (no driving-control effects)."""
import pytest

from openpilot.selfdrive.carrot.route_traffic import extract_route_traffic


def route(sections, total=20_000):
  return {"total_distance_m": total, "traffic_sections": sections}


def seg(start, length, congestion):
  return {"start_distance_m": start, "length_m": length, "congestion": congestion}


def test_good_bad_very_bad_none_and_uncovered_gaps():
  present, sections = extract_route_traffic(route([
    seg(0, 5_000, "Good"), seg(6_000, 2_000, "BAD"),
    seg(9_000, 4_000, "VeryBad"), seg(15_000, 5_000, "none"),
  ]))
  assert present
  assert sections == [
    {"startDistanceM": 0, "lengthM": 5000, "congestionCode": 1},
    {"startDistanceM": 6000, "lengthM": 2000, "congestionCode": 2},
    {"startDistanceM": 9000, "lengthM": 4000, "congestionCode": 3},
    {"startDistanceM": 15000, "lengthM": 5000, "congestionCode": 0},
  ]


def test_sender_labels_and_float_integer_meters():
  present, sections = extract_route_traffic(route([
    seg(0.0, 1000.0, " very_bad "), seg(1000, 19000, "GOOD"),
  ]))
  assert present and [part["congestionCode"] for part in sections] == [3, 1]


@pytest.mark.parametrize("bad", [
  None, {}, {"total_distance_m": 20000}, route([]),
  route([seg(0, 1, "good")], total=0),
  route([seg(0, 1, "good")], total=2_000_001),
  route([seg(0, 1, "good")], total=True),
  route([seg(0, 1, "good")], total=float("inf")),
  route([seg(0, 1, "good")], total=10 ** 1000),
  route([seg(4000, 1000, "good"), seg(4500, 1000, "bad")]),
  route([seg(21000, 1000, "good")]),
  route([seg(-1, 500, "good")]),
  route([seg(0, -1, "good")]),
  route([seg(0, 0, "good")]),
  route([seg(0, 500, "not_known")]),
  route([seg(True, 500, "good")]),
  route([seg(1.5, 500, "good")]),
  route([seg(0, float("nan"), "good")]),
  route([seg(0, 500, None)]),
  route([seg(0, 1, "good")] * 129),
])
def test_invalid_traffic_clears_all_sections(bad):
  assert extract_route_traffic(bad) == (False, [])


def test_new_route_without_traffic_does_not_retain_old_route_colors():
  assert extract_route_traffic(route([seg(0, 5000, "very_bad")]))[0]
  assert extract_route_traffic({"total_distance_m": 25_000}) == (False, [])
