"""Unit tests for opt-in-by-mode upcoming route congestion cruise cap."""
from types import SimpleNamespace as NS

from openpilot.selfdrive.carrot.route_congestion_cruise import (
  RouteCongestionCruiseCap, imminent_route_jam,
)

NOW = 1000.0


def meta(sequence=1, age=0.1, present=True):
  return NS(present=present, sequence=sequence,
            receivedMonoTimeNanos=int((NOW - age) * 1e9))


class NaviSM:
  def __init__(self, *, remaining=11_100, code=3, sequence=1):
    self.seen = {'carrotNavi': True}
    self.alive = {'carrotNavi': True}
    self.valid = {'carrotNavi': True}
    self.recv_time = {'carrotNavi': NOW}
    self.nav = NS(
      connected=True, sessionId='session-test',
      navigationStatus=NS(meta=meta(), guidanceActive=True, offRoute=False, routePresent=True),
      route=NS(
        meta=meta(sequence), totalDistanceM=20_000, remainingDistanceM=remaining,
        trafficPresent=True, trafficSections=[
          NS(startDistanceM=0, lengthM=9000, congestionCode=1),
          NS(startDistanceM=9000, lengthM=5000, congestionCode=code),
          NS(startDistanceM=14000, lengthM=6000, congestionCode=1),
        ],
      ),
    )

  def __getitem__(self, key):
    assert key == 'carrotNavi'
    return self.nav


def update(cap, sm, *, target=110., speed=90., engaged=True, manual=False, gas=False):
  return cap.update(target, speed, sm, engaged=engaged, manual_speed_change=manual,
                    gas_pressed=gas, now_s=NOW)


def test_300m_boundary_and_outside():
  sm = NaviSM(remaining=11_301)  # traveled 8699: jam starts 301m ahead
  assert imminent_route_jam(sm, now_s=NOW) is None
  sm.nav.route.remainingDistanceM = 11_300
  assert imminent_route_jam(sm, now_s=NOW).distance_m == 300
  sm.nav.route.remainingDistanceM = 11_000
  assert imminent_route_jam(sm, now_s=NOW).distance_m == 0
  sm.nav.route.remainingDistanceM = 10_999  # already inside congestion
  assert imminent_route_jam(sm, now_s=NOW) is None


def test_confirm_two_distinct_fresh_messages_before_reducing():
  sm, cap = NaviSM(), RouteCongestionCruiseCap()
  assert update(cap, sm) == (110., False)
  assert update(cap, sm) == (110., False)  # 100Hz reuse of same 1Hz packet
  sm.nav.route.meta.sequence = 2
  assert update(cap, sm) == (80., True)
  assert update(cap, sm, target=120., speed=65.) == (80., False)


def test_only_very_bad_and_vehicle_at_least_90():
  sm, cap = NaviSM(code=2), RouteCongestionCruiseCap()
  assert update(cap, sm) == (110., False)
  sm.nav.route.trafficSections[1].congestionCode = 3
  assert update(cap, sm, speed=89.9) == (110., False)
  assert update(cap, sm, speed=90.) == (110., False)
  sm.nav.route.meta.sequence = 2
  assert update(cap, sm, speed=90.) == (80., True)


def test_never_raise_a_lower_cruise_target():
  sm, cap = NaviSM(), RouteCongestionCruiseCap()
  assert update(cap, sm, target=70.) == (70., False)
  sm.nav.route.meta.sequence = 2
  assert update(cap, sm, target=70.) == (70., False)


def test_missing_stale_off_route_traffic_invalid_or_gas_does_not_trigger():
  for invalidate in (
    lambda sm: setattr(sm.nav.route, 'trafficPresent', False),
    lambda sm: setattr(sm.nav.navigationStatus, 'offRoute', True),
    lambda sm: setattr(sm.nav.navigationStatus.meta, 'receivedMonoTimeNanos', int((NOW - 4) * 1e9)),
    lambda sm: setattr(sm.nav.route.meta, 'receivedMonoTimeNanos', int((NOW - 4) * 1e9)),
    lambda sm: sm.valid.update({'carrotNavi': False}),
    lambda sm: sm.recv_time.update({'carrotNavi': NOW - 2}),
    lambda sm: setattr(sm.nav.route, 'remainingDistanceM', 20_001),
    lambda sm: setattr(sm.nav.route.trafficSections[1], 'startDistanceM', 8000),
  ):
    sm = NaviSM()
    invalidate(sm)
    assert imminent_route_jam(sm, now_s=NOW) is None
  cap, sm = RouteCongestionCruiseCap(), NaviSM()
  assert update(cap, sm, gas=True) == (110., False)


def test_manual_override_stops_retrigger_and_disengage_rearms():
  cap, sm = RouteCongestionCruiseCap(), NaviSM()
  update(cap, sm)
  sm.nav.route.meta.sequence = 2
  assert update(cap, sm) == (80., True)
  assert update(cap, sm, target=90., manual=True) == (90., False)
  for seq in (3, 4, 5):
    sm.nav.route.meta.sequence = seq
    assert update(cap, sm, target=105.) == (105., False)
  assert update(cap, sm, engaged=False) == (105., False)
  assert update(cap, sm) == (110., False)
  sm.nav.route.meta.sequence = 6
  assert update(cap, sm) == (80., True)


def test_latched_speed_does_not_auto_restore_when_traffic_disappears():
  cap, sm = RouteCongestionCruiseCap(), NaviSM()
  update(cap, sm)
  sm.nav.route.meta.sequence = 2
  assert update(cap, sm) == (80., True)
  sm.nav.route.trafficPresent = False
  sm.alive['carrotNavi'] = False
  assert update(cap, sm, target=115.) == (80., False)
