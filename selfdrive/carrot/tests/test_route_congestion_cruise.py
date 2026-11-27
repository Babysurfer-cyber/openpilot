"""Upcoming Naver route traffic caps in Auto Mode: slow 200m/90, jam 300m/70."""
from types import SimpleNamespace as NS

from openpilot.selfdrive.carrot.route_congestion_cruise import (
  RouteCongestionCruiseCap, imminent_route_jam, imminent_route_traffic,
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


def confirm(cap, sm, *, target=110., speed=20.):
  assert update(cap, sm, target=target, speed=speed) == (target, False)
  sm.nav.route.meta.sequence += 1
  return update(cap, sm, target=target, speed=speed)


def test_congestion_300m_boundary_and_outside():
  sm = NaviSM(remaining=11_301)
  assert imminent_route_traffic(sm, now_s=NOW) is None
  sm.nav.route.remainingDistanceM = 11_300
  assert imminent_route_jam(sm, now_s=NOW).distance_m == 300
  assert imminent_route_traffic(sm, now_s=NOW).cap_kph == 70.
  sm.nav.route.remainingDistanceM = 11_000
  assert imminent_route_jam(sm, now_s=NOW).distance_m == 0
  sm.nav.route.remainingDistanceM = 10_999
  assert imminent_route_jam(sm, now_s=NOW) is None


def test_slow_200m_boundary_and_outside():
  sm = NaviSM(code=2, remaining=11_201)
  assert imminent_route_traffic(sm, now_s=NOW) is None
  sm.nav.route.remainingDistanceM = 11_200
  assert imminent_route_traffic(sm, now_s=NOW).distance_m == 200
  assert imminent_route_traffic(sm, now_s=NOW).cap_kph == 90.
  assert imminent_route_jam(sm, now_s=NOW) is None
  sm.nav.route.remainingDistanceM = 11_000
  assert imminent_route_traffic(sm, now_s=NOW).distance_m == 0
  sm.nav.route.remainingDistanceM = 10_999
  assert imminent_route_traffic(sm, now_s=NOW) is None


def test_confirm_two_distinct_fresh_messages_and_ignore_vehicle_speed():
  for code, target_cap in ((2, 90.), (3, 70.)):
    sm, cap = NaviSM(code=code), RouteCongestionCruiseCap()
    assert update(cap, sm, speed=0.) == (110., False)
    assert update(cap, sm, speed=0.) == (110., False)
    sm.nav.route.meta.sequence = 2
    assert update(cap, sm, speed=0.) == (target_cap, True)
    assert update(cap, sm, target=120., speed=65.) == (target_cap, False)


def test_slow_90_then_congestion_70_in_same_cruise_session():
  sm, cap = NaviSM(code=2), RouteCongestionCruiseCap()
  assert confirm(cap, sm) == (90., True)
  sm.nav.route.trafficSections[1].congestionCode = 3
  sm.nav.route.meta.sequence += 1
  assert update(cap, sm, target=110., speed=12.) == (90., False)
  sm.nav.route.meta.sequence += 1
  assert update(cap, sm, target=110., speed=12.) == (70., True)
  assert cap.cap_kph == 70.


def test_improving_congestion_to_slow_then_good_never_restores_speed():
  sm, cap = NaviSM(), RouteCongestionCruiseCap()
  assert confirm(cap, sm) == (70., True)
  sm.nav.route.trafficSections[1].congestionCode = 2
  sm.nav.route.meta.sequence += 1
  assert update(cap, sm, target=110.) == (70., False)
  sm.nav.route.trafficSections[1].congestionCode = 1
  sm.nav.route.meta.sequence += 1
  assert update(cap, sm, target=110.) == (70., False)


def test_improving_slow_to_good_never_restores_speed():
  sm, cap = NaviSM(code=2), RouteCongestionCruiseCap()
  assert confirm(cap, sm) == (90., True)
  sm.nav.route.trafficSections[1].congestionCode = 1
  sm.nav.route.meta.sequence += 1
  assert update(cap, sm, target=110.) == (90., False)


def test_worse_jam_wins_when_slow_and_jam_both_approaching():
  sm = NaviSM(remaining=11_100, code=2)
  sm.nav.route.trafficSections[1].lengthM = 100
  sm.nav.route.trafficSections.insert(2, NS(startDistanceM=9100, lengthM=4900, congestionCode=3))
  section = imminent_route_traffic(sm, now_s=NOW)
  assert section.cap_kph == 70.
  assert section.distance_m == 200


def test_never_raise_a_lower_set_speed():
  for code, target, cap_speed in ((2, 80., 90.), (3, 60., 70.)):
    sm, cap = NaviSM(code=code), RouteCongestionCruiseCap()
    assert update(cap, sm, target=target) == (target, False)
    sm.nav.route.meta.sequence = 2
    assert update(cap, sm, target=target) == (target, False)
    assert cap.cap_kph == cap_speed
    assert update(cap, sm, target=110.) == (cap_speed, False)


def test_good_unknown_and_severity_below_range_never_trigger():
  for code in (0, 1):
    sm, cap = NaviSM(code=code), RouteCongestionCruiseCap()
    assert update(cap, sm) == (110., False)
    sm.nav.route.meta.sequence = 2
    assert update(cap, sm) == (110., False)


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
    assert imminent_route_traffic(sm, now_s=NOW) is None
  cap, sm = RouteCongestionCruiseCap(), NaviSM()
  assert update(cap, sm, gas=True) == (110., False)


def test_manual_override_stops_retrigger_until_disengaged():
  cap, sm = RouteCongestionCruiseCap(), NaviSM()
  assert confirm(cap, sm) == (70., True)
  assert update(cap, sm, target=95., manual=True) == (95., False)
  for seq in (3, 4, 5):
    sm.nav.route.meta.sequence = seq
    assert update(cap, sm, target=105.) == (105., False)
  assert update(cap, sm, engaged=False, target=105.) == (105., False)
  assert update(cap, sm) == (110., False)
  sm.nav.route.meta.sequence = 6
  assert update(cap, sm) == (70., True)


def test_latched_speed_does_not_auto_restore_when_navigation_disappears():
  cap, sm = RouteCongestionCruiseCap(), NaviSM()
  assert confirm(cap, sm) == (70., True)
  sm.nav.route.trafficPresent = False
  sm.alive['carrotNavi'] = False
  assert update(cap, sm, target=115.) == (70., False)


def test_route_severity_change_requires_distinct_second_fresh_heartbeat():
  cap, sm = RouteCongestionCruiseCap(), NaviSM(code=2)
  assert update(cap, sm) == (110., False)
  sm.nav.route.trafficSections[1].congestionCode = 3
  assert update(cap, sm) == (110., False)
  sm.nav.route.meta.sequence += 1
  assert update(cap, sm) == (70., True)
