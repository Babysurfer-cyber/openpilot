"""Route ownership never mixes an external instruction with another sender's geometry."""

from types import SimpleNamespace

import pytest

from cereal import car, custom, log
from openpilot.selfdrive.carrot.carrot_navi_control import get_carrot_navi_control, parse_carrot_navi_control
from openpilot.selfdrive.carrot.carrot_navi_route import MAX_ROUTE_POINTS, RouteSource, coordinates


MAPBOX = ((37., 127.), (37.001, 127.))
LEGACY = ((37., 128.), (37.001, 128.))
V2 = ((38., 127.), (38.001, 127.))


def route_dicts(points):
  return [{'latitude': lat, 'longitude': lon} for lat, lon in points]


def navigation_message(points=V2, *, present=True, off_route=False, connected=True):
  return custom.CarrotNaviState.new_message(
    schemaVersion=1, connected=connected, sessionId='current-session',
    navigationStatus={'meta': {'present': True, 'sequence': 1, 'receivedMonoTimeNanos': 100_000_000_000},
                      'guidanceActive': True, 'offRoute': off_route},
    route={'meta': {'present': present, 'sequence': 1, 'receivedMonoTimeNanos': 100_000_000_000}, 'polyline': route_dicts(points),
           'remainingDistanceM': 4000, 'remainingTimeSec': 300},
  )


def control(points=V2, **kwargs):
  return parse_carrot_navi_control(navigation_message(points, **kwargs), now_s=100.)


def test_v2_route_takes_priority_and_discard_pre_v2_legacy_geometry():
  source = RouteSource()
  source.mapbox, source.legacy = MAPBOX, LEGACY
  assert source.select(True) == route_dicts(LEGACY)
  assert source.select(True, control()) == route_dicts(V2)
  assert source.legacy == ()
  assert source.select(False) == route_dicts(MAPBOX)
  # A legacy reconnect needs a new route, not the geometry from before v2.
  assert source.select(True) == []
  source.legacy = LEGACY
  assert source.select(True) == route_dicts(LEGACY)


@pytest.mark.parametrize(('present', 'off_route', 'points'), (
  (False, False, V2),
  (True, False, ()),
  (True, True, V2),
))
def test_external_navigation_without_usable_geometry_suppresses_mapbox(present, off_route, points):
  source = RouteSource()
  source.mapbox, source.legacy = MAPBOX, LEGACY
  assert source.select(True, control(points, present=present, off_route=off_route)) == []
  assert source.selected == ('external', ())


def test_mapbox_changes_are_cached_until_the_external_owner_disconnects():
  source = RouteSource()
  source.mapbox = MAPBOX
  assert source.select(False, control()) == route_dicts(V2)
  later = ((36., 126.), (36.001, 126.))
  source.mapbox = later
  assert source.select(False, control()) is None
  assert source.select(False) == route_dicts(later)


def test_legacy_disconnect_clears_route_before_reconnect():
  source = RouteSource()
  source.mapbox, source.legacy = MAPBOX, LEGACY
  assert source.select(True) == route_dicts(LEGACY)
  assert source.select(False) == route_dicts(MAPBOX)
  assert source.select(True) == []


@pytest.mark.parametrize('point', (
  {'latitude': float('nan'), 'longitude': 127.},
  {'latitude': 37., 'longitude': float('inf')},
  {'latitude': 90., 'longitude': 127.},
  {'latitude': 37., 'longitude': 181.},
  {'longitude': 127.},
  {'latitude': 'invalid', 'longitude': 127.},
))
def test_bad_route_coordinate_invalidates_geometry_instead_of_bridging_it(point):
  assert coordinates([{'latitude': 37., 'longitude': 127.}, point]) == ()


def test_route_coordinate_count_is_bounded_and_native_coordinates_are_accepted():
  assert coordinates(route_dicts(MAPBOX)) == MAPBOX
  assert coordinates([SimpleNamespace(latitude=37., longitude=127.)]) == ((37., 127.),)
  assert coordinates([{'latitude': 37., 'longitude': 127.}] * (MAX_ROUTE_POINTS + 1)) == ()


@pytest.fixture
def route_manager(monkeypatch):
  from openpilot.selfdrive.carrot import carrot_man

  clock = SimpleNamespace(now=100., onroad=False)
  monkeypatch.setattr(carrot_man.time, 'monotonic', lambda: clock.now)

  def native_message(service, **kwargs):
    message = log.Event.new_message(valid=kwargs.get('valid', True))
    message.init(service)
    return message

  monkeypatch.setattr(carrot_man.messaging, 'new_message', native_message, raising=False)
  sent = []
  manager = carrot_man.CarrotMan.__new__(carrot_man.CarrotMan)
  manager.pm = SimpleNamespace(send=lambda name, message: sent.append((name, message)))
  manager.params = SimpleNamespace(get_bool=lambda _key: clock.onroad)
  manager.route_source = RouteSource()
  manager.route_source.mapbox = MAPBOX
  manager.carrot_serv = SimpleNamespace(
    carrot_navi_active=False, _external_navigation_connected=lambda: False,
    active_carrot=2, vpPosPointLon=127., vpPosPointLat=37., bearing=0.,
    nRoadLimitSpeed=80, autoNaviSpeedDecelRate=1., autoNaviSpeedCtrlEnd=2.,
  )
  manager.navi_points = []
  manager.navi_points_start_index = 0
  manager.navi_points_active = manager.navd_active = False
  manager.nav_route_next = 0.
  manager.broadcast_port = 7705
  manager.active_carrot_last = 0
  manager.sm = {'carState': car.CarState.new_message(vEgo=15.)}
  return SimpleNamespace(manager=manager, clock=clock, sent=sent)


def published_route(sent):
  assert sent[-1][0] == 'navRoute'
  return tuple((point.latitude, point.longitude) for point in sent[-1][1].navRoute.coordinates)


def assert_published_route(sent, expected):
  actual = published_route(sent)
  assert len(actual) == len(expected)
  for actual_point, expected_point in zip(actual, expected, strict=True):
    # navRoute's existing coordinates use float32, unlike the input control.
    assert actual_point == pytest.approx(expected_point, rel=0., abs=4e-6)


def test_carrot_man_native_route_publication_and_late_consumer_heartbeat(route_manager):
  manager, clock, sent = route_manager.manager, route_manager.clock, route_manager.sent
  manager._select_navigation_route(control())
  assert_published_route(sent, V2)
  assert manager.navi_points == [(lon, lat) for lat, lon in V2]
  assert manager.navi_points_active and manager.navd_active
  manager._select_navigation_route(control())
  assert len(sent) == 1
  clock.now += 2.01
  manager._select_navigation_route(control())
  assert len(sent) == 2
  assert_published_route(sent, V2)


def test_native_mapbox_route_is_cached_while_v2_owns_navigation(route_manager):
  manager, sent = route_manager.manager, route_manager.sent
  manager._select_navigation_route(control())
  later = ((36., 126.), (36.001, 126.))
  manager.send_routes(route_dicts(later), from_navd=True)
  assert len(sent) == 1
  manager._select_navigation_route(control())
  assert len(sent) == 1
  manager._select_navigation_route(None)
  assert_published_route(sent, later)


def test_stale_or_disconnected_v2_never_resurrects_expired_legacy_route(route_manager):
  manager, clock, sent = route_manager.manager, route_manager.clock, route_manager.sent
  manager.route_source.legacy = LEGACY
  message = navigation_message()

  class Sm:
    seen = alive = valid = {'carrotNavi': True}
    recv_time = {'carrotNavi': 100.}

    def __getitem__(self, _key):
      return message

  sm = Sm()
  manager._select_navigation_route(get_carrot_navi_control(sm, now=100.))
  assert_published_route(sent, V2)
  manager.carrot_serv.carrot_navi_active = True
  manager.carrot_serv._external_navigation_connected = lambda: True
  clock.now = 101.01
  manager._select_navigation_route(get_carrot_navi_control(sm, now=clock.now))
  assert_published_route(sent, MAPBOX)
  assert manager.route_source.legacy == ()
  message.connected = False
  sm.recv_time['carrotNavi'] = clock.now
  manager._select_navigation_route(get_carrot_navi_control(sm, now=clock.now))
  assert_published_route(sent, MAPBOX)


def test_route_received_offroad_remains_available_for_the_next_onroad_update(route_manager):
  manager = route_manager.manager
  manager._select_navigation_route(control(MAPBOX))
  before = list(manager.navi_points)
  assert manager.carrot_navi_route() == ([], [], 300)
  assert manager.navi_points == before
  assert manager.navi_points_active
  assert manager.route_source.selected == ('external', MAPBOX)


def test_discovery_ip_uses_the_interface_reaching_the_phone_network(route_manager, monkeypatch):
  from openpilot.selfdrive.carrot import carrot_man

  calls = []

  class DatagramSocket:
    def __enter__(self):
      return self

    def __exit__(self, *_args):
      pass

    def setsockopt(self, level, name, value):
      calls.append(('option', level, name, value))

    def connect(self, address):
      calls.append(('connect', address))

    def getsockname(self):
      return '192.168.50.154', 45000

  monkeypatch.setattr(carrot_man.socket, 'socket', lambda family, kind: DatagramSocket())
  manager = route_manager.manager
  assert manager.get_local_ip('192.168.50.255') == '192.168.50.154'
  assert ('option', carrot_man.socket.SOL_SOCKET, carrot_man.socket.SO_BROADCAST, 1) in calls
  assert ('connect', ('192.168.50.255', manager.broadcast_port)) in calls
  assert all(call != ('connect', ('8.8.8.8', manager.broadcast_port)) for call in calls)
