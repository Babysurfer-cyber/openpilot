"""Native navigation-control messages feed the existing TBT/SDI publication path."""

from types import SimpleNamespace

import pytest

from cereal import car, custom, log
from openpilot.selfdrive.carrot import carrot_serv


def metadata(sequence=1, *, present=True, received_ns=100_000_000_000):
  return {'present': present, 'sequence': sequence, 'receivedMonoTimeNanos': received_ns}


def navigation_message(*, turn_type=12, distance=20, sdi=False):
  return custom.CarrotNaviState.new_message(
    schemaVersion=1, connected=True, sessionId='navigation-session',
    navigationStatus={'meta': metadata(), 'guidanceActive': True},
    vehicle={'meta': metadata(), 'latitude': 37., 'longitude': 127.,
             'headingDeg': 90., 'speedKph': 54., 'roadName': 'Current road'},
    laneCurrent={'meta': metadata(), 'roadCategory': 1, 'roadCategoryValid': True},
    speed={'meta': metadata(), 'roadLimitValid': True, 'roadLimitKph': 80,
           'sdiPresent': sdi, 'sdiType': 1, 'sdiDistanceM': 100, 'sdiSpeedLimitKph': 60},
    guidanceCurrent={'meta': metadata(), 'turnType': turn_type, 'distanceM': distance,
                     'mainText': 'Current maneuver'},
    guidanceNext={'meta': metadata(present=False)},
    route={'meta': metadata(), 'remainingDistanceM': 4000, 'remainingTimeSec': 300,
           'polyline': [{'latitude': 37., 'longitude': 127.}, {'latitude': 37.001, 'longitude': 127.}]},
    trafficSignal={'meta': metadata(present=False)},
  )


class MemoryParams:
  def __init__(self, *_args, **_kwargs):
    self.values = {}

  def get_int(self, key):
    return {'AutoNaviSpeedBumpSpeed': 30, 'AutoNaviSpeedBumpTime': 2,
            'AutoNaviSpeedCtrlEnd': 2, 'AutoNaviSpeedCtrlMode': 1,
            'AutoNaviSpeedSafetyFactor': 100, 'AutoNaviSpeedDecelRate': 100,
            'AutoTurnControl': 2, 'AutoTurnControlSpeedTurn': 30,
            'AutoTurnControlTurnEnd': 5, 'AutoRoadSpeedLimitOffset': -1,
            'MyDrivingMode': 3}.get(key, 0)

  def get_float(self, key):
    return 100. if key == 'MapTurnSpeedFactor' else float(self.get_int(key))

  def get_bool(self, key):
    return key == 'IsMetric'

  def get(self, key, **_kwargs):
    return b'30' if key == 'AutoCurveSpeedLowerLimit' else self.values.get(key)

  def put_nonblocking(self, key, value):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class NavigationSubMaster:
  def __init__(self):
    self.data = {
      'carrotNavi': navigation_message(),
      'carState': car.CarState.new_message(vEgo=15., vEgoCluster=15.),
      'carControl': car.CarControl.new_message(enabled=True, latActive=True),
      'selfdriveState': log.SelfdriveState.new_message(distanceTraveled=0.),
      'modelV2': log.ModelDataV2.new_message(meta={'modelTurnSpeed': 250.}),
      'gpsLocationExternal': log.GpsLocationData.new_message(hasFix=False),
      'navInstruction': log.NavInstruction.new_message(),
    }
    self.seen = dict.fromkeys(self.data, True)
    self.alive = dict.fromkeys(self.data, True)
    self.valid = dict.fromkeys(self.data, True)
    self.updated = dict.fromkeys(self.data, True)
    self.recv_time = dict.fromkeys(self.data, 100.)
    self.alive['navInstruction'] = self.valid['navInstruction'] = False

  def __getitem__(self, name):
    return self.data[name]


def native_message(service):
  message = log.Event.new_message(valid=True)
  message.init(service)
  return message


@pytest.fixture
def navi_drive(monkeypatch):
  monkeypatch.setattr(carrot_serv, 'Params', MemoryParams)
  monkeypatch.setattr(carrot_serv.messaging, 'new_message', native_message, raising=False)
  clock = SimpleNamespace(now=100.)
  monkeypatch.setattr(carrot_serv.time, 'monotonic', lambda: clock.now)
  serv = carrot_serv.CarrotServ()
  sm = NavigationSubMaster()
  publications = {}
  publisher = SimpleNamespace(send=lambda name, message: publications.__setitem__(name, message))

  class Drive:
    def __init__(self):
      self.serv, self.sm, self.clock = serv, sm, clock
      self.publications = publications

    def update(self, now=100., *, distance=0., message=None, received=True):
      clock.now = now
      if message is not None:
        sm.data['carrotNavi'] = message
      sm.updated['carrotNavi'] = received
      if received:
        sm.recv_time['carrotNavi'] = now
      sm.data['selfdriveState'].distanceTraveled = distance
      serv.update_navi('127.0.0.1', sm, publisher, 250., (), (), 300., 'gpsLocationExternal')
      return publications['carrotMan'].carrotMan

  return Drive()


@pytest.mark.parametrize(('turn_type', 'distance', 'expected_type', 'turn_info'), (
  (12, 20, 'turn left', 1),
  (13, 20, 'turn right', 2),
  (7, 100, 'fork left', 3),
  (6, 100, 'fork right', 4),
))
def test_navigation_control_reaches_published_automatic_turn(navi_drive, turn_type, distance, expected_type, turn_info):
  result = navi_drive.update(message=navigation_message(turn_type=turn_type, distance=distance))
  assert result.atcType == expected_type
  assert result.xTurnInfo == turn_info
  assert result.xDistToTurn == distance
  assert result.activeCarrot > 1
  assert result.desiredSpeed < 200
  instruction = navi_drive.publications['navInstructionCarrot']
  assert instruction.valid
  assert instruction.navInstructionCarrot.maneuverPrimaryText == 'Current maneuver'


def test_navigation_control_speed_camera_reaches_published_deceleration(navi_drive):
  message = navigation_message(sdi=True)
  message.guidanceCurrent.meta.present = False
  navi_drive.sm.data['carState'].vEgo = navi_drive.sm.data['carState'].vEgoCluster = 100. / 3.6
  result = navi_drive.update(message=message)
  assert result.xSpdType == 1
  assert result.xSpdLimit == 60
  assert result.xSpdDist == 100
  assert result.desiredSource == 'CAM'
  assert 60 <= result.desiredSpeed < 100


@pytest.mark.parametrize('loss', ('invalid', 'stale', 'disconnected', 'empty_session', 'off_route', 'absent_items'))
def test_lost_or_withdrawn_navigation_clears_automatic_turn_and_sdi(navi_drive, loss):
  message = navigation_message(sdi=True)
  assert navi_drive.update(message=message).xTurnInfo == 1
  if loss == 'invalid':
    navi_drive.sm.valid['carrotNavi'] = False
  elif loss == 'disconnected':
    message.connected = False
  elif loss == 'empty_session':
    message.sessionId = ''
  elif loss == 'off_route':
    message.navigationStatus.offRoute = True
  elif loss == 'absent_items':
    message.guidanceCurrent.meta.present = message.speed.meta.present = False
  result = navi_drive.update(101.1 if loss == 'stale' else 100.1,
                             message=message, received=loss != 'stale')
  assert result.atcType == 'none'
  assert result.xTurnInfo == -1
  assert result.xSpdType == -1
  assert result.xSpdDist == 0


def test_heartbeats_do_not_rewind_distance_or_renew_gps_and_road_category(navi_drive):
  message = navigation_message(distance=100)
  first = navi_drive.update(message=message)
  assert first.xDistToTurn == 100
  assert first.roadCategoryValid
  assert navi_drive.serv.last_update_gps_time_navi == 100.
  second = navi_drive.update(100.5, distance=10., message=message)
  assert second.xDistToTurn == 90
  assert navi_drive.serv.last_update_gps_time_navi == 100.
  assert second.roadCategoryMonoTimeNanos == 100_000_000_000


def test_legacy_navigation_still_drives_the_existing_atc_path(navi_drive):
  navi_drive.sm.seen['carrotNavi'] = False
  serv = navi_drive.serv
  serv.active_count = serv.active_sdi_count = 80
  serv.nTBTTurnType, serv.nTBTDist = 12, 20
  serv._update_tbt()
  result = navi_drive.update(received=False)
  assert result.atcType == 'turn left'
  assert result.xTurnInfo == 1
  assert result.xDistToTurn == 20
  assert not serv.carrot_navi_active


def test_section_control_reaches_existing_section_speed_path(navi_drive):
  message = navigation_message()
  message.guidanceCurrent.meta.present = False
  message.speed.sectionPresent = message.speed.sectionActive = True
  message.speed.sectionSpeedLimitKph = 80
  message.speed.sectionRemainingDistanceM = 500
  result = navi_drive.update(message=message)
  assert result.xSpdType == 4
  assert result.xSpdLimit == 80
  assert result.xSpdDist == 500
  assert result.desiredSource == 'section'
  assert result.desiredSpeed == 80


def test_same_sequence_tombstone_and_reappearance_clear_and_restore_guidance(navi_drive):
  message = navigation_message()
  assert navi_drive.update(message=message).atcType == 'turn left'
  message.guidanceCurrent.meta.present = False
  assert navi_drive.update(100.1, message=message).atcType == 'none'
  message.guidanceCurrent.meta.present = True
  assert navi_drive.update(100.2, message=message).atcType == 'turn left'


def test_inactive_guidance_clears_atc_but_preserves_independent_speed_camera(navi_drive):
  message = navigation_message(sdi=True)
  assert navi_drive.update(message=message).atcType == 'turn left'
  message.navigationStatus.guidanceActive = False
  result = navi_drive.update(100.1, message=message)
  assert result.atcType == 'none'
  assert result.xTurnInfo == -1
  assert result.xSpdType == 1
  assert result.xSpdLimit == 60


def test_next_guidance_update_does_not_restore_travelled_current_distance(navi_drive):
  message = navigation_message(distance=100)
  message.guidanceNext.meta = metadata()
  message.guidanceNext.turnType = 13
  message.guidanceNext.distanceM = 200
  assert navi_drive.update(message=message).xDistToTurn == 100
  assert navi_drive.serv.xDistToTurnNext == 300
  assert navi_drive.update(100.1, distance=10., message=message).xDistToTurn == 90
  message.guidanceNext.meta.sequence = 2
  message.guidanceNext.distanceM = 150
  assert navi_drive.update(100.2, distance=10., message=message).xDistToTurn == 90
  assert navi_drive.serv.xDistToTurnNext == 240


def test_fresh_service_heartbeats_cannot_keep_expired_items_in_control(navi_drive):
  message = navigation_message(sdi=True)
  assert navi_drive.update(message=message).xTurnInfo == 1
  result = navi_drive.update(110.1, message=message)
  assert result.atcType == 'none'
  assert result.xTurnInfo == -1
  assert result.xSpdType == -1
  assert not result.roadCategoryValid
  assert navi_drive.serv.last_update_gps_time_navi == 0.


def test_legacy_packet_cannot_overwrite_active_v2_guidance_but_commands_still_arrive(navi_drive):
  assert navi_drive.update().atcType == 'turn left'
  serv = navi_drive.serv
  serv.update({'carrotIndex': 9, 'nRoadLimitSpeed': 30, 'nTBTTurnType': 13, 'nTBTDist': 200,
               'carrotCmd': 'SPEED', 'carrotArg': 'UP'})
  result = navi_drive.update(100.1)
  assert result.atcType == 'turn left'
  assert result.xDistToTurn == 20
  assert serv.nRoadLimitSpeed == 80
  assert result.carrotCmd == 'SPEED'
  assert result.carrotArg == 'UP'


def test_route_progress_remains_available_without_route_geometry(navi_drive):
  message = navigation_message()
  message.route.polyline = []
  result = navi_drive.update(message=message)
  assert result.atcType == 'turn left'
  assert result.nGoPosDist == 4000
  assert result.nGoPosTime == 300
  instruction = navi_drive.publications['navInstructionCarrot'].navInstructionCarrot
  assert instruction.distanceRemaining == 4000
  assert instruction.timeRemaining == 300


@pytest.mark.parametrize('section', (False, True))
def test_status_expiry_clears_speed_control_even_when_speed_sequence_is_still_fresh(navi_drive, section):
  message = navigation_message(sdi=True)
  if section:
    message.speed.sectionPresent = message.speed.sectionActive = True
    message.speed.sectionSpeedLimitKph = 80
    message.speed.sectionRemainingDistanceM = 500
  assert navi_drive.update(message=message).xSpdType == (4 if section else 1)
  # The receiver can refresh a speed item without refreshing navigation status.
  # Its unchanged sequence/presence must not preserve derived SDI permission.
  message.speed.meta.receivedMonoTimeNanos = 103_100_000_000
  result = navi_drive.update(103.1, message=message)
  assert result.xSpdType == -1
  assert result.xSpdLimit == 0
  assert result.xSpdDist == 0


def test_lane_category_item_expiry_is_not_extended_by_other_fresh_items(navi_drive):
  message = navigation_message()
  assert navi_drive.update(message=message).roadCategoryValid
  for item in (message.navigationStatus, message.speed, message.guidanceCurrent):
    item.meta.receivedMonoTimeNanos = 103_100_000_000
  result = navi_drive.update(103.1, message=message)
  assert result.activeCarrot > 1
  assert not result.roadCategoryValid
  assert result.roadCategoryMonoTimeNanos == 0


@pytest.mark.parametrize(('road_limit', 'distance_m'), (
  (0, 160.), (30, 160.), (40, 180.), (50, 200.),
  (60, 230.), (80, 290.), (100, 350.), (120, 350.),
))
def test_atc_hud_guide_start_distance_matches_deceleration_window(road_limit, distance_m):
  # The value for the HUD comes from the exact same helper as ATC's
  # speed-dependent "prepare" transition, not the AUTO mode display label.
  assert carrot_serv.CarrotServ.atc_guide_start_distance(road_limit) == distance_m


def test_atc_hud_guide_window_is_published_from_live_road_limit(navi_drive):
  message = navigation_message(turn_type=13, distance=200)
  message.speed.roadLimitKph = 80
  result = navi_drive.update(message=message)
  assert result.atcGuideStartDistanceM == 290.
  assert result.xTurnInfo == 2
  assert result.xDistToTurn == 200


def test_mode4_one_shot_current_atc_skip_and_next_guidance_restore(navi_drive, monkeypatch):
  # Integration check: the short-click event reaches CarrotServ via the
  # ephemeral request, silences only the current ATC speed/type source,
  # and a new navigation item is eligible again.
  serv, sm = navi_drive.serv, navi_drive.sm
  orig_get_int = serv.params.get_int
  serv.params.get_int = lambda key: 4 if key == 'PaddleMode' else orig_get_int(key)
  serv.params.get_bool = lambda key: key in ('IsMetric', 'AutoTurnControlEnabled')
  sm.data['carState'].cruiseState.available = True
  sm.data['carState'].vCruiseCluster = 15.0
  monkeypatch.setattr(carrot_serv.time, 'monotonic_ns', lambda: 100_000_000_000)

  current = navigation_message(turn_type=13, distance=200)
  normal = navi_drive.update(message=current)
  assert normal.xTurnInfo == 2
  assert normal.atcType in ('atc right', 'turn right')
  assert not serv.atc_one_shot_skip.skipped

  serv.params_memory.put_nonblocking('AtcSkipRequest', '99990000000')
  skipped = navi_drive.update(message=current)
  assert serv.atc_one_shot_skip.skipped
  assert skipped.atcType == 'none'
  assert skipped.desiredSource != 'atc'

  skipped_again = navi_drive.update(message=current, distance=10.)
  assert serv.atc_one_shot_skip.skipped
  assert skipped_again.atcType == 'none'

  next_turn = navigation_message(turn_type=13, distance=250)
  next_turn.guidanceCurrent.meta.sequence = 2
  restored = navi_drive.update(message=next_turn, distance=10.)
  assert not serv.atc_one_shot_skip.skipped
  assert restored.atcType in ('atc right', 'turn right')


def test_mode4_atc_request_rejected_after_deceleration_has_started(navi_drive, monkeypatch):
  serv, sm = navi_drive.serv, navi_drive.sm
  orig_get_int = serv.params.get_int
  serv.params.get_int = lambda key: 4 if key == 'PaddleMode' else orig_get_int(key)
  serv.params.get_bool = lambda key: key in ('IsMetric', 'AutoTurnControlEnabled')
  sm.data['carState'].cruiseState.available = True
  sm.data['carState'].vCruiseCluster = 40.0  # 144 km/h, ATC target is lower
  monkeypatch.setattr(carrot_serv.time, 'monotonic_ns', lambda: 100_000_000_000)

  current = navigation_message(turn_type=13, distance=200)
  navi_drive.update(message=current)
  serv.params_memory.put_nonblocking('AtcSkipRequest', '99990000000')
  result = navi_drive.update(message=current)
  assert not serv.atc_one_shot_skip.skipped
  assert result.atcType != 'none'
