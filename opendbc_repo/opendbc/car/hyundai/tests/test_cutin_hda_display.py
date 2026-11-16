"""Confirmed CUT-IN -> real controller receipt gates -> ADRV cluster field."""
from types import SimpleNamespace as NS

import pytest

from cereal import log
from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller, hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags


def lead(source='corner180', track_id=1304, **changes):
  result = dict(status=True, radar=True, radarSource=source, radarTrackId=track_id,
                dRel=25., yRel=1.5, vRel=-2., vLead=23., vLeadK=23., score=.8)
  result.update(changes)
  return result


def radar_state(*, confirmed=(), legacy=False, risk=False, ordinary=None):
  state = log.RadarState.new_message()
  state.leadOne = lead('frontRadar', 61, yRel=0.)
  if legacy:
    state.leadOne = lead('frontRadar', -1)
  state.leadsCutIn = list(confirmed)
  if risk:
    state.leadCutInRisk = lead(score=.9)
  if ordinary is not None:
    if ordinary == 'leadTwo':
      state.leadTwo = lead()
    else:
      setattr(state, ordinary, [lead()])
  return state


def detected(state, *, valid=True, received=1., now=1.1):
  return hyundaicanfd.radar_cutin_detected(state, valid=valid, received_s=received, now_s=now)


@pytest.fixture(autouse=True)
def isolated_settings(monkeypatch):
  class MemoryParams:
    def get_int(self, key):
      return 0

  monkeypatch.setattr(hyundaicanfd, 'Params', MemoryParams)
  for name, value in (('_lane_line_check', 0), ('_blinker_timer', 0),
                      ('_lever_max_level', 0), ('_lane_changed_this_session', False)):
    monkeypatch.setattr(hyundaicanfd.create_ccnc_messages, name, value, raising=False)
  monkeypatch.setattr(hyundaicanfd._make_ccnc_values, '_filt_curve', 0., raising=False)


@pytest.mark.parametrize('source,identifier', [('corner180', 1304), ('corner235', 2501), ('frontRadar', 39)])
def test_confirmed_event_survives_real_message_serialization_without_a_corner_lead_one(source, identifier):
  state = radar_state(confirmed=[lead(source, identifier)])
  with log.RadarState.from_bytes(state.to_bytes()) as received:
    assert received.leadOne.radarSource == 'frontRadar'
    assert received.leadOne.radarTrackId == 61
    assert not received.leadTwo.status
    assert detected(received)


def test_old_distance_based_corner_lead_remains_supported():
  assert detected(radar_state(legacy=True))


@pytest.mark.parametrize('observation', ['leadTwo', 'leadsCenter', 'leadsLeft', 'leadsRight'])
def test_an_ordinary_corner_observation_is_not_a_cutin_event(observation):
  assert not detected(radar_state(ordinary=observation))


def test_unconfirmed_early_risk_alone_does_not_show_the_confirmed_cutin_icon():
  assert not detected(radar_state(risk=True))


def test_no_event_does_not_show_the_icon():
  assert not detected(radar_state())


@pytest.mark.parametrize('legacy', [False, True])
@pytest.mark.parametrize('changes', [
  {'status': False}, {'radar': False}, {'dRel': 0.}, {'dRel': .1}, {'dRel': -1.},
  {'dRel': float('nan')}, {'dRel': float('inf')},
  {'vRel': float('nan')}, {'vRel': float('inf')},
])
def test_invalid_candidate_never_activates_the_display(legacy, changes):
  candidate = lead(track_id=-1 if legacy else 1304, **changes)
  state = radar_state(confirmed=[] if legacy else [candidate])
  if legacy:
    state.leadOne = candidate
  assert not detected(state)


@pytest.mark.parametrize('valid,received,now', [
  (False, 1., 1.1), (True, 1., 1.251), (True, 1.01, 1.),
  (True, 0., .1), (True, -1., -.9),
  (True, float('nan'), 1.), (True, float('inf'), 1.),
  (True, 1., float('nan')), (True, 1., float('inf')),
])
def test_invalid_stale_future_or_missing_receipt_clears_even_a_confirmed_event(valid, received, now):
  assert not detected(radar_state(confirmed=[lead()]), valid=valid, received=received, now=now)


@pytest.mark.parametrize('error', ['canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'])
def test_radar_faults_suppress_confirmed_cutin_display(error):
  state = radar_state(confirmed=[lead()])
  setattr(state.radarErrors, error, True)
  assert not detected(state)


class PrefixComplete(Exception):
  pass


class BeforeControlCalculation:
  """Stop after radar receipt processing, before unrelated steering work."""
  @property
  def actuators(self):
    raise PrefixComplete


class RadarSubMaster:
  def __init__(self, state):
    self.state = state
    self.model = log.ModelDataV2.new_message()
    self.valid = {'radarState': True, 'modelV2': True}
    self.alive = {'radarState': True, 'modelV2': True}
    self.updated = {'radarState': True, 'modelV2': True}
    self.recv_time = {'radarState': 1., 'modelV2': 1.}
    # Receipt timestamps use local MONOTONIC, not the publisher's clock.
    self.logMonoTime = {'radarState': 900_000_000_000, 'modelV2': 900_000_000_000}
    self.polls = []

  def update(self, timeout):
    self.polls.append(timeout)

  def __getitem__(self, service):
    return self.state if service == 'radarState' else self.model


def controller_prefix(monkeypatch, state):
  controller = carcontroller.CarController.__new__(carcontroller.CarController)
  controller.frame = 1  # Avoid the unrelated parameter polling branches.
  controller.blinker_runtime = None
  controller.blinker_test_request = 0
  controller.blinker_test_status = 'unsupported'
  controller.corner_radar_cutin = False
  controller.sm = RadarSubMaster(state)
  cs = NS(out=structs.CarState())
  cs.out.canValid = True
  clock = NS(now=1.1)
  monkeypatch.setattr(carcontroller.time, 'monotonic', lambda: clock.now)

  def update():
    with pytest.raises(PrefixComplete):
      controller.update(BeforeControlCalculation(), cs, 1_100_000_000)
    return controller.corner_radar_cutin

  return controller, cs, clock, update


def wire_packet(flag, *, bsd=False):
  packer = CANPacker('hyundai_canfd_generated')
  stock = {name: 0 for name in packer.dbc.name_to_msg['ADRV_0x1ea'].sigs}
  stock.update(COUNTER=71, NEW_SIGNAL_1=1)
  model = log.ModelDataV2.new_message()
  model.meta.desireState = [0.] * 8
  out = structs.CarState()
  out.canValid = True
  out.gearShifter = 'drive'
  out.vEgo = 25.
  if bsd:
    out.leftBlinker = out.leftBlindspot = True
    out.blinkerLever = 1
    hyundaicanfd.create_ccnc_messages._blinker_timer = 25
    hyundaicanfd.create_ccnc_messages._lever_max_level = 1
  cs = NS(MD=model, MD_valid=True, out=out, scc_control=None, lfahda_cluster=None,
          adrv_0x161=None, adrv_0x200=None, adrv_0x1ea=stock, ccnc_0x162=None)
  control = structs.CarControl()
  control.latActive = True
  cp = NS(flags=HyundaiFlags.CAMERA_SCC.value, openpilotLongitudinalControl=False)
  messages = hyundaicanfd.create_ccnc_messages(
    cp, packer, NS(ECAN=0, CAM=2), 5, control, cs, control.hudControl,
    0., 0, 0, flag, 2, False, 0,
  )
  assert len(messages) == 1 and messages[0][0] == 0x1EA and messages[0][2] == 0
  parser = CANParser('hyundai_canfd_generated', [('ADRV_0x1ea', 0)], 0)
  parser.update([(1_000_000_000, messages)])
  return messages[0][1], parser.vl['ADRV_0x1ea']


@pytest.mark.parametrize('source', ['corner180', 'corner235', 'frontRadar'])
def test_controller_receipt_to_actual_cluster_can_and_event_clear(monkeypatch, source):
  controller, cs, clock, update = controller_prefix(monkeypatch, radar_state(confirmed=[lead(source)]))
  assert update()
  assert cs.radarState is controller.sm.state
  assert controller.sm.polls == [0]
  assert wire_packet(controller.corner_radar_cutin)[1]['HDA_MODE2'] == 4
  controller.sm.state = radar_state()
  clock.now = controller.sm.recv_time['radarState'] = 1.15
  assert not update()
  assert wire_packet(controller.corner_radar_cutin)[1]['HDA_MODE2'] == 1


def test_controller_clears_stale_state_without_waiting_for_another_radar_update(monkeypatch):
  controller, cs, clock, update = controller_prefix(monkeypatch, radar_state(confirmed=[lead()]))
  assert update()
  controller.sm.updated['radarState'] = False
  clock.now = 1.2
  assert update()  # Normal gaps between 20Hz messages retain a still-fresh event.
  clock.now = 1.251
  assert not update()
  assert wire_packet(controller.corner_radar_cutin)[1]['HDA_MODE2'] == 1


@pytest.mark.parametrize('fault', ['invalid', 'not_alive', 'can_invalid'])
def test_controller_retracts_previous_warning_on_feed_or_vehicle_fault(monkeypatch, fault):
  controller, cs, clock, update = controller_prefix(monkeypatch, radar_state(confirmed=[lead()]))
  assert update()
  controller.sm.updated['radarState'] = False
  if fault == 'invalid':
    controller.sm.valid['radarState'] = False
  elif fault == 'not_alive':
    controller.sm.alive['radarState'] = False
  else:
    cs.out.canValid = False
  assert not update()


@pytest.mark.parametrize('flag,bsd,expected', [(True, False, 4), (True, True, 4),
                                             (False, True, 2), (False, False, 1)])
def test_cutin_icon_priority_and_existing_bsd_normal_fallback(flag, bsd, expected):
  assert wire_packet(flag, bsd=bsd)[1]['HDA_MODE2'] == expected


def test_cutin_display_changes_only_hda_mode2_bits_and_checksum():
  plain, plain_fields = wire_packet(False)
  alert, alert_fields = wire_packet(True)
  assert plain_fields['HDA_MODE2'] == 1 and alert_fields['HDA_MODE2'] == 4
  plain, alert = bytearray(plain), bytearray(alert)
  plain[:2] = alert[:2] = bytes(2)
  plain[4] &= ~0x07
  alert[4] &= ~0x07
  assert plain == alert


@pytest.mark.parametrize('source', ['corner180', 'corner235'])
def test_actual_trajectory_detector_to_controller_and_cluster_can(monkeypatch, source):
  from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
  from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import Point, model_with_lead

  detector = DPathRadarController(prefer_corner_radar=True)
  controller, cs, clock, update = controller_prefix(monkeypatch, radar_state())
  confirmed_frames, early_only_frames = [], []
  for index in range(36):
    elapsed = index * .05
    distance, lateral = 8. - 1.7 * elapsed, -3.30 + .70 * elapsed
    output = detector.update(
      time_s=elapsed, v_ego=7.9,
      radar_points=(
        Point(35, 25., 0., v_rel=0., source='frontRadar'),
        Point(45, distance, lateral, v_rel=-1.7, source='frontRadar'),
        Point(3504, distance - .1, lateral - .1, v_rel=-1.6, yv_rel=.70,
              source=source, trackState=2),
      ),
      model=model_with_lead(25., 0., 7.9),
    )
    state = log.RadarState.new_message()
    state.leadOne = output.lead_one
    if output.lead_two is not None:
      state.leadTwo = output.lead_two
    state.leadsCutIn = list(output.leads_cutin)
    if output.lead_cutin_risk is not None:
      state.leadCutInRisk = output.lead_cutin_risk
    controller.sm.state = state
    clock.now = controller.sm.recv_time['radarState'] = 1.1 + elapsed
    flag = update()
    assert state.leadOne.radarSource == 'frontRadar'
    assert state.leadOne.radarTrackId == 35
    assert flag == bool(output.leads_cutin)
    assert wire_packet(flag)[1]['HDA_MODE2'] == (4 if output.leads_cutin else 1)
    if output.leads_cutin:
      assert any(item['radarSource'] == source and item['radarTrackId'] == 3504
                 for item in output.leads_cutin)
      confirmed_frames.append(elapsed)
    elif output.lead_cutin_risk is not None:
      early_only_frames.append(elapsed)
  assert confirmed_frames and confirmed_frames[0] <= 1.
  assert early_only_frames
  controller.sm.state = radar_state()
  clock.now += .05
  controller.sm.recv_time['radarState'] = clock.now
  assert not update()
  assert wire_packet(controller.corner_radar_cutin)[1]['HDA_MODE2'] == 1
