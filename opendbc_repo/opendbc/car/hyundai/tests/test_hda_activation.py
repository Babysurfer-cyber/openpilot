"""Stock HDA recovery requests respect stop/fault states and vehicle navigation."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags


@pytest.fixture(autouse=True)
def settings(monkeypatch):
  class MemoryParams:
    def get_int(self, key):
      return 0

  monkeypatch.setattr(hyundaicanfd, 'Params', MemoryParams)
  monkeypatch.setattr(hyundaicanfd.create_ccnc_messages, '_lane_line_check', 0, raising=False)


def vehicle(*, speed=20., main=False, acc_mode=0, info=0, lfa=0):
  state = structs.CarState()
  state.canValid = True
  state.gearShifter = 'drive'
  state.vEgo = speed
  return NS(
    out=state, MD=None, MainMode_ACC=main, ACCMode=acc_mode,
    scc_control={'InfoDisplay': info},
    lfahda_cluster={'HDA_CntrlModSta': 0, 'HDA_LFA_SymSta': lfa,
                   'HDA_OptUsmSta': 2, 'HDA_InfoPUDis': 0},
    cruise_btns_msg_canfd='CRUISE_BUTTONS',
    cruise_buttons_msg={'COUNTER': 7, 'SET_ME_1': 1, 'LFA_BTN': 0, 'CRUISE_BUTTONS': 0,
                        'ADAPTIVE_CRUISE_MAIN_BTN': 0, 'NORMAL_CRUISE_MAIN_BTN': 0},
    adrv_0x161=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    navi_segment_4b9=None,
  )


def emit(cs, *, frame=12, enabled=True, corner=2, stopping=False):
  cp = NS(flags=HyundaiFlags.CAMERA_SCC.value, openpilotLongitudinalControl=True)
  cc = structs.CarControl()
  cc.enabled = enabled
  return hyundaicanfd.create_ccnc_messages(
    cp, CANPacker('hyundai_canfd_generated'), NS(CAM=2, ECAN=0), frame,
    cc, cs, cc.hudControl, 0., False, False, False, corner, stopping, 0,
  )


def buttons(messages):
  parser = CANParser('hyundai_canfd_generated', [('CRUISE_BUTTONS', 0)], 2)
  selected = [message for message in messages if message[0] == 0x1CF]
  assert len(selected) == 1
  parser.update([(1_000_000_000, selected)])
  values = parser.vl['CRUISE_BUTTONS']
  return {name: values[name] for name in
          ('LFA_BTN', 'CRUISE_BUTTONS', 'ADAPTIVE_CRUISE_MAIN_BTN', 'NORMAL_CRUISE_MAIN_BTN')}


def navigation(messages):
  return [message for message in messages if message[0] == 0x4B9]


def apply_condition(cs, condition):
  if condition in ('park', 'neutral', 'reverse'):
    cs.out.gearShifter = condition
  elif condition == 'stopped':
    cs.out.vEgo = 0.
    cs.out.standstill = True
  elif condition == 'low_speed':
    cs.out.vEgo = 3.
  elif condition == 'invalid_can':
    cs.out.canValid = False
  elif condition == 'nan':
    cs.out.vEgo = float('nan')
  elif condition == 'infinite':
    cs.out.vEgo = float('inf')
  elif condition == 'brake':
    cs.out.brakePressed = True
  elif condition == 'missing_cluster':
    cs.lfahda_cluster = None
  elif condition == 'hda_option_fail':
    cs.lfahda_cluster['HDA_OptUsmSta'] = 7
  elif condition == 'hda_popup_fail':
    cs.lfahda_cluster['HDA_InfoPUDis'] = 4
  elif condition == 'ccnc_fail':
    packer = CANPacker('hyundai_canfd_generated')
    cs.ccnc_0x162 = {name: 0 for name in packer.dbc.name_to_msg['CCNC_0x162'].sigs}
    cs.ccnc_0x162['FAULT_HDA'] = 1
  else:
    raise AssertionError(condition)


@pytest.mark.parametrize('activation', ['lfa', 'main', 'set', 'navigation'])
@pytest.mark.parametrize('condition', [
  'park', 'neutral', 'reverse', 'stopped', 'low_speed', 'invalid_can', 'nan', 'infinite',
  'brake', 'missing_cluster', 'hda_option_fail', 'hda_popup_fail', 'ccnc_fail',
])
def test_unavailable_vehicle_state_does_not_inject_activation(activation, condition):
  cs = vehicle(main=activation == 'set')
  apply_condition(cs, condition)
  frame = {'lfa': 2, 'main': 12, 'set': 12, 'navigation': 20}[activation]
  messages = emit(cs, frame=frame)
  assert not any(buttons(messages).values())
  assert not navigation(messages)


@pytest.mark.parametrize('activation,expected_frames', [
  ('lfa', [2, 4, 6, 8, 10]), ('main', [12, 14, 16]), ('set', [12, 14, 16]),
])
def test_healthy_moving_activation_keeps_original_button_windows(activation, expected_frames):
  cs = vehicle(main=activation == 'set', lfa=0 if activation == 'lfa' else 1)
  field = {'lfa': 'LFA_BTN', 'main': 'ADAPTIVE_CRUISE_MAIN_BTN', 'set': 'CRUISE_BUTTONS'}[activation]
  selected = []
  for frame in range(0, 200, 2):
    if buttons(emit(cs, frame=frame))[field]:
      selected.append(frame)
  assert selected == expected_frames


@pytest.mark.parametrize('enabled', [False, True])
def test_healthy_moving_main_recovery_works_with_corner_standby_or_active_control(enabled):
  cs = vehicle(lfa=1)
  assert buttons(emit(cs, enabled=enabled))['ADAPTIVE_CRUISE_MAIN_BTN'] == 1


def test_corner_disabled_does_not_enable_main_in_standby():
  assert not any(buttons(emit(vehicle(lfa=1), enabled=False, corner=0)).values())


def test_healthy_absent_vehicle_navigation_keeps_the_original_six_frame_pattern():
  cs = vehicle(lfa=1)
  results = [(frame, navigation(emit(cs, frame=frame))) for frame in range(500)]
  sent = [(frame, messages[0]) for frame, messages in results if messages]
  assert [frame for frame, _ in sent] == [10, 20, 30, 40, 50, 60]
  assert all(message[2] == 2 for _, message in sent)
  assert [message[1] for _, message in sent[:3]] == [bytes.fromhex('0000808a32300100')] * 3
  assert [message[1] for _, message in sent[3:]] == [bytes.fromhex('ffffffffffffffff')] * 3


@pytest.mark.parametrize('fill', [0, 255])
def test_existing_vehicle_navigation_even_all_ones_blocks_fake_route_overwrite(fill):
  cs = vehicle()
  cs.navi_segment_4b9 = {f'BYTE_{index}': fill for index in range(1, 9)}
  source = deepcopy(cs.navi_segment_4b9)
  assert not any(navigation(emit(cs, frame=frame)) for frame in (10, 20, 30, 40, 50, 60))
  assert cs.navi_segment_4b9 == source


def test_fault_recovery_allows_new_moving_requests_without_mutating_stock_state():
  cs = vehicle(lfa=1)
  cs.lfahda_cluster['HDA_OptUsmSta'] = 7
  source = deepcopy(cs.lfahda_cluster)
  assert not navigation(emit(cs, frame=20))
  assert cs.lfahda_cluster == source
  cs.lfahda_cluster['HDA_OptUsmSta'] = 2
  assert navigation(emit(cs, frame=20))
  assert buttons(emit(cs))['ADAPTIVE_CRUISE_MAIN_BTN'] == 1


@pytest.mark.parametrize('stopping,braking,enabled,expected', [
  (False, False, True, 2), (True, False, True, 0),
  (False, True, True, 0), (False, False, False, 0),
])
def test_real_standstill_stop_and_go_resume_is_preserved(stopping, braking, enabled, expected):
  cs = vehicle(speed=0., main=True, acc_mode=1, info=4, lfa=1)
  cs.out.standstill = True
  cs.out.brakePressed = braking
  assert buttons(emit(cs, stopping=stopping, enabled=enabled))['CRUISE_BUTTONS'] == expected


@pytest.mark.parametrize('fault', ['hda_option_fail', 'hda_popup_fail', 'ccnc_fail', 'invalid_can', 'park'])
def test_standstill_resume_does_not_override_a_fault_or_unavailable_car(fault):
  cs = vehicle(speed=0., main=True, acc_mode=1, info=4, lfa=1)
  apply_condition(cs, fault)
  assert not any(buttons(emit(cs)).values())


@pytest.mark.parametrize('field,value', [
  ('CRUISE_BUTTONS', 1), ('CRUISE_BUTTONS', 4),
  ('ADAPTIVE_CRUISE_MAIN_BTN', 1), ('NORMAL_CRUISE_MAIN_BTN', 1),
])
@pytest.mark.parametrize('frame', [2, 12])
def test_driver_buttons_are_forwarded_without_automatic_activation(field, value, frame):
  cs = vehicle()
  cs.cruise_buttons_msg[field] = value
  source = deepcopy(cs.cruise_buttons_msg)
  values = buttons(emit(cs, frame=frame))
  assert values[field] == value
  assert all(v == 0 for k, v in values.items() if k != field)
  assert cs.cruise_buttons_msg == source


def test_existing_physical_lfa_suppression_does_not_trigger_an_automatic_replacement():
  cs = vehicle()
  cs.cruise_buttons_msg['LFA_BTN'] = 1
  source = deepcopy(cs.cruise_buttons_msg)
  assert not any(buttons(emit(cs, frame=2)).values())
  assert cs.cruise_buttons_msg == source


def test_reported_hda_fault_popup_is_preserved_with_a_valid_cluster_checksum():
  packer = CANPacker('hyundai_canfd_generated')
  source = {name: 0 for name in packer.dbc.name_to_msg['LFAHDA_CLUSTER'].sigs}
  source.update(COUNTER=170, HDA_OptUsmSta=7, HDA_InfoPUDis=4, HDA_LFA_SymSta=1)
  before = deepcopy(source)
  cs = vehicle(speed=0.)
  cs.lfahda_cluster = source
  messages = hyundaicanfd.create_lfahda_cluster(packer, cs, NS(ECAN=0), False, False, structs.CarControl().hudControl)
  assert len(messages) == 1
  parser = CANParser('hyundai_canfd_generated', [('LFAHDA_CLUSTER', 0)], 0)
  parser.update([(1_000_000_000, messages)])
  values = parser.vl['LFAHDA_CLUSTER']
  assert values['HDA_OptUsmSta'] == 7 and values['HDA_InfoPUDis'] == 4
  assert int.from_bytes(messages[0][1][:2], 'little') == hyundaicanfd.hkg_can_fd_checksum(
    0x1E0, None, bytearray(messages[0][1]),
  )
  assert source == before


def test_reported_ccnc_hda_fault_is_not_hidden_by_the_recovery_guard():
  cs = vehicle()
  apply_condition(cs, 'ccnc_fail')
  source = deepcopy(cs.ccnc_0x162)
  messages = emit(cs, frame=20)
  selected = [message for message in messages if message[0] == 0x162]
  assert len(selected) == 1 and not navigation(messages)
  parser = CANParser('hyundai_canfd_generated', [('CCNC_0x162', 0)], 0)
  parser.update([(1_000_000_000, selected)])
  assert parser.vl['CCNC_0x162']['FAULT_HDA'] == 1
  assert int.from_bytes(selected[0][1][:2], 'little') == hyundaicanfd.hkg_can_fd_checksum(
    0x162, None, bytearray(selected[0][1]),
  )
  assert cs.ccnc_0x162 == source
