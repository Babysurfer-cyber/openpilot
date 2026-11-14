"""Queued lane-change request transport; no SPAS activation or parking ECU use."""
from types import SimpleNamespace as NS

import pytest

from cereal import log
from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags


@pytest.mark.parametrize('requested,current,expected', [
  ('left', 0, 3), ('right', 0, 4), ('left', 4, 3), ('right', 3, 4),
  ('none', 3, 3), ('none', 4, 4), ('none', 0, 0), ('none', 1, 0),
])
def test_request_preference_and_existing_current_change_fallback(requested, current, expected):
  md = NS(meta=NS(laneChangeBlinkerHold=requested))
  assert hyundaicanfd._lane_change_blink_hold(md, current, lateral_active=True, model_valid=True) == expected


@pytest.mark.parametrize('md,current,expected', [(None, 3, 0), (NS(meta=NS()), 3, 3), (NS(), 4, 4)])
def test_missing_model_or_older_metadata_is_handled(md, current, expected):
  assert hyundaicanfd._lane_change_blink_hold(md, current, lateral_active=True, model_valid=True) == expected


@pytest.mark.parametrize('active,valid', [(False, True), (True, False), (False, False)])
def test_inactive_or_invalid_model_releases_both_request_and_fallback(active, valid):
  md = NS(meta=NS(laneChangeBlinkerHold='left'))
  assert hyundaicanfd._lane_change_blink_hold(md, 4, lateral_active=active, model_valid=valid) == 0


@pytest.mark.parametrize('side', ['none', 'left', 'right'])
def test_real_model_metadata_serialization(side):
  md = log.ModelDataV2.new_message()
  assert str(md.meta.laneChangeBlinkerHold) == 'none'
  md.meta.laneChangeBlinkerHold = side
  md.meta.desire = 'laneChangeLeft'
  md.meta.desireState = [0., 0., 0., 0.95, 0.]
  with log.ModelDataV2.from_bytes(md.to_bytes()) as received:
    assert str(received.meta.laneChangeBlinkerHold) == side
    assert hyundaicanfd._lane_change_blink_hold(received, 3, lateral_active=True, model_valid=True) == {
      'none': 3, 'left': 3, 'right': 4,
    }[side]


def make_adrv(request, *, current=0, active=True, valid=True, compatibility=False):
  packer = CANPacker('hyundai_canfd_generated')
  stock_fields = {name: 0 for name in packer.dbc.name_to_msg['ADRV_0x1ea'].sigs}
  stock_fields.update(COUNTER=71, LEFT_BLINK_HOLD=1, RIGHT_BLINK_HOLD=1, NEW_SIGNAL_1=1)
  md = log.ModelDataV2.new_message()
  md.meta.laneChangeBlinkerHold = request
  md.meta.desireState = [0.] * 5
  if current:
    md.meta.desireState[current] = 0.95
  state = structs.CarState()
  state.vEgo = 25.
  state.gearShifter = 'drive'
  cs = NS(MD=md, out=state, scc_control=None, lfahda_cluster=None, adrv_0x161=None,
          adrv_0x200=None, adrv_0x1ea=stock_fields, ccnc_0x162=None)
  if not compatibility:
    cs.MD_valid = valid
  cp = NS(flags=HyundaiFlags.CAMERA_SCC.value, openpilotLongitudinalControl=False)
  control = structs.CarControl()
  control.latActive = active
  result = hyundaicanfd.create_ccnc_messages(cp, packer, NS(ECAN=0, CAM=2), 5, control, cs,
                                            control.hudControl, 0., 0, 0, False, 0, False, 0)
  assert len(result) == 1 and result[0][0] == 0x1EA and result[0][2] == 0
  parser = CANParser('hyundai_canfd_generated', [('ADRV_0x1ea', 0)], 0)
  parser.update([(1_000_000_000, result)])
  return result[0][1], parser.vl['ADRV_0x1ea']


@pytest.mark.parametrize('requested,current,active,valid,expected', [
  ('left', 0, True, True, (1, 0)), ('right', 0, True, True, (0, 1)),
  ('left', 4, True, True, (1, 0)), ('right', 3, True, True, (0, 1)),
  ('none', 3, True, True, (1, 0)), ('none', 4, True, True, (0, 1)),
  ('left', 4, False, True, (0, 0)), ('right', 3, True, False, (0, 0)),
])
def test_actual_adrv_message_carries_only_one_valid_hold_side(requested, current, active, valid, expected):
  _, fields = make_adrv(requested, current=current, active=active, valid=valid)
  assert (fields['LEFT_BLINK_HOLD'], fields['RIGHT_BLINK_HOLD']) == expected


def test_older_carstate_without_validity_attribute_retains_compatibility():
  _, fields = make_adrv('left', compatibility=True)
  assert (fields['LEFT_BLINK_HOLD'], fields['RIGHT_BLINK_HOLD']) == (1, 0)


def test_queued_transport_changes_only_hold_bits_and_checksum():
  baseline, _ = make_adrv('none')
  for side in ('left', 'right'):
    actual, _ = make_adrv(side)
    before, after = bytearray(baseline), bytearray(actual)
    before[:2] = after[:2] = bytes(2)
    before[3] &= ~0x60
    after[3] &= ~0x60
    assert before == after
