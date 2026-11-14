"""Real FSM -> production metadata assignments -> Cap'n Proto -> ADRV CAN.

Native model inference, IPC scheduling and the physical EV6 are outside this
simulation. Only the two existing model/controller metadata boundaries are
extracted; the lane-change FSM, schema, CAN packing and parsing are real.
"""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from cereal import log
from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.selfdrive.controls.lib import desire_helper as desire
from openpilot.selfdrive.controls.tests.test_desire_continuation import MemoryParams, Rig


ROOT = Path(__file__).resolve().parents[3]


def assignment_subset(path, names):
  tree = ast.parse((ROOT / path).read_text())
  nodes = [node for node in ast.walk(tree) if isinstance(node, ast.Assign)
           and len(node.targets) == 1 and ast.unparse(node.targets[0]) in names]
  assert {ast.unparse(node.targets[0]) for node in nodes} == names
  assert len(nodes) == len(names)
  return compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), path, 'exec')


PUBLISH = assignment_subset('selfdrive/modeld/modeld.py', {
  'modelv2_send.modelV2.meta.laneChangeState',
  'modelv2_send.modelV2.meta.laneChangeDirection',
  'modelv2_send.modelV2.meta.laneChangeBlinkerHold',
  'modelv2_send.modelV2.meta.desire',
  'modelv2_send.modelV2.meta.laneChangeProb',
})
RECEIVE = assignment_subset('opendbc_repo/opendbc/car/hyundai/carcontroller.py', {'CS.MD', 'CS.MD_valid'})


@pytest.fixture(autouse=True)
def no_device_settings(monkeypatch):
  monkeypatch.setattr(desire, 'Params', MemoryParams)
  monkeypatch.setattr(hyundaicanfd, 'Params', MemoryParams)
  monkeypatch.setattr(hyundaicanfd._make_ccnc_values, '_filt_curve', 0., raising=False)
  monkeypatch.setattr(hyundaicanfd.create_ccnc_messages, '_lane_line_check', 0, raising=False)


class ModelTransport:
  def __init__(self, model, valid, alive):
    self.model = model
    self.valid = {'modelV2': valid}
    self.alive = {'modelV2': alive}

  def __getitem__(self, key):
    assert key == 'modelV2'
    return self.model


def wire_packet(rig, *, active=True, valid=True, alive=True, override_hold=None):
  event = log.Event.new_message()
  event.valid = valid
  event.init('modelV2')
  event.modelV2 = rig.md.as_reader()
  exec(PUBLISH, {'modelv2_send': event, 'DH': rig.h})
  if override_hold is not None:
    event.modelV2.meta.laneChangeBlinkerHold = override_hold

  with log.Event.from_bytes(event.to_bytes()) as received:
    packer = CANPacker('hyundai_canfd_generated')
    stock = {name: 0 for name in packer.dbc.name_to_msg['ADRV_0x1ea'].sigs}
    stock.update(COUNTER=73, NEW_SIGNAL_1=1)
    cs = NS(out=rig.cs, scc_control=None, lfahda_cluster=None, adrv_0x161=None,
            adrv_0x200=None, adrv_0x1ea=stock, ccnc_0x162=None)
    exec(RECEIVE, {'self': NS(sm=ModelTransport(received.modelV2, received.valid, alive)), 'CS': cs})
    control = structs.CarControl()
    control.latActive = active
    cp = NS(flags=HyundaiFlags.CAMERA_SCC.value, openpilotLongitudinalControl=False)
    messages = hyundaicanfd.create_ccnc_messages(
      cp, packer, NS(ECAN=0, CAM=2), 5, control, cs, control.hudControl,
      0., 0, 0, False, 0, False, 0,
    )
    hold = str(received.modelV2.meta.laneChangeBlinkerHold)

  # The normal indication path never uses SPAS, diagnostic ECU requests or the
  # parked experimental mode. No other CAN address is introduced here.
  assert [address for address, _, _ in messages] == [0x1EA]
  parser = CANParser('hyundai_canfd_generated', [('ADRV_0x1ea', 0)], 0)
  parser.update([(1_000_000_000, messages)])
  fields = parser.vl['ADRV_0x1ea']
  return hold, (fields['LEFT_BLINK_HOLD'], fields['RIGHT_BLINK_HOLD']), messages[0][1]


@pytest.mark.parametrize('side,expected', [('left', (1, 0)), ('right', (0, 1))])
def test_physical_tap_request_survives_publication_and_queued_handoff(side, expected):
  rig = Rig(side)
  rig.queue()
  # Model confidence itself is zero in this simulation; only the explicit
  # request supplies the signal, exercising the new transport rather than the
  # old >0.9 model-probability fallback.
  assert all(value == 0 for value in rig.md.meta.desireState)
  assert wire_packet(rig)[:2] == (side, expected)
  assert rig.finish() == log.LaneChangeState.preLaneChange
  assert wire_packet(rig)[:2] == (side, expected)
  assert rig.step() == log.LaneChangeState.laneChangeStarting
  assert not rig.h.queued_lane_change
  assert wire_packet(rig)[:2] == (side, expected)
  rig.geometry()
  rig.step()
  assert wire_packet(rig)[:2] == (side, expected)
  rig.cross()
  assert wire_packet(rig)[:2] == ('none', (0, 0))


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('condition', ['invalid_envelope', 'stale_model', 'inactive_lateral'])
def test_received_stale_or_disabled_control_withdraws_the_explicit_hold(side, condition):
  rig = Rig(side)
  rig.queue()
  flags = {'valid': condition != 'invalid_envelope', 'alive': condition != 'stale_model',
           'active': condition != 'inactive_lateral'}
  hold, fields, _ = wire_packet(rig, **flags)
  assert hold == side  # Serialized request is unchanged; the receiver withdraws it.
  assert fields == (0, 0)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_queue_metadata_changes_only_hold_bits_and_crc_in_the_existing_adrv_packet(side):
  rig = Rig(side)
  rig.queue()
  _, _, held = wire_packet(rig)
  _, baseline_hold, baseline = wire_packet(rig, override_hold='none')
  assert baseline_hold == (0, 0)
  baseline, held = bytearray(baseline), bytearray(held)
  baseline[:2] = held[:2] = bytes(2)
  baseline[3] &= ~0x60
  held[3] &= ~0x60
  assert baseline == held
