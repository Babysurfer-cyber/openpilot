"""Fresh stock-navigation section state from real Hyundai 0x4B4 frames."""

import math

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, HyundaiFlags


DBC = "hyundai_canfd_generated"
MESSAGE = "NEW_MSG_4B4"
START = 2_000_000_000


def section_parser():
  return CANParser(DBC, [(MESSAGE, math.nan)], 0)


def feed_average(parser, timestamp, average):
  frame = CANPacker(DBC).make_can_msg(MESSAGE, 0, {"POS_RANGE_AVG_SPEED": average})
  assert parser.update([(timestamp, [frame])]) == {0x4B4}


def section_state(parser, ret=None):
  ret = structs.CarState.new_message() if ret is None else ret
  CarState.__new__(CarState)._update_vehicle_navi_section(parser, ret)
  return ret


def test_zero_average_entry_and_exit_follow_real_messages():
  parser = section_parser()
  feed_average(parser, START, 0)
  ret = section_state(parser)
  assert ret.navSectionValid and not ret.navSectionActive

  feed_average(parser, START + 100_000_000, 92)
  section_state(parser, ret)
  assert ret.navSectionValid and ret.navSectionActive

  feed_average(parser, START + 200_000_000, 0)
  section_state(parser, ret)
  assert ret.navSectionValid and not ret.navSectionActive


@pytest.mark.parametrize("average", (1, 92, 510))
def test_all_valid_positive_average_values_indicate_section(average):
  parser = section_parser()
  feed_average(parser, START, average)
  ret = section_state(parser)
  assert ret.navSectionValid and ret.navSectionActive


@pytest.mark.parametrize("age,valid", ((0, True), (1_000_000_000, True), (1_000_000_001, False), (-1, False)))
def test_section_state_requires_fresh_nonfuture_can_timestamp(age, valid):
  parser = section_parser()
  feed_average(parser, START, 92)
  # Advance the parser's receive clock independently of this optional message.
  parser.update([(START + age, [])])
  ret = section_state(parser)
  assert ret.navSectionValid == valid
  assert ret.navSectionActive == valid


def test_registered_but_unseen_message_is_not_valid_zero_average():
  parser = section_parser()
  parser.update([(START, [])])
  ret = section_state(parser)
  assert not ret.navSectionValid and not ret.navSectionActive


def test_timestamp_zero_is_not_a_received_section_sample():
  parser = section_parser()
  feed_average(parser, 0, 92)
  ret = section_state(parser)
  assert not ret.navSectionValid and not ret.navSectionActive


def test_invalid_average_clears_previous_active_output():
  parser = section_parser()
  feed_average(parser, START, 92)
  ret = section_state(parser)
  assert ret.navSectionActive

  feed_average(parser, START + 100_000_000, 511)
  section_state(parser, ret)
  assert not ret.navSectionValid and not ret.navSectionActive


def test_stale_sample_clears_both_flags_and_new_sample_recovers():
  parser = section_parser()
  feed_average(parser, START, 92)
  ret = section_state(parser)
  assert ret.navSectionActive

  parser.update([(START + 1_000_000_001, [])])
  section_state(parser, ret)
  assert not ret.navSectionValid and not ret.navSectionActive

  feed_average(parser, START + 1_100_000_000, 93)
  section_state(parser, ret)
  assert ret.navSectionValid and ret.navSectionActive


def test_logged_frame_decodes_and_activates_section():
  parser = section_parser()
  frame = (0x4B4, bytes.fromhex("0d0085cbb18948a5"), 0)
  assert parser.update([(START, [frame])]) == {0x4B4}
  assert parser.vl[MESSAGE]["POS_OFFSET"] == 13
  assert parser.vl[MESSAGE]["POS_RANGE_AVG_SPEED"] == 92
  ret = section_state(parser)
  assert ret.navSectionValid and ret.navSectionActive


@pytest.mark.parametrize("average,map_source,forward_info,active", ((0, 2, 2, False), (92, 1, 0, True)))
def test_generic_camera_and_forward_info_do_not_override_section_sample(average, map_source, forward_info, active):
  parser = section_parser()
  feed_average(parser, START, average)
  ret = structs.CarState.new_message(mapSource=map_source, navFrwinfo=forward_info)
  section_state(parser, ret)
  assert ret.navSectionValid and ret.navSectionActive == active


def test_omitted_section_fields_default_to_unknown_in_serialized_car_state():
  ret = structs.CarState.new_message(vEgo=20.0, mapSource=2)
  with structs.CarState.from_bytes(ret.to_bytes()) as decoded:
    assert not decoded.navSectionValid and not decoded.navSectionActive


def test_optional_message_loss_does_not_make_can_invalid():
  parser = section_parser()
  for timestamp in (START, START + 1_000_000_000, START + 30_000_000_000):
    parser.update([(timestamp, [])])
    assert parser.can_valid and not parser.bus_timeout
    ret = section_state(parser)
    assert not ret.navSectionValid and not ret.navSectionActive


def test_canfd_parser_registers_section_message_without_required_frequency():
  cp = structs.CarParams.new_message(
    brand="hyundai", carFingerprint=CAR.KIA_EV6,
    flags=int(HyundaiFlags.CANFD | HyundaiFlags.CANFD_ALT_BUTTONS),
  )
  cp.init("safetyConfigs", 1)
  parser = CarState.__new__(CarState).get_can_parsers_canfd(cp)[Bus.pt]
  assert MESSAGE in parser.vl
  assert parser.message_states[0x4B4].ignore_alive
  # Other optional navigation messages may also be absent indefinitely.
  parser.update([(START + 30_000_000_000, [])])
  assert parser.can_valid and not parser.bus_timeout
