"""Read-only EV6 prevehicle evidence gap report from *saved* CAN CSV captures.

This is an evidence inventory, not an authorization mechanism. It does not
communicate with a CAN device or determine safe vehicle actuation. The reports
always remain NO_GO unless an independent hardware qualification process outside
this tool has been completed. Source bus metadata is not authenticated.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

from ev6_spas_control import check_stock_frame
from ev6_spas_factory_replay import _decode_capture, lever_checksum

TARGETS = (0x35, 0x175, 0xA0, 0x3C1, 0x165, 0x16A, 0x413, 0x7B1, 0x7B9)
PR46_STOCK_GUARD = (0x35, 0x175, 0xA0)


def inspect_saved_capture(path):
  counts = Counter()
  bad_spas_crc = 0
  bad_stalk_crc = 0
  bad_lamp_length = 0
  neutral_stalk = 0
  active_stalk = 0
  ignored_other_bus = 0
  for _stamp, addr, bus, payload in _decode_capture(path):
    if bus != 0:
      ignored_other_bus += 1
      continue
    if addr not in TARGETS:
      continue
    counts[addr] += 1
    if addr in (0x165, 0x16A) and not check_stock_frame(addr, payload):
      bad_spas_crc += 1
    elif addr == 0x3C1:
      if len(payload) != 8 or payload[0] != lever_checksum(payload):
        bad_stalk_crc += 1
      elif payload[3] == 0 and payload[4] == 0:
        neutral_stalk += 1
      else:
        active_stalk += 1
    elif addr == 0x413 and len(payload) != 8:
      bad_lamp_length += 1
  missing = [f'0x{addr:X}' for addr in PR46_STOCK_GUARD if counts[addr] == 0]
  blockers = [
    'INDEPENDENT_ISOLATED_ECU_HANDOFF_NOT_VERIFIED',
    'INDEPENDENT_MONITOR_HEALTH_AND_ERROR_COUNTERS_NOT_AVAILABLE',
    'FULL_COMPILED_PANDA_SAFETY_TEST_AND_TOPOLOGY_NOT_VERIFIED',
  ]
  if missing:
    blockers.append('MISSING_PANDA_PARK_BRAKE_WHEEL_BUS0_INPUTS:' + ','.join(missing))
  if not counts[0x7B1] or not counts[0x7B9]:
    blockers.append('NO_MATCHED_APRK_DIAGNOSTIC_REQUEST_RESPONSE_EVIDENCE')
  if not counts[0x165] or not counts[0x16A]:
    blockers.append('NO_COMPLETE_BUS0_FACTORY_SPAS1_SPAS2_EVIDENCE')
  if not counts[0x3C1] or not counts[0x413]:
    blockers.append('MISSING_STALK_OR_LAMP_OBSERVATIONS')
  if bad_spas_crc or bad_stalk_crc or bad_lamp_length:
    blockers.append('CAPTURE_CRC_OR_LENGTH_ERRORS')
  return {
    'capture_filename': Path(path).name,
    'verdict': 'NO_GO_FOR_VEHICLE_CAN_TRANSMIT',
    'transmitted_by_tool': 0,
    'evidence_is_authenticated': False,
    'reported_bus0_counts': {f'0x{a:X}': counts[a] for a in TARGETS},
    'ignored_nonzero_bus_records': ignored_other_bus,
    'neutral_0x3c1_samples': neutral_stalk,
    'nonneutral_0x3c1_samples': active_stalk,
    'invalid_spas_crc': bad_spas_crc,
    'invalid_stalk_crc_or_length': bad_stalk_crc,
    'invalid_lamp_length': bad_lamp_length,
    'park_guard_missing_bus0_identifiers': missing,
    'blockers': blockers,
    'limitations': (
      'Source bus field and ECU origin are unverified; gear/brake/wheel presence '
      'does not prove their values, freshness, or panda RX quality. 0x7B1/0x7B9 '
      'presence alone cannot prove ECU handoff. This tool NEVER grants TX permission.'
    ),
  }


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--left', type=Path, required=True)
  ap.add_argument('--right', type=Path, required=True)
  ap.add_argument('--output', type=Path)
  a = ap.parse_args()
  result = {'kind': 'EV6_OFFLINE_PREVEHICLE_EVIDENCE_GAP_INVENTORY',
            'merging_or_installing_is_authorized': False,
            'left': inspect_saved_capture(a.left),
            'right': inspect_saved_capture(a.right)}
  result['shared_blockers'] = sorted(set(result['left']['blockers']) | set(result['right']['blockers']))
  payload = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
  if a.output:
    a.output.write_text(payload, encoding='utf-8')
  print(payload)


if __name__ == '__main__':
  main()
