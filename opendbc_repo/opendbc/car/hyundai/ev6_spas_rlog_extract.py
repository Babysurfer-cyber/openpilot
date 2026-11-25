"""Extract only selected EV6 receive-side CAN IDs from an already saved rlog.

No on-car code, CAN transport or TX. Does NOT use sendcan events. Exports no
camera/GPS/driver-monitor streams, just timestamp/bus/ID/payload from can.
Panda bus 0/1/2 topology may differ across cars; copied/forwarded messages
are not independent ECU origin proof. Use a saved offline copy of the log.
"""
import argparse
import csv
import json
from pathlib import Path
import tempfile

# Reuse the IDs inventoried by the offline prevehicle evidence gap classifier.
from ev6_spas_evidence_gate import TARGETS, inspect_saved_capture

EXPORT_BUSES = frozenset((0, 1, 2))


def extract_receive_rows(events):
  """Convert read-only LogReader events to rows; never inspect 'sendcan'."""
  first_ns = None
  last_ns = None
  for event in events:
    if event.which() != 'can':
      continue
    stamp_ns = int(event.logMonoTime)
    if stamp_ns < 0 or (last_ns is not None and stamp_ns < last_ns):
      raise ValueError('Nonmonotonic or negative rlog timestamp')
    last_ns = stamp_ns
    if first_ns is None:
      first_ns = stamp_ns
    for msg in event.can:
      bus = int(msg.src)
      addr = int(msg.address)
      if bus not in EXPORT_BUSES or addr not in TARGETS:
        continue
      payload = bytes(msg.dat)
      if not payload:
        raise ValueError('Empty CAN payload for selected ID')
      yield (f'{(stamp_ns-first_ns) / 1e9:.9f}', f'0x{addr:X}', bus, payload.hex())


def export_saved_rlog(events, output_path):
  """Write a complete CSV atomically; never leave a partial sensitive trace."""
  output_path = Path(output_path)
  counts = 0
  tmp = None
  try:
    with tempfile.NamedTemporaryFile('w', encoding='utf-8', newline='',
                                     dir=output_path.parent, prefix='.ev6_can_',
                                     suffix='.tmp', delete=False) as out:
      tmp = Path(out.name)
      writer = csv.writer(out)
      writer.writerow(('t_sec', 'can_id', 'bus', 'payload_hex'))
      for row in extract_receive_rows(events):
        writer.writerow(row)
        counts += 1
    if not counts:
      raise ValueError('No selected receive-side CAN frames in saved rlog')
    tmp.replace(output_path)
  finally:
    if tmp is not None:
      tmp.unlink(missing_ok=True)
  return counts


def write_bus_summary(filtered_csv, summary_path):
  """Optional privacy-minimized counts: no raw CAN payloads or timestamps."""
  report = inspect_saved_capture(filtered_csv)
  summary = {
    'kind': 'EV6_SAVED_RLOG_RECEIVE_ONLY_BUS_COUNTS',
    'can_tx_by_extractor': 0,
    'merged_vehicle_code_changed': False,
    'permits_vehicle_actuation': False,
    'captured_bus_metadata_is_authenticated': False,
    'counts_by_bus_and_can_id': report['reported_can_id_counts_by_bus'],
    'panda_guard_input_counts_by_bus': report['park_guard_input_counts_by_bus'],
    'buses_with_all_three_guard_ids_observed': report['buses_with_all_three_park_guard_ids_observed'],
    'panda_guard_safety_validated': False,
    'independent_ecu_handoff_tested': False,
    'warning': ('Counts are from saved CAN receive events only. bus numbers in rlogs '
                'may represent forwarded traffic; no ECU provenance, signal validity, '
                'or TX permission is established.'),
  }
  Path(summary_path).write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
  return summary


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--input', required=True, type=Path, help='Previously recorded rlog(.zst) on disk')
  parser.add_argument('--output', required=True, type=Path, help='Filtered CSV to inspect offline')
  parser.add_argument('--summary-output', type=Path, help='Optional counts-only JSON (no CAN payloads)')
  opts = parser.parse_args()
  # This import is only a file decoder, not a CAN interface or device client.
  from openpilot.tools.lib.logreader import LogReader
  number = export_saved_rlog(LogReader(str(opts.input)), opts.output)
  if opts.summary_output is not None:
    write_bus_summary(opts.output, opts.summary_output)
  print(f'Wrote {number} filtered, receive-side frames to {opts.output}. Vehicle TX: zero.')


if __name__ == '__main__':
  main()
