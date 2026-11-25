"""Extract only selected EV6 receive-side CAN IDs from an already saved rlog.

No on-car code, CAN transport or TX. Does NOT use sendcan events. Exports no
camera/GPS/driver-monitor streams, just timestamp/bus/ID/payload from can.
Panda bus 0/1/2 topology may differ across cars; copied/forwarded messages
are not independent ECU origin proof. Use a saved offline copy of the log.
"""
import argparse
import csv
from pathlib import Path

# Reuse the IDs inventoried by the offline prevehicle evidence gap classifier.
from ev6_spas_evidence_gate import TARGETS

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
  output_path = Path(output_path)
  counts = 0
  with output_path.open('w', encoding='utf-8', newline='') as out:
    writer = csv.writer(out)
    writer.writerow(('t_sec', 'can_id', 'bus', 'payload_hex'))
    for row in extract_receive_rows(events):
      writer.writerow(row)
      counts += 1
  if not counts:
    output_path.unlink(missing_ok=True)
    raise ValueError('No selected receive-side CAN frames in saved rlog')
  return counts


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--input', required=True, type=Path, help='Previously recorded rlog(.zst) on disk')
  parser.add_argument('--output', required=True, type=Path, help='Filtered CSV to inspect offline')
  opts = parser.parse_args()
  # This import is only a file decoder, not a CAN interface or device client.
  from openpilot.tools.lib.logreader import LogReader
  number = export_saved_rlog(LogReader(str(opts.input)), opts.output)
  print(f'Wrote {number} filtered, receive-side frames to {opts.output}. Vehicle TX: zero.')


if __name__ == '__main__':
  main()
