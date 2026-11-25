#!/usr/bin/env python3
"""Read-only EV6 SPAS1/SPAS2 bus source audit of previously exported rlog CSV.

CSV requires t_sec,can_id,bus,payload_hex. This is *not* a CAN transmit or
UDS client. A returned/forwarded src=130 is not proof of another OEM ECU.
Example: python tools/signal_analysis/ev6_spas_bus_route_audit.py left.csv right.csv
"""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

from ev6_spas_onset_sequence import check_spas2


def checked_spas2_control(payload_hex):
  """Decode direction only from a complete, CRC-valid, 32-byte OEM frame."""
  try:
    data = bytes.fromhex(payload_hex)
  except (TypeError, ValueError):
    return None
  if not check_spas2(data):
    return None
  return (data[16] >> 5) & 7

WATCH = {0x165, 0x16A}
SOURCES = (0, 2, 130)


def summarize(path: Path):
  packets = {addr: {src: [] for src in SOURCES} for addr in WATCH}
  with path.open(newline="", encoding="utf-8") as f:
    for row in csv.DictReader(f):
      addr, src = int(row["can_id"], 0), int(row["bus"])
      if addr in WATCH and src in SOURCES:
        packets[addr][src].append((float(row["t_sec"]), row["payload_hex"].lower()))
  result = {"source_file": path.name, "addresses": {}, "limits": [
    "cereal src=130 is a returned/forwarded frame, NOT a second physical parking ECU",
    "Matching timestamps/payloads across buses do not prove the physical ECU source",
    "No external CAN/UDS commands; read-only historic CAN evidence only",
    "Raw source counts include all observed rows; direction inference requires intact CRC/length",
  ]}
  for addr in sorted(WATCH):
    g = packets[addr]
    s0, s2, s130 = (set(g[src]) for src in SOURCES)
    summary = {
      "bus0_rx": len(g[0]),
      "bus2_rx": len(g[2]),
      "src130_forwarded_or_returned": len(g[130]),
      "bus2_first_s": round(g[2][0][0], 3) if g[2] else None,
      "bus2_last_s": round(g[2][-1][0], 3) if g[2] else None,
      "src130_first_s": round(g[130][0][0], 3) if g[130] else None,
      "src130_last_s": round(g[130][-1][0], 3) if g[130] else None,
      "exact_timestamp_payload_matches_bus2_to_bus0": len(s2 & s0),
      "exact_timestamp_payload_matches_src130_to_bus0": len(s130 & s0),
      "bus0_after_5s": sum(t > 5 for t, _ in g[0]),
    }
    if addr == 0x16A:
      directions = {src: [(t, checked_spas2_control(payload)) for t, payload in g[src]]
                    for src in SOURCES}
      invalid = {src: sum(direction is None for _, direction in directions[src]) for src in SOURCES}
      summary["invalid_spas2_crc_or_length_by_source"] = {
        str(src): invalid[src] for src in SOURCES
      }
      # If *any* original bus-0 SPAS2 frame is invalid, the apparent first
      # direction may hide a missing/corrupt earlier command. Do not conclude.
      trustworthy_bus0 = bool(g[0]) and invalid[0] == 0
      summary["spas2_direction_inference_valid_bus0"] = trustworthy_bus0
      counts = Counter(direction for _, direction in directions[0]) if trustworthy_bus0 else Counter()
      active = [t for t, direction in directions[0] if direction in (3, 4)] if trustworthy_bus0 else []
      summary["spas2_control_histogram_bus0"] = {str(k): v for k, v in sorted(counts.items())}
      summary["first_direction_command_s"] = round(min(active), 3) if active else None
      # Corrupt direct bus-2 frames cannot be counted as evidence of zero commands.
      summary["direct_bus2_rx_during_direction_command_count"] = (
        sum(direction in (3, 4) for _, direction in directions[2])
        if invalid[2] == 0 else None
      )
    result["addresses"][f"0x{addr:03x}"] = summary
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("captures", nargs="+", type=Path)
  parser.add_argument("--output", type=Path)
  args = parser.parse_args()
  report = {"kind": "EV6_STOCK_SPAS_BUS_SOURCE_AUDIT_NO_TX",
            "cases": [summarize(path) for path in args.captures]}
  output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
  if args.output:
    args.output.write_text(output, encoding="utf-8")
  print(output)


if __name__ == "__main__":
  main()
