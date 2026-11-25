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
      def command(payload):
        return (int(payload[32:34], 16) >> 5) & 7 if len(payload) >= 34 else None
      counts = Counter(command(payload) for _, payload in g[0])
      active = [t for t, payload in g[0] if command(payload) in (3, 4)]
      summary["spas2_control_histogram_bus0"] = {str(k): v for k, v in sorted(counts.items())}
      summary["first_direction_command_s"] = round(active[0], 3) if active else None
      summary["direct_bus2_rx_during_direction_command_count"] = sum(
        command(payload) in (3, 4) for _, payload in g[2])
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
