#!/usr/bin/env python3
"""Read-only check for route congestion actually received from CarrotNM.

Prints metadata and counts, never route coordinates or other personal data.
Run during active navigation: python3 selfdrive/carrot/inspect_route_traffic.py
"""
from __future__ import annotations

import argparse
import json
from urllib.request import urlopen


def main() -> None:
  parser = argparse.ArgumentParser(description="Inspect live CarrotNM congestion stream (read-only)")
  parser.add_argument("--url", default="http://127.0.0.1:7714/api/navi/latest")
  args = parser.parse_args()
  try:
    with urlopen(args.url, timeout=3) as reply:
      snapshot = json.load(reply)
  except (OSError, ValueError) as exc:
    print("CarrotNavi receiver unavailable:", exc)
    return

  record = (snapshot.get("items") or {}).get("route") or {}
  route = record.get("value") or {}
  if not isinstance(route, dict):
    route = {}
  status = (snapshot.get("items") or {}).get("navigation_status") or {}
  sections = route.get("traffic_sections")
  counts: dict[str, int] = {}
  if isinstance(sections, list):
    for section in sections:
      label = str(section.get("congestion", "unknown")) if isinstance(section, dict) else "invalid"
      counts[label] = counts.get(label, 0) + 1

  print("Receiver connected:", bool(snapshot.get("connected")))
  print("Route current:", bool(record.get("present")))
  print("Guidance current:", bool(status.get("present")))
  print("Route distance (m):", route.get("total_distance_m"))
  print("Traffic source:", route.get("traffic_source", "(none)"))
  print("Traffic sections:", len(sections) if isinstance(sections, list) else "(not sent)")
  print("Congestion types:", counts)


if __name__ == "__main__":
  main()
