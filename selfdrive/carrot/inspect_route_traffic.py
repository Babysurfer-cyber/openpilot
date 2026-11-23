#!/usr/bin/env python3
"""Read-only probe: which route/traffic fields does CarrotNM actually send?

Does not modify CarrotNM, send commands, expose route coordinates, or change
vehicle controls. Run on comma while CarrotNM is actively navigating.
"""
import argparse
import json
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--url", default="http://127.0.0.1:7714/api/navi/latest",
                      help="Local CarrotNavi receiver status endpoint")
  args = parser.parse_args()
  try:
    with urlopen(args.url, timeout=3) as response:
      snapshot = json.load(response)
  except (OSError, ValueError, HTTPError, URLError) as exc:
    print(f"Receiver unavailable: {exc}")
    return

  items = snapshot.get("items") or {}
  route_record = items.get("route") or {}
  route = route_record.get("value") or {}
  print("Receiver connected:", bool(snapshot.get("connected")))
  print("Route present:", bool(route_record.get("present")))
  if not isinstance(route, dict):
    print("Route value not an object")
    return
  print("Route field names:", sorted(route.keys()))
  candidates = {key: value for key, value in route.items()
                if "traffic" in key.lower() or "congestion" in key.lower()}
  print("Congestion-related field names:", sorted(candidates.keys()))
  for name, value in candidates.items():
    print(f"{name}: type={type(value).__name__}, count={len(value) if isinstance(value, list) else 'n/a'}")
  if not candidates:
    print("No congestion fields observed from the current sender.")


if __name__ == "__main__":
  main()
