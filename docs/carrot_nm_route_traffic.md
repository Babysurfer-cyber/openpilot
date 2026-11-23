# CarrotNM route congestion receiver (display only)

## Scope / current compatibility

This is an **optional receive-only extension** to the existing CarrotNM 1.30
WebSocket v2 `route` item. It does **not** modify the APK, the fixed
28-item catalog, the discovery protocol, the EV6 head unit, cruise control,
AutoMode, or any steering/braking logic.

The inspected CarrotNM 1.30 APK includes Naver Maps SDK traffic models,
but **does not demonstrate that the installed app currently sends route
congestion sections over WebSocket**. With the unmodified current APK,
the bar will normally remain neutral. This patch is not a way to
magically obtain traffic data which the sender never sends.

## Optional wire extension

A sender capable of providing per-section congestion may add this
key to an otherwise unchanged `json/route` item value:

```json
{
  "total_distance_m": 20000,
  "remain_distance_m": 12600,
  "traffic_sections": [
    {"start_distance_m": 0, "length_m": 5000, "congestion": "good"},
    {"start_distance_m": 5000, "length_m": 3000, "congestion": "bad"},
    {"start_distance_m": 8000, "length_m": 4000, "congestion": "very_bad"},
    {"start_distance_m": 12000, "length_m": 8000, "congestion": "good"}
  ]
}
```

All distances are meters along **this route from its start**.
Supported labels: `good`, `bad`, `very_bad`/`VeryBad`, `none`.
Codes: 1 green, 2 amber, 3 red, 0 unknown gray.
Missing or uncovered sections remain gray; no inference is made from speed
limits, congestion averages, camera alerts, or traffic light data.

## Data validation and safe invalidation

- A `route` must remain fresh through the existing 3-second timeout.
- `navigation_status` must be valid, guidance active, and not off-route.
- Sections must be ordered, nonoverlapping, integer-meter lengths, nonempty,
  and within the current total route distance (<=2,000,000 m).
- Maximum 128 sections. One invalid section invalidates the entire list.
- Connection loss, invalid route/status, rerouting without fresh traffic,
  or missing sections clears the display; no old congestion is reused.
- Current position is taken from the existing **total minus remaining**
  route distance; it is not an independently matched GPS position.
- **No vehicle control** uses these fields.

## Files

- `selfdrive/carrot/route_traffic.py`: strict JSON parser
- `selfdrive/carrot/carrot_navi_cereal.py`: optional route data mapping
- `cereal/custom.capnp`: appended `Route.trafficPresent` and
  `Route.trafficSections` fields
- `selfdrive/ui/carrot.cc`: colored sections in compact route progress HUD
- `selfdrive/carrot/tests/test_route_traffic.py`: parser tests

## Validation required before installation

Run full cereal schema build, openpilot unit tests, Qt/NanoVG UI compilation,
and static or parked comma 3X UI tests. Validate live `route` JSON via
`/api/navi/latest` to confirm which fields the stock CarrotNM APK actually
emits. Until actual sender fields are captured, congestion display should not
be described as operational on real drives.
