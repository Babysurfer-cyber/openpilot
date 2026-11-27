#pragma once

// Display-only route progress. Optional per-section congestion from the
// patched CarrotNM sender is validated and rendered separately in carrot.cc.
// If traffic is absent or invalid, the original neutral/cyan progress remains.
namespace route_progress {

// A navigation SDK can round remaining distance up by a few metres at route
// start. Tolerate only this small overshoot; larger mismatches still clear
// the HUD marker rather than displaying incorrect position or traffic.
constexpr int MAX_REMAINING_OVERSHOOT_M = 5;

struct Snapshot {
  bool visible = false;
  int total_m = 0;
  int traveled_m = 0;
  int remaining_m = 0;
  float fraction = 0.0f;
};

// Live route and guidance status must agree. On disconnect, reroute, off-route
// or stale/absent data we never retain the previous progress marker.
inline Snapshot calculate(bool connected, bool status_present,
                          bool guidance_active, bool off_route,
                          bool route_present, bool route_meta_present,
                          int total_m, int remaining_m) {
  if (!connected || !status_present || !guidance_active || off_route ||
      !route_present || !route_meta_present ||
      total_m <= 0 || total_m > 2000000 ||
      remaining_m < 0 || remaining_m > total_m + MAX_REMAINING_OVERSHOOT_M) {
    return {};
  }
  const int normalized_remaining_m = remaining_m > total_m ? total_m : remaining_m;
  const int traveled_m = total_m - normalized_remaining_m;
  return {true, total_m, traveled_m, normalized_remaining_m,
          static_cast<float>(traveled_m) / static_cast<float>(total_m)};
}

// Match the navigational maneuvers handled by CarrotServ's TBT mapping,
// without consulting AutoTurnControl mode or any longitudinal control state.
// Exclude informational-only notices and tollgates, just as the existing HUD
// flag did for xTurnInfo=0 and xTurnInfo=6.
inline bool atc_route_maneuver(int turn_type) {
  switch (turn_type) {
    // Left/right turns, forks, exits, roundabouts, U-turn, destination.
    case 12: case 16: case 13: case 19:
    case 102: case 105: case 112: case 115:
    case 101: case 104: case 111: case 114:
    case 7: case 44: case 17: case 75: case 76: case 118:
    case 6: case 43: case 73: case 74: case 123: case 124: case 117:
    case 131: case 132: case 140: case 141: case 133: case 134:
    case 135: case 136: case 137: case 138: case 139: case 142:
    case 14: case 201:
      return true;
    default:
      return false;
  }
}

// Read-only marker for the current upcoming ATC maneuver on the *same* route.
// Never infer its position from a different route, stale guidance, or a turn
// distance beyond the verified remaining journey. No vehicle control changes.
struct AtcMarker {
  bool visible = false;
  float fraction = 0.0f;
};

inline AtcMarker atc_marker(const Snapshot &route, bool guidance_present,
                            bool atc_maneuver, int maneuver_distance_m) {
  if (!route.visible || !guidance_present || !atc_maneuver ||
      maneuver_distance_m <= 0 || maneuver_distance_m > route.remaining_m) {
    return {};
  }
  const int atc_position_m = route.traveled_m + maneuver_distance_m;
  if (atc_position_m > route.total_m) return {};
  return {true, static_cast<float>(atc_position_m) / static_cast<float>(route.total_m)};
}

}  // namespace route_progress
