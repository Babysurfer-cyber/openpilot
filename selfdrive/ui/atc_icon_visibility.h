#pragma once

#include <cmath>

// HUD-only visibility gate. The threshold is the same road-speed-dependent
// preparation distance calculated by CarrotServ, not the AUTO mode display speed.
// This must not change the ATC deceleration target or any driving command.
inline bool atc_guide_icon_in_range(int maneuver_distance_m, float prepare_distance_m) {
  return maneuver_distance_m > 0 && std::isfinite(prepare_distance_m) &&
         prepare_distance_m > 0.0f &&
         static_cast<float>(maneuver_distance_m) <= prepare_distance_m;
}
