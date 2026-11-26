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

// Single source of truth for whether the existing TBT icon is blinking.
// The icon is hidden on the pulse's off phase only when current ATC
// actually limits speed. The AUTO/NORM badge must follow this exact gate.
inline bool atc_guide_icon_blink_active(int maneuver, int distance_m, float guide_start_m,
                                        bool current_atc_decel_active, bool canceled) {
  return maneuver > 0 && atc_guide_icon_in_range(distance_m, guide_start_m) &&
         current_atc_decel_active && !canceled;
}
