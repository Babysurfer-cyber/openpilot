#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

#include "selfdrive/ui/atc_icon_visibility.h"

// Active longitudinal ATC may start reducing the target speed before the
// road-speed-dependent TBT icon/guide threshold. Use the selected *current*
// ATC limit, not the next-turn "atc2" source or the ATC setting alone.
// Speeds published by CarrotMan and CarState.vCruiseCluster are both in km/h.
inline bool atc_mode_background_current_decel_active(std::string_view desired_source,
                                                      float desired_kph, float cruise_kph) {
  return desired_source == "atc" && std::isfinite(desired_kph) &&
         std::isfinite(cruise_kph) && cruise_kph > 0.0f &&
         desired_kph > 0.0f && desired_kph < cruise_kph - 0.5f;
}

// Blink for a valid current turn/fork while in the normal HUD guide window,
// OR while its speed reduction is actually selected, even if that reduction
// began farther away than the HUD icon window. A "prepare" label outside the
// icon window is eligible only in the latter case. Ignored, canceled, stale
// or unrelated guidance does not blink the mode badge.
inline bool atc_mode_background_should_blink(bool enabled, int driving_mode,
                                              int x_turn_info, int distance_m,
                                              float guide_start_m, std::string_view atc_type,
                                              bool current_atc_decel_active = false) {
  if (!enabled || (driving_mode != 3 && driving_mode != 5)) return false;
  if (x_turn_info < 1 || x_turn_info > 4 || distance_m <= 0) return false;

  const bool normal_type = atc_type == "atc left" || atc_type == "atc right" ||
                           atc_type == "turn left" || atc_type == "turn right" ||
                           atc_type == "fork left" || atc_type == "fork right";
  constexpr std::string_view prepare_suffix = " prepare";
  const bool preparing = atc_type.size() > prepare_suffix.size() &&
                         atc_type.substr(atc_type.size() - prepare_suffix.size()) == prepare_suffix;
  const auto unqualified_type = preparing ? atc_type.substr(0, atc_type.size() - prepare_suffix.size()) : atc_type;
  const bool valid_prepare = preparing &&
                             (unqualified_type == "turn left" || unqualified_type == "turn right" ||
                              unqualified_type == "fork left" || unqualified_type == "fork right" ||
                              unqualified_type == "atc left" || unqualified_type == "atc right");

  if (!normal_type && !valid_prepare) return false;
  if (current_atc_decel_active) return true;
  return normal_type && atc_guide_icon_in_range(distance_m, guide_start_m);
}

// Exactly 700ms per color: [0,700) native, [700,1400) ATC yellow,
// [1400,2100) native, etc. Use elapsed monotonic time, never frame count.
inline bool atc_mode_background_warning_phase(int64_t elapsed_ms) {
  return elapsed_ms >= 0 && ((elapsed_ms / 700) % 2) == 1;
}

// When ATC is enabled but not in its guide/active window, retain the old
// steady ATC color (AUTO: dark yellow; NORM: pale yellow).
inline bool atc_mode_background_yellow(bool enabled, bool blinking, int64_t elapsed_ms) {
  return enabled && (!blinking || atc_mode_background_warning_phase(elapsed_ms));
}
