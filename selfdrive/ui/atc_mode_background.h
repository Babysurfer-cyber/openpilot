#pragma once

#include <cstdint>
#include <string_view>

#include "selfdrive/ui/atc_icon_visibility.h"

// Blink only when the current turn/fork is in the same preparation window
// used for navigation icon visibility. A "prepare" state outside the window,
// cancelled guidance, stale/default/unknown guidance, or disabled ATC never
// triggers the mode badge. This is a visual-only signal.
inline bool atc_mode_background_should_blink(bool enabled, int driving_mode,
                                              int x_turn_info, int distance_m,
                                              float guide_start_m, std::string_view atc_type) {
  if (!enabled || (driving_mode != 3 && driving_mode != 5)) return false;
  if (x_turn_info < 1 || x_turn_info > 4 ||
      !atc_guide_icon_in_range(distance_m, guide_start_m)) return false;

  return atc_type == "atc left" || atc_type == "atc right" ||
         atc_type == "turn left" || atc_type == "turn right" ||
         atc_type == "fork left" || atc_type == "fork right";
}

// Exactly 700ms per color: [0,700) native, [700,1400) ATC yellow,
// [1400,2100) native, etc. Use elapsed monotonic time, never frame count.
inline bool atc_mode_background_warning_phase(int64_t elapsed_ms) {
  return elapsed_ms >= 0 && ((elapsed_ms / 700) % 2) == 1;
}
