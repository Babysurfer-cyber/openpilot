#include "selfdrive/ui/atc_icon_visibility.h"
#include "selfdrive/ui/atc_mode_background.h"

#include <cassert>
#include <initializer_list>

// This test verifies the actual shared icon state, not a second ATC badge
// timer or an independently interpreted maneuver type.
int main() {
  const bool blinking_icon = atc_guide_icon_blink_active(2, 140, 200.0f, true, false);
  assert(blinking_icon);
  for (bool pulse : {false, true}) {
    const bool icon_drawn = atc_guide_icon_in_range(140, 200.0f) &&
                            (!blinking_icon || pulse);
    // With the same pulse the badge's yellow phase is exactly when the
    // blinking TBT icon is visible; otherwise the native mode color shows.
    assert(atc_mode_background_yellow(true, blinking_icon, pulse) == icon_drawn);
    assert(atc_mode_background_yellow(false, blinking_icon, pulse) == icon_drawn);
  }

  // A visible, but non-blinking icon must leave the badge steady.
  const bool icon_not_blinking = atc_guide_icon_blink_active(2, 140, 200.0f, false, false);
  assert(!icon_not_blinking);
  assert(atc_mode_background_yellow(true, icon_not_blinking, true));
  assert(atc_mode_background_yellow(true, icon_not_blinking, false));
  assert(!atc_mode_background_yellow(false, icon_not_blinking, true));
  assert(!atc_mode_background_yellow(false, icon_not_blinking, false));

  // Phase is shared over arbitrary repeated frames: no 700ms local timer.
  for (int frame = 0; frame < 140; ++frame) {
    const bool actual_icon_pulse = (frame % 14) < 6;
    assert(atc_mode_background_yellow(true, blinking_icon, actual_icon_pulse) == actual_icon_pulse);
  }

  // The canceled, outside-window, invalid-distance, invalid-guide and
  // unrendered (straight/tollgate) cases must never blink.
  assert(!atc_guide_icon_blink_active(2, 140, 200.0f, true, true));
  assert(!atc_guide_icon_blink_active(2, 201, 200.0f, true, false));
  assert(!atc_guide_icon_blink_active(2, 0, 200.0f, true, false));
  assert(!atc_guide_icon_blink_active(2, 140, 0.0f, true, false));
  assert(!atc_guide_icon_blink_active(5, 140, 200.0f, true, false));
  assert(!atc_guide_icon_blink_active(6, 140, 200.0f, true, false));
  assert(!atc_guide_icon_blink_active(-1, 140, 200.0f, true, false));
  for (int type : {1, 2, 3, 4, 7, 8}) {
    assert(atc_guide_icon_blink_active(type, 140, 200.0f, true, false));
  }
}
