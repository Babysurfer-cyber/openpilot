#include "selfdrive/ui/atc_mode_background.h"

#include <cassert>
#include <cstdint>
#include <initializer_list>

int main() {
  // Every color lasts exactly 700ms; each full cycle lasts 1400ms.
  assert(!atc_mode_background_warning_phase(-1));
  assert(!atc_mode_background_warning_phase(0));
  assert(!atc_mode_background_warning_phase(699));
  assert(atc_mode_background_warning_phase(700));
  assert(atc_mode_background_warning_phase(1399));
  assert(!atc_mode_background_warning_phase(1400));
  assert(!atc_mode_background_warning_phase(2099));
  assert(atc_mode_background_warning_phase(2100));

  // Outside the preparation window: ATC enabled keeps the old steady
  // yellow, and ATC disabled keeps the native mode background.
  assert(atc_mode_background_yellow(true, false, 0));
  assert(atc_mode_background_yellow(true, false, 700));
  assert(atc_mode_background_yellow(true, false, 1400));
  assert(!atc_mode_background_yellow(false, false, 0));
  assert(!atc_mode_background_yellow(false, false, 700));

  // Inside the preparation window: native color for 0-699ms, warning
  // yellow for 700-1399ms, and repeat. Disabling ATC stops blinking.
  assert(!atc_mode_background_yellow(true, true, 0));
  assert(!atc_mode_background_yellow(true, true, 699));
  assert(atc_mode_background_yellow(true, true, 700));
  assert(atc_mode_background_yellow(true, true, 1399));
  assert(!atc_mode_background_yellow(true, true, 1400));
  assert(!atc_mode_background_yellow(false, true, 700));

  for (int driving_mode : {3, 5}) {
    for (int maneuver : {1, 2, 3, 4}) {
      const char* active = maneuver == 1 ? "atc left" : maneuver == 2 ? "turn right"
                           : maneuver == 3 ? "fork left" : "fork right";
      assert(atc_mode_background_should_blink(true, driving_mode, maneuver, 200, 200.0f, active));
      assert(!atc_mode_background_should_blink(true, driving_mode, maneuver, 201, 200.0f, active));
      assert(!atc_mode_background_should_blink(true, driving_mode, maneuver, 0, 200.0f, active));
      assert(!atc_mode_background_should_blink(false, driving_mode, maneuver, 100, 200.0f, active));
      assert(!atc_mode_background_should_blink(true, driving_mode, maneuver, 100, 200.0f, "atc right prepare"));
      assert(!atc_mode_background_should_blink(true, driving_mode, maneuver, 100, 200.0f, "fork right canceled"));
    }
  }

  assert(!atc_mode_background_should_blink(true, 2, 2, 100, 200.0f, "atc right"));
  assert(!atc_mode_background_should_blink(true, 4, 2, 100, 200.0f, "turn right"));
  assert(!atc_mode_background_should_blink(true, 5, 8, 100, 200.0f, "straight"));
  assert(!atc_mode_background_should_blink(true, 5, 2, 100, 200.0f, ""));
  assert(!atc_mode_background_should_blink(true, 5, 2, 100, 200.0f, "none"));

  // Regression: ATC can already select a speed limit farther away than the
  // normal TBT icon window (e.g. cruise 80, turn target 20, guide 290m).
  assert(atc_mode_background_current_decel_active("atc", 74.0f, 80.0f));
  assert(!atc_mode_background_current_decel_active("atc2", 74.0f, 80.0f));
  assert(!atc_mode_background_current_decel_active("CAM", 74.0f, 80.0f));
  assert(!atc_mode_background_current_decel_active("atc", 80.0f, 80.0f));
  assert(!atc_mode_background_current_decel_active("atc", 79.7f, 80.0f));  // tolerance
  assert(!atc_mode_background_current_decel_active("atc", 0.0f, 80.0f));
  assert(!atc_mode_background_current_decel_active("atc", 74.0f, 0.0f));

  // An active current ATC reduction blinks regardless of icon threshold.
  // "prepare" may still be published in this interval.
  assert(atc_mode_background_should_blink(true, 5, 2, 320, 290.0f, "turn right prepare", true));
  assert(atc_mode_background_should_blink(true, 5, 2, 320, 290.0f, "atc right", true));
  assert(atc_mode_background_should_blink(true, 3, 3, 350, 290.0f, "fork left prepare", true));
  assert(!atc_mode_background_should_blink(true, 5, 2, 320, 290.0f, "turn right prepare", false));
  assert(!atc_mode_background_should_blink(true, 5, 2, 320, 290.0f, "turn right", false));
  assert(!atc_mode_background_should_blink(true, 5, 2, 120, 290.0f, "turn right canceled", true));
  assert(!atc_mode_background_should_blink(false, 5, 2, 320, 290.0f, "atc right", true));
  assert(!atc_mode_background_should_blink(true, 5, 2, 0, 290.0f, "atc right", true));
  assert(!atc_mode_background_should_blink(true, 5, 5, 120, 290.0f, "straight", true));
  assert(!atc_mode_background_should_blink(true, 2, 2, 320, 290.0f, "atc right", true));

  // Once ATC is actively limiting, the existing 700ms phase continues to
  // alternate; no speed or steering commands are touched by this gate.
  const bool active = atc_mode_background_should_blink(true, 5, 2, 320, 290.0f, "turn right prepare", true);
  assert(!atc_mode_background_yellow(true, active, 0));
  assert(atc_mode_background_yellow(true, active, 700));
  assert(!atc_mode_background_yellow(true, active, 1400));
}
