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
}
