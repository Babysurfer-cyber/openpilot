#include "selfdrive/ui/atc_icon_visibility.h"

#include <cassert>
#include <initializer_list>
#include <limits>

int main() {
  // At the ATC start distance the icon becomes visible, not one metre earlier.
  for (float threshold : {160.0f, 200.0f, 230.0f, 290.0f, 350.0f}) {
    const int d = static_cast<int>(threshold);
    assert(!atc_guide_icon_in_range(d + 1, threshold));
    assert(atc_guide_icon_in_range(d, threshold));
    assert(atc_guide_icon_in_range(100, threshold));
  }
  assert(!atc_guide_icon_in_range(0, 160.0f));
  assert(!atc_guide_icon_in_range(-1, 160.0f));
  assert(!atc_guide_icon_in_range(30, 0.0f));
  assert(!atc_guide_icon_in_range(30, -10.0f));
  assert(!atc_guide_icon_in_range(30, std::numeric_limits<float>::quiet_NaN()));
  assert(!atc_guide_icon_in_range(30, std::numeric_limits<float>::infinity()));

  // The icon and AUTO badge consume exactly this same blinking predicate.
  for (int maneuver : {1, 2, 3, 4, 7, 8}) {
    assert(atc_guide_icon_blink_active(maneuver, 100, 200.0f, true, false));
    assert(!atc_guide_icon_blink_active(maneuver, 100, 200.0f, true, true));
    assert(!atc_guide_icon_blink_active(maneuver, 100, 200.0f, false, false));
    assert(!atc_guide_icon_blink_active(maneuver, 201, 200.0f, true, false));
  }
  assert(!atc_guide_icon_blink_active(5, 100, 200.0f, true, false));
  assert(!atc_guide_icon_blink_active(6, 100, 200.0f, true, false));
}
