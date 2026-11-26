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
}
