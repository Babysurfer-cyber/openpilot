#include <cassert>
#include <limits>
#include "selfdrive/ui/nav_hud_hold.h"

int main() {
  using nav_hud_hold::Decision;
  using nav_hud_hold::State;

  State hud;
  assert(hud.update(100.0, false, false, false) == Decision::hidden);
  assert(hud.update(100.0, true, false, false) == Decision::fresh);
  assert(hud.update(100.1, false, false, false) == Decision::held);
  assert(hud.update(102.0, false, false, false) == Decision::held);
  assert(hud.update(102.001, false, false, false) == Decision::hidden);
  assert(hud.update(102.1, false, false, false) == Decision::hidden);

  // A temporary blank/zero route counter does not extend the freshness timer.
  assert(hud.update(200.0, true, false, false) == Decision::fresh);
  for (double t = 200.1; t < 202.0; t += .1) {
    assert(hud.update(t, false, false, false) == Decision::held);
  }
  assert(hud.update(202.1, false, false, false) == Decision::hidden);

  // Once fresh data resumes, the window updates and the timer restarts.
  assert(hud.update(300.0, true, false, false) == Decision::fresh);
  assert(hud.update(301.5, false, false, false) == Decision::held);
  assert(hud.update(301.6, true, false, false) == Decision::fresh);
  assert(hud.update(303.5, false, false, false) == Decision::held);

  // Explicit route end, off-route, disconnect, and session/reroute changes
  // always invalidate the cache instead of holding stale directions.
  assert(hud.update(400.0, true, true, false) == Decision::hidden);
  assert(hud.update(400.1, false, false, false) == Decision::hidden);
  assert(hud.update(410.0, true, false, false) == Decision::fresh);
  assert(hud.update(410.1, true, false, true) == Decision::hidden);
  assert(hud.update(410.2, false, false, false) == Decision::hidden);

  // Clock reset or bad monotonic timestamp must invalidate the old view.
  assert(hud.update(500.0, true, false, false) == Decision::fresh);
  assert(hud.update(499.0, false, false, false) == Decision::hidden);
  assert(hud.update(std::numeric_limits<double>::quiet_NaN(), true, false, false) == Decision::hidden);
  return 0;
}
