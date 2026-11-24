#include <cassert>
#include <cmath>
#include "selfdrive/ui/route_progress.h"

int main() {
  using route_progress::calculate;
  auto normal = calculate(true, true, true, false, true, true, 20000, 12600);
  assert(normal.visible && normal.total_m == 20000 && normal.traveled_m == 7400);
  assert(std::fabs(normal.fraction - 0.37f) < 0.0001f);
  // A re-route is a fresh snapshot rather than the old traveled-distance accumulator.
  auto rerouted = calculate(true, true, true, false, true, true, 25000, 18000);
  assert(rerouted.visible && rerouted.traveled_m == 7000);
  assert(std::fabs(rerouted.fraction - 0.28f) < 0.0001f);
  assert(!calculate(true, true, true, true, true, true, 20000, 12000).visible);
  assert(!calculate(true, false, true, false, true, true, 20000, 12000).visible);
  assert(!calculate(true, true, true, false, true, false, 20000, 12000).visible);
  assert(!calculate(false, true, true, false, true, true, 20000, 12000).visible);
  assert(!calculate(true, true, true, false, true, true, 20000, 21000).visible);
  // Regression: CarrotNM can report total=6329m and remaining=6330m at
  // route start. Keep the HUD visible and clamp the marker to 0% traveled.
  auto rounded_start = calculate(true, true, true, false, true, true, 6329, 6330);
  assert(rounded_start.visible && rounded_start.remaining_m == 6329);
  assert(rounded_start.traveled_m == 0 && rounded_start.fraction == 0.0f);
  auto rounding_limit = calculate(true, true, true, false, true, true, 6329, 6334);
  assert(rounding_limit.visible && rounding_limit.fraction == 0.0f);
  // Do not mask an actual route mismatch or loss of navigation validity.
  assert(!calculate(true, true, true, false, true, true, 6329, 6335).visible);
  assert(!calculate(true, true, true, true, true, true, 6329, 6330).visible);
  assert(!calculate(true, true, true, false, true, true, 0, 0).visible);
  assert(calculate(true, true, true, false, true, true, 20000, 20000).fraction == 0.0f);
  return 0;
}
