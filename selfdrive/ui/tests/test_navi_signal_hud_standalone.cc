#include "selfdrive/ui/navi_signal_hud.h"
#include <cassert>

using namespace navi_signal_hud;
int main() {
  const uint64_t now = 10000000000ULL;
  std::vector<Lamp> lamps = {{"STRAIGHT", "GREEN", true, 12, 0, 0}};
  auto strip = four_lights(true, true, now, now-100000000ULL, lamps);
  assert(strip.valid && strip.straight && !strip.left && !strip.red && !strip.amber);
  assert(strip.remaining_valid && strip.remaining_seconds == 12);
  assert(!four_lights(false, true, now, now-100000000ULL, lamps).valid);
  assert(!four_lights(true, false, now, now-100000000ULL, lamps).valid);
  assert(!four_lights(true, true, now, now-3000000000ULL, lamps).valid);
  assert(!four_lights(true, true, now, now+1ULL, lamps).valid);

  lamps = {{"STRAIGHT", "RED", true, 25, 0, 0}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && !strip.straight && strip.countdown_shade == Shade::red);
  lamps = {{"STRAIGHT", "YELLOW", true, 2, 0, 0}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.amber && !strip.red);
  lamps = {{"LEFT", "GREEN_BLINK", true, 15, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.left && strip.left_blink && !strip.straight);

  // Simultaneous straight + left may have different expiry seconds.
  lamps = {{"STRAIGHT", "RED", true, 25, 0, 0},
           {"LEFT", "GREEN", true, 15, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && strip.left && !strip.remaining_valid);

  lamps[1].remaining_seconds=25;
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.remaining_valid && strip.remaining_seconds == 25);

  lamps[1].remaining_valid=false;
  assert(!four_lights(true, true, now, now, lamps).remaining_valid);
  lamps[1].remaining_valid=true;

  // Must not merge signals from different junctions or same approach twice.
  lamps[1].group_index=1;
  assert(!four_lights(true, true, now, now, lamps).valid);
  lamps[1].group_index=0;
  lamps[1].direction="STRAIGHT";
  assert(!four_lights(true, true, now, now, lamps).valid);

  lamps = {{"RIGHT", "GREEN", true, 10, 0, 0}};
  assert(!four_lights(true, true, now, now, lamps).valid);
  lamps = {{"STRAIGHT", "BLUE", true, 10, 0, 0}};
  assert(!four_lights(true, true, now, now, lamps).valid);
  lamps = {{"STRAIGHT", "RED", true, 700, 0, 0}};
  assert(!four_lights(true, true, now, now, lamps).valid);

  lamps = {{"STRAIGHT", "RED_BLINK", false, 0, 0, 0},
           {"LEFT", "GREEN", false, 0, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && strip.red_blink && strip.left && !strip.remaining_valid);
}
