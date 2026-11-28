#include "selfdrive/ui/navi_signal_hud.h"
#include <cassert>
#include <utility>

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
  assert(four_lights(true, true, now, now-FRESH_NS, lamps).valid);
  assert(!four_lights(true, true, now, now-FRESH_NS-1ULL, lamps).valid);

  lamps = {{"STRAIGHT", "RED", true, 25, 0, 0}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && !strip.straight && strip.countdown_shade == Shade::red);
  lamps = {{"STRAIGHT", "YELLOW", true, 2, 0, 0}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.amber && !strip.red && !strip.remaining_valid);
  lamps = {{"LEFT", "GREEN_BLINK", true, 15, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.left && strip.left_blink && !strip.straight && !strip.remaining_valid);

  // Simultaneous straight + left may have different expiry seconds.
  lamps = {{"STRAIGHT", "RED", true, 25, 0, 0},
           {"LEFT", "GREEN", true, 15, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && strip.left && strip.remaining_valid);
  assert(strip.remaining_seconds == 15 && strip.countdown_shade == Shade::green);

  lamps[1].remaining_seconds=25;
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.remaining_valid && strip.remaining_seconds == 25);
  assert(strip.countdown_shade == Shade::green);

  lamps[1].remaining_valid=false;
  assert(!four_lights(true, true, now, now, lamps).remaining_valid);
  lamps[1].remaining_valid=true;

  // A lower-priority missing countdown must not hide the selected green one.
  lamps[0].remaining_valid=false;
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.remaining_valid && strip.remaining_seconds == 25);
  lamps[0].remaining_valid=true;

  // Select by state before testing the six-second threshold. No red fallback.
  lamps[1].remaining_seconds=4;
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && strip.left && !strip.remaining_valid);
  lamps[1].remaining_seconds=15;

  lamps = {{"STRAIGHT", "GREEN", true, 20, 0, 0},
           {"LEFT", "RED", true, 30, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && strip.straight && !strip.left);
  assert(strip.remaining_valid && strip.remaining_seconds == 20 && strip.countdown_shade == Shade::green);

  lamps = {{"STRAIGHT", "RED", true, 25, 0, 0},
           {"LEFT", "YELLOW", true, 8, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && strip.amber && !strip.straight && !strip.left);
  assert(strip.remaining_valid && strip.remaining_seconds == 8 && strip.countdown_shade == Shade::yellow);

  // Equal priority uses wire-list order, even if signal_index or time differs.
  lamps = {{"LEFT", "GREEN", true, 20, 0, 9},
           {"STRAIGHT", "GREEN", true, 8, 0, 0}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.straight && strip.left && strip.remaining_valid && strip.remaining_seconds == 20);
  std::swap(lamps[0], lamps[1]);
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.remaining_valid && strip.remaining_seconds == 8);
  lamps[0].remaining_valid=false;
  assert(!four_lights(true, true, now, now, lamps).remaining_valid);
  lamps[0].remaining_valid=true;
  lamps[0].remaining_seconds=5;
  assert(!four_lights(true, true, now, now, lamps).remaining_valid);

  // Only steady states rank for countdowns; blinking lights still illuminate.
  lamps = {{"STRAIGHT", "RED", true, 25, 0, 0},
           {"LEFT", "GREEN_BLINK", true, 15, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && strip.left && strip.left_blink);
  assert(strip.remaining_valid && strip.remaining_seconds == 25 && strip.countdown_shade == Shade::red);
  lamps[0].state="RED_BLINK";
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && strip.red_blink && strip.left && strip.left_blink && !strip.remaining_valid);

  for (int seconds : {0, 5, 6, 600}) {
    lamps = {{"STRAIGHT", "RED", true, seconds, 0, 0}};
    strip = four_lights(true, true, now, now, lamps);
    assert(strip.valid && strip.red);
    assert(strip.remaining_valid == (seconds >= 6));
    if (strip.remaining_valid) assert(strip.remaining_seconds == seconds && strip.countdown_shade == Shade::red);
  }

  // All steady straight/left combinations retain their four original lamps.
  const char *states[] = {"RED", "YELLOW", "GREEN"};
  const Shade shades[] = {Shade::red, Shade::yellow, Shade::green};
  for (int straight = 0; straight < 3; ++straight) {
    for (int left = 0; left < 3; ++left) {
      lamps = {{"STRAIGHT", states[straight], true, 12, 0, 0},
               {"LEFT", states[left], true, 18, 0, 1}};
      strip = four_lights(true, true, now, now, lamps);
      assert(strip.valid && strip.remaining_valid);
      assert(strip.red == (straight == 0 || left == 0));
      assert(strip.amber == (straight == 1 || left == 1));
      assert(strip.straight == (straight == 2));
      assert(strip.left == (left == 2));
      assert(!strip.red_blink && !strip.amber_blink && !strip.straight_blink && !strip.left_blink);
      assert(strip.remaining_seconds == (straight >= left ? 12 : 18));
      assert(strip.countdown_shade == shades[straight >= left ? straight : left]);
    }
  }

  // Countdown follows SDK-eligible received indications, without drawing
  // right/U-turn signals as straight or left lamps.
  lamps = {{"STRAIGHT", "RED", true, 25, 0, 0},
           {"RIGHT", "GREEN", true, 15, 0, 1}};
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.valid && strip.red && !strip.left && !strip.straight && !strip.amber);
  assert(strip.remaining_valid && strip.remaining_seconds == 15 && strip.countdown_shade == Shade::green);
  lamps[1].state="YELLOW";
  lamps[1].remaining_seconds=8;
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.red && !strip.amber && strip.remaining_valid && strip.remaining_seconds == 8);
  assert(strip.countdown_shade == Shade::yellow);

  lamps[1].direction="U_TURN";
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.remaining_valid && strip.remaining_seconds == 25 && strip.countdown_shade == Shade::red);
  lamps[1].state="RED";
  lamps[1].remaining_seconds=40;
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.remaining_valid && strip.remaining_seconds == 25);
  lamps[1].state="GREEN";
  strip = four_lights(true, true, now, now, lamps);
  assert(strip.red && !strip.left && !strip.straight && strip.remaining_valid && strip.remaining_seconds == 40);
  lamps[1].remaining_seconds=4;
  assert(!four_lights(true, true, now, now, lamps).remaining_valid);

  lamps = {{"STRAIGHT", "RED", true, 25, 0, 0},
           {"LEFT", "GREEN", true, 15, 0, 1}};

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
