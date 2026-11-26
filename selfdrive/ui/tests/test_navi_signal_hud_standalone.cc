#include "selfdrive/ui/navi_signal_hud.h"
#include <cassert>

using namespace navi_signal_hud;
int main() {
  const uint64_t now = 10000000000ULL;
  std::vector<Lamp> lamps = {{"STRAIGHT", "RED", true, 25}};
  assert(visible(true, true, now, now-100000000ULL, lamps));
  assert(compact_visible(true, true, now, now-100000000ULL, lamps));
  assert(!compact_visible(true, true, now, now-3000000000ULL, lamps));
  assert(shade(lamps[0].state) == Shade::red);
  assert(!visible(false, true, now, now-100000000ULL, lamps));
  assert(!visible(true, false, now, now-100000000ULL, lamps));
  assert(!visible(true, true, now, now-3000000000ULL, lamps));
  assert(!visible(true, true, now, now+1ULL, lamps));
  assert(!visible(true, true, now, 0, lamps));
  lamps[0].state="YELLOW_BLINK";
  assert(shade(lamps[0].state) == Shade::yellow);
  lamps[0].remaining_seconds=700;
  assert(!visible(true,true,now,now,lamps));
  lamps[0].remaining_valid=false;
  assert(visible(true,true,now,now,lamps));
  lamps[0].state="BLUE";
  assert(!visible(true,true,now,now,lamps));
  lamps[0].state="GREEN";
  lamps[0].direction="UNKNOWN";
  assert(!visible(true,true,now,now,lamps));
  lamps[0].direction="LEFT";
  lamps.push_back({"RIGHT","GREEN_BLINK",true,10});
  assert(visible(true,true,now,now,lamps));
  assert(!compact_visible(true,true,now,now,lamps)); // two directions: no arbitrary lane selection
  lamps.push_back({"STRAIGHT","RED",true,1});
  assert(!visible(true,true,now,now,lamps));
}
