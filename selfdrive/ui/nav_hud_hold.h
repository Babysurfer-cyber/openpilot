#pragma once

#include <cmath>

// Presentation-only tolerance for brief navigation IPC dropouts.
// Never use this state for cruise, ATC, braking or route-control decisions.
namespace nav_hud_hold {

constexpr double MAX_HOLD_S = 2.0;

enum class Decision {
  hidden,
  fresh,
  held,
};

class State {
public:
  Decision update(double now_s, bool fresh_route_sample, bool explicit_end, bool route_changed) {
    if (!std::isfinite(now_s) || explicit_end || route_changed) {
      reset();
      return Decision::hidden;
    }
    if (fresh_route_sample) {
      last_good_s_ = now_s;
      return Decision::fresh;
    }
    if (last_good_s_ >= 0.0 && now_s >= last_good_s_ &&
        now_s - last_good_s_ <= MAX_HOLD_S) {
      return Decision::held;
    }
    reset();
    return Decision::hidden;
  }

  void reset() { last_good_s_ = -1.0; }

private:
  double last_good_s_ = -1.0;
};

}  // namespace nav_hud_hold
