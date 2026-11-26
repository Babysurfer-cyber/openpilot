#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Informational overlay ONLY; never consumed by controls, cruise, or ATC.
namespace navi_signal_hud {

constexpr uint64_t FRESH_NS = 2500000000ULL;

enum class Shade { unknown, red, yellow, green };

struct Lamp {
  std::string direction;
  std::string state;
  bool remaining_valid = false;
  int remaining_seconds = 0;
};

inline Shade shade(const std::string &state) {
  if (state == "RED" || state == "RED_BLINK") return Shade::red;
  if (state == "YELLOW" || state == "YELLOW_BLINK") return Shade::yellow;
  if (state == "GREEN" || state == "GREEN_BLINK") return Shade::green;
  return Shade::unknown;
}

inline const char *direction_label(const std::string &direction) {
  if (direction == "STRAIGHT") return "직진";
  if (direction == "LEFT") return "좌회전";
  if (direction == "RIGHT") return "우회전";
  if (direction == "U_TURN") return "유턴";
  return nullptr;
}

inline const char *state_label(const std::string &state) {
  if (state == "RED") return "빨강";
  if (state == "YELLOW") return "노랑";
  if (state == "GREEN") return "초록";
  if (state == "RED_BLINK") return "빨강 점멸";
  if (state == "YELLOW_BLINK") return "노랑 점멸";
  if (state == "GREEN_BLINK") return "초록 점멸";
  return nullptr;
}

inline bool visible(bool connected, bool signal_present, uint64_t now_ns, uint64_t receive_ns,
                    const std::vector<Lamp> &lamps) {
  if (!connected || !signal_present || receive_ns == 0 || now_ns < receive_ns ||
      now_ns - receive_ns > FRESH_NS || lamps.empty() || lamps.size() > 2) return false;
  for (const auto &lamp : lamps) {
    if (!direction_label(lamp.direction) || !state_label(lamp.state) ||
        (lamp.remaining_valid && (lamp.remaining_seconds < 0 || lamp.remaining_seconds > 600))) return false;
  }
  return true;
}

// Compact one-row panel is only safe for a single unambiguous signal.
// Multi-direction candidates must not be silently mapped to the driver's lane.
inline bool compact_visible(bool connected, bool signal_present, uint64_t now_ns, uint64_t receive_ns,
                            const std::vector<Lamp> &lamps) {
  return lamps.size() == 1 && visible(connected, signal_present, now_ns, receive_ns, lamps);
}

}  // namespace navi_signal_hud
