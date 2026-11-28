#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Informational overlay only: never consumed by controls, cruise, or ATC.
namespace navi_signal_hud {
constexpr uint64_t FRESH_NS = 2500000000ULL;
constexpr int MIN_COUNTDOWN_SECONDS = 6;
enum class Shade { unknown, red, yellow, green };

struct Lamp {
  std::string direction;
  std::string state;
  bool remaining_valid = false;
  int remaining_seconds = 0;
  int group_index = 0;
  int signal_index = 0;
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
      now_ns - receive_ns > FRESH_NS || lamps.empty() || lamps.size() > 4) return false;
  for (const auto &lamp : lamps) {
    if (!direction_label(lamp.direction) || !state_label(lamp.state) ||
        lamp.group_index < 0 || lamp.group_index > 7 ||
        lamp.signal_index < 0 || lamp.signal_index > 11 ||
        (lamp.remaining_valid && (lamp.remaining_seconds < 0 || lamp.remaining_seconds > 600))) return false;
  }
  return true;
}

// Four *positions* in the compact pill: red / amber / left arrow / straight green.
// These are explicit indicators; no first-result-wins assumption about lanes.
struct FourLights {
  bool valid = false;
  bool red = false;
  bool amber = false;
  bool left = false;
  bool straight = false;
  bool red_blink = false;
  bool amber_blink = false;
  bool left_blink = false;
  bool straight_blink = false;
  bool remaining_valid = false;
  int remaining_seconds = 0;
  Shade countdown_shade = Shade::unknown;
};

inline int countdown_priority(const Lamp &lamp) {
  // Match the SDK's steady-state icon eligibility, including directions which
  // have no dedicated position in this compact strip. Blinks have priority 0.
  if (lamp.direction == "U_TURN" && lamp.state != "GREEN") return 0;
  if (lamp.state == "GREEN") return 3;
  if (lamp.state == "YELLOW") return 2;
  if (lamp.state == "RED") return 1;
  return 0;
}

inline FourLights four_lights(bool connected, bool present, uint64_t now, uint64_t received,
                             const std::vector<Lamp> &lamps) {
  FourLights result;
  if (!visible(connected, present, now, received, lamps)) return result;
  const int group = lamps.front().group_index;
  bool seen_left = false;
  bool seen_straight = false;
  bool recognized = false;
  const Lamp *countdown = nullptr;
  int selected_priority = 0;

  for (const auto &lamp : lamps) {
    if (lamp.group_index != group) return {};
    // Choose before checking its countdown, so a short/missing high-priority
    // value never falls back to a lower-priority signal's longer countdown.
    const int priority = countdown_priority(lamp);
    if (priority > selected_priority) {
      countdown = &lamp;
      selected_priority = priority;
    }
    // Data for a different approach cannot be projected onto a straight/left lamp.
    if (lamp.direction != "STRAIGHT" && lamp.direction != "LEFT") continue;
    if (lamp.direction == "STRAIGHT") {
      if (seen_straight) return {};  // conflicting indications for same approach
      seen_straight = true;
    } else {
      if (seen_left) return {};
      seen_left = true;
    }
    recognized = true;
    const Shade c = shade(lamp.state);
    const bool blink = lamp.state.find("_BLINK") != std::string::npos;
    if (c == Shade::red) {
      result.red = true;
      result.red_blink |= blink;
    } else if (c == Shade::yellow) {
      result.amber = true;
      result.amber_blink |= blink;
    } else if (lamp.direction == "LEFT") {
      result.left = true;
      result.left_blink = blink;
    } else {
      result.straight = true;
      result.straight_blink = blink;
    }
  }
  result.valid = recognized;
  if (recognized && countdown != nullptr) {
    result.remaining_seconds = countdown->remaining_seconds;
    result.countdown_shade = shade(countdown->state);
    result.remaining_valid = countdown->remaining_valid && countdown->remaining_seconds >= MIN_COUNTDOWN_SECONDS;
  }
  return result;
}
}  // namespace navi_signal_hud
