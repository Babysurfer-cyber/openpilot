#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>

// This helper only chooses a requested mode transition. A successful USB write
// is not a hardware acknowledgement: card must also observe fresh pandaStates.
struct HyundaiBlinkerSafetyEligibility {
  std::string brand;
  std::string fingerprint;
  uint64_t flags = 0;
  bool passive = false;
  bool dashcam_only = false;
  size_t panda_count = 0;
  size_t safety_config_count = 0;
  bool hyundai_canfd_mode = false;
  uint16_t safety_param = 0;

  bool supported() const {
    constexpr uint64_t kCanfdFlag = 1ULL << 13;
    constexpr uint64_t kCameraSccFlag = 1ULL << 3;
    constexpr uint64_t kBlinkerTestFlag = 1ULL << 27;
    constexpr uint16_t kEvCameraParam = 1U | 8U;
    return brand == "hyundai" && fingerprint == "KIA_EV6" &&
           (flags & (kCanfdFlag | kCameraSccFlag)) == (kCanfdFlag | kCameraSccFlag) && !(flags & kBlinkerTestFlag) &&
           !passive && !dashcam_only && panda_count == 1 && safety_config_count == 1 &&
           hyundai_canfd_mode && (safety_param & kEvCameraParam) == kEvCameraParam && !(safety_param & 1024U);
  }
};

class HyundaiBlinkerSafetyTransition {
public:
  enum class Action { NONE, TEST, NORMAL, ERROR };

  void reset(const std::string &initial_request = {}) {
    ignored_request_ = initial_request;
    last_request_.clear();
    active_nonce_.clear();
    ack_.clear();
    used_nonces_.clear();
  }

  Action request(const std::string &value, bool supported) {
    // Removing a request, stopping card, or losing a file must not restore
    // driving safety while the parking ECU might still be disabled.
    if (value.empty() || value == ignored_request_ || value == last_request_) return Action::NONE;
    last_request_ = value;

    const auto colon = value.find(':');
    if (colon == std::string::npos) return reject(value);
    const std::string mode = value.substr(0, colon);
    const std::string nonce = value.substr(colon + 1);
    if (!validNonce(nonce)) return reject(value);

    if (mode == "test") {
      if (!supported) return reject(value);
      if (active_nonce_ == nonce) {
        ack_ = value;  // An idempotent replay never reinitializes Panda state.
        return Action::NONE;
      }
      if (!active_nonce_.empty() || used_nonces_.count(nonce)) return reject(value);
      active_nonce_ = nonce;
      used_nonces_.insert(nonce);
      ack_ = value;
      return Action::TEST;
    }
    if (mode == "normal") {
      if (active_nonce_.empty()) {
        // card may replace test:<nonce> with normal:<nonce> before pandad's
        // next poll. No ECU action has been authorized by this coordinator;
        // acknowledge cancellation without another USB safety reset. card
        // must still observe a new normal pandaState before resuming output.
        used_nonces_.insert(nonce);
        ack_ = value;
        return Action::NONE;
      }
      if (active_nonce_ == nonce) {
        active_nonce_.clear();
        ack_ = value;
        return Action::NORMAL;
      }
    }
    return reject(value);
  }

  const std::string &ack() const { return ack_; }
  const std::string &activeNonce() const { return active_nonce_; }

private:
  static bool validNonce(const std::string &nonce) {
    if (nonce.empty() || nonce.size() > 64) return false;
    for (unsigned char c : nonce) {
      if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') || c == '-' || c == '_')) return false;
    }
    return true;
  }

  Action reject(const std::string &value) {
    ack_ = "error:" + value.substr(0, 128);
    return Action::ERROR;
  }

  std::string ignored_request_;
  std::string last_request_;
  std::string active_nonce_;
  std::string ack_;
  std::unordered_set<std::string> used_nonces_;
};
