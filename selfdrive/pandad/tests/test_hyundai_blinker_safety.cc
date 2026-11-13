#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
// Panda's can.h defines this macro before the production helper is included.
#define CANFD
#include "selfdrive/pandad/hyundai_blinker_safety.h"

using Transition = HyundaiBlinkerSafetyTransition;
using Action = Transition::Action;

static HyundaiBlinkerSafetyEligibility eligible() {
  return {"hyundai", "KIA_EV6", (1ULL << 13) | (1ULL << 3), false, false, 1, 1, true, 1 | 4 | 8 | 16};
}

TEST_CASE("Only the original supported EV6 bus-zero layout authorizes a test") {
  auto valid = eligible();
  REQUIRE(valid.supported());
  auto verify = [](auto change) {
    auto e = eligible();
    change(e);
    REQUIRE_FALSE(e.supported());
  };
  verify([](auto &e) { e.brand = "kia"; });
  verify([](auto &e) { e.fingerprint = "KIA_EV6_PE"; });
  verify([](auto &e) { e.flags &= ~(1ULL << 13); });
  verify([](auto &e) { e.flags &= ~(1ULL << 3); });
  verify([](auto &e) { e.flags |= 1ULL << 27; });
  verify([](auto &e) { e.passive = true; });
  verify([](auto &e) { e.dashcam_only = true; });
  verify([](auto &e) { e.panda_count = 2; });
  verify([](auto &e) { e.safety_config_count = 2; });
  verify([](auto &e) { e.hyundai_canfd_mode = false; });
  verify([](auto &e) { e.safety_param &= ~1U; });
  verify([](auto &e) { e.safety_param &= ~8U; });
  verify([](auto &e) { e.safety_param |= 1024U; });
}

TEST_CASE("New supported sessions enter and explicitly restore exactly once") {
  Transition t;
  REQUIRE(t.request("test:session-1", true) == Action::TEST);
  REQUIRE(t.activeNonce() == "session-1");
  REQUIRE(t.ack() == "test:session-1");
  for (int i = 0; i < 10; ++i) REQUIRE(t.request("test:session-1", true) == Action::NONE);
  REQUIRE(t.request("normal:session-1", true) == Action::NORMAL);
  REQUIRE(t.activeNonce().empty());
  REQUIRE(t.ack() == "normal:session-1");
  for (int i = 0; i < 10; ++i) REQUIRE(t.request("normal:session-1", true) == Action::NONE);
  REQUIRE(t.request("test:session-2", true) == Action::TEST);
  REQUIRE(t.request("normal:session-2", true) == Action::NORMAL);
}

TEST_CASE("Missing requests never release an active test") {
  Transition t;
  REQUIRE(t.request("test:1", true) == Action::TEST);
  for (int i = 0; i < 10; ++i) REQUIRE(t.request("", true) == Action::NONE);
  REQUIRE(t.activeNonce() == "1");
  REQUIRE(t.ack() == "test:1");
  REQUIRE(t.request("normal:wrong-session", true) == Action::ERROR);
  REQUIRE(t.activeNonce() == "1");
  REQUIRE(t.request("test:new-session", true) == Action::ERROR);
  REQUIRE(t.activeNonce() == "1");
  REQUIRE(t.request("normal:1", true) == Action::NORMAL);
}

TEST_CASE("An unsupported request makes no transition and cancellation never changes safety") {
  Transition t;
  REQUIRE(t.request("test:1", false) == Action::ERROR);
  REQUIRE(t.ack() == "error:test:1");
  REQUIRE(t.activeNonce().empty());
  REQUIRE(t.request("normal:1", false) == Action::NONE);
  REQUIRE(t.ack() == "normal:1");
  REQUIRE(t.activeNonce().empty());
}

TEST_CASE("Malformed requests fail without changing an active session") {
  Transition t;
  REQUIRE(t.request("test:valid_nonce", true) == Action::TEST);
  for (const auto &value : {"1", "0", "test:", "normal:", "test:with:colon", "test:with space", "normal:valid_nonce\n", "anything:valid_nonce"}) {
    REQUIRE(t.request(value, true) == Action::ERROR);
    REQUIRE(t.activeNonce() == "valid_nonce");
  }
  REQUIRE(t.request("test:" + std::string(65, 'a'), true) == Action::ERROR);
  REQUIRE(t.request("normal:valid_nonce", true) == Action::NORMAL);
}

TEST_CASE("Completed nonces cannot replay a diagnostic experiment") {
  Transition t;
  REQUIRE(t.request("test:first", true) == Action::TEST);
  REQUIRE(t.request("normal:first", true) == Action::NORMAL);
  REQUIRE(t.request("test:second", true) == Action::TEST);
  REQUIRE(t.request("normal:second", true) == Action::NORMAL);
  REQUIRE(t.request("test:first", true) == Action::ERROR);
  REQUIRE(t.activeNonce().empty());
}

TEST_CASE("Startup and offroad reset ignore previously stored requests") {
  Transition t;
  t.reset("test:old");
  REQUIRE(t.request("test:old", true) == Action::NONE);
  REQUIRE(t.ack().empty());
  REQUIRE(t.activeNonce().empty());
  REQUIRE(t.request("test:fresh", true) == Action::TEST);
  t.reset("test:fresh");  // New ignition or a new PandaSafety instance.
  REQUIRE(t.request("test:fresh", true) == Action::NONE);
  REQUIRE(t.request("normal:fresh", true) == Action::NONE);
  REQUIRE(t.request("test:next", true) == Action::TEST);
}

TEST_CASE("Idempotent active requests can recover acknowledgement after a wrong token") {
  Transition t;
  REQUIRE(t.request("test:1", true) == Action::TEST);
  REQUIRE(t.request("normal:2", true) == Action::ERROR);
  REQUIRE(t.request("test:1", true) == Action::NONE);
  REQUIRE(t.ack() == "test:1");
  REQUIRE(t.activeNonce() == "1");
}

TEST_CASE("A cancellation received before its test request is acknowledged without entering safety") {
  Transition t;
  REQUIRE(t.request("normal:cancel-before-poll", true) == Action::NONE);
  REQUIRE(t.ack() == "normal:cancel-before-poll");
  REQUIRE(t.activeNonce().empty());
  REQUIRE(t.request("test:cancel-before-poll", true) == Action::ERROR);
  REQUIRE(t.activeNonce().empty());
  REQUIRE(t.request("test:fresh", true) == Action::TEST);
  REQUIRE(t.request("normal:another-cancel", true) == Action::ERROR);
  REQUIRE(t.activeNonce() == "fresh");
  REQUIRE(t.request("normal:fresh", true) == Action::NORMAL);
}
