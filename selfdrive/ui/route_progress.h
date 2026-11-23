#pragma once

// Display-only route progress. Optional per-section congestion from the
// patched CarrotNM sender is validated and rendered separately in carrot.cc.
// If traffic is absent or invalid, the original neutral/cyan progress remains.
namespace route_progress {

struct Snapshot {
  bool visible = false;
  int total_m = 0;
  int traveled_m = 0;
  int remaining_m = 0;
  float fraction = 0.0f;
};

// Live route and guidance status must agree. On disconnect, reroute, off-route
// or stale/absent data we never retain the previous progress marker.
inline Snapshot calculate(bool connected, bool status_present,
                          bool guidance_active, bool off_route,
                          bool route_present, bool route_meta_present,
                          int total_m, int remaining_m) {
  if (!connected || !status_present || !guidance_active || off_route ||
      !route_present || !route_meta_present ||
      total_m <= 0 || total_m > 2000000 ||
      remaining_m < 0 || remaining_m > total_m) {
    return {};
  }
  const int traveled_m = total_m - remaining_m;
  return {true, total_m, traveled_m, remaining_m,
          static_cast<float>(traveled_m) / static_cast<float>(total_m)};
}

}  // namespace route_progress
