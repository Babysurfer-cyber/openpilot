#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

// Observation-only display state. None of this memory is a control input.
namespace signal_display {
constexpr double FRESH_AGE = .35;
constexpr double DISPLAY_HOLD = 2.;
constexpr double CAMERA_SKEW = .25;
constexpr int REFERENCE_WIDTH = 1344;
constexpr int REFERENCE_HEIGHT = 760;
constexpr int MAX_TRACKS = 20;
constexpr int MAX_BYTES = 8192;

enum class Color { unknown = 0, red = 1, green = 2, mixed = 3 };

struct Track {
  int id = 0;
  std::array<double, 4> box = {};
  Color color = Color::unknown;
  bool confirmed = false;
  bool held = false;
};

struct Record {
  std::string session;
  uint32_t frame_id = 0;
  double timestamp = -INFINITY;
  int source_width = REFERENCE_WIDTH;
  int source_height = REFERENCE_HEIGHT;
  std::vector<Track> tracks;
};

inline bool supported_geometry(int width, int height) {
  return (width == REFERENCE_WIDTH && height == REFERENCE_HEIGHT) || (width == 1928 && height == 1208);
}

inline std::array<double, 3> reference_transform(int width, int height) {
  const double scale = double(REFERENCE_WIDTH) / width;
  return {scale, 0., (REFERENCE_HEIGHT - height * scale) / 2.};
}

inline std::array<double, 4> source_box(const Track &track, int width, int height) {
  const auto transform = reference_transform(width, height);
  return {(track.box[0] - transform[1]) / transform[0], (track.box[1] - transform[2]) / transform[0],
          (track.box[2] - transform[1]) / transform[0], (track.box[3] - transform[2]) / transform[0]};
}

inline bool valid_record(const Record &record, double now) {
  const double age = now - record.timestamp;
  return std::isfinite(now) && std::isfinite(record.timestamp) && age >= 0 && age <= DISPLAY_HOLD &&
         !record.session.empty() && record.session.size() <= 256 &&
         supported_geometry(record.source_width, record.source_height) && record.tracks.size() <= MAX_TRACKS;
}

inline bool valid_track(const Track &track) {
  const auto &b = track.box;
  return track.id > 0 && track.id < 1000000 &&
         std::all_of(b.begin(), b.end(), [](double x) { return std::isfinite(x); }) &&
         0 <= b[0] && b[0] < b[2] && b[2] <= REFERENCE_WIDTH &&
         0 <= b[1] && b[1] < b[3] && b[3] <= REFERENCE_HEIGHT;
}

inline double intersection_over_union(const Track &a, const Track &b) {
  const double area = std::max(0., std::min(a.box[2], b.box[2]) - std::max(a.box[0], b.box[0])) *
                      std::max(0., std::min(a.box[3], b.box[3]) - std::max(a.box[1], b.box[1]));
  const double total = (a.box[2] - a.box[0]) * (a.box[3] - a.box[1]) +
                       (b.box[2] - b.box[0]) * (b.box[3] - b.box[1]) - area;
  return area / std::max(total, 1.);
}

class State {
public:
  void reset() { *this = State(); }

  void update(const Record &record, double now) {
    if (!valid_record(record, now)) {
      reset();
      return;
    }
    if (have_record && (record.session != current.session || record.timestamp < current.timestamp ||
                        record.frame_id < current.frame_id || record.source_width != current.source_width ||
                        record.source_height != current.source_height)) {
      reset();
    }
    const bool changed = !have_record || record.session != current.session || record.timestamp != current.timestamp ||
                         record.frame_id != current.frame_id;
    if (!changed) return;  // Repeated reads never renew capture-based expiry.

    for (auto it = saved.begin(); it != saved.end();) {
      if (now - it->second.observed > DISPLAY_HOLD) it = saved.erase(it); else ++it;
    }

    std::map<int, Saved> inherited;
    int colors = 0;
    for (const auto &track : record.tracks) {
      if (!valid_track(track)) continue;
      colors |= int(track.color);
      if (track.color != Color::unknown || saved.count(track.id)) continue;
      std::vector<Saved> matches;
      for (const auto &[old_id, old] : saved) {
        const bool old_present = std::any_of(record.tracks.begin(), record.tracks.end(),
                                            [old_id](const Track &t) { return t.id == old_id; });
        if (!old_present && now - old.color_time <= DISPLAY_HOLD && intersection_over_union(old.track, track) >= .5) {
          matches.push_back(old);
        }
      }
      if (matches.size() == 1 && std::count_if(record.tracks.begin(), record.tracks.end(), [&matches](const Track &t) {
            return intersection_over_union(matches[0].track, t) >= .5;
          }) == 1) {
        inherited[track.id] = matches[0];
      }
    }
    if (colors) {
      summary_color = Color(colors);
      summary_color_time = record.timestamp;
    }
    // A replacement ID at the same housing must not leave duplicate old boxes.
    for (auto it = saved.begin(); it != saved.end();) {
      const int old_id = it->first;
      const bool old_present = std::any_of(record.tracks.begin(), record.tracks.end(),
                                          [old_id](const Track &t) { return t.id == old_id; });
      const auto &a = it->second.track.box;
      const bool overlaps = std::any_of(record.tracks.begin(), record.tracks.end(), [&a](const Track &t) {
        return valid_track(t) && std::max(a[0], t.box[0]) < std::min(a[2], t.box[2]) &&
               std::max(a[1], t.box[1]) < std::min(a[3], t.box[3]);
      });
      if (!old_present && overlaps) it = saved.erase(it); else ++it;
    }
    for (auto track : record.tracks) {
      if (!valid_track(track)) continue;
      const Saved *previous = saved.count(track.id) ? &saved.at(track.id) :
                              inherited.count(track.id) ? &inherited.at(track.id) : nullptr;
      double color_time = record.timestamp;
      if (track.color == Color::unknown && previous && now - previous->color_time <= DISPLAY_HOLD) {
        track.color = previous->track.color;
        track.confirmed = false;
        color_time = previous->color_time;
      }
      saved[track.id] = {track, record.timestamp, color_time, record.frame_id};
    }
    while (saved.size() > MAX_TRACKS) {
      auto oldest = std::min_element(saved.begin(), saved.end(), [](const auto &a, const auto &b) {
        return a.second.observed < b.second.observed;
      });
      saved.erase(oldest);
    }
    current = record;
    have_record = true;
  }

  Color color(double now) const {
    return now >= summary_color_time && now - summary_color_time <= DISPLAY_HOLD ? summary_color : Color::unknown;
  }
  bool held(double now) const {
    return color(now) != Color::unknown &&
           (now - summary_color_time > FRESH_AGE || summary_color_time != current.timestamp);
  }
  bool visible(double now) const { return have_record && valid_record(current, now); }

  std::vector<Track> tracks(double now, uint32_t frame_id, double camera_timestamp, int width, int height, bool wide) const {
    if (wide || !visible(now) || width != current.source_width || height != current.source_height ||
        !std::isfinite(camera_timestamp) || now < camera_timestamp || now - camera_timestamp > FRESH_AGE ||
        std::abs(camera_timestamp - current.timestamp) > DISPLAY_HOLD ||
        std::abs(int64_t(frame_id) - current.frame_id) > 40) return {};
    std::vector<std::pair<double, Track>> sorted;
    for (const auto &[id, old] : saved) {
      if (now < old.observed || now - old.observed > DISPLAY_HOLD) continue;
      Track track = old.track;
      if (now - old.color_time > DISPLAY_HOLD) {
        track.color = Color::unknown;
        track.confirmed = false;
      }
      const bool fresh = old.observed == current.timestamp && now - current.timestamp <= FRESH_AGE &&
                         std::abs(camera_timestamp - current.timestamp) <= CAMERA_SKEW &&
                         std::abs(int64_t(frame_id) - current.frame_id) <= 5;
      track.held = !fresh || old.color_time != old.observed;
      if (track.held) track.confirmed = false;
      sorted.emplace_back(old.observed, track);
    }
    std::stable_sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
    std::vector<Track> result;
    for (const auto &item : sorted) {
      if (result.size() >= MAX_TRACKS) break;
      result.push_back(item.second);
    }
    return result;
  }

  const Record &record() const { return current; }

private:
  struct Saved {
    Track track;
    double observed;
    double color_time;
    uint32_t frame_id;
  };
  bool have_record = false;
  Record current;
  std::map<int, Saved> saved;
  Color summary_color = Color::unknown;
  double summary_color_time = -INFINITY;
};

}  // namespace signal_display

class SignalDisplayReader {
public:
  explicit SignalDisplayReader(std::string path = "/dev/shm/carrot_signal_display.json") : path(std::move(path)) {}
  const signal_display::State &read(bool enabled, double now);

private:
  std::string path;
  std::string bytes;
  signal_display::State state;
  double next_read = 0.;
};
