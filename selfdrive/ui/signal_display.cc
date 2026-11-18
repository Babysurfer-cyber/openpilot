#include "selfdrive/ui/signal_display.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

namespace {
bool number(const QJsonValue &value, double *result) {
  if (!value.isDouble() || !std::isfinite(value.toDouble())) return false;
  *result = value.toDouble();
  return true;
}

bool integer(const QJsonValue &value, int64_t minimum, int64_t maximum, int64_t *result) {
  double x;
  if (!number(value, &x) || x < minimum || x > maximum || x != std::floor(x)) return false;
  *result = int64_t(x);
  return true;
}

bool parse(const QByteArray &bytes, signal_display::Record *record) {
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(bytes, &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) return false;
  const auto obj = document.object();
  const auto size = obj.value("size").toArray();
  int64_t frame_id;
  if (obj.value("version").toInt(-1) != 1 || obj.value("stream").toString() != "road" || size.size() != 2 ||
      size[0].toInt(-1) != signal_display::REFERENCE_WIDTH || size[1].toInt(-1) != signal_display::REFERENCE_HEIGHT ||
      !obj.value("session").isString() || !number(obj.value("timestamp"), &record->timestamp) ||
      !integer(obj.value("frame_id"), 0, UINT32_MAX, &frame_id) || !obj.value("tracks").isArray()) return false;
  record->frame_id = uint32_t(frame_id);
  record->session = obj.value("session").toString().toStdString();

  // Reference-sized old packets are still accepted. Other camera geometries
  // require the producer's explicit, matching scale/crop metadata.
  if (obj.contains("source_size") || obj.contains("reference_transform")) {
    const auto source = obj.value("source_size").toArray();
    const auto transform = obj.value("reference_transform").toArray();
    int64_t width, height;
    if (source.size() != 2 || transform.size() != 3 ||
        !integer(source[0], 1, 8192, &width) || !integer(source[1], 1, 8192, &height) ||
        !signal_display::supported_geometry(int(width), int(height))) return false;
    record->source_width = int(width);
    record->source_height = int(height);
    const auto expected = signal_display::reference_transform(int(width), int(height));
    for (int i = 0; i < 3; ++i) {
      double value;
      if (!number(transform[i], &value) || std::abs(value - expected[i]) > 1e-6) return false;
    }
  }
  const auto tracks = obj.value("tracks").toArray();
  if (tracks.size() > signal_display::MAX_TRACKS) return false;
  std::map<int, bool> ids;
  for (const auto &value : tracks) {
    if (!value.isObject()) continue;
    const auto obj = value.toObject();
    const auto box = obj.value("box").toArray();
    int64_t id, age;
    if (box.size() != 4 || !integer(obj.value("id"), 1, 999999, &id) ||
        !integer(obj.value("age"), 0, 0, &age) || ids.count(int(id))) continue;
    signal_display::Track track;
    track.id = int(id);
    bool coordinates = true;
    for (int i = 0; i < 4; ++i) coordinates &= number(box[i], &track.box[i]);
    if (!coordinates || !signal_display::valid_track(track)) continue;
    const auto raw = obj.value("raw").toString();
    track.color = raw == "red" ? signal_display::Color::red :
                  raw == "green" ? signal_display::Color::green : signal_display::Color::unknown;
    track.confirmed = track.color != signal_display::Color::unknown && obj.value("state").toString() == raw;
    record->tracks.push_back(track);
    ids[track.id] = true;
  }
  return true;
}
}  // namespace

const signal_display::State &SignalDisplayReader::read(bool enabled, double now) {
  if (!enabled || !std::isfinite(now) || now < next_read - .05 - 1e-9) {
    state.reset();
    bytes.clear();
    next_read = 0.;
    if (!enabled || !std::isfinite(now)) return state;
  }
  if (now < next_read) return state;
  next_read = now + .05;
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::ReadOnly)) {
    state.reset();
    bytes.clear();
    return state;
  }
  const QByteArray raw = file.read(signal_display::MAX_BYTES + 1);
  if (raw.size() > signal_display::MAX_BYTES) {
    state.reset();
    bytes.clear();
    return state;
  }
  const auto current_bytes = raw.toStdString();
  if (current_bytes != bytes) {
    signal_display::Record record;
    if (!parse(raw, &record) || !signal_display::valid_record(record, now)) {
      state.reset();
      bytes.clear();
      return state;
    }
    state.update(record, now);
    bytes = current_bytes;
  }
  return state;
}
