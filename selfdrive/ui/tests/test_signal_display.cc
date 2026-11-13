#include "catch2/catch.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include "selfdrive/ui/signal_display.h"

using namespace signal_display;

namespace {
Track lamp(int id = 1, Color color = Color::red) {
  return {id, {600., 200., 650., 220.}, color, color != Color::unknown, false};
}
Record packet(double timestamp = 10., uint32_t frame_id = 200) {
  return {"session-a", frame_id, timestamp, 1344, 760, {lamp()}};
}
QJsonObject json_packet() {
  return {{"version", 1}, {"stream", "road"}, {"size", QJsonArray{1344, 760}},
          {"frame_id", 200}, {"timestamp", 10.}, {"session", "session-a"},
          {"tracks", QJsonArray{QJsonObject{{"id", 1}, {"box", QJsonArray{600, 200, 650, 220}},
            {"raw", "red"}, {"state", "red"}, {"age", 0}}}}};
}
void write(const QString &path, const QJsonObject &obj) {
  QFile file(path);
  REQUIRE(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
  const auto bytes = QJsonDocument(obj).toJson(QJsonDocument::Compact);
  REQUIRE(file.write(bytes) == bytes.size());
}
}  // namespace

TEST_CASE("signal display capture time bounds repeated packets and unknown color") {
  State state;
  auto record = packet();
  state.update(record, 10.1);
  REQUIRE(state.color(10.1) == Color::red);
  REQUIRE_FALSE(state.held(10.1));
  state.update(record, 10.5);
  REQUIRE(state.held(10.5));
  record.timestamp = 11.; record.frame_id = 220;
  record.tracks[0] = lamp(1, Color::unknown);
  state.update(record, 11.1);
  REQUIRE(state.color(11.1) == Color::red);
  REQUIRE(state.held(11.1));
  REQUIRE(state.tracks(11.1, 220, 11.1, 1344, 760, false)[0].held);
  REQUIRE(state.color(12.01) == Color::unknown);
  state.update(record, 12.1);
  REQUIRE(state.color(12.1) == Color::unknown);
  REQUIRE_FALSE(state.visible(13.01));
}

TEST_CASE("signal display fresh red green and mixed replace previous colors") {
  State state;
  auto record = packet();
  state.update(record, 10.1);
  record.timestamp = 10.2; record.frame_id += 4;
  record.tracks = {lamp(1, Color::green)};
  state.update(record, 10.25);
  REQUIRE(state.color(10.25) == Color::green);
  REQUIRE_FALSE(state.held(10.25));
  record.timestamp = 10.3; record.frame_id += 2;
  record.tracks.push_back(lamp(2, Color::red));
  state.update(record, 10.35);
  REQUIRE(state.color(10.35) == Color::mixed);
}

TEST_CASE("signal display reset rejects time reversal session changes and invalid packets") {
  State state;
  auto record = packet();
  state.update(record, 10.1);
  SECTION("session change") { record.session = "session-b"; record.timestamp = 10.2; }
  SECTION("frame reversal") { record.frame_id = 199; record.timestamp = 10.2; }
  SECTION("capture reversal") { record.timestamp = 9.9; }
  record.tracks = {};
  state.update(record, 10.3);
  REQUIRE(state.color(10.3) == Color::unknown);
  record.timestamp = NAN;
  state.update(record, 10.3);
  REQUIRE_FALSE(state.visible(10.3));
  record = packet(); record.timestamp = 11.;
  state.update(record, 10.3);
  REQUIRE_FALSE(state.visible(10.3));
}

TEST_CASE("signal boxes require a fresh matching road camera and geometry") {
  State state;
  state.update(packet(), 10.1);
  REQUIRE(state.tracks(10.1, 200, 10.1, 1344, 760, false).size() == 1);
  REQUIRE(state.tracks(10.1, 200, 10.1, 1344, 760, true).empty());
  REQUIRE(state.tracks(10.1, 200, 10.1, 1928, 1208, false).empty());
  REQUIRE(state.tracks(10.5, 200, 10.1, 1344, 760, false).empty());
  REQUIRE(state.tracks(10.1, 200, 10.2, 1344, 760, false).empty());
  REQUIRE(state.tracks(10.1, 241, 10.1, 1344, 760, false).empty());
  auto held = state.tracks(10.5, 210, 10.5, 1344, 760, false);
  REQUIRE(held.size() == 1);
  REQUIRE(held[0].held);
  REQUIRE_FALSE(held[0].confirmed);
  REQUIRE(state.tracks(12.01, 240, 12.01, 1344, 760, false).empty());
}

TEST_CASE("signal display identity handoff preserves bounded UI color only") {
  State state;
  auto record = packet();
  state.update(record, 10.1);
  record.timestamp = 10.2; record.frame_id += 4;
  record.tracks = {lamp(2, Color::unknown)};
  state.update(record, 10.25);
  auto boxes = state.tracks(10.25, 204, 10.25, 1344, 760, false);
  REQUIRE(boxes.size() == 1);
  REQUIRE(boxes[0].id == 2);
  REQUIRE(boxes[0].color == Color::red);
  REQUIRE(boxes[0].held);
  REQUIRE_FALSE(boxes[0].confirmed);
  REQUIRE(state.color(12.01) == Color::unknown);
}

TEST_CASE("signal display inverse crop maps the reference centre and full width") {
  const auto identity = source_box(lamp(), 1344, 760);
  REQUIRE(identity == lamp().box);
  Track reference{1, {0., 0., 1344., 760.}};
  const auto raw = source_box(reference, 1928, 1208);
  REQUIRE(raw[0] == Approx(0.));
  REQUIRE(raw[2] == Approx(1928.));
  REQUIRE((raw[1] + raw[3]) / 2. == Approx(604.));
  REQUIRE(raw[1] > 0.);
  REQUIRE(raw[3] < 1208.);
  const auto t = reference_transform(1928, 1208);
  REQUIRE(raw[1] * t[0] + t[2] == Approx(0.).margin(1e-9));
  REQUIRE(raw[3] * t[0] + t[2] == Approx(760.));
  REQUIRE_FALSE(supported_geometry(2688, 1520));
}

TEST_CASE("Qt signal display packet reader rejects malformed stale and oversize input") {
  QTemporaryDir temp;
  REQUIRE(temp.isValid());
  const auto path = temp.filePath("display.json");
  SignalDisplayReader reader(path.toStdString());
  auto record = json_packet();
  write(path, record);
  REQUIRE(reader.read(true, 10.1).color(10.1) == Color::red);
  SECTION("future") { record["timestamp"] = 12.; }
  SECTION("expired") { record["timestamp"] = 7.; }
  SECTION("oversize") { record["extra"] = QString(9000, 'a'); }
  SECTION("wrong stream") { record["stream"] = "wideRoad"; }
  SECTION("wrong dimensions") { record["size"] = QJsonArray{1928, 1208}; }
  SECTION("missing session") { record.remove("session"); }
  SECTION("bad frame") { record["frame_id"] = -1; }
  SECTION("fractional frame") { record["frame_id"] = 200.5; }
  SECTION("nonlist tracks") { record["tracks"] = "invalid"; }
  SECTION("too many tracks") { QJsonArray tracks; for (int i = 0; i < 21; ++i) tracks.append(QJsonObject{}); record["tracks"] = tracks; }
  write(path, record);
  const auto &result = reader.read(true, 10.2);
  REQUIRE_FALSE(result.visible(10.2));
  REQUIRE(result.color(10.2) == Color::unknown);
}

TEST_CASE("Qt signal display geometry metadata must match the actual producer transform") {
  QTemporaryDir temp;
  const auto path = temp.filePath("display.json");
  SignalDisplayReader reader(path.toStdString());
  auto record = json_packet();
  const auto transform = reference_transform(1928, 1208);
  record["source_size"] = QJsonArray{1928, 1208};
  record["reference_transform"] = QJsonArray{transform[0], transform[1], transform[2]};
  write(path, record);
  REQUIRE(reader.read(true, 10.1).record().source_width == 1928);
  REQUIRE(reader.read(true, 10.1).tracks(10.1, 200, 10.1, 1928, 1208, false).size() == 1);
  record["reference_transform"] = QJsonArray{1., 0., 0.};
  write(path, record);
  REQUIRE_FALSE(reader.read(true, 10.2).visible(10.2));
}

TEST_CASE("Qt signal display disabled mode never consumes an old observation") {
  QTemporaryDir temp;
  const auto path = temp.filePath("display.json");
  SignalDisplayReader reader(path.toStdString());
  write(path, json_packet());
  REQUIRE_FALSE(reader.read(false, 10.1).visible(10.1));
  REQUIRE(reader.read(true, 10.2).color(10.2) == Color::red);
  REQUIRE_FALSE(reader.read(false, 10.3).visible(10.3));
  REQUIRE(reader.read(false, 10.3).color(10.3) == Color::unknown);
  REQUIRE_FALSE(reader.read(true, 12.1).visible(12.1));
}
