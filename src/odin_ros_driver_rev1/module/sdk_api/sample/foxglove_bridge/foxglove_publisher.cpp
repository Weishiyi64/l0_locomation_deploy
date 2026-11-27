#include "foxglove_publisher.h"

#include <foxglove/foxglove.hpp>
#include <foxglove/websocket.hpp>

#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace odin::sdk;
using namespace std::chrono_literals;

// ============================================================================
// JSON schema definitions for Foxglove well-known types
// ============================================================================

// foxglove.PointCloud — binary point buffer + field layout
static const std::string kSchemaPointCloud = R"({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object","properties":{"sec":{"type":"integer"},"nsec":{"type":"integer"}}},
    "frame_id":{"type":"string"},
    "pose":{"type":"object"},
    "point_stride":{"type":"integer"},
    "fields":{"type":"array","items":{"type":"object"}},
    "data":{"type":"string"}
  }
})";

// foxglove.CompressedImage — JPEG/PNG binary blob
static const std::string kSchemaCompressedImage = R"({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object","properties":{"sec":{"type":"integer"},"nsec":{"type":"integer"}}},
    "frame_id":{"type":"string"},
    "data":{"type":"string"},
    "format":{"type":"string"}
  }
})";

// foxglove.FrameTransform — translation + quaternion rotation
static const std::string kSchemaFrameTransform = R"({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object","properties":{"sec":{"type":"integer"},"nsec":{"type":"integer"}}},
    "parent_frame_id":{"type":"string"},
    "child_frame_id":{"type":"string"},
    "translation":{"type":"object"},
    "rotation":{"type":"object"}
  }
})";

// foxglove.PoseInFrame — position + orientation
static const std::string kSchemaPoseInFrame = R"({
  "type":"object",
  "properties":{
    "timestamp":{"type":"object","properties":{"sec":{"type":"integer"},"nsec":{"type":"integer"}}},
    "frame_id":{"type":"string"},
    "pose":{"type":"object"}
  }
})";

// ============================================================================
// Protobuf FileDescriptorSet builder — generates schema bytes at runtime
//   so Foxglove Studio can parse protobuf-encoded binary messages.
// ============================================================================

namespace fds {  // file-descriptor-set helpers

inline void pv(std::vector<uint8_t>& b, uint64_t v) {
  while (v > 0x7F) { b.push_back(static_cast<uint8_t>(v | 0x80)); v >>= 7; }
  b.push_back(static_cast<uint8_t>(v));
}
inline void pt(std::vector<uint8_t>& b, uint32_t f, uint32_t w) {
  pv(b, (static_cast<uint64_t>(f) << 3) | w);
}
inline void ps(std::vector<uint8_t>& b, uint32_t f, const char* s) {
  size_t len = std::strlen(s);
  pt(b, f, 2); pv(b, len);
  b.insert(b.end(), s, s + len);
}
inline void pu(std::vector<uint8_t>& b, uint32_t f, uint32_t v) {
  pt(b, f, 0); pv(b, v);
}
inline void pb(std::vector<uint8_t>& b, uint32_t f, const std::vector<uint8_t>& d) {
  pt(b, f, 2); pv(b, d.size());
  b.insert(b.end(), d.begin(), d.end());
}

// DescriptorProto.field: FieldDescriptorProto(name=1, number=3, type=5, label=6)
inline std::vector<uint8_t> F(const char* name, uint32_t num, uint32_t type) {
  std::vector<uint8_t> b;
  ps(b, 1, name); pu(b, 3, num); pu(b, 5, type); pu(b, 4, 1); // label=OPTIONAL
  return b;
}
// Field with type_name (nested message type, e.g. ".Time")
inline std::vector<uint8_t> FN(const char* name, uint32_t num, const char* tn) {
  auto f = F(name, num, 11); // TYPE_MESSAGE
  ps(f, 6, tn);
  return f;
}
// Field: repeated message type
inline std::vector<uint8_t> FR(const char* name, uint32_t num, const char* tn) {
  auto f = FN(name, num, tn);
  // overwrite label byte: field 4 = LABEL_REPEATED(3)
  for (size_t i = 0; i < f.size(); ++i) {
    if (f[i] == 0x20 && i + 1 < f.size() && f[i + 1] == 0x01) {
      f[i + 1] = 0x03; break;
    }
  }
  return f;
}

// DescriptorProto (message): name=1, field=2, nested_type=3
inline std::vector<uint8_t> Msg(const char* name,
    std::initializer_list<std::vector<uint8_t>> fields,
    std::initializer_list<std::vector<uint8_t>> nested = {}) {
  std::vector<uint8_t> b;
  ps(b, 1, name);
  for (auto& f : fields) pb(b, 2, f);
  for (auto& n : nested) pb(b, 3, n);
  return b;
}

// FileDescriptorProto: name=1, package=2, message_type=4, syntax=12
inline std::vector<uint8_t> FDP(const char* pkg,
    std::initializer_list<std::vector<uint8_t>> msgs) {
  std::vector<uint8_t> b;
  ps(b, 1, "odin_foxglove.proto");
  ps(b, 2, pkg);
  for (auto& m : msgs) pb(b, 4, m);
  ps(b, 12, "proto3");
  return b;
}

// FileDescriptorSet: file=1
inline std::vector<uint8_t> FDS(std::initializer_list<std::vector<uint8_t>> files) {
  std::vector<uint8_t> b;
  for (auto& f : files) pb(b, 1, f);
  return b;
}

// ---- Well-known nested message types ----

inline std::vector<uint8_t> TimeMsg() {
  return Msg("Time", {F("sec", 1, 13), F("nsec", 2, 13)});  // uint32, uint32
}
inline std::vector<uint8_t> Vector3Msg() {
  return Msg("Vector3", {F("x", 1, 2), F("y", 2, 2), F("z", 3, 2)});  // float, float, float
}
inline std::vector<uint8_t> QuaternionMsg() {
  return Msg("Quaternion",
      {F("x", 1, 2), F("y", 2, 2), F("z", 3, 2), F("w", 4, 2)});  // float
}
inline std::vector<uint8_t> PoseMsg() {
  return Msg("Pose",
      {FN("position", 1, ".Vector3"), FN("orientation", 2, ".Quaternion")},
      {Vector3Msg(), QuaternionMsg()});
}

// ---- Full FileDescriptorSet for each schema ----

inline std::vector<uint8_t> BuildPointCloudFDS() {
  auto pef = Msg("PackedElementField",
      {F("name", 1, 9), F("offset", 2, 13), F("type", 3, 5)});  // string,uint32,int32
  auto pc  = Msg("PointCloud", {
      FN("timestamp", 1, ".Time"),
      F("frame_id", 2, 9),          // string
      FN("pose", 3, ".Pose"),
      F("point_stride", 4, 13),     // uint32
      FR("fields", 5, ".PackedElementField"),
      F("data", 6, 12),             // bytes
  }, {TimeMsg(), PoseMsg(), pef});
  return FDS({FDP("foxglove", {pc})});
}

inline std::vector<uint8_t> BuildCompressedImageFDS() {
  auto ci = Msg("CompressedImage", {
      FN("timestamp", 1, ".Time"),
      F("frame_id", 2, 9),   // string
      F("data", 3, 12),      // bytes
      F("format", 4, 9),     // string
  }, {TimeMsg()});
  return FDS({FDP("foxglove", {ci})});
}

}  // namespace fds

// Custom telemetry schema (no built-in Foxglove panel — shown as raw JSON)
static const std::string kSchemaTelemetry = R"({
  "type":"object",
  "properties":{
    "uptime_s":{"type":"integer"},
    "cpu_pct":{"type":"integer"},
    "mem_used_kb":{"type":"integer"},
    "mem_total_kb":{"type":"integer"},
    "mem_use_rate":{"type":"integer"},
    "load_1":{"type":"number"},
    "load_5":{"type":"number"},
    "load_15":{"type":"number"}
  }
})";

// ============================================================================
// Helpers
// ============================================================================

std::string FoxglovePublisher::Base64Encode(const uint8_t* data, size_t len) {
  static constexpr char kTable[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((len + 2) / 3) * 4);
  for (size_t i = 0; i < len; i += 3) {
    uint32_t a = data[i];
    uint32_t b = (i + 1 < len) ? data[i + 1] : 0;
    uint32_t c = (i + 2 < len) ? data[i + 2] : 0;
    uint32_t triple = (a << 16) | (b << 8) | c;
    out += kTable[(triple >> 18) & 0x3F];
    out += kTable[(triple >> 12) & 0x3F];
    out += (i + 1 < len) ? kTable[(triple >> 6) & 0x3F] : '=';
    out += (i + 2 < len) ? kTable[triple & 0x3F] : '=';
  }
  return out;
}

std::string FoxglovePublisher::TimestampJson(uint64_t ns) {
  return "{\"sec\":" + std::to_string(ns / 1'000'000'000ULL) +
         ",\"nsec\":" + std::to_string(ns % 1'000'000'000ULL) + "}";
}

std::string FoxglovePublisher::EscapeJson(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:   out += c;      break;
    }
  }
  return out;
}

/// Helper: log a JSON string to a RawChannel with optional timestamp.
static void LogJson(foxglove::RawChannel& ch, const std::string& json,
                    std::optional<uint64_t> ts = std::nullopt) {
  ch.log(reinterpret_cast<const std::byte*>(json.data()), json.size(), ts);
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

FoxglovePublisher::FoxglovePublisher(uint16_t port) : port_(port) {}

FoxglovePublisher::~FoxglovePublisher() {
  Stop();
}

// ============================================================================
// Start / Stop / Run
// ============================================================================

bool FoxglovePublisher::Start() {
  foxglove::WebSocketServerOptions options;
  options.port = port_;
  options.host = "0.0.0.0";
  options.name = "Odin LiDAR Bridge";
  options.supported_encodings = {"json", "protobuf"};

  auto srv = foxglove::WebSocketServer::create(std::move(options));
  if (!srv.has_value()) {
    std::cerr << "Failed to create WebSocket server: "
              << foxglove::strerror(srv.error()) << "\n";
    return false;
  }
  server_ = std::make_unique<foxglove::WebSocketServer>(std::move(srv.value()));
  std::cout << "WebSocket server started on ws://0.0.0.0:" << port_ << "\n\n";

  // Helper lambda: create a channel with a JSON schema.
  auto makeChannel = [this](const std::string& topic,
                            const std::string& schema_name,
                            const std::string& schema_data) -> foxglove::RawChannel* {
    foxglove::Schema schema;
    schema.name = schema_name;
    schema.encoding = "jsonschema";
    schema.data = reinterpret_cast<const std::byte*>(schema_data.data());
    schema.data_len = schema_data.size();

    auto result = foxglove::RawChannel::create(topic, "json", schema);
    if (!result.has_value()) {
      std::cerr << "  [FAIL] " << topic << ": "
                << foxglove::strerror(result.error()) << "\n";
      return nullptr;
    }
    std::cout << "  [OK]   " << topic << " (" << schema_name << ")\n";
    return new foxglove::RawChannel(std::move(result.value()));
  };

  // Helper: same as makeChannel but for binary (protobuf) message encoding.
  //   schema_data must be a protobuf FileDescriptorSet binary.
  auto makeBinaryChannel = [this](const std::string& topic,
                                  const std::string& schema_name,
                                  const std::vector<uint8_t>& schema_data) -> foxglove::RawChannel* {
    foxglove::Schema schema;
    schema.name = schema_name;
    schema.encoding = "protobuf";
    schema.data = reinterpret_cast<const std::byte*>(schema_data.data());
    schema.data_len = schema_data.size();

    auto result = foxglove::RawChannel::create(topic, "protobuf", schema);
    if (!result.has_value()) {
      std::cerr << "  [FAIL] " << topic << ": "
                << foxglove::strerror(result.error()) << "\n";
      return nullptr;
    }
    std::cout << "  [OK]   " << topic << " (" << schema_name << ", protobuf)\n";
    return new foxglove::RawChannel(std::move(result.value()));
  };

  // Build protobuf FileDescriptorSet for each schema type.
  auto fdsPointCloud      = fds::BuildPointCloudFDS();
  auto fdsCompressedImage = fds::BuildCompressedImageFDS();

  std::cout << "Advertising channels:\n";
  ch_points_ = makeBinaryChannel("/lidar/points", "foxglove.PointCloud", fdsPointCloud);
  ch_slam_   = makeBinaryChannel("/slam/points",  "foxglove.PointCloud", fdsPointCloud);
  ch_img0_   = makeBinaryChannel("/camera/left",  "foxglove.CompressedImage", fdsCompressedImage);
  ch_img1_   = makeBinaryChannel("/camera/right", "foxglove.CompressedImage", fdsCompressedImage);
  ch_odom_    = makeChannel("/odom",     "foxglove.PoseInFrame", kSchemaPoseInFrame);
  ch_odom_hf_ = makeChannel("/odom_hf",  "foxglove.PoseInFrame", kSchemaPoseInFrame);
  ch_tf_      = makeChannel("/tf",        "foxglove.FrameTransform", kSchemaFrameTransform);
  ch_telem_   = makeChannel("/telemetry", "odin.Telemetry", kSchemaTelemetry);
  std::cout << "\n";

  return true;
}

void FoxglovePublisher::Stop() {
  // Close channels first
  if (ch_points_) { ch_points_->close(); delete ch_points_; ch_points_ = nullptr; }
  if (ch_slam_)   { ch_slam_->close();   delete ch_slam_;   ch_slam_ = nullptr; }
  if (ch_img0_)   { ch_img0_->close();   delete ch_img0_;   ch_img0_ = nullptr; }
  if (ch_img1_)   { ch_img1_->close();   delete ch_img1_;   ch_img1_ = nullptr; }
  if (ch_odom_)    { ch_odom_->close();    delete ch_odom_;    ch_odom_ = nullptr; }
  if (ch_odom_hf_) { ch_odom_hf_->close(); delete ch_odom_hf_; ch_odom_hf_ = nullptr; }
  if (ch_tf_)      { ch_tf_->close();      delete ch_tf_;      ch_tf_ = nullptr; }
  if (ch_telem_)  { ch_telem_->close();  delete ch_telem_;  ch_telem_ = nullptr; }
  if (server_)    { server_->stop();     server_.reset(); }
}

void FoxglovePublisher::Run() {
  auto last_print = std::chrono::steady_clock::now();
  last_static_tf_ = last_print;
  while (g_running.load()) {
    std::this_thread::sleep_for(200ms);
    auto now = std::chrono::steady_clock::now();

    // Emit static identity TF entries every 1 s so Foxglove can resolve
    // imu→lidar, imu→camera0, imu→camera1 (device only sends odom→imu).
    if (now - last_static_tf_ >= 1s) {
      PublishStaticTf();
      last_static_tf_ = now;
    }

    if (now - last_print >= 2s) {
      std::cout << "  pts=" << cnt_pts_.load()
                << " slam=" << cnt_slam_.load()
                << " img0=" << cnt_img0_.load()
                << " img1=" << cnt_img1_.load()
                << " odom=" << cnt_odom_.load()
                << " odom_hf=" << cnt_odom_hf_.load()
                << " tf=" << cnt_tf_.load()
                << " telem=" << cnt_telem_.load()
                << "  clients=" << (server_ ? server_->clientCount() : 0)
                << "    \r" << std::flush;
      last_print = now;
    }
  }
}

// ============================================================================
// Static callbacks — forward to publish methods
// ============================================================================

void FoxglovePublisher::OnPointCloud(const OdinPointCloudPacket& p, void* ud) {
  static_cast<FoxglovePublisher*>(ud)->PublishPointCloud(p);
}
void FoxglovePublisher::OnSlam(const OdinSlamPacket& p, void* ud) {
  static_cast<FoxglovePublisher*>(ud)->PublishSlam(p);
}
void FoxglovePublisher::OnCamera(const OdinCameraPacket& p, void* ud) {
  static_cast<FoxglovePublisher*>(ud)->PublishCamera(p, false);
}
void FoxglovePublisher::OnCamera2(const OdinCameraPacket& p, void* ud) {
  static_cast<FoxglovePublisher*>(ud)->PublishCamera(p, true);
}
void FoxglovePublisher::OnOdom(const OdinOdomPacket& p, OdomSourceType t, void* ud) {
  static_cast<FoxglovePublisher*>(ud)->PublishOdom(p, t);
}
void FoxglovePublisher::OnTf(const OdinTfPacket& p, void* ud) {
  static_cast<FoxglovePublisher*>(ud)->PublishTf(p);
}
void FoxglovePublisher::OnTelemetry(const OdinTelemetryPacket& p, void* ud) {
  static_cast<FoxglovePublisher*>(ud)->PublishTelemetry(p);
}

// ============================================================================
// Protobuf serialization helpers for foxglove.PointCloud
// ============================================================================

namespace {
inline void PbVarint(std::vector<uint8_t>& b, uint64_t v) {
  while (v > 0x7F) { b.push_back(static_cast<uint8_t>(v | 0x80)); v >>= 7; }
  b.push_back(static_cast<uint8_t>(v));
}
inline void PbTag(std::vector<uint8_t>& b, uint32_t f, uint32_t w) {
  PbVarint(b, (static_cast<uint64_t>(f) << 3) | w);
}
// Encode a foxglove Pose (identity: position=0, orientation w=1) into buf.
inline void PbPoseIdentity(std::vector<uint8_t>& b) {
  // orientation.w = 1.0f  (field 4, wire=5=fixed32)
  uint8_t ow[5] = {0x25, 0x00, 0x00, 0x80, 0x3f};
  b.insert(b.end(), ow, ow + 5);
}
// Encode a PackedElementField {name, offset, type}.
inline void PbField(std::vector<uint8_t>& b, const char* name,
                    uint32_t offset, uint32_t type) {
  size_t nLen = std::strlen(name);
  // field 1: name (string)
  b.push_back(0x0a); PbVarint(b, nLen);
  b.insert(b.end(), name, name + nLen);
  // field 2: offset (uint32 varint)
  b.push_back(0x10); PbVarint(b, offset);
  // field 3: type (uint32 varint)
  b.push_back(0x18); PbVarint(b, type);
}
}  // namespace

// ============================================================================
// PublishPointCloud — OdinRawPoint<float> (x,y,z,intensity,confidence)
//   → foxglove.PointCloud protobuf  (point_stride=16, fields=x,y,z,intensity)
// ============================================================================

void FoxglovePublisher::PublishPointCloud(const OdinPointCloudPacket& pkt) {
  if (!ch_points_) return;
  const size_t n = pkt.points.size();
  if (n == 0) return;

  // Binary point data: 16 bytes per point (float32 x,y,z,intensity)
  std::vector<uint8_t> data(n * 16);
  for (size_t i = 0; i < n; ++i) {
    const auto& pt = pkt.points[i];
    // Odin SDK points are in millimeters; Foxglove expects meters.
    constexpr float kMmToM = 0.001f;
    float v[4] = {pt.x * kMmToM, pt.y * kMmToM, pt.z * kMmToM,
                  static_cast<float>(pt.intensity)};
    std::memcpy(data.data() + i * 16, v, 16);
  }

  std::vector<uint8_t> msg;
  msg.reserve(256 + data.size());

  // field 1: timestamp (Time {sec:varint, nsec:fixed32})
  uint64_t sec = pkt.timestamp / 1'000'000'000ULL;
  uint32_t nsec = static_cast<uint32_t>(pkt.timestamp % 1'000'000'000ULL);
  std::vector<uint8_t> ts;
  ts.push_back(0x08); PbVarint(ts, sec);
  ts.push_back(0x15);
  ts.push_back(nsec & 0xFF); ts.push_back((nsec >> 8) & 0xFF);
  ts.push_back((nsec >> 16) & 0xFF); ts.push_back((nsec >> 24) & 0xFF);
  PbTag(msg, 1, 2); PbVarint(msg, ts.size());
  msg.insert(msg.end(), ts.begin(), ts.end());

  // field 2: frame_id = "lidar" (matches ROS driver raw_frame_id)
  static const char* kLidar = "lidar";
  PbTag(msg, 2, 2); PbVarint(msg, 5);
  msg.insert(msg.end(), kLidar, kLidar + 5);

  // field 3: pose (identity)
  PbTag(msg, 3, 2); PbVarint(msg, 5); PbPoseIdentity(msg);

  // field 4: point_stride = 16
  PbTag(msg, 4, 0); PbVarint(msg, 16);

  // field 5: fields (repeated PackedElementField)
  auto addF = [&msg](const char* n, uint32_t o, uint32_t t) {
    std::vector<uint8_t> inner; PbField(inner, n, o, t);
    PbTag(msg, 5, 2); PbVarint(msg, inner.size());
    msg.insert(msg.end(), inner.begin(), inner.end());
  };
  addF("x", 0, 7); addF("y", 4, 7); addF("z", 8, 7); addF("intensity", 12, 7);

  // field 6: data (raw bytes)
  PbTag(msg, 6, 2); PbVarint(msg, data.size());
  msg.insert(msg.end(), data.begin(), data.end());

  ch_points_->log(reinterpret_cast<const std::byte*>(msg.data()), msg.size(),
                  pkt.timestamp);
  cnt_pts_++;
}

// ============================================================================
// PublishSlam — OdinSlamPoint<float> (x,y,z,r,g,b,a)
//   → foxglove.PointCloud protobuf  (point_stride=16)
// ============================================================================

void FoxglovePublisher::PublishSlam(const OdinSlamPacket& pkt) {
  if (!ch_slam_) return;
  const size_t n = pkt.points.size();
  if (n == 0) return;

  // 16 bytes per point: float32 x,y,z + uint8 r,g,b,a + 4 padding
  std::vector<uint8_t> data(n * 16);
  for (size_t i = 0; i < n; ++i) {
    const auto& pt = pkt.points[i];
    // Odin SDK points are in millimeters; Foxglove expects meters.
    constexpr float kMmToM = 0.001f;
    float xyz[3] = {pt.x * kMmToM, pt.y * kMmToM, pt.z * kMmToM};
    std::memcpy(data.data() + i * 16, xyz, 12);
    data[i * 16 + 12] = pt.r; data[i * 16 + 13] = pt.g;
    data[i * 16 + 14] = pt.b; data[i * 16 + 15] = pt.a;
  }

  std::vector<uint8_t> msg;
  msg.reserve(256 + data.size());

  uint64_t sec = pkt.timestamp / 1'000'000'000ULL;
  uint32_t nsec = static_cast<uint32_t>(pkt.timestamp % 1'000'000'000ULL);
  std::vector<uint8_t> ts;
  ts.push_back(0x08); PbVarint(ts, sec);
  ts.push_back(0x15);
  ts.push_back(nsec & 0xFF); ts.push_back((nsec >> 8) & 0xFF);
  ts.push_back((nsec >> 16) & 0xFF); ts.push_back((nsec >> 24) & 0xFF);
  PbTag(msg, 1, 2); PbVarint(msg, ts.size());
  msg.insert(msg.end(), ts.begin(), ts.end());

  // frame_id = "odom" (SLAM points are transformed using odom pose,
  // so they live in the odom frame — matches ROS driver convention)
  static const char* kOdomFrame = "odom";
  PbTag(msg, 2, 2); PbVarint(msg, 4);
  msg.insert(msg.end(), kOdomFrame, kOdomFrame + 4);

  PbTag(msg, 3, 2); PbVarint(msg, 5); PbPoseIdentity(msg);
  PbTag(msg, 4, 0); PbVarint(msg, 16);

  auto addF = [&msg](const char* n, uint32_t o, uint32_t t) {
    std::vector<uint8_t> inner; PbField(inner, n, o, t);
    PbTag(msg, 5, 2); PbVarint(msg, inner.size());
    msg.insert(msg.end(), inner.begin(), inner.end());
  };
  addF("x", 0, 7); addF("y", 4, 7); addF("z", 8, 7);
  addF("red", 12, 1); addF("green", 13, 1);
  addF("blue", 14, 1); addF("alpha", 15, 1);

  PbTag(msg, 6, 2); PbVarint(msg, data.size());
  msg.insert(msg.end(), data.begin(), data.end());

  ch_slam_->log(reinterpret_cast<const std::byte*>(msg.data()), msg.size(),
                pkt.timestamp);
  cnt_slam_++;
}

// ============================================================================
// PublishCamera — OdinCameraPacket (MJPEG / H264 payload)
//   → foxglove.CompressedImage JSON  (format="jpeg", data=base64)
// ============================================================================

void FoxglovePublisher::PublishCamera(const OdinCameraPacket& pkt, bool is_right) {
  foxglove::RawChannel* ch = is_right ? ch_img1_ : ch_img0_;
  if (!ch) return;
  if (pkt.payload.empty()) return;

  // Serialize CompressedImage as protobuf binary for Foxglove Studio Image panel.
  // Protobuf field layout:
  //   field 1 (Time)      : {sec:varint32, nsec:fixed32}
  //   field 2 (frame_id)  : string
  //   field 3 (data)      : bytes (raw JPEG/H264)
  //   field 4 (format)    : string
  std::vector<uint8_t> buf;
  buf.reserve(128 + pkt.payload.size());

  auto putVarint = [&buf](uint64_t v) {
    while (v > 0x7F) { buf.push_back(static_cast<uint8_t>(v | 0x80)); v >>= 7; }
    buf.push_back(static_cast<uint8_t>(v));
  };
  auto putTag = [&buf, &putVarint](uint32_t field, uint32_t wire) {
    putVarint((static_cast<uint64_t>(field) << 3) | wire);
  };

  // --- field 1: timestamp (nested message) ---
  uint64_t sec = pkt.timestamp / 1'000'000'000ULL;
  uint32_t nsec = static_cast<uint32_t>(pkt.timestamp % 1'000'000'000ULL);
  // Build inner message bytes first to know length
  std::vector<uint8_t> tsInner;
  // sec = field 1, varint (wire=0)
  tsInner.push_back(0x08);
  { uint64_t v = sec; while (v > 0x7F) { tsInner.push_back(static_cast<uint8_t>(v | 0x80)); v >>= 7; } tsInner.push_back(static_cast<uint8_t>(v)); }
  // nsec = field 2, fixed32 (wire=5)
  tsInner.push_back(0x15);
  tsInner.push_back(nsec & 0xFF); tsInner.push_back((nsec >> 8) & 0xFF);
  tsInner.push_back((nsec >> 16) & 0xFF); tsInner.push_back((nsec >> 24) & 0xFF);
  // Write outer field 1 + length + inner
  putTag(1, 2); putVarint(tsInner.size());
  buf.insert(buf.end(), tsInner.begin(), tsInner.end());

  // --- field 2: frame_id (string) ---
  std::string frameId = is_right ? "camera1" : "camera0";  // ROS driver naming
  putTag(2, 2); putVarint(frameId.size());
  buf.insert(buf.end(), frameId.begin(), frameId.end());

  // --- field 3: data (bytes — raw JPEG) ---
  putTag(3, 2); putVarint(pkt.payload.size());
  buf.insert(buf.end(), pkt.payload.begin(), pkt.payload.end());

  // --- field 4: format (string) ---
  const char* format = "jpg";
  if (pkt.format == OdinDataFormat::kImageH264) format = "h264";
  else if (pkt.format == OdinDataFormat::kImageH265) format = "h265";
  size_t fmtLen = std::strlen(format);
  putTag(4, 2); putVarint(fmtLen);
  buf.insert(buf.end(), reinterpret_cast<const uint8_t*>(format),
             reinterpret_cast<const uint8_t*>(format) + fmtLen);

  ch->log(reinterpret_cast<const std::byte*>(buf.data()), buf.size(), pkt.timestamp);
  if (is_right) cnt_img1_++; else cnt_img0_++;
}

// ============================================================================
// PublishOdom — OdinOdomData (pos[3], orient[4], lin_vel[3], ang_vel[3])
//   → foxglove.PoseInFrame JSON
// ============================================================================

void FoxglovePublisher::PublishOdom(const OdinOdomPacket& pkt,
                                    OdomSourceType odom_type) {
  // Dispatch to low-freq or high-freq channel
  foxglove::RawChannel* ch =
      (odom_type == OdomSourceType::kOdom2ImuHigh) ? ch_odom_hf_ : ch_odom_;
  if (!ch) return;
  if (pkt.data.empty()) return;

  // Use the latest odom entry in the packet
  const auto& d = pkt.data.back();
  std::string json =
      "{"
      "\"timestamp\":" + TimestampJson(pkt.timestamp) + ","
      "\"frame_id\":\"" +
          std::string(odom_type == OdomSourceType::kOdom2ImuHigh ? "odom_hf" : "odom") + "\","
      "\"pose\":{"
        "\"position\":{\"x\":" + std::to_string(d.pos[0]) +
                     ",\"y\":" + std::to_string(d.pos[1]) +
                     ",\"z\":" + std::to_string(d.pos[2]) + "},"
        "\"orientation\":{\"x\":" + std::to_string(d.orient[0]) +
                        ",\"y\":" + std::to_string(d.orient[1]) +
                        ",\"z\":" + std::to_string(d.orient[2]) +
                        ",\"w\":" + std::to_string(d.orient[3]) + "}"
      "}"
      "}";

  LogJson(*ch, json, pkt.timestamp);
  if (odom_type == OdomSourceType::kOdom2ImuHigh) cnt_odom_hf_++; else cnt_odom_++;
}

// ============================================================================
// PublishTf — OdinTfPacket (translation[3], rotation[4], frame_ids)
//   → foxglove.FrameTransform JSON
// ============================================================================

void FoxglovePublisher::PublishTf(const OdinTfPacket& pkt) {
  if (!ch_tf_) return;

  std::string json =
      "{"
      "\"timestamp\":" + TimestampJson(pkt.timestamp) + ","
      "\"parent_frame_id\":\"" + EscapeJson(pkt.frame_id) + "\","
      "\"child_frame_id\":\"" + EscapeJson(pkt.child_frame_id) + "\","
      "\"translation\":{\"x\":" + std::to_string(pkt.translation[0]) +
                       ",\"y\":" + std::to_string(pkt.translation[1]) +
                       ",\"z\":" + std::to_string(pkt.translation[2]) + "},"
      "\"rotation\":{\"x\":" + std::to_string(pkt.rotation[0]) +
                  ",\"y\":" + std::to_string(pkt.rotation[1]) +
                  ",\"z\":" + std::to_string(pkt.rotation[2]) +
                  ",\"w\":" + std::to_string(pkt.rotation[3]) + "}"
      "}";

  LogJson(*ch_tf_, json, pkt.timestamp);
  cnt_tf_++;
}

// ============================================================================
// PublishStaticTf — identity transforms to complete the TF tree.
//   Device only sends odom→imu; Foxglove needs imu→lidar, imu→camera0,
//   imu→camera1 so that point cloud / image frame_ids resolve.
// ============================================================================

void FoxglovePublisher::PublishStaticTf() {
  if (!ch_tf_) return;

  // Current time in ns (approximate)
  auto now_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
  std::string ts = TimestampJson(now_ns);

  // Identity transform: translation=(0,0,0), rotation=(0,0,0,1)
  auto identityJson = [&ts](const char* parent, const char* child) {
    return std::string(
        "{"
        "\"timestamp\":") + ts + ","
        "\"parent_frame_id\":\"" + parent + "\","
        "\"child_frame_id\":\"" + child + "\","
        "\"translation\":{\"x\":0,\"y\":0,\"z\":0},"
        "\"rotation\":{\"x\":0,\"y\":0,\"z\":0,\"w\":1}"
        "}";
  };

  static const struct { const char* parent; const char* child; } kStaticTf[] = {
      {"imu", "lidar"},
      {"imu", "camera0"},
      {"imu", "camera1"},
  };

  for (const auto& tf : kStaticTf) {
    auto json = identityJson(tf.parent, tf.child);
    LogJson(*ch_tf_, json, now_ns);
  }
}

// ============================================================================
// PublishTelemetry — OdinTelemetryData (cpu, mem, load, etc.)
//   → custom JSON (no built-in Foxglove panel)
// ============================================================================

void FoxglovePublisher::PublishTelemetry(const OdinTelemetryPacket& pkt) {
  if (!ch_telem_) return;

  const auto& d = pkt.data;
  std::string json =
      "{"
      "\"uptime_s\":" + std::to_string(d.uptime_s) + ","
      "\"cpu_pct\":" + std::to_string(d.cpu_pct) + ","
      "\"mem_used_kb\":" + std::to_string(d.mem_used_kb) + ","
      "\"mem_total_kb\":" + std::to_string(d.mem_total_kb) + ","
      "\"mem_use_rate\":" + std::to_string(d.mem_use_rate) + ","
      "\"load_1\":" + std::to_string(d.load_1) + ","
      "\"load_5\":" + std::to_string(d.load_5) + ","
      "\"load_15\":" + std::to_string(d.load_15) +
      "}";

  LogJson(*ch_telem_, json, pkt.timestamp);
  cnt_telem_++;
}
