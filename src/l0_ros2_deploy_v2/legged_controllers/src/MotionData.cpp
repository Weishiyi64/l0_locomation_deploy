#include "MotionData.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <codecvt>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace legged {
namespace motion {
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
std::int64_t nowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
std::int64_t steadyNs() {
  // The same Linux monotonic epoch is used by all three processes, independently of ROS time.
  timespec value{};
  require(::clock_gettime(CLOCK_MONOTONIC, &value) == 0, "Cannot read monotonic clock");
  return static_cast<std::int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
}
void checkDeadline(std::int64_t deadline) {
  require(deadline > steadyNs(), "Text generation exceeded its 3 second deadline");
}
std::int64_t integer(const Json& value, std::int64_t minimum, std::int64_t maximum) {
  require(value.is_number_integer(), "Expected an integer");
  if (value.is_number_unsigned())
    require(value.get<std::uint64_t>() <= static_cast<std::uint64_t>(maximum), "Integer exceeds limit");
  const auto result = value.get<std::int64_t>();
  require(result >= minimum && result <= maximum, "Integer outside permitted range");
  return result;
}
double finiteNumber(const Json& value) {
  require(value.is_number(), "Expected a number");
  const double result = value.get<double>();
  require(std::isfinite(result), "Expected a finite number");
  return result;
}
Json objectJson(const std::string& text) {
  require(!text.empty() && text.size() <= kJsonLimit, "JSON exceeds transport limit or is empty");
  auto result = Json::parse(text);
  require(result.is_object(), "Expected a JSON object");
  return result;
}
std::string encode(const Json& value) {
  // dump uses round-trip float formatting and strict UTF-8, unlike a YAML emitter.
  auto result = value.dump(-1, ' ', false, Json::error_handler_t::strict);
  require(result.size() <= kJsonLimit, "JSON exceeds 4 MiB transport limit");
  return result;
}
std::string readFile(const std::string& path, std::size_t maximum) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  require(file.good(), "Cannot open asset: " + path);
  const auto length = file.tellg();
  require(length > 0 && static_cast<std::uint64_t>(length) <= maximum, "Invalid asset size: " + path);
  std::string data(static_cast<std::size_t>(length), '\0');
  file.seekg(0);
  require(bool(file.read(&data[0], length)), "Short asset read: " + path);
  return data;
}
Json readJson(const std::string& path) { return objectJson(readFile(path)); }
std::string parentPath(const std::string& path) {
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? "." : (slash == 0 ? "/" : path.substr(0, slash));
}
std::string assetPath(const std::string& root, const std::string& relative) {
  require(!relative.empty() && relative.front() != '/', "Expected relative asset path");
  std::unique_ptr<char, decltype(&std::free)> base(::realpath(root.c_str(), nullptr), std::free);
  std::unique_ptr<char, decltype(&std::free)> path(::realpath((root + "/" + relative).c_str(), nullptr), std::free);
  require(base && path, "Cannot resolve deployment asset: " + relative);
  const std::string prefix = std::string(base.get()) == "/" ? "/" : std::string(base.get()) + "/";
  const std::string resolved(path.get());
  struct stat info{};
  require(resolved.compare(0, prefix.size(), prefix) == 0 && ::stat(path.get(), &info) == 0 &&
          S_ISREG(info.st_mode), "Deployment asset escapes package: " + relative);
  return resolved;
}
bool validId(const std::string& id) {
  return !id.empty() && id.size() <= 32 && std::all_of(id.begin(), id.end(), [](unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
}
std::string newId(const std::string& prefix) {
  // Linux supplies independent random bytes; IDs do not consume the gesture's seeded RNG.
  std::ifstream source("/dev/urandom", std::ios::binary);
  std::array<unsigned char, 14> bytes{};
  require(bool(source.read(reinterpret_cast<char*>(bytes.data()), bytes.size())), "Cannot create request ID");
  std::ostringstream result;
  result << prefix << std::hex << std::setfill('0');
  for (auto byte : bytes) result << std::setw(2) << static_cast<unsigned>(byte);
  return result.str();
}
bool contains(const std::deque<std::string>& ids, const std::string& id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}
void remember(std::deque<std::string>& ids, const std::string& id) {
  if (!contains(ids, id)) ids.push_back(id);
  if (ids.size() > 256) ids.pop_front();
}
std::vector<std::string> textCharacters(const std::string& text) {
  std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> codec;
  const auto points = codec.from_bytes(text);
  require(codec.to_bytes(points) == text, "Invalid UTF-8 text");
  std::vector<std::string> result;
  result.reserve(points.size());
  for (char32_t point : points) {
    require(point <= 0x10ffff && !(point >= 0xd800 && point <= 0xdfff), "Invalid Unicode character");
    result.push_back(codec.to_bytes(point));
  }
  return result;
}
void validateText(const std::string& text) {
  const auto characters = textCharacters(text);
  require(!characters.empty() && characters.size() <= 128, "text must contain 1..128 characters");
  std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> codec;
  bool nonspace = false;
  for (char32_t c : codec.from_bytes(text)) {
    const bool space = (c >= 9 && c <= 13) || (c >= 0x1c && c <= 0x20) || c == 0x85 || c == 0xa0 ||
        c == 0x1680 || (c >= 0x2000 && c <= 0x200a) || c == 0x2028 || c == 0x2029 ||
        c == 0x202f || c == 0x205f || c == 0x3000;
    nonspace = nonspace || !space;
  }
  require(nonspace, "text must not be whitespace");
}
std::uint32_t requestSeed(const Json& options) {
  require(options.is_object() && options.count("seed") && options.size() <= 2 &&
          (options.size() == 1 || options.count("deadline_monotonic_ns")), "Text options accept seed and internal deadline only");
  return static_cast<std::uint32_t>(integer(options.at("seed"), 1, 4294967295LL));
}
std::int64_t requestDeadline(const Json& options, std::int64_t fallback) {
  if (!options.count("deadline_monotonic_ns")) return fallback;
  const auto deadline = integer(options.at("deadline_monotonic_ns"), 1, std::numeric_limits<std::int64_t>::max());
  require(deadline <= steadyNs() + kGenerationNs, "Generation deadline exceeds 3 seconds");
  checkDeadline(deadline);
  return std::min(deadline, fallback);
}
Pose mirrorArms(const Pose& pose) {
  Pose result = pose;
  for (std::size_t j = 0; j < 3; ++j) {
    const float sign = j == 0 ? 1.0f : -1.0f;
    result[j] = pose[3 + j] * sign;
    result[3 + j] = pose[j] * sign;
  }
  return result;
}
Assets loadAssets(const std::string& deployment) {
  Assets result;
  const auto layout = readJson(deployment);
  const bool base = layout.at("format") == "wbc_base_graph_v1";
  const bool fixed = (base || layout.at("format") == "wbc_lora_graph_v1") && layout.value("kind", "") == "policy";
  require(fixed || layout.at("format") == "wbc_motion_deployment_v1", "Unsupported motion deployment");
  const auto root = parentPath(deployment);
  result.graph = fixed ? layout : readJson(assetPath(root, layout.at("model_manifest").get<std::string>()));
  std::vector<std::string> catalogs;
  if (fixed) catalogs.push_back(deployment);
  else for (const auto& path : layout.at("catalogs")) catalogs.push_back(assetPath(root, path.get<std::string>()));
  for (const auto& path : catalogs) {
    const auto catalog = readJson(path);
    require(catalog.at("format") == "wbc_motion_catalog_v1" ||
            ((base || catalog.at("format") == "wbc_lora_graph_v1") && catalog.value("kind", "") == "policy"), "Unsupported motion catalog");
    for (auto item : catalog.at("actions")) {
      const auto id = item.at("action_id").get<std::string>();
      require(validId(id) && !result.actions.count(id), "Duplicate/invalid motion ID");
      item["config_path"] = assetPath(parentPath(path), item.at("config").get<std::string>());
      if (!base) item["adapter_path"] = assetPath(parentPath(path), item.at("adapter_manifest").get<std::string>());
      result.actions[id] = std::move(item);
    }
  }
  require(!result.actions.empty(), "Empty motion catalog");
  if (!fixed) {
    result.tracker = result.actions.at(layout.at("text_action").get<std::string>());
    require(result.tracker.at("action_id") == "UpperBody_Enhanced" && result.tracker.value("json_only", false) &&
            result.tracker.at("parameter_source") == "lora", "Text tracker must be JSON-only UpperBody_Enhanced LoRA");
  }
  const auto& config = fixed ? result.actions.begin().value() : result.tracker;
  const auto robot = YAML::LoadFile(config.at("config_path").get<std::string>())["RobotCfg"];
  result.names = robot["joint_order_mj"].as<std::vector<std::string>>();
  require(result.names.size() == 21 && std::set<std::string>(result.names.begin(), result.names.end()).size() == 21,
          "WBC requires 21 unique joint names");
  if (!fixed) {
    const auto standing = YAML::LoadFile(assetPath(root, layout.at("standing_config").get<std::string>()));
    const double height = standing["standing_root_height_m"].as<double>();
    require(std::isfinite(height) && height > 0.05 && height < 2.0, "Invalid standing height");
    const auto legs = standing["standing_leg_joint_angles"];
    require(legs.IsMap() && legs.size() == 12, "Standing config needs 12 leg joints");
    for (std::size_t j = 0; j < 12; ++j)
      require(legs[result.names[j]] && std::isfinite(legs[result.names[j]].as<double>()), "Invalid standing joint");
  }
  return result;
}
Limits loadLimits(const std::string& mapping, const std::vector<std::string>& names) {
  const auto motors = YAML::LoadFile(mapping)["motors"];
  require(motors.IsSequence(), "Mapping needs a motors sequence");
  Limits result;
  for (const auto& name : names) {
    unsigned count = 0;
    std::pair<double, double> bounds;
    for (const auto& motor : motors) {
      if (motor["joint_name"].as<std::string>() != name) continue;
      ++count;
      bounds = {motor["joint_lower"].as<double>(), motor["joint_upper"].as<double>()};
      require(motor["enabled"].as<bool>() && std::isfinite(bounds.first) && std::isfinite(bounds.second) &&
              bounds.first < bounds.second, "Invalid/disabled mapping joint");
    }
    require(count == 1, "Missing/duplicate mapping joint: " + name);
    result.push_back(bounds);
  }
  return result;
}
Json checkSnapshot(Json snapshot, const std::vector<std::string>& names, const Limits& limits) {
  require(snapshot.at("joint_names") == Json(names) && limits.size() == 21, "Snapshot joint order differs from WBC Base");
  auto& joints = snapshot.at("joint_positions");
  const auto& position = snapshot.at("base_position");
  auto& orientation = snapshot.at("base_quaternion_wxyz");
  require(joints.is_array() && joints.size() == 21 && position.is_array() && position.size() == 3 &&
          orientation.is_array() && orientation.size() == 4, "Invalid snapshot dimensions");
  for (std::size_t j = 0; j < 21; ++j) {
    const auto q = finiteNumber(joints[j]);
    require(q >= limits[j].first - 1e-6 && q <= limits[j].second + 1e-6, "Snapshot exceeds robot limits");
    joints[j] = std::max(limits[j].first, std::min(q, limits[j].second));
  }
  for (const auto& value : position) finiteNumber(value);
  std::array<double, 4> q;
  double squared = 0;
  for (std::size_t j = 0; j < 4; ++j) { q[j] = finiteNumber(orientation[j]); squared += q[j] * q[j]; }
  require(squared > 1e-12 && std::abs(squared - 1) <= 0.1, "Invalid IMU quaternion");
  for (auto& value : q) value /= std::sqrt(squared);
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double roll = std::atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y));
  const double pitch = std::asin(std::max(-1.0, std::min(2 * (w*y - z*x), 1.0)));
  const double cr = std::cos(roll/2), sr = std::sin(roll/2), cp = std::cos(pitch/2), sp = std::sin(pitch/2);
  orientation = {cr*cp, sr*cp, cr*sp, -sr*sp};
  return snapshot;
}
Json makeDocument(Json trajectory, const Json& snapshot, const Assets& assets) {
  if (trajectory.is_null()) return nullptr;
  auto diagnostics = trajectory.at("model_diagnostics");
  for (const auto* name : {"token_ids", "material_ids", "backend", "inference_ms"}) diagnostics[name] = trajectory.at(name);
  Json frames = Json::array();
  const auto& positions = trajectory.at("positions");
  frames.get_ref<Json::array_t&>().reserve(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i)
    frames.push_back({{"frame_index", i}, {"time", i / 50.0}, {"joint_positions", positions[i]}});
  return {{"request_id", trajectory.at("request_id")}, {"format", "gmr_upper_body_joint_trajectory_v1"},
      {"robot", "l0_v5"}, {"angle_unit", "rad"}, {"fps", 50}, {"coordinate_contract", "robot_anatomical_v1"},
      {"joint_names", trajectory.at("joint_names")}, {"frame_count", positions.size()}, {"frames", std::move(frames)},
      {"wbc_start_state", snapshot},
      {"root_reference_source", "configured_standing"}, {"wbc_base_action", assets.tracker.at("action_id")},
      {"model_diagnostics", std::move(diagnostics)}, {"generation_request", trajectory.at("generation_request")},
      {"safety_adjustments", trajectory.at("limit_adjustments")}, {"expires_at_ms", nowMs() + 5000}};
}
std::string temporaryDirectory() {
  const char* configured = std::getenv("TMPDIR");
  std::string path = std::string(configured && *configured ? configured : "/tmp") + "/legged_txt2motion_XXXXXX";
  require(::mkdtemp(&path[0]) != nullptr, "Cannot create private trajectory directory");
  return path;
}
void saveDocument(const std::string& directory, const std::string& encoded, std::int64_t deadline) {
  checkDeadline(deadline);
  const auto temporary = directory + "/.reference.tmp";
  const auto final = directory + "/reference.json";
  const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
  require(fd >= 0, "Cannot create trajectory file");
  try {
    std::size_t done = 0;
    while (done < encoded.size()) {
      checkDeadline(deadline);
      const auto written = ::write(fd, encoded.data() + done, encoded.size() - done);
      if (written < 0 && errno == EINTR) continue;
      require(written > 0, "Trajectory write failed");
      done += static_cast<std::size_t>(written);
    }
  } catch (...) { ::close(fd); throw; }
  require(::close(fd) == 0, "Trajectory close failed");
  checkDeadline(deadline);
  // link publishes atomically without replacing an existing trajectory.
  require(::link(temporary.c_str(), final.c_str()) == 0, "Cannot publish trajectory file");
  require(::unlink(temporary.c_str()) == 0, "Cannot remove temporary trajectory file");
  checkDeadline(deadline);
}
void removeTemporaryDirectory(const std::string& directory) {
  if (directory.empty()) return;
  for (const auto* name : {"reference.json", ".reference.tmp"})
    require(::unlink((directory + "/" + name).c_str()) == 0 || errno == ENOENT, "Trajectory removal failed");
  require(::rmdir(directory.c_str()) == 0 || errno == ENOENT, "Trajectory directory removal failed");
}
}  // namespace motion
}  // namespace legged
