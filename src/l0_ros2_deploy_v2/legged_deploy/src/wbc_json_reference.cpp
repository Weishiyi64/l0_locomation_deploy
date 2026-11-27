#include "wbc_json_reference.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
std::int64_t nowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
double finite(const YAML::Node& value) {
  const double result = value.as<double>();
  require(std::isfinite(result), "non-finite JSON reference value");
  return result;
}
}  // namespace

std::vector<std::pair<double, double>> wbcJsonJointLimits(
    const std::string& mapping_file, const std::vector<std::string>& joint_names) {
  const auto motors = YAML::LoadFile(mapping_file)["motors"];
  require(motors.IsSequence(), "mapping has no motors sequence");
  std::vector<std::pair<double, double>> result;
  for (const auto& name : joint_names) {
    unsigned count = 0;
    std::pair<double, double> bounds;
    for (const auto& motor : motors) {
      if (motor["joint_name"].as<std::string>() != name) continue;
      ++count;
      bounds = {finite(motor["joint_lower"]), finite(motor["joint_upper"])};
      require(motor["enabled"].as<bool>() && bounds.first < bounds.second, "invalid/disabled joint limit");
    }
    require(count == 1, "missing/duplicate JSON WBC mapping joint");
    result.push_back(bounds);
  }
  return result;
}

WbcJsonReference::WbcJsonReference(const YAML::Node& root, const std::vector<std::string>& joint_names,
    const std::vector<std::pair<double, double>>& limits,
    const Joints& standing_joints, double standing_root_height_m) {
  require(joint_names.size() == 21 && limits.size() == 21, "JSON reference requires 21-joint WBC ABI");
  require(root.IsMap() && root["format"].as<std::string>("") == "gmr_upper_body_joint_trajectory_v1" &&
          root["robot"].as<std::string>("") == "l0_v5" && root["angle_unit"].as<std::string>("") == "rad" &&
          root["coordinate_contract"].as<std::string>("") == "robot_anatomical_v1" &&
          root["fps"].as<double>(0) == 50.0, "JSON reference format/robot/coordinate/FPS mismatch");
  id_ = root["request_id"].as<std::string>();
  require(id_.size() >= 3 && id_.size() <= 32 && id_.compare(0, 2, "T_") == 0 &&
      std::all_of(id_.begin(), id_.end(), [](unsigned char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9') || c == '-' || c == '_';
      }), "invalid text motion request ID");
  expires_at_ms_ = root["expires_at_ms"].as<std::int64_t>();
  const auto now = nowMs();
  require(expires_at_ms_ > now && expires_at_ms_ <= now + 10000, "expired/excessive JSON admission TTL");
  const auto names = root["joint_names"];
  require(names.IsSequence() && names.size() == 9, "JSON must contain nine upper-body joint names");
  for (std::size_t i = 0; i < 9; ++i)
    require(names[i].as<std::string>() == joint_names[12 + i], "JSON joint order differs from WBC Base");
  const auto snapshot = root["wbc_start_state"];
  const auto snapshot_names = snapshot["joint_names"];
  const auto snapshot_joints = snapshot["joint_positions"];
  require(snapshot_names.IsSequence() && snapshot_names.size() == 21 &&
          snapshot_joints.IsSequence() && snapshot_joints.size() == 21, "invalid WBC start snapshot");
  for (std::size_t j = 0; j < 21; ++j) {
    require(snapshot_names[j].as<std::string>() == joint_names[j], "snapshot joint order differs from WBC Base");
    const double q = finite(snapshot_joints[j]);
    require(q >= limits[j].first - 1e-6 && q <= limits[j].second + 1e-6, "snapshot outside robot joint limits");
    start_[j] = static_cast<float>(std::max(limits[j].first, std::min(q, limits[j].second)));
  }
  require(root["root_reference_source"].as<std::string>("") == "configured_standing", "text reference must use standing configuration");
  require(std::isfinite(standing_root_height_m) && standing_root_height_m > 0.05 && standing_root_height_m < 2.0,
          "invalid configured standing height");
  root_position_ = {{0.0, 0.0, standing_root_height_m}};
  root_orientation_ = {{1.0, 0.0, 0.0, 0.0}};
  for (std::size_t j = 0; j < 12; ++j)
    require(std::isfinite(standing_joints[j]) && standing_joints[j] >= limits[j].first &&
            standing_joints[j] <= limits[j].second, "text standing target outside robot joint limits");
  const auto token_ids = root["model_diagnostics"]["token_ids"];
  require(token_ids.IsSequence() && token_ids.size() == 3, "JSON needs head/left/right token identity");
  const std::array<int, 3> tokens{{token_ids[0].as<int>(), token_ids[1].as<int>(), token_ids[2].as<int>()}};
  require(tokens[0] >= 0 && tokens[0] < 47 && tokens[1] >= 0 && tokens[1] < 43 && tokens[2] >= 0 && tokens[2] < 43,
          "invalid text model token IDs");
  require(tokens[0] || tokens[1] || tokens[2], "no active body part");
  const auto frames = root["frames"];
  const auto count = root["frame_count"].as<std::size_t>(0);
  require(frames.IsSequence() && count == frames.size() && count >= 2 && count <= 5000,
          "invalid JSON frame count");
  frames_.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto frame = frames[i];
    require(frame["frame_index"].as<std::size_t>(count) == i &&
            std::abs(finite(frame["time"]) - i / 50.0) <= 1e-5, "invalid JSON frame timeline");
    const auto values = frame["joint_positions"];
    require(values.IsSequence() && values.size() == 9, "invalid upper-body frame size");
    Joints row = start_;
    std::copy_n(standing_joints.begin(), 12, row.begin());
    for (std::size_t j = 0; j < 9; ++j) {
      const double q = finite(values[j]);
      const auto& bound = limits[12 + j];
      require(q >= bound.first - 1e-6 && q <= bound.second + 1e-6, "JSON frame exceeds robot joint limits");
      if (i == 0 || i + 1 == count)
        require(std::abs(q - start_[12 + j]) <= 0.005, "JSON endpoint differs from captured upper-body pose");
      const bool active = tokens[j < 3 ? 1 : (j < 6 ? 2 : 0)] != 0;
      if (!active) require(std::abs(q - start_[12 + j]) <= 0.005, "inactive body part differs from captured pose");
      else row[12 + j] = static_cast<float>(std::max(bound.first, std::min(q, bound.second)));
    }
    frames_.push_back(row);
  }
  // JSON already includes upper-body entry and return; policy handoff owns the strategy switch.
  require(!expired(), "JSON expired during full validation");
}

const WbcJsonReference::Joints& WbcJsonReference::frame(int index) const {
  return frames_.at(static_cast<std::size_t>(std::max(0, std::min(index, static_cast<int>(frames_.size()) - 1))));
}

bool WbcJsonReference::expired() const { return expires_at_ms_ <= nowMs(); }
