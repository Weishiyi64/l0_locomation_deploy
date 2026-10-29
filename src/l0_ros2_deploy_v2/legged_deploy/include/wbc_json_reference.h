#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

std::vector<std::pair<double, double>> wbcJsonJointLimits(
    const std::string& mapping_file, const std::vector<std::string>& joint_names);

// The deployed command_short ABI consumes only root pose + 21 joint positions.
// For an upper-body-only reference with fixed root/lower body, these are exactly
// the values used by the previous MuJoCo -> reference-ONNX -> RKNN path.
class WbcJsonReference {
 public:
  using Joints = std::array<float, 21>;
  WbcJsonReference(const std::string& payload, const std::vector<std::string>& joint_names,
                   const std::vector<std::pair<double, double>>& limits, const std::string& base_hash,
                   const std::array<double, 3>* base_position = nullptr,
                   const std::array<double, 4>* base_orientation = nullptr);
  const std::string& id() const { return id_; }
  const Joints& frame(int index) const;
  const Joints& start() const { return start_; }
  const std::array<double, 3>& rootPosition() const { return root_position_; }
  const std::array<double, 4>& rootOrientation() const { return root_orientation_; }
  std::size_t size() const { return frames_.size(); }
  bool expired() const;

 private:
  std::string id_;
  std::int64_t expires_at_ms_{0};
  Joints start_{};
  std::array<double, 3> root_position_{};
  std::array<double, 4> root_orientation_{};
  std::vector<Joints> frames_;
};
