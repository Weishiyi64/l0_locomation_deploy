#pragma once

#include <cstddef>
#include <cstdint>
#include <luvbot_hardware_interfaces/msg/motor_route.hpp>
#include <luvbot_hardware_interfaces/msg/motor_state.hpp>
#include <luvbot_hardware_interfaces/msg/motor_target.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace system_command_arbiter {

constexpr unsigned kMotorErrorFaultSamples = 100U;
constexpr double kJointPositionTolerance = 0.1;

struct MotorLimits {
  double p_min{};
  double p_max{};
  double v_min{};
  double v_max{};
  double t_min{};
  double t_max{};
  double kp_min{};
  double kp_max{};
  double kd_min{};
  double kd_max{};
  double rotation_direction{1.0};
};

enum class MotorFamily { DM, RS, ENC, MR, LS };

struct MotorMappingEntry {
  std::string joint_name;
  luvbot_hardware_interfaces::msg::MotorRoute route;
  MotorFamily family{};
  bool enabled{false};
  double zero_offset{};
  double motor_direction{1.0};
  double reduction{1.0};
  double joint_lower{};
  double joint_upper{};
  MotorLimits limits;
};

struct JointFeedback {
  double q{};
  double dq{};
  double tau{};
  uint16_t error{};
  uint8_t operating_mode{};
  bool valid{};
  bool initializing{};
  bool error_pending{};
};

struct FeedbackWrapState {
  double offset{};
  unsigned stable_samples{};
  bool locked{};
  bool fault{};
  uint16_t error_code{};
  unsigned error_samples{};

  bool ready() const { return stable_samples >= 3U && !fault; }
};

class MotorMapping {
 public:
  explicit MotorMapping(const std::string& path);

  std::size_t size() const { return entries_.size(); }
  std::size_t enabledSize() const { return enabled_size_; }
  const std::vector<MotorMappingEntry>& entries() const { return entries_; }
  const MotorMappingEntry* findJoint(const std::string& joint_name) const;
  int indexOfJoint(const std::string& joint_name) const;
  int indexOfRoute(const luvbot_hardware_interfaces::msg::MotorRoute& route) const;

  luvbot_hardware_interfaces::msg::MotorTarget commandFor(const MotorMappingEntry& mapping, double q, double dq,
                                                         double tau, double kp, double kd,
                                                         double wrap_offset = 0.0) const;
  double rawJointPosition(const MotorMappingEntry& mapping, double motor_q) const;
  double initialWrapOffset(const MotorMappingEntry& mapping, double motor_q,
                           double tolerance = kJointPositionTolerance) const;
  JointFeedback feedbackFor(const MotorMappingEntry& mapping, const luvbot_hardware_interfaces::msg::MotorState& state,
                            double wrap_offset) const;
  JointFeedback updateFeedback(const MotorMappingEntry& mapping,
                               const luvbot_hardware_interfaces::msg::MotorState& state, FeedbackWrapState& wrap) const;

 private:
  std::vector<MotorMappingEntry> entries_;
  std::unordered_map<std::string, std::size_t> joint_indices_;
  std::size_t enabled_size_{0};
};

}  // namespace system_command_arbiter
