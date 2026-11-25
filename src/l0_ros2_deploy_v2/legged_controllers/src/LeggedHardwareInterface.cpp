// Copyright information
//
// Copyright (c) 2022, Qiayuan (Original author)
// Copyright (c) 2024, Shibo Mu (Modifications)
// All rights reserved.

#include "legged_controllers/LeggedHardwareInterface.h"

#include <stdexcept>

namespace legged {

// ====== YAML utility helpers ======
void requireNode(const YAML::Node& n, const std::string& key_path) {
  if (!n) throw std::runtime_error("Missing required YAML node: " + key_path);
}

std::vector<double> asDoubleList(const YAML::Node& n, const std::string& path) {
  requireNode(n, path);
  std::vector<double> out;
  for (const auto& it : n) out.push_back(it.as<double>());
  return out;
}

std::vector<std::string> asStringList(const YAML::Node& n, const std::string& path) {
  requireNode(n, path);
  std::vector<std::string> out;
  for (const auto& it : n) out.push_back(it.as<std::string>());
  return out;
}

// ====== LeggedHardwareInterface base implementations (no-ops / stubs) ======

bool LeggedHardwareInterface::robot_configure(rclcpp::Node::SharedPtr nh, const std::string& /*yaml_path*/) {
  nh_ = nh;
  RCLCPP_INFO(nh_->get_logger(), "LeggedHardwareInterface::robot_configure");
  return true;
}

bool LeggedHardwareInterface::read(rclcpp::Time /*time*/, rclcpp::Duration /*period*/) { return true; }

bool LeggedHardwareInterface::write(rclcpp::Time /*time*/, rclcpp::Duration /*period*/) { return true; }

// ====== Getter implementations ======

std::vector<double> LeggedHardwareInterface::getBodyPosition() { return {base_px_, base_py_, base_pz_}; }

std::vector<double> LeggedHardwareInterface::getBodyOrientation() { return {base_qx_, base_qy_, base_qz_, base_qw_}; }

std::vector<JointMotorState> LeggedHardwareInterface::getHyridJointMotorState() { return joint_motor_states_; }

std::vector<JointMotorCommand> LeggedHardwareInterface::getJointMotorCommands() { return joint_motor_commands_; }

std::vector<double> LeggedHardwareInterface::getImuQuaternion() { return {imu_qx_, imu_qy_, imu_qz_, imu_qw_}; }

std::vector<double> LeggedHardwareInterface::getImuAngularVelocity() { return {imu_gyx_, imu_gyy_, imu_gyz_}; }

std_msgs::msg::Header LeggedHardwareInterface::getImuHeader() { return imu_header_; }

bool LeggedHardwareInterface::set_joint_motor_commands(int index, const JointMotorCommand& cmd) {
  if (index < 0 || index >= static_cast<int>(joint_motor_commands_.size()))
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "set_joint_motor_commands: index " << index << " out of range (size="
                                                                              << joint_motor_commands_.size() << ")");
    throw std::runtime_error("set_joint_motor_commands: index out of range");
    return false;
  }
  joint_motor_commands_[index] = cmd;
  return true;
}

}  // namespace legged
