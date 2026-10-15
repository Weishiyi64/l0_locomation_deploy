#pragma once

#include <legged_controllers/IHardwareLayer.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace legged {

// LeggedHardwareInterface is a convenience base that provides shared data
// storage (joint states/commands, IMU, body pose) and default getter/setter
// implementations on top of IHardwareLayer.  Both DirectHardwareLayer (real
// robot) and SimHardwareAdapter (MuJoCo) inherit from this class.
class LeggedHardwareInterface : public IHardwareLayer {
 public:
  LeggedHardwareInterface() = default;
  ~LeggedHardwareInterface() override = default;

  // Lifecycle – base implementations are no-ops; subclasses must override.
  bool robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path) override;
  bool read(rclcpp::Time time, rclcpp::Duration period) override;
  bool write(rclcpp::Time time, rclcpp::Duration period) override;

  // ====== Hardware management (default: warn + return false) ======
  bool zeroMotors(const std::vector<std::string>& /* joint_names */) override {
    RCLCPP_WARN(nh_->get_logger(), "zeroMotors() not implemented");
    return false;
  }
  bool zeroMotorsByTag(const std::string& /* tag */) override {
    RCLCPP_WARN(nh_->get_logger(), "zeroMotorsByTag() not implemented");
    return false;
  }
  bool resetAllMotors() override {
    RCLCPP_WARN(nh_->get_logger(), "resetAllMotors() not implemented");
    return false;
  }
  bool enableAllMotors() override {
    RCLCPP_WARN(nh_->get_logger(), "enableAllMotors() not implemented");
    return false;
  }
  bool disableAllMotors() override {
    RCLCPP_WARN(nh_->get_logger(), "disableAllMotors() not implemented");
    return false;
  }

  // ====== State accessors (read from protected member storage) ======
  std::vector<double> getBodyPosition() override;
  std::vector<double> getBodyOrientation() override;
  std::vector<JointMotorState> getHyridJointMotorState() override;
  std::vector<double> getImuQuaternion() override;
  std::vector<double> getImuAngularVelocity() override;

  // ====== Command accessors ======
  std::vector<JointMotorCommand> getJointMotorCommands() override;
  inline int getJointMotorCommandsSize() override { return static_cast<int>(joint_motor_commands_.size()); }
  bool set_joint_motor_commands(int index, const JointMotorCommand& cmd) override final;

 protected:
  // ====== IMU state storage ======
  double imu_qx_{0.0}, imu_qy_{0.0}, imu_qz_{0.0}, imu_qw_{1.0};
  double imu_gyx_{0.0}, imu_gyy_{0.0}, imu_gyz_{0.0};
  double imu_accx_{0.0}, imu_accy_{0.0}, imu_accz_{0.0};

  // ====== Body/base state storage ======
  double base_px_{0.0}, base_py_{0.0}, base_pz_{0.0};
  double base_qx_{0.0}, base_qy_{0.0}, base_qz_{0.0}, base_qw_{1.0};
  double base_vx_{0.0}, base_vy_{0.0}, base_vz_{0.0};
  double base_wx_{0.0}, base_wy_{0.0}, base_wz_{0.0};

  // ====== Timestamp / FSM state ======
  double timestamp_ns_{0.0};
  double fsm_state_{0.0};

  // ====== Motor vectors ======
  std::vector<JointMotorState> joint_motor_states_;
  std::vector<JointMotorCommand> joint_motor_commands_;

  // ====== Node + metadata ======
  rclcpp::Node::SharedPtr nh_;
  std::string name_;
  std::string robot_type_;
  std::vector<std::string> joint_names_;
};

}  // namespace legged
