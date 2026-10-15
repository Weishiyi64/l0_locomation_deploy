#pragma once

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace legged {

// ====== YAML utility helpers (declared here, implemented in
// LeggedHardwareInterface.cpp) ======
void requireNode(const YAML::Node& n, const std::string& key_path);
std::vector<double> asDoubleList(const YAML::Node& n, const std::string& path);
std::vector<std::string> asStringList(const YAML::Node& n, const std::string& path);

// ====== Shared data types ======
struct JointMotorState {
  double q;    // Position
  double dq;   // Velocity
  double tau;  // Effort
  JointMotorState() : q(0.0), dq(0.0), tau(0.0) {}
  JointMotorState(double _q, double _dq, double _tau) : q(_q), dq(_dq), tau(_tau) {}
};

struct JointMotorCommand {
  double q_des;   // desired position
  double dq_des;  // desired velocity
  double kp;      // stiffness
  double kd;      // damping
  double tau_ff;  // feedforward torque

  JointMotorCommand() : q_des(0.0), dq_des(0.0), kp(0.0), kd(0.0), tau_ff(0.0) {}
  JointMotorCommand(double _q_des, double _dq_des, double _kp, double _kd, double _tau_ff)
      : q_des(_q_des), dq_des(_dq_des), kp(_kp), kd(_kd), tau_ff(_tau_ff) {}
};

// ====== Pure virtual hardware abstraction interface ======
// Implemented by DirectHardwareLayer (real robot) and SimHardwareAdapter (sim).
class IHardwareLayer {
 public:
  virtual ~IHardwareLayer() = default;

  // Lifecycle
  virtual bool robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path) = 0;
  virtual bool read(rclcpp::Time time, rclcpp::Duration period) = 0;
  virtual bool write(rclcpp::Time time, rclcpp::Duration period) = 0;

  // Optional calibration mode flag (sim can ignore)
  virtual bool set_calibration_mode(bool enabled) {
    (void)enabled;
    return true;
  }

  // ====== State accessors (filled by read) ======
  virtual std::vector<double> getBodyPosition() = 0;
  virtual std::vector<double> getBodyOrientation() = 0;
  virtual std::vector<JointMotorState> getHyridJointMotorState() = 0;
  virtual std::vector<double> getImuQuaternion() = 0;
  virtual std::vector<double> getImuAngularVelocity() = 0;

  // ====== Command accessors (written by controller, consumed by write) ======
  virtual std::vector<JointMotorCommand> getJointMotorCommands() = 0;
  virtual int getJointMotorCommandsSize() = 0;
  virtual bool set_joint_motor_commands(int index, const JointMotorCommand& cmd) = 0;

  // ====== Optional hardware management (no-op in sim) ======
  virtual bool zeroMotors(const std::vector<std::string>& joint_names) {
    (void)joint_names;
    return false;
  }
  virtual bool zeroMotorsByTag(const std::string& tag) {
    (void)tag;
    return false;
  }
  virtual bool resetAllMotors() { return false; }
  virtual bool enableAllMotors() { return false; }
  virtual bool disableAllMotors() { return false; }
};

}  // namespace legged
