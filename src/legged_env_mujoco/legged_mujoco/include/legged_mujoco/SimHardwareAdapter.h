#pragma once

#include <legged_controllers/LeggedHardwareInterface.h>

#include <memory>

#include "legged_mujoco/LeggedMujocoSim.h"

namespace legged {

// SimHardwareAdapter wraps LeggedMujocoSim and implements IHardwareLayer,
// so the same LeggedControlloop can drive both real hardware and simulation.
class SimHardwareAdapter : public LeggedHardwareInterface {
 public:
  explicit SimHardwareAdapter(std::shared_ptr<LeggedMujocoSim> sim) : sim_(std::move(sim)) {}
  ~SimHardwareAdapter() override = default;

  // robot_configure: set up node handle and resize internal buffers
  bool robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path) override;

  // read: copy sim state into parent-class storage (joint_motor_states_,
  // imu_*, base_*, timestamp_ns_)
  bool read(rclcpp::Time time, rclcpp::Duration period) override;

  // write: pack joint_motor_commands_ into LowCmd and publish via sim
  bool write(rclcpp::Time time, rclcpp::Duration period) override;

  // Simulation does not support calibration / motor management
  bool zeroMotors(const std::vector<std::string>&) override { return false; }
  bool zeroMotorsByTag(const std::string&) override { return false; }
  bool resetAllMotors() override { return false; }
  bool enableAllMotors() override { return false; }
  bool disableAllMotors() override { return false; }

 private:
  std::shared_ptr<LeggedMujocoSim> sim_;
};

}  // namespace legged
