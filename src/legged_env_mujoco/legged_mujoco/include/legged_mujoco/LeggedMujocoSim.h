#pragma once

// LeggedMujocoSim is a pure data + I/O class: it subscribes to /LOWSTATE and
// publishes to /LOWCMD.  It has NO hardware_interface inheritance.
// SimHardwareAdapter wraps it and implements IHardwareLayer for the control
// loop.

#include <pthread.h>

#include <rclcpp/rclcpp.hpp>

#include "legged_msgs/msg/low_cmd.hpp"
#include "legged_msgs/msg/low_state.hpp"

namespace legged {

struct MujocoMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
  double prev_pos_, prev_vel_;
};

struct MujocoImuData {
  double ori_[4];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];  // NOLINT(modernize-avoid-c-arrays)
  double prev_angularVel_[3];
};

class LeggedMujocoSim {
 public:
  LeggedMujocoSim() = default;
  ~LeggedMujocoSim();

  bool robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path);

  // Called by SimHardwareAdapter to copy current state out
  bool get_current_state(legged_msgs::msg::LowState& state_out);

  // Called by SimHardwareAdapter to publish a command
  bool publish_command(const legged_msgs::msg::LowCmd& cmd);

  int joint_count() const { return joint_count_; }

 private:
  rclcpp::Node::SharedPtr nh_;

  rclcpp::Subscription<legged_msgs::msg::LowState>::SharedPtr stateSubscriber_;
  rclcpp::Publisher<legged_msgs::msg::LowCmd>::SharedPtr cmdPublisher_;

  legged_msgs::msg::LowState lowState_;
  int64_t last_timestamp_{0};

  pthread_mutex_t recvMutex_;

  void HandleLowState(const legged_msgs::msg::LowState::ConstSharedPtr msg);

  int joint_count_{0};
};

}  // namespace legged
