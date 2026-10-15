#include "legged_mujoco/SimHardwareAdapter.h"

namespace legged {

bool SimHardwareAdapter::robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path) {
  // Store node handle in parent class
  nh_ = nh;

  // sim_ must already be configured (joint_count known)
  const int n = sim_->joint_count();
  if (n <= 0)
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "[SimHardwareAdapter] sim joint_count=%d, call "
                 "LeggedMujocoSim::robot_configure first",
                 n);
    return false;
  }

  joint_motor_states_.resize(n);
  joint_motor_commands_.resize(n);

  RCLCPP_INFO(nh_->get_logger(), "[SimHardwareAdapter] Initialised with %d joints", n);
  (void)yaml_path;
  return true;
}

bool SimHardwareAdapter::read(rclcpp::Time time, rclcpp::Duration /*period*/) {
  legged_msgs::msg::LowState state;
  sim_->get_current_state(state);

  const int n = sim_->joint_count();
  for (int i = 0; i < n; ++i)
  {
    joint_motor_states_[i].q = state.motor_state[i].q;
    joint_motor_states_[i].dq = state.motor_state[i].dq;
    joint_motor_states_[i].tau = state.motor_state[i].tau;
  }

  // IMU: LowState stores quaternion as [w,x,y,z], remap to [x,y,z,w]
  imu_qx_ = state.imu.quaternion[1];
  imu_qy_ = state.imu.quaternion[2];
  imu_qz_ = state.imu.quaternion[3];
  imu_qw_ = state.imu.quaternion[0];

  imu_gyx_ = state.imu.gyroscope[0];
  imu_gyy_ = state.imu.gyroscope[1];
  imu_gyz_ = state.imu.gyroscope[2];

  // Body pose
  base_px_ = state.body_xpos[0];
  base_py_ = state.body_xpos[1];
  base_pz_ = state.body_xpos[2];

  base_qw_ = state.body_xquat[0];
  base_qx_ = state.body_xquat[1];
  base_qy_ = state.body_xquat[2];
  base_qz_ = state.body_xquat[3];

  timestamp_ns_ = static_cast<double>(time.nanoseconds());

  return true;
}

bool SimHardwareAdapter::write(rclcpp::Time time, rclcpp::Duration /*period*/) {
  const int n = sim_->joint_count();

  legged_msgs::msg::LowCmd cmd;
  cmd.shifted_time = time;
  cmd.motor_cmd.resize(n);

  for (int i = 0; i < n; ++i)
  {
    cmd.motor_cmd[i].q = joint_motor_commands_[i].q_des;
    cmd.motor_cmd[i].dq = joint_motor_commands_[i].dq_des;
    cmd.motor_cmd[i].kp = joint_motor_commands_[i].kp;
    cmd.motor_cmd[i].kd = joint_motor_commands_[i].kd;
    cmd.motor_cmd[i].tau = joint_motor_commands_[i].tau_ff;
  }

  sim_->publish_command(cmd);
  return true;
}

}  // namespace legged
