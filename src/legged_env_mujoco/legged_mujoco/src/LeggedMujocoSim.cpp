#include "legged_mujoco/LeggedMujocoSim.h"

#include <legged_controllers/IHardwareLayer.h>

namespace legged {

bool LeggedMujocoSim::robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path) {
  nh_ = nh;

  // Read joint names from YAML to determine joint count
  YAML::Node root = YAML::LoadFile(yaml_path);
  YAML::Node rc = root["RobotCfg"];
  requireNode(rc, "RobotCfg");

  std::vector<std::string> joint_names = asStringList(rc["joint_order_mj"], "RobotCfg.joint_order_mj");

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(nh_->get_logger(), "LeggedMujocoSim joint[%zu]: %s", i, joint_names[i].c_str());
  }

  joint_count_ = static_cast<int>(joint_names.size());

  lowState_.motor_state.resize(joint_count_);

  pthread_mutex_init(&recvMutex_, NULL);

  stateSubscriber_ = nh_->create_subscription<legged_msgs::msg::LowState>(
      "/LOWSTATE", rclcpp::QoS(100), std::bind(&LeggedMujocoSim::HandleLowState, this, std::placeholders::_1));
  cmdPublisher_ = nh_->create_publisher<legged_msgs::msg::LowCmd>("/LOWCMD", rclcpp::QoS(100));

  return true;
}

void LeggedMujocoSim::HandleLowState(const legged_msgs::msg::LowState::ConstSharedPtr msg) {
  pthread_mutex_lock(&recvMutex_);
  builtin_interfaces::msg::Time t = msg->shifted_time;
  int64_t msg_timestamp = static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);

  if (msg_timestamp > last_timestamp_)
  {
    lowState_ = *msg;
    last_timestamp_ = msg_timestamp;
  }
  else
  {
    RCLCPP_WARN(nh_->get_logger(), "[LeggedMujocoSim] Stale timestamp: msg=%ld last=%ld", msg_timestamp,
                last_timestamp_);
  }
  pthread_mutex_unlock(&recvMutex_);
}

bool LeggedMujocoSim::get_current_state(legged_msgs::msg::LowState& state_out) {
  pthread_mutex_lock(&recvMutex_);
  state_out = lowState_;
  pthread_mutex_unlock(&recvMutex_);
  return true;
}

bool LeggedMujocoSim::publish_command(const legged_msgs::msg::LowCmd& cmd) {
  cmdPublisher_->publish(cmd);
  return true;
}

LeggedMujocoSim::~LeggedMujocoSim() { pthread_mutex_destroy(&recvMutex_); }

}  // namespace legged
