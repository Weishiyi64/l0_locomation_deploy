#include "legged_real/LeggedReal.h"

namespace legged {

// Note: The following setup functions use ROS1-style hardware interfaces
// that are not compatible with ros2_control. They are kept for reference.

bool LeggedReal::setupTimestampInterface() {
  // timestampInterface_.registerHandle(legged::TimestampHandle("timestamp",
  // &timestamp_));
  return true;
}

bool LeggedReal::setupBodyStateInterface() {
  // bodyStateInterface_.registerHandle(legged::BodyStateHandle("body",
  // body_xpos_, body_xquat_));
  return true;
}

bool LeggedReal::setupHybridJointInterface() {
  std::vector<std::string> joint_names;
  // 关节顺序与 mcs_ 中的 actuators 名称一一对应，确保索引与电机索引一致
  if (!mcs_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "LeggedReal::setupJoints: mcs_ is null");
    return false;
  }

  joint_names = mcs_->motorNames();

  if (joint_names.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "LeggedReal::setupJoints: no actuators loaded from "
                 "ActuatorBridge (~buses)");
    return false;
  }

  if (joint_names.size() != mcs_->motorNum())
  {
    RCLCPP_WARN(nh_->get_logger(), "LeggedReal::setupJoints: names size(%zu) != motorNum(%zu)", joint_names.size(),
                mcs_->motorNum());
  }

  // 确保内部缓存大小匹配
  if (jointData_.size() != mcs_->motorNum())
  {
    jointData_.resize(mcs_->motorNum());
  }

  // Use vector-based storage for ROS2 compatibility
  // hybridJointInterface_ registration is handled by
  // export_command_interfaces() in derived class
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(nh_->get_logger(), "LeggedReal::setupJoints: registering joint %zu: %s", i, joint_names[i].c_str());
    // hybridJointInterface_.registerHandle(HybridJointHandle(...));
  }

  RCLCPP_INFO(nh_->get_logger(), "LeggedReal registered %zu joints", joint_names.size());
  return true;
}

bool LeggedReal::setupImuSensorInterface() {
  // imuSensorInterface_.registerHandle(legged::ImuSensorHandle("imu_link",
  // "imu_link", imuData_.ori, imuData_.angular_vel));
  return true;
}

bool LeggedReal::setupHardwareCommandInterface() {
  // commandInterface_.registerHandle(HardwareCommandHandle("hardware_cmd",
  // this));
  return true;
}

bool LeggedReal::getFSMStateInterface() {
  // auto* fsm_state_iface = get<legged::FSMStateInterface>();
  // fsm_state_handle_ = fsm_state_iface->getHandle("fsm_state");
  return true;
}

}  // namespace legged
