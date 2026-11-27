#include "LeggedReal.h"

namespace legged {

bool LeggedReal::setupTimestampInterface() { return true; }
bool LeggedReal::setupBodyStateInterface() { return true; }

bool LeggedReal::setupHybridJointInterface() {
  if (joint_names_.empty() || joint_names_.size() != joint_motor_commands_.size())
  {
    RCLCPP_ERROR(nh_->get_logger(), "LeggedReal joint order is empty or inconsistent");
    return false;
  }
  RCLCPP_INFO(nh_->get_logger(), "LeggedReal registered %zu topic-controlled joints", joint_names_.size());
  return true;
}

bool LeggedReal::setupImuSensorInterface() { return true; }
bool LeggedReal::setupHardwareCommandInterface() { return true; }
bool LeggedReal::getFSMStateInterface() { return true; }

}  // namespace legged
