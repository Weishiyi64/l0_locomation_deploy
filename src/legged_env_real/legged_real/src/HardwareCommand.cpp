#include <chrono>
#include <thread>

#include "legged_real/LeggedReal.h"

namespace legged {

bool LeggedReal::zeroMotorByJointName(const std::string& joint_name) {
  if (!mcs_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "[ZERO_MOTOR] ActuatorBridge not initialized");
    return false;
  }

  int motor_index = mcs_->indexOf(joint_name);
  if (motor_index < 0)
  {
    RCLCPP_WARN(nh_->get_logger(), "[ZERO_MOTOR] Joint '%s' not found", joint_name.c_str());
    return false;
  }

  const size_t idx = static_cast<size_t>(motor_index);
  const double zero_tolerance = 0.005;
  const int max_attempts = 5;
  const double wait_time = 0.4;
  double current_position = 180.0;

  for (int attempt = 1; attempt <= max_attempts; ++attempt)
  {
    RCLCPP_INFO(nh_->get_logger(), "[ZERO_MOTOR] Attempt %d/%d to zero joint '%s'", attempt, max_attempts,
                joint_name.c_str());

    mcs_->setReceiveFlag(idx, false);

    mcs_->zero(idx);
    mcs_->reset(idx);

    auto start_time = nh_->now();
    rclcpp::Duration timeout(std::chrono::nanoseconds(static_cast<int64_t>(wait_time * 1e9)));

    while (nh_->now() - start_time < timeout)
    {
      if (mcs_->getReceiveFlag(idx))
      {
        mcs_->setReceiveFlag(idx, false);

        auto joint_state = mcs_->getJointState(idx);
        current_position = static_cast<double>(joint_state.pos);
        float zero_offset = mcs_->getZeroOffset(idx);
        if (std::isfinite(current_position) && std::fabs(current_position - zero_offset) <= zero_tolerance)
        {
          RCLCPP_INFO(nh_->get_logger(),
                      "[ZERO_MOTOR] Joint '%s' successfully zeroed on attempt "
                      "%d (position: %.6f rad)",
                      joint_name.c_str(), attempt, current_position);
          return true;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(nh_->get_logger(),
                "[ZERO_MOTOR] Joint '%s' attempt %d failed - position: %.6f "
                "rad (tolerance: %.6f rad)",
                joint_name.c_str(), attempt, current_position, zero_tolerance);
  }

  RCLCPP_ERROR(nh_->get_logger(), "[ZERO_MOTOR] Failed to zero joint '%s' after %d attempts", joint_name.c_str(),
               max_attempts);
  return false;
}

int LeggedReal::zeroMotorsByJointNames(const std::vector<std::string>& joint_names) {
  if (!mcs_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "[ZERO_MOTOR] ActuatorBridge not initialized");
    return 0;
  }

  int success_count = 0;
  std::vector<std::string> failed_joints;

  for (const auto& joint_name : joint_names)
  {
    if (zeroMotorByJointName(joint_name))
    {
      success_count++;
    }
    else
    {
      failed_joints.push_back(joint_name);
    }
  }

  if (success_count == static_cast<int>(joint_names.size()))
  {
    RCLCPP_INFO(nh_->get_logger(), "[ZERO_MOTOR] Successfully zeroed all %zu joints", joint_names.size());
  }
  else
  {
    RCLCPP_WARN(nh_->get_logger(), "[ZERO_MOTOR] Successfully zeroed %d out of %zu joints", success_count,
                joint_names.size());
    if (!failed_joints.empty())
    {
      std::string failed_list;
      for (size_t i = 0; i < failed_joints.size(); ++i)
      {
        if (i > 0) failed_list += ", ";
        failed_list += failed_joints[i];
      }
      RCLCPP_ERROR(nh_->get_logger(), "[ZERO_MOTOR] Failed joints: %s", failed_list.c_str());
    }
  }

  return success_count;
}

bool LeggedReal::zeroMotors(const std::vector<std::string>& joint_names) {
  RCLCPP_INFO(nh_->get_logger(), "[HW_CMD] Zeroing motors: %zu joints", joint_names.size());
  int success_count = zeroMotorsByJointNames(joint_names);
  bool all_success = (success_count == static_cast<int>(joint_names.size()));

  if (all_success)
  {
    RCLCPP_INFO(nh_->get_logger(), "[HW_CMD] All %zu motors zeroed successfully", joint_names.size());
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "[HW_CMD] Motor zeroing failed: %d/%zu succeeded", success_count,
                 joint_names.size());
  }

  return all_success;
}

bool LeggedReal::zeroMotorsByTag(const std::string& tag) {
  if (!mcs_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "[HW_CMD] ActuatorBridge not initialized");
    return false;
  }

  RCLCPP_INFO(nh_->get_logger(), "[HW_CMD] Zeroing motors by tag: %s", tag.c_str());
  std::vector<std::string> joint_names = mcs_->getJointNamesByTag(tag);

  if (joint_names.empty())
  {
    RCLCPP_WARN(nh_->get_logger(), "[HW_CMD] No joints found with tag: %s", tag.c_str());
    return false;
  }

  RCLCPP_INFO(nh_->get_logger(), "[HW_CMD] Found %zu joints with tag '%s'", joint_names.size(), tag.c_str());
  return zeroMotors(joint_names);
}

bool LeggedReal::resetAllMotors() {
  RCLCPP_INFO(nh_->get_logger(), "[HW_CMD] Resetting all motors");
  if (mcs_)
  {
    mcs_->resetAll();
    return true;
  }
  return false;
}

bool LeggedReal::enableAllMotors() {
  RCLCPP_INFO(nh_->get_logger(), "[HW_CMD] Enabling all motors");
  if (mcs_)
  {
    return mcs_->enableAll();
  }
  return false;
}

bool LeggedReal::disableAllMotors() {
  RCLCPP_INFO(nh_->get_logger(), "[HW_CMD] Disabling all motors");
  if (mcs_)
  {
    mcs_->disableAll();
    return true;
  }
  return false;
}

}  // namespace legged
