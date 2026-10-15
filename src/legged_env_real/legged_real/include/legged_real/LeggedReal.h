#pragma once

#include <legged_controllers/LeggedHardwareInterface.h>
#include <math.h>
#include <memory.h>
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <memory>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "actuator_bridge/actuator_bridge.h"
#include "legged_msgs/msg/low_state.hpp"
#include "legged_real/transmission/transmission_mapping.h"
#include "rclcpp/rclcpp.hpp"

namespace legged {

struct MotorData {
  double pos_, vel_, tau_;
  double pos_des_, vel_des_, kp_, kd_, ff_;
  double prev_pos_, prev_vel_;
};

struct ImuData {
  double ori[4];
  double angular_vel[3];

  double prev_angular_vel[3];
};

class LeggedReal : public LeggedHardwareInterface {
 public:
  LeggedReal() = default;
  LeggedReal(std::shared_ptr<rclcpp::Node> node) { nh_ = node; }
  ~LeggedReal() = default;

  // Configuration method (ROS2 ros2_control style)
  bool robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path) override;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subscription_;
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  bool read(rclcpp::Time time, rclcpp::Duration period) override;
  bool write(rclcpp::Time time, rclcpp::Duration period) override;

  // Implement hardware command interface
  bool zeroMotors(const std::vector<std::string>& joint_names) override;
  bool zeroMotorsByTag(const std::string& tag);
  bool resetAllMotors() override;
  bool enableAllMotors() override;
  bool disableAllMotors() override;

  // Setup methods for hardware interfaces
  bool setupTimestampInterface();
  bool setupBodyStateInterface();
  bool setupHybridJointInterface();
  bool setupImuSensorInterface();
  bool setupHardwareCommandInterface();
  bool getFSMStateInterface();

 private:
  // Member variables (nh_ is inherited from LeggedHardwareInterface)

  bool zeroMotorByJointName(const std::string& joint_name);
  int zeroMotorsByJointNames(const std::vector<std::string>& joint_names);

  double body_xpos_[3] = {0.0, 0.0, 0.0};
  double body_xquat_[4] = {1.0, 0.0, 0.0, 0.0};

  int64_t timestamp_;

  std::unique_ptr<actuator_bridge::ActuatorBridge> mcs_;

  std::vector<int> consecutive_missing_feedback_;
  std::vector<bool> startup_received_once_;
  int max_consecutive_missing_;
  bool emergency_stop_triggered_ = false;
  std::vector<MotorData> jointData_;
  ImuData imuData_{};

  std::vector<float> joint_lower_limits_;
  std::vector<float> joint_upper_limits_;

  double joint_filter_coeff_real_;
  double imu_filter_coeff_real_;
  bool joint_first_read_ = true;
  bool imu_first_read_ = true;

  bool imu_enabled_ = true;
  bool imu_connection_status_ = false;
  rclcpp::Time imu_startup_time_;
  rclcpp::Time last_imu_received_time_;
  double imu_startup_timeout_;
  double imu_connection_timeout_;

  bool joint_check_failed_ = false;

  // Emergency publish helpers
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> emergency_pub_;
  bool emergency_published_ = false;

  // Hardware ready status
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> motors_ready_pub_;
  bool motors_ready_published_ = false;

  std::string robot_type_;
  bool use_ankle_transmission_ = false;
  std::unique_ptr<TransmissionMapping> left_ankle_transmission_;
  std::unique_ptr<TransmissionMapping> right_ankle_transmission_;

  // Ankle joint indices
  int left_ankle_pitch_idx_ = -1;
  int left_ankle_roll_idx_ = -1;
  int left_ankle_upper_idx_ = -1;
  int left_ankle_lower_idx_ = -1;
  int right_ankle_pitch_idx_ = -1;
  int right_ankle_roll_idx_ = -1;
  int right_ankle_upper_idx_ = -1;
  int right_ankle_lower_idx_ = -1;
};

}  // namespace legged
