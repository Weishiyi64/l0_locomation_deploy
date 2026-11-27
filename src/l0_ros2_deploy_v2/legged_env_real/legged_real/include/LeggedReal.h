#pragma once

#include <LeggedHardwareInterface.h>

#include <atomic>
#include <array>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <luvbot_hardware_interfaces/msg/motor_command.hpp>
#include <luvbot_hardware_interfaces/msg/motor_enable_command.hpp>
#include <luvbot_hardware_interfaces/msg/motor_state_array.hpp>
#include <luvbot_hardware_interfaces/srv/motor_set_zero.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <motor_mapping.hpp>
#include <utility>
#include <vector>

namespace legged {

struct MotorData {
  double pos_{0.0};
  double vel_{0.0};
  double tau_{0.0};
  double prev_vel_{0.0};
};

struct ImuData {
  double ori[4]{0.0, 0.0, 0.0, 1.0};
  double angular_vel[3]{0.0, 0.0, 0.0};
  double prev_angular_vel[3]{0.0, 0.0, 0.0};
};

class LeggedReal : public LeggedHardwareInterface {
 public:
  LeggedReal() = default;
  explicit LeggedReal(std::shared_ptr<rclcpp::Node> node) { nh_ = std::move(node); }
  ~LeggedReal() override = default;

  bool robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path) override;
  bool read(rclcpp::Time time, rclcpp::Duration period) override;
  bool write(rclcpp::Time time, rclcpp::Duration period) override;
  bool set_calibration_mode(bool enabled) override;

  bool zeroMotors(const std::vector<std::string>& joint_names) override;
  bool zeroMotorsByTag(const std::string& tag) override;
  bool resetAllMotors() override;
  bool enableAllMotors() override;
  bool disableAllMotors() override;
  // Call after the executor stops; the control thread may keep sending zero-gain MIT until process exit.
  bool disableMotorsBeforeShutdown();

  bool setupTimestampInterface();
  bool setupBodyStateInterface();
  bool setupHybridJointInterface();
  bool setupImuSensorInterface();
  bool setupHardwareCommandInterface();
  bool getFSMStateInterface();

 private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void motorStateCallback(const luvbot_hardware_interfaces::msg::MotorStateArray::SharedPtr msg);
  bool requestAllMotorsEnabled(bool enabled);
  void lifecycleTick();
  void zeroTick();
  void verifyZeroFeedback(const luvbot_hardware_interfaces::msg::MotorStateArray& msg);
  void publishHardwareFault();
  void captureHardwareStatus(bool protection, bool command_invalid);
  void publishHardwareStatus();
  struct MotorStatus {
    bool fault{false}, position_ready{false}, valid{false}, initializing{false}, error_pending{false};
    uint16_t error_code{0};
    int missing_cycles{0};
  };
  struct HardwareStatus {
    std::chrono::steady_clock::time_point sampled{};
    std::array<MotorStatus, 256> motors{};
    bool protection{false}, motor_fault{false}, imu_fault{false}, lifecycle_fault{false};
    bool command_invalid{false}, motors_enabled{false}, calibration{false};
    double imu_age_ms{-1.0};
  };
  HardwareStatus hardware_status_;
  std::mutex hardware_status_mutex_;
  std::atomic<int64_t> imu_accepted_ns_{0};
  double feedback_timeout_ms_{0.0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_detail_pub_;
  rclcpp::TimerBase::SharedPtr fault_detail_timer_;
  void publishMotorsReady(bool ready);

  std::unique_ptr<system_command_arbiter::MotorMapping> motor_mapping_;
  std::vector<double> default_joint_positions_;
  std::vector<system_command_arbiter::JointFeedback> latest_feedback_;
  std::vector<bool> feedback_updated_;
  std::vector<int> consecutive_missing_mask_frames_;
  std::vector<system_command_arbiter::FeedbackWrapState> feedback_wrap_;
  std::mutex feedback_mutex_;
  // Control-thread snapshot: a command must use the same turn as its observation.
  std::vector<system_command_arbiter::JointFeedback> control_feedback_;
  std::vector<bool> control_feedback_updated_;
  std::vector<double> control_wrap_offsets_;
  bool control_feedback_ready_{false};
  luvbot_hardware_interfaces::msg::MotorCommand motor_command_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<luvbot_hardware_interfaces::msg::MotorStateArray>::SharedPtr motor_state_subscription_;
  rclcpp::Publisher<luvbot_hardware_interfaces::msg::MotorCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<luvbot_hardware_interfaces::msg::MotorEnableCommand>::SharedPtr enable_publisher_;
  rclcpp::TimerBase::SharedPtr lifecycle_timer_;

  std::mutex lifecycle_mutex_;
  bool lifecycle_active_{false};
  bool lifecycle_target_enabled_{false};
  rclcpp::Time lifecycle_started_{0, 0, RCL_ROS_TIME};
  std::size_t lifecycle_next_motor_{0};
  unsigned lifecycle_round_{1};
  std::chrono::steady_clock::time_point lifecycle_next_send_{};
  std::atomic<bool> motors_enabled_{false};
  std::atomic<bool> reset_feedback_watchdogs_{false};
  uint32_t enable_sequence_{0};

  std::vector<int> consecutive_missing_feedback_;
  std::vector<bool> feedback_fault_;
  int max_consecutive_missing_{200};
  std::atomic<bool> emergency_stop_triggered_{false};
  std::vector<MotorData> joint_data_;
  ImuData imu_data_{};
  std::mutex sensor_mutex_;
  double odom_position_[3]{0.0, 0.0, 0.0};
  double odom_orientation_[4]{0.0, 0.0, 0.0, 1.0};
  bool odom_received_{false};

  double joint_filter_coeff_real_{1.0};
  double imu_filter_coeff_real_{1.0};
  bool imu_first_read_{true};
  bool imu_enabled_{true};
  bool imu_connection_status_{false};
  rclcpp::Time imu_startup_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_received_time_{0, 0, RCL_ROS_TIME};
  double imu_startup_timeout_{5.0};
  double imu_connection_timeout_{5.0};
  double emergency_damping_kd_{5.0};

  std::atomic<bool> calibration_mode_{false};
  rclcpp::Client<luvbot_hardware_interfaces::srv::MotorSetZero>::SharedPtr zero_client_;
  // Protected by lifecycle_mutex_; only post-request position feedback confirms zeroing.
  bool zero_pending_{false};
  bool zero_failed_{false};
  bool zero_request_pending_{false};
  std::chrono::steady_clock::time_point zero_next_send_{};
  std::vector<int64_t> zero_feedback_after_ns_;
  std::vector<uint8_t> zero_queued_indices_;
  std::vector<uint8_t> zero_pending_indices_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hardware_fault_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_ready_pub_;
  std::mutex status_mutex_;
  std::atomic<bool> motor_fault_{false};
  std::atomic<bool> imu_fault_{false};
  std::atomic<bool> lifecycle_fault_{false};
  bool emergency_published_{false};
  bool hardware_fault_published_{false};
  bool motors_ready_state_{false};
  uint32_t command_sequence_{0};
};

}  // namespace legged
