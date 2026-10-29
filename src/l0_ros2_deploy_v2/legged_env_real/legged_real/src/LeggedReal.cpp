#include "legged_real/LeggedReal.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <luvbot_hardware_interfaces/msg/motor_state.hpp>
#include <stdexcept>
#include <unordered_set>

namespace legged {

bool LeggedReal::robot_configure(rclcpp::Node::SharedPtr nh, const std::string&) {
  nh_ = std::move(nh);
  auto declare = [this](const std::string& name, const auto& value) {
    if (!nh_->has_parameter(name)) nh_->declare_parameter(name, value);
  };
  declare("/RobotHw/joint_filter_coeff_real", 1.0);
  declare("/RobotHw/imu_filter_coeff_real", 1.0);
  declare("/RobotHw/imu_startup_timeout", 5.0);
  declare("/RobotHw/imu_connection_timeout", 5.0);
  declare("/RobotHw/max_consecutive_missing", 200);
  declare("/RobotHw/imu_enabled", true);
  declare("emergency.damping_kd", 5.0);
  declare("motor_lifecycle_period_ms", 20);
  declare("robot_type", std::string{});
  declare("robot_cfg_path", std::string{});
  declare("legged_controller_config_path", std::string{});
  declare("mapping_file", std::string{});

  joint_filter_coeff_real_ = nh_->get_parameter("/RobotHw/joint_filter_coeff_real").as_double();
  imu_filter_coeff_real_ = nh_->get_parameter("/RobotHw/imu_filter_coeff_real").as_double();
  imu_startup_timeout_ = nh_->get_parameter("/RobotHw/imu_startup_timeout").as_double();
  imu_connection_timeout_ = nh_->get_parameter("/RobotHw/imu_connection_timeout").as_double();
  max_consecutive_missing_ = nh_->get_parameter("/RobotHw/max_consecutive_missing").as_int();
  imu_enabled_ = nh_->get_parameter("/RobotHw/imu_enabled").as_bool();
  const double emergency_damping_kd = nh_->get_parameter("emergency.damping_kd").as_double();
  if (!std::isfinite(emergency_damping_kd))
  {
    RCLCPP_ERROR(nh_->get_logger(), "emergency.damping_kd must be finite");
    return false;
  }
  emergency_damping_kd_ = std::max(0.0, emergency_damping_kd);
  const int motor_lifecycle_period_ms = nh_->get_parameter("motor_lifecycle_period_ms").as_int();
  robot_type_ = nh_->get_parameter("robot_type").as_string();
  const std::string robot_cfg_path = nh_->get_parameter("robot_cfg_path").as_string();
  const std::string mapping_file = nh_->get_parameter("mapping_file").as_string();

  if (joint_filter_coeff_real_ < 0.0 || joint_filter_coeff_real_ > 1.0 || imu_filter_coeff_real_ < 0.0 ||
      imu_filter_coeff_real_ > 1.0 || imu_startup_timeout_ <= 0.0 || imu_connection_timeout_ <= 0.0 ||
      max_consecutive_missing_ <= 0 || motor_lifecycle_period_ms <= 0)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Invalid hardware filter or timeout parameter");
    return false;
  }
  RCLCPP_INFO(nh_->get_logger(), "Motor feedback watchdog: warning after %d missing control cycles, emergency after %d",
              std::max(1, max_consecutive_missing_ / 2), max_consecutive_missing_);

  if ((robot_type_ != "l0_v3" && robot_type_ != "l0_v4" && robot_type_ != "l0_v5") || robot_cfg_path.empty() ||
      mapping_file.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Topic motor control requires robot_type=l0_v3/l0_v4/l0_v5 and both config paths");
    return false;
  }

  try
  {
    motor_mapping_ = std::make_unique<system_command_arbiter::MotorMapping>(mapping_file);
    if (motor_mapping_->enabledSize() == 0U) throw std::runtime_error("motor mapping has no enabled motors");
    const YAML::Node robot_cfg = YAML::LoadFile(robot_cfg_path)["RobotCfg"];
    if (!robot_cfg || !robot_cfg["joint_order_mj"] || !robot_cfg["joint_order_mj"].IsSequence() ||
        !robot_cfg["default_joint_angles"] || !robot_cfg["default_joint_angles"].IsSequence())
      throw std::runtime_error("RobotCfg.joint_order_mj and default_joint_angles are required");
    for (const auto& joint : robot_cfg["joint_order_mj"]) joint_names_.push_back(joint.as<std::string>());
    if (joint_names_.size() != motor_mapping_->size()) throw std::runtime_error("joint order and mapping size differ");
    if (robot_cfg["default_joint_angles"].size() != joint_names_.size())
      throw std::runtime_error("default joint angle count and mapping size differ");
    std::unordered_set<std::string> seen;
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      const auto& name = joint_names_[i];
      if (!seen.emplace(name).second || motor_mapping_->entries()[i].joint_name != name)
        throw std::runtime_error("joint order does not match motor mapping at index " + std::to_string(i));
    }
    default_joint_positions_.reserve(joint_names_.size());
    for (const auto& angle : robot_cfg["default_joint_angles"])
    {
      const double value = angle.as<double>();
      if (!std::isfinite(value)) throw std::runtime_error("default joint angles must be finite");
      default_joint_positions_.push_back(value);
    }
  } catch (const std::exception& error)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to configure topic motor mapping: %s", error.what());
    return false;
  }

  const auto motor_command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto safety_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  imu_subscription_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS(), std::bind(&LeggedReal::imuCallback, this, std::placeholders::_1));
  odom_subscription_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&LeggedReal::odomCallback, this, std::placeholders::_1));
  motor_state_subscription_ = nh_->create_subscription<robot_interfaces::msg::MotorStateArray>(
      "actuator/motor/state", rclcpp::SensorDataQoS(),
      std::bind(&LeggedReal::motorStateCallback, this, std::placeholders::_1));
  command_publisher_ =
      nh_->create_publisher<robot_interfaces::msg::MotorCommand>("actuator/motor/command", motor_command_qos);
  enable_publisher_ =
      nh_->create_publisher<robot_interfaces::msg::MotorEnableCommand>("actuator/motor/enable", motor_command_qos);
  zero_client_ = nh_->create_client<robot_interfaces::srv::MotorSetZero>("actuator/motor/set_zero");
  emergency_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("emergency_stop", safety_qos);
  hardware_fault_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("hardware_fault", safety_qos);
  motors_ready_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("motors_ready", safety_qos);

  const std::size_t count = motor_mapping_->size();
  latest_feedback_.resize(count);
  feedback_updated_.assign(count, false);
  feedback_wrap_.resize(count);
  control_wrap_offsets_.assign(count, 0.0);
  consecutive_missing_feedback_.assign(count, 0);
  feedback_fault_.assign(count, false);
  joint_data_.resize(count);
  joint_motor_states_.resize(count);
  joint_motor_commands_.resize(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    if (motor_mapping_->entries()[i].enabled) continue;
    joint_data_[i].pos_ = default_joint_positions_[i];
    joint_motor_states_[i].q = default_joint_positions_[i];
  }

  imu_startup_time_ = nh_->now();
  lifecycle_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(motor_lifecycle_period_ms),
                                            std::bind(&LeggedReal::lifecycleTick, this));
  if (!requestAllMotorsEnabled(true)) return false;

  return setupTimestampInterface() && setupBodyStateInterface() && setupHybridJointInterface() &&
         setupImuSensorInterface() && setupHardwareCommandInterface();
}

void LeggedReal::motorStateCallback(const robot_interfaces::msg::MotorStateArray::SharedPtr msg) {
  verifyZeroFeedback(*msg);
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  for (const auto& state : msg->states)
  {
    const std::size_t i = state.route.motor_type;
    if (i >= motor_mapping_->size())
    {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                           "Ignoring feedback for unknown HPM motor index %u",
                           static_cast<unsigned>(state.route.motor_type));
      continue;
    }
    if (!motor_mapping_->entries()[i].enabled) continue;
    const auto& mapping = motor_mapping_->entries()[i];
    luvbot_hardware_interfaces::msg::MotorState mapped_state;
    mapped_state.route = mapping.route;
    mapped_state.valid = state.valid;
    mapped_state.error_code = state.error_code;
    mapped_state.operating_mode = state.operating_mode;
    mapped_state.q = state.q;
    mapped_state.dq = state.dq;
    mapped_state.tau = state.tau;
    const double previous_offset = feedback_wrap_[i].offset;
    const bool was_ready = feedback_wrap_[i].ready();
    const bool was_faulted = feedback_wrap_[i].fault;
    latest_feedback_[i] = motor_mapping_->updateFeedback(mapping, mapped_state, feedback_wrap_[i]);
    if (latest_feedback_[i].error_pending)
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                           "Motor %s transient error_code=%u: %u/%u consecutive feedback frames; "
                           "holding last good feedback",
                           mapping.joint_name.c_str(), static_cast<unsigned>(state.error_code),
                           feedback_wrap_[i].error_samples, system_command_arbiter::kMotorErrorFaultSamples);
    if (latest_feedback_[i].initializing && !latest_feedback_[i].error_pending &&
        feedback_wrap_[i].stable_samples == 0U)
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                           "Motor %s feedback received but joint position/turn is unconfirmed: "
                           "motor_q=%.6f -> joint_q=%.6f wrap_offset=%.6f; "
                           "joint_limits=[%.6f, %.6f] tolerance=%.3f rad; check calibration and mechanical pose",
                           mapping.joint_name.c_str(), state.q, latest_feedback_[i].q, feedback_wrap_[i].offset,
                           mapping.joint_lower, mapping.joint_upper, system_command_arbiter::kJointPositionTolerance);
    if ((!was_ready && feedback_wrap_[i].ready()) || previous_offset != feedback_wrap_[i].offset)
      RCLCPP_INFO(nh_->get_logger(), "Motor %s turn check: motor_q=%.6f offset=%.6f joint_q=%.6f ready=%s",
                  mapping.joint_name.c_str(), state.q, feedback_wrap_[i].offset, latest_feedback_[i].q,
                  feedback_wrap_[i].ready() ? "true" : "false");
    if (!was_faulted && feedback_wrap_[i].fault)
      RCLCPP_ERROR(nh_->get_logger(),
                   "Motor %s feedback fault latched: motor_q=%.6f offset=%.6f joint_q=%.6f "
                   "error_code=%u consecutive_error_frames=%u; "
                   "active commands blocked; emergency damping remains available; "
                   "inspect feedback and disable before recovery",
                   mapping.joint_name.c_str(), state.q, feedback_wrap_[i].offset, latest_feedback_[i].q,
                   static_cast<unsigned>(state.error_code), feedback_wrap_[i].error_samples);
    feedback_updated_[i] = true;
  }
  // unsigned int feedback_mask =  msg->feedback_mask & 0x001F_FFFF;
  constexpr unsigned int ALL_FEEDBACK_MASK = 0x001FFFFFu;

  const uint32_t feedback_mask = static_cast<uint32_t>(msg->feedback_mask) & ALL_FEEDBACK_MASK;

  if (feedback_mask != ALL_FEEDBACK_MASK)
  {
    std::string missing_motors;
    for (unsigned int i = 0; i < 21; ++i)
    {
      if ((feedback_mask & (1u << i)) == 0u)
      {
        if (!missing_motors.empty())
        {
          missing_motors += ", ";
        }
        missing_motors += std::to_string(i + 1);
      }
    }
    /*
    RCLCPP_WARN(
        this->get_logger(),
        "电机反馈异常，缺少反馈的电机号: [%s]，feedback_mask=0x%08X",
        missing_motors.c_str(),
        static_cast<unsigned int>(feedback_mask));
    */
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000,
                         "电机反馈异常，缺少反馈的电机号: [%s]，feedback_mask=0x%08X", missing_motors.c_str(),
                         static_cast<unsigned int>(feedback_mask));
  }
}

bool LeggedReal::read(rclcpp::Time time, rclcpp::Duration) {
  std::vector<system_command_arbiter::JointFeedback> feedback;
  std::vector<bool> updated;
  bool reset_feedback_watchdogs = false;
  bool motors_enabled_at_snapshot = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    reset_feedback_watchdogs = reset_feedback_watchdogs_.exchange(false);
    motors_enabled_at_snapshot = motors_enabled_.load();
    feedback = latest_feedback_;
    updated = feedback_updated_;
    control_feedback_ready_ = true;
    for (std::size_t i = 0; i < feedback_wrap_.size(); ++i)
    {
      control_wrap_offsets_[i] = feedback_wrap_[i].offset;
      if (motor_mapping_->entries()[i].enabled && !feedback_wrap_[i].ready()) control_feedback_ready_ = false;
    }
    std::fill(feedback_updated_.begin(), feedback_updated_.end(), false);
  }
  if (reset_feedback_watchdogs)
  {
    std::fill(consecutive_missing_feedback_.begin(), consecutive_missing_feedback_.end(), 0);
    std::fill(feedback_fault_.begin(), feedback_fault_.end(), false);
  }

  bool motor_fault = false;
  for (std::size_t i = 0; i < feedback.size(); ++i)
  {
    const auto& mapping = motor_mapping_->entries()[i];
    if (!mapping.enabled)
    {
      joint_motor_states_[i].q = default_joint_positions_[i];
      joint_motor_states_[i].dq = 0.0;
      joint_motor_states_[i].tau = 0.0;
      continue;
    }

    if (updated[i] && feedback[i].valid && feedback[i].error == 0U)
    {
      const bool recovered = feedback_fault_[i];
      feedback_fault_[i] = false;
      consecutive_missing_feedback_[i] = 0;
      joint_data_[i].pos_ = feedback[i].q;
      joint_data_[i].vel_ =
          joint_filter_coeff_real_ * feedback[i].dq + (1.0 - joint_filter_coeff_real_) * joint_data_[i].prev_vel_;
      joint_data_[i].tau_ = feedback[i].tau;
      joint_data_[i].prev_vel_ = joint_data_[i].vel_;
      if (recovered)
        RCLCPP_INFO(nh_->get_logger(), "Motor %s feedback recovered; motor commands may resume",
                    joint_names_[i].c_str());
    }
    else if (updated[i] && feedback[i].error_pending)
    {
      // Count received error frames in the callback, not control cycles; keep the last good observation.
      consecutive_missing_feedback_[i] = 0;
    }
    else if (updated[i] && feedback[i].initializing)
    {
      // An uncalibrated joint can be online without a usable control observation.
      const bool recovered = feedback_fault_[i];
      feedback_fault_[i] = false;
      consecutive_missing_feedback_[i] = 0;
      if (recovered)
        RCLCPP_INFO(nh_->get_logger(),
                    "Motor %s feedback received again; joint position/turn still unconfirmed",
                    joint_names_[i].c_str());
    }
    else if (updated[i])
    {
      const bool newly_faulted = !feedback_fault_[i];
      feedback_fault_[i] = true;
      consecutive_missing_feedback_[i] = 0;
      if (newly_faulted)
      {
        RCLCPP_ERROR(nh_->get_logger(),
                     "\033[1;31mMotor %s (HPM index %zu) feedback fault: valid=%s error_code=%u; switching to "
                     "emergency damping\033[0m",
                     joint_names_[i].c_str(), i, feedback[i].valid ? "true" : "false",
                     static_cast<unsigned>(feedback[i].error));
      }
    }
    else if (motors_enabled_at_snapshot && !calibration_mode_.load())
    {
      const int previous_missing = consecutive_missing_feedback_[i];
      consecutive_missing_feedback_[i] = std::min(previous_missing + 1, max_consecutive_missing_);
      if (consecutive_missing_feedback_[i] == std::max(1, max_consecutive_missing_ / 2))
        RCLCPP_WARN(nh_->get_logger(),
                    "\033[1;33mMotor %s (HPM index %zu) no new motor feedback for %d control cycles; "
                    "watchdog grace period remains active\033[0m",
                    joint_names_[i].c_str(), i, consecutive_missing_feedback_[i]);
      if (previous_missing < max_consecutive_missing_ && consecutive_missing_feedback_[i] == max_consecutive_missing_)
      {
        feedback_fault_[i] = true;
        RCLCPP_ERROR(
            nh_->get_logger(),
            "\033[1;31mMotor %s (HPM index %zu) no new motor feedback for %d consecutive control cycles; "
            "switching to emergency damping\033[0m",
            joint_names_[i].c_str(), i, max_consecutive_missing_);
      }
    }

    joint_motor_states_[i].q = joint_data_[i].pos_;
    joint_motor_states_[i].dq = joint_data_[i].vel_;
    joint_motor_states_[i].tau = joint_data_[i].tau_;
  }

  motor_fault = std::any_of(feedback_fault_.begin(), feedback_fault_.end(), [](bool value) { return value; });
  motor_fault_.store(motor_fault);
  emergency_stop_triggered_.store(motor_fault);
  publishMotorsReady(motors_enabled_.load() && !motor_fault);
  publishHardwareFault();

  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    imu_qx_ = imu_data_.ori[0];
    imu_qy_ = imu_data_.ori[1];
    imu_qz_ = imu_data_.ori[2];
    imu_qw_ = imu_data_.ori[3];
    imu_gyx_ = imu_data_.angular_vel[0];
    imu_gyy_ = imu_data_.angular_vel[1];
    imu_gyz_ = imu_data_.angular_vel[2];
    if (odom_received_)
    {
      base_px_ = odom_position_[0];
      base_py_ = odom_position_[1];
      base_pz_ = odom_position_[2];
      base_qx_ = odom_orientation_[0];
      base_qy_ = odom_orientation_[1];
      base_qz_ = odom_orientation_[2];
      base_qw_ = odom_orientation_[3];
    }
    else
    {
      base_qx_ = imu_qx_;
      base_qy_ = imu_qy_;
      base_qz_ = imu_qz_;
      base_qw_ = imu_qw_;
    }
  }
  timestamp_ns_ = static_cast<double>(time.nanoseconds());
  return true;
}

void LeggedReal::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double quaternion_norm_squared =
      msg->orientation.x * msg->orientation.x + msg->orientation.y * msg->orientation.y +
      msg->orientation.z * msg->orientation.z + msg->orientation.w * msg->orientation.w;
  const double values[]{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
                        quaternion_norm_squared};
  if (!std::all_of(std::begin(values), std::end(values), [](double value) { return std::isfinite(value); }) ||
      quaternion_norm_squared < 1e-12)
  {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "Ignoring invalid IMU sample");
    return;
  }
  const double inverse_norm = 1.0 / std::sqrt(quaternion_norm_squared);
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  imu_connection_status_ = true;
  last_imu_received_time_ = nh_->now();
  for (int i = 0; i < 3; ++i)
  {
    imu_data_.angular_vel[i] = imu_first_read_ ? values[i]
                                               : imu_filter_coeff_real_ * values[i] +
                                                     (1.0 - imu_filter_coeff_real_) * imu_data_.prev_angular_vel[i];
    imu_data_.prev_angular_vel[i] = imu_data_.angular_vel[i];
  }
  imu_first_read_ = false;
  imu_data_.ori[0] = msg->orientation.x * inverse_norm;
  imu_data_.ori[1] = msg->orientation.y * inverse_norm;
  imu_data_.ori[2] = msg->orientation.z * inverse_norm;
  imu_data_.ori[3] = msg->orientation.w * inverse_norm;
}

void LeggedReal::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto& position = msg->pose.pose.position;
  const auto& orientation = msg->pose.pose.orientation;
  const double norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
                              orientation.z * orientation.z + orientation.w * orientation.w;
  if (msg->header.frame_id.empty() || msg->child_frame_id.empty() || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z) || !std::isfinite(norm_squared) || norm_squared < 1e-12)
  {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "Ignoring invalid odometry pose");
    return;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  odom_position_[0] = position.x;
  odom_position_[1] = position.y;
  odom_position_[2] = position.z;
  odom_orientation_[0] = orientation.x * inverse_norm;
  odom_orientation_[1] = orientation.y * inverse_norm;
  odom_orientation_[2] = orientation.z * inverse_norm;
  odom_orientation_[3] = orientation.w * inverse_norm;
  odom_received_ = true;
}

bool LeggedReal::write(rclcpp::Time, rclcpp::Duration) {
  bool imu_failed = false;
  const rclcpp::Time current = nh_->now();
  const bool motors_enabled = motors_enabled_.load();
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    // No actuator command is published before lifecycle enable completes, so
    // the IMU startup watchdog must not expire while its backend is still absent.
    if (imu_enabled_ && motors_enabled && !imu_connection_status_)
      imu_failed = (current - imu_startup_time_).seconds() >= imu_startup_timeout_;
    else if (imu_enabled_ && motors_enabled && (current - last_imu_received_time_).seconds() > imu_connection_timeout_)
    {
      imu_connection_status_ = false;
      imu_failed = true;
    }
  }

  imu_fault_.store(imu_failed);
  publishHardwareFault();
  const bool stop = emergency_stop_triggered_.load() || imu_failed || lifecycle_fault_.load();
  if (stop != emergency_published_)
  {
    std_msgs::msg::Bool message;
    message.data = stop;
    emergency_pub_->publish(message);
    emergency_published_ = stop;
    if (stop)
      RCLCPP_ERROR(nh_->get_logger(),
                   "\033[1;31mMotor commands switched to emergency damping: emergency_stop=%s motor_feedback_fault=%s "
                   "imu_fault=%s lifecycle_fault=%s\033[0m",
                   emergency_stop_triggered_.load() ? "true" : "false", motor_fault_.load() ? "true" : "false",
                   imu_failed ? "true" : "false", lifecycle_fault_.load() ? "true" : "false");
    else
      RCLCPP_INFO(nh_->get_logger(), "Hardware fault cleared; controller state determines motor commands");
  }
  // Serialize publication with disable, calibration transitions and SETZERO requests.
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  // Serialize turn selection with publication, including the first powered command.
  std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
  bool zero_gain_command = !motors_enabled_.load() || calibration_mode_.load() || lifecycle_active_ ||
                           lifecycle_fault_.load();
  bool active_command = false;
  bool damping_command = stop || zero_gain_command;
  bool feedback_ready = control_feedback_ready_;
  for (std::size_t i = 0; i < feedback_wrap_.size(); ++i)
  {
    if (!motor_mapping_->entries()[i].enabled) continue;
    damping_command = damping_command || feedback_wrap_[i].fault;
    const auto& command = joint_motor_commands_[i];
    active_command = active_command || command.kp != 0.0 || command.dq_des != 0.0 || command.tau_ff != 0.0;
    feedback_ready =
        feedback_ready && feedback_wrap_[i].ready() && control_wrap_offsets_[i] == feedback_wrap_[i].offset;
  }
  if (!damping_command && !feedback_ready)
  {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                         "Publishing zero-gain MIT until all feedback turns are confirmed in the control snapshot");
    zero_gain_command = true;
    damping_command = true;
  }
  robot_interfaces::msg::MotorCommand output;
  output.stamp = current;
  output.sequence_id = ++command_sequence_;
  output.control_mode = robot_interfaces::msg::MotorCommand::CONTROL_MODE_MIT;
  output.commands.reserve(motor_mapping_->enabledSize());
  try
  {
    for (std::size_t i = 0; i < joint_motor_commands_.size(); ++i)
    {
      const auto& mapping = motor_mapping_->entries()[i];
      if (!mapping.enabled) continue;
      const auto& command = joint_motor_commands_[i];
      // Disabled/calibration states take precedence over emergency damping.
      const auto mapped = motor_mapping_->commandFor(
          mapping, damping_command ? joint_motor_states_[i].q : command.q_des,
          damping_command ? 0.0 : command.dq_des, damping_command ? 0.0 : command.tau_ff,
          damping_command ? 0.0 : command.kp,
          zero_gain_command ? 0.0 : (damping_command ? emergency_damping_kd_ : command.kd),
          feedback_wrap_[i].offset);
      robot_interfaces::msg::MotorTarget target;
      target.route.motor_type = static_cast<uint8_t>(i);
      target.q = mapped.q;
      target.dq = mapped.dq;
      target.tau = mapped.tau;
      target.kp = mapped.kp;
      target.kd = mapped.kd;
      output.commands.push_back(target);
    }
  } catch (const std::exception& error)
  {
    emergency_stop_triggered_.store(true);
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "\033[1;31mRejected motor command: %s\033[0m",
                          error.what());
    return false;
  }
  if (command_publisher_->get_subscription_count() == 0U)
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                         "\033[1;33mPublishing motor commands without an HPM subscriber\033[0m");
  if (!damping_command && active_command)
    for (auto& wrap : feedback_wrap_) wrap.locked = true;
  command_publisher_->publish(output);
  return true;
}

void LeggedReal::publishHardwareFault() {
  const bool fault = motor_fault_.load() || imu_fault_.load() || lifecycle_fault_.load();
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (fault == hardware_fault_published_) return;
  std_msgs::msg::Bool message;
  message.data = fault;
  hardware_fault_pub_->publish(message);
  hardware_fault_published_ = fault;
}

void LeggedReal::publishMotorsReady(bool ready) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (ready == motors_ready_state_) return;
  std_msgs::msg::Bool message;
  message.data = ready;
  motors_ready_pub_->publish(message);
  motors_ready_state_ = ready;
  if (ready)
    RCLCPP_INFO(nh_->get_logger(),
                "Motor command path ready for %zu enabled motors after publishing the HPM enable command",
                motor_mapping_->enabledSize());
  else if (motors_enabled_.load())
    RCLCPP_ERROR(nh_->get_logger(), "\033[1;31mMotor readiness lost; inspect preceding per-motor errors\033[0m");
  else
    RCLCPP_INFO(nh_->get_logger(), "Motors disabled; readiness is false");
}

bool LeggedReal::requestAllMotorsEnabled(bool enabled) {
  if (!motor_mapping_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Motor lifecycle request blocked before motor mapping is configured");
    return false;
  }
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (enabled && (calibration_mode_.load() || zero_pending_ || zero_failed_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Motor enable blocked by active or failed calibration");
    return false;
  }
  lifecycle_target_enabled_ = enabled;
  lifecycle_active_ = true;
  lifecycle_started_ = nh_->now();
  lifecycle_fault_.store(false);
  if (!enabled)
  {
    motors_enabled_.store(false);
    publishMotorsReady(false);
  }
  publishHardwareFault();
  return true;
}

void LeggedReal::lifecycleTick() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (!lifecycle_active_) return;
  const rclcpp::Time current = nh_->now();
  if (enable_publisher_->get_subscription_count() == 0U)
  {
    if ((current - lifecycle_started_).seconds() > 20.0)
    {
      lifecycle_active_ = false;
      motors_enabled_.store(false);
      lifecycle_fault_.store(true);
      publishMotorsReady(false);
      publishHardwareFault();
      RCLCPP_ERROR(nh_->get_logger(),
                   "\033[1;31mTimed out waiting for /actuator/motor/enable subscriber; motors remain disabled\033[0m");
    }
    else
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                           "\033[1;33mWaiting for HPM subscriber on /actuator/motor/enable\033[0m");
    return;
  }

  robot_interfaces::msg::MotorEnableCommand command;
  command.stamp = current;
  command.sequence_id = ++enable_sequence_;
  command.motor_types.reserve(motor_mapping_->enabledSize());
  command.enabled.reserve(motor_mapping_->enabledSize());
  for (std::size_t i = 0; i < motor_mapping_->size(); ++i)
  {
    if (!motor_mapping_->entries()[i].enabled) continue;
    command.motor_types.push_back(static_cast<uint8_t>(i));
    command.enabled.push_back(lifecycle_target_enabled_);
  }
  lifecycle_active_ = false;
  {
    // Do not count feedback received before this enable publication toward the
    // first complete post-enable feedback round.
    std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
    if (lifecycle_target_enabled_)
    {
      std::fill(feedback_updated_.begin(), feedback_updated_.end(), false);
      for (auto& wrap : feedback_wrap_)
      {
        wrap.stable_samples = 0U;
        wrap.error_code = 0U;
        wrap.error_samples = 0U;
      }
      for (auto& feedback : latest_feedback_) feedback.valid = false;
      reset_feedback_watchdogs_.store(true);
    }
    else
    {
      for (auto& wrap : feedback_wrap_) wrap = system_command_arbiter::FeedbackWrapState{};
      for (auto& feedback : latest_feedback_) feedback.valid = false;
      std::fill(feedback_updated_.begin(), feedback_updated_.end(), false);
    }
    enable_publisher_->publish(command);
    motors_enabled_.store(lifecycle_target_enabled_);
  }
  if (lifecycle_target_enabled_)
  {
    std::lock_guard<std::mutex> sensor_lock(sensor_mutex_);
    if (!imu_connection_status_) imu_startup_time_ = current;
  }
  lifecycle_fault_.store(false);
  publishHardwareFault();
  RCLCPP_INFO(nh_->get_logger(), "Published HPM %s command for %zu motors",
              lifecycle_target_enabled_ ? "enable" : "disable", motor_mapping_->enabledSize());
}

}  // namespace legged
