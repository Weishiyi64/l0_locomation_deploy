#include "legged_real/LeggedReal.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
using actuator_bridge::JointCmd;
using actuator_bridge::JointState;
namespace legged {
namespace {

bool usesAnkleTransmission(const std::string& robot_type) { return robot_type == "y1_v2" || robot_type == "y1_v3"; }

}  // namespace

bool LeggedReal::robot_configure(rclcpp::Node::SharedPtr nh, const std::string& yaml_path) {
  nh_ = nh;

  int error = 0;
  try
  {
    nh_->declare_parameter("/RobotHw/joint_filter_coeff_real", 1.0);
    nh_->declare_parameter("/RobotHw/imu_filter_coeff_real", 1.0);
    nh_->declare_parameter("/RobotHw/imu_startup_timeout", 5.0);
    nh_->declare_parameter("/RobotHw/imu_connection_timeout", 5.0);
    nh_->declare_parameter("/RobotHw/max_consecutive_missing", 100);
    nh_->declare_parameter("/RobotHw/imu_enabled", true);
  } catch (const std::exception& e)
  { RCLCPP_WARN(nh_->get_logger(), "Parameters already declared"); }

  try
  {
    joint_filter_coeff_real_ = nh_->get_parameter("/RobotHw/joint_filter_coeff_real").as_double();
    imu_filter_coeff_real_ = nh_->get_parameter("/RobotHw/imu_filter_coeff_real").as_double();
    imu_startup_timeout_ = nh_->get_parameter("/RobotHw/imu_startup_timeout").as_double();
    imu_connection_timeout_ = nh_->get_parameter("/RobotHw/imu_connection_timeout").as_double();
    max_consecutive_missing_ = nh_->get_parameter("/RobotHw/max_consecutive_missing").as_int();
    imu_enabled_ = nh_->get_parameter("/RobotHw/imu_enabled").as_bool();
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "[LeggedReal] Failed to get hardware parameters: %s", e.what());
    return false;
  }

  RCLCPP_INFO(nh_->get_logger(), "[LeggedReal] joint_filter_coeff_real_: %f", joint_filter_coeff_real_);
  RCLCPP_INFO(nh_->get_logger(), "[LeggedReal] imu_filter_coeff_real_: %f", imu_filter_coeff_real_);
  RCLCPP_INFO(nh_->get_logger(), "[LeggedReal] imu_startup_timeout_: %f", imu_startup_timeout_);
  RCLCPP_INFO(nh_->get_logger(), "[LeggedReal] imu_connection_timeout_: %f", imu_connection_timeout_);
  RCLCPP_INFO(nh_->get_logger(), "[LeggedReal] max_consecutive_missing_: %d", max_consecutive_missing_);
  RCLCPP_INFO(nh_->get_logger(), "[LeggedReal] imu_enabled_: %d", imu_enabled_);
  joint_first_read_ = true;
  imu_first_read_ = true;
  imu_startup_time_ = nh_->now();

  imu_subscription_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 100, std::bind(&LeggedReal::imu_callback, this, std::placeholders::_1));
  emergency_pub_ = nh_->create_publisher<std_msgs::msg::Float32>("/emergency_stop", 1);
  motors_ready_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("/hardware_status/motors_ready", 1);

  // Declare launch params (may already be declared if node options
  // auto-declare)
  auto try_declare = [&](const std::string& name, const std::string& def) {
    try
    { nh_->declare_parameter<std::string>(name, def); } catch (...)
    {}
  };
  try_declare("config_file", "");
  try_declare("robot_type", "");
  try_declare("robot_cfg_path", "");
  try_declare("legged_controller_config_path", "");

  // 初始化新的执行器桥
  mcs_ = std::make_unique<actuator_bridge::ActuatorBridge>(nh_);
  if (!mcs_->loadFromParams())
  {
    RCLCPP_ERROR(nh_->get_logger(), "ActuatorBridge failed to load from ~buses params");
    return false;
  }

  // 检查是否需要使用传动解算，并在电机 reset/enable 前验证索引
  robot_type_ = "";
  if (!nh_->get_parameter("robot_type", robot_type_))
  {
    RCLCPP_WARN(nh_->get_logger(), "[LeggedReal] robot_type parameter not set; ankle transmission disabled");
  }
  use_ankle_transmission_ = usesAnkleTransmission(robot_type_);

  if (use_ankle_transmission_)
  {
    RCLCPP_INFO(nh_->get_logger(), "[LeggedReal] Ankle transmission mapping enabled for robot_type=%s",
                robot_type_.c_str());

    // 创建传动解算实例
    left_ankle_transmission_ = std::make_unique<TransmissionMapping>(1.0, -1.0, 0.0851720675, 0.0);
    right_ankle_transmission_ = std::make_unique<TransmissionMapping>(1.0, 1.0, 0.0851720675, 0.0);

    // 查找关节索引
    left_ankle_pitch_idx_ = mcs_->indexOf("left_ankle_pitch_joint");
    left_ankle_roll_idx_ = mcs_->indexOf("left_ankle_roll_joint");
    left_ankle_upper_idx_ = mcs_->indexOf("left_ankle_upper_motor");
    left_ankle_lower_idx_ = mcs_->indexOf("left_ankle_lower_motor");

    right_ankle_pitch_idx_ = mcs_->indexOf("right_ankle_pitch_joint");
    right_ankle_roll_idx_ = mcs_->indexOf("right_ankle_roll_joint");
    right_ankle_upper_idx_ = mcs_->indexOf("right_ankle_upper_motor");
    right_ankle_lower_idx_ = mcs_->indexOf("right_ankle_lower_motor");

    // 验证索引有效性
    struct {
      const char* name;
      int* idx;
    } checks[] = {
        {"left_ankle_pitch_joint", &left_ankle_pitch_idx_},   {"left_ankle_roll_joint", &left_ankle_roll_idx_},
        {"left_ankle_upper_motor", &left_ankle_upper_idx_},   {"left_ankle_lower_motor", &left_ankle_lower_idx_},
        {"right_ankle_pitch_joint", &right_ankle_pitch_idx_}, {"right_ankle_roll_joint", &right_ankle_roll_idx_},
        {"right_ankle_upper_motor", &right_ankle_upper_idx_}, {"right_ankle_lower_motor", &right_ankle_lower_idx_},
    };
    bool indices_valid = true;
    for (const auto& c : checks)
    {
      if (*c.idx == -1)
      {
        RCLCPP_ERROR(nh_->get_logger(), "[LeggedReal] Failed to find %s", c.name);
        indices_valid = false;
      }
    }
    if (!indices_valid)
    {
      RCLCPP_ERROR(nh_->get_logger(),
                   "[LeggedReal] Failed to initialize transmission mapping: "
                   "missing joint indices");
      return false;
    }

    RCLCPP_INFO(nh_->get_logger(),
                "[LeggedReal] Transmission mapping indices - Left ankle: "
                "pitch=%d, roll=%d, upper=%d, lower=%d",
                left_ankle_pitch_idx_, left_ankle_roll_idx_, left_ankle_upper_idx_, left_ankle_lower_idx_);
    RCLCPP_INFO(nh_->get_logger(),
                "[LeggedReal] Transmission mapping indices - Right ankle: "
                "pitch=%d, roll=%d, upper=%d, lower=%d",
                right_ankle_pitch_idx_, right_ankle_roll_idx_, right_ankle_upper_idx_, right_ankle_lower_idx_);
  }
  else
  {
    RCLCPP_INFO(nh_->get_logger(), "[LeggedReal] Ankle transmission mapping disabled for robot_type=%s",
                robot_type_.c_str());
  }

  // enable mcs_
  mcs_->resetAll();
  if (!mcs_->enableAll())
  {
    RCLCPP_ERROR(nh_->get_logger(), "[LeggedReal] Motor enable failed during init");
  }

  consecutive_missing_feedback_.resize(mcs_->motorNum(), 0);
  startup_received_once_.assign(mcs_->motorNum(), false);
  jointData_.resize(mcs_->motorNum());
  // Size the shared interface vectors so controller can read/write via them
  joint_motor_states_.resize(mcs_->motorNum());
  joint_motor_commands_.resize(mcs_->motorNum());

  joint_lower_limits_.resize(mcs_->motorNum());
  joint_upper_limits_.resize(mcs_->motorNum());
  for (size_t i = 0; i < mcs_->motorNum(); i++)
  {
    joint_lower_limits_[i] = mcs_->getJointLowerLimit(i);
    joint_upper_limits_[i] = mcs_->getJointUpperLimit(i);
  }

  // Note: These setup functions use ROS1-style hardwareinterfaces that are not
  // compatible with ros2_control They are kept for potential future migration
  setupTimestampInterface();
  setupBodyStateInterface();
  setupHybridJointInterface();
  setupImuSensorInterface();
  setupHardwareCommandInterface();

  return true;
}

bool LeggedReal::read(rclcpp::Time time, rclcpp::Duration period) {
  std::vector<JointState> joint_states(mcs_->motorNum());
  std::vector<bool> received_flags(mcs_->motorNum());
  // 获取关节名称，用于更清晰的报警信息
  std::vector<std::string> joint_names = mcs_->motorNames();

  for (int i = 0; i < static_cast<int>(mcs_->motorNum()); i++)
  {
    joint_states[i] = mcs_->getJointState(i);
    received_flags[i] = mcs_->getReceiveFlag(i);
  }

  // 传动解算 - 电机空间 -> 关节空间
  if (use_ankle_transmission_)
  {
    // 左脚踝
    {
      double theta_u = joint_states[left_ankle_upper_idx_].pos;
      double theta_l = joint_states[left_ankle_lower_idx_].pos;
      double theta_u_dot = joint_states[left_ankle_upper_idx_].vel;
      double theta_l_dot = joint_states[left_ankle_lower_idx_].vel;

      bool calc_success = left_ankle_transmission_->calc_state(theta_u, theta_l, theta_u_dot, theta_l_dot);

      if (calc_success)
      {
        Eigen::Vector4d state = left_ankle_transmission_->get_state();
        double theta_p_urdf = state(0);
        double theta_r_urdf = state(1);
        double theta_p_dot_urdf = state(2);
        double theta_r_dot_urdf = state(3);

        joint_states[left_ankle_pitch_idx_].pos = theta_p_urdf;
        joint_states[left_ankle_roll_idx_].pos = theta_r_urdf;
        joint_states[left_ankle_pitch_idx_].vel = theta_p_dot_urdf;
        joint_states[left_ankle_roll_idx_].vel = theta_r_dot_urdf;
      }
      else
      {
        double theta_p_urdf = left_ankle_transmission_->theta_p_urdf_;
        double theta_r_urdf = left_ankle_transmission_->theta_r_urdf_;
        double theta_p_dot_urdf = left_ankle_transmission_->theta_p_dot_urdf_;
        double theta_r_dot_urdf = left_ankle_transmission_->theta_r_dot_urdf_;
        double theta_u = left_ankle_transmission_->theta_u_;
        double theta_l = left_ankle_transmission_->theta_l_;
        RCLCPP_ERROR(
            nh_->get_logger(),
            "Left ankle calc_state failed: theta_p_urdf: %.3f, theta_r_urdf: %.3f, theta_u: %.3f, theta_l: %.3f",
            theta_p_urdf, theta_r_urdf, theta_u, theta_l);
        joint_states[left_ankle_pitch_idx_].pos = theta_p_urdf;
        joint_states[left_ankle_roll_idx_].pos = theta_r_urdf;
        joint_states[left_ankle_pitch_idx_].vel = theta_p_dot_urdf;
        joint_states[left_ankle_roll_idx_].vel = theta_r_dot_urdf;
      }
    }

    // 右脚踝
    {
      double theta_u = joint_states[right_ankle_upper_idx_].pos;
      double theta_l = joint_states[right_ankle_lower_idx_].pos;
      double theta_u_dot = joint_states[right_ankle_upper_idx_].vel;
      double theta_l_dot = joint_states[right_ankle_lower_idx_].vel;

      bool calc_success = right_ankle_transmission_->calc_state(theta_u, theta_l, theta_u_dot, theta_l_dot);

      if (calc_success)
      {
        Eigen::Vector4d state = right_ankle_transmission_->get_state();
        double theta_p_urdf = state(0);
        double theta_r_urdf = state(1);
        double theta_p_dot_urdf = state(2);
        double theta_r_dot_urdf = state(3);

        joint_states[right_ankle_pitch_idx_].pos = theta_p_urdf;
        joint_states[right_ankle_roll_idx_].pos = theta_r_urdf;
        joint_states[right_ankle_pitch_idx_].vel = theta_p_dot_urdf;
        joint_states[right_ankle_roll_idx_].vel = theta_r_dot_urdf;
      }
      else
      {
        double theta_p_urdf = right_ankle_transmission_->theta_p_urdf_;
        double theta_r_urdf = right_ankle_transmission_->theta_r_urdf_;
        double theta_p_dot_urdf = right_ankle_transmission_->theta_p_dot_urdf_;
        double theta_r_dot_urdf = right_ankle_transmission_->theta_r_dot_urdf_;
        double theta_u = right_ankle_transmission_->theta_u_;
        double theta_l = right_ankle_transmission_->theta_l_;
        RCLCPP_ERROR(
            nh_->get_logger(),
            "Right ankle calc_state failed: theta_p_urdf: %.3f, theta_r_urdf: %.3f, theta_u: %.3f, theta_l: %.3f",
            theta_p_urdf, theta_r_urdf, theta_u, theta_l);
        joint_states[right_ankle_pitch_idx_].pos = theta_p_urdf;
        joint_states[right_ankle_roll_idx_].pos = theta_r_urdf;
        joint_states[right_ankle_pitch_idx_].vel = theta_p_dot_urdf;
        joint_states[right_ankle_roll_idx_].vel = theta_r_dot_urdf;
      }
    }
  }

  // Check motor faults
  static std::vector<rclcpp::Time> motor_fault_warn_last_time;
  const int motor_n = static_cast<int>(mcs_->motorNum());
  if (static_cast<int>(motor_fault_warn_last_time.size()) != motor_n)
  {
    motor_fault_warn_last_time.assign(motor_n, rclcpp::Time(0));
  }
  constexpr double kMotorFaultWarnPeriod = 0.5;
  for (int i = 0; i < motor_n; i++)
  {
    if (received_flags[i] && (joint_states[i].error != 0))
    {
      const std::string name_str = (i < static_cast<int>(joint_names.size())) ? joint_names[i] : std::string("unknown");
      if (!emergency_stop_triggered_)
      {
        RCLCPP_ERROR(nh_->get_logger(),
                     "EMERGENCY STOP: Motor %d (%s) reported fault (error code %d)! System entering safe mode.", i + 1,
                     name_str.c_str(), joint_states[i].error);
        emergency_stop_triggered_ = true;
      }
      else
      {
        const bool due =
            (motor_fault_warn_last_time[i].nanoseconds() == 0) ||
            (time - motor_fault_warn_last_time[i]).nanoseconds() >= static_cast<int64_t>(kMotorFaultWarnPeriod * 1e9);
        if (due)
        {
          RCLCPP_WARN(nh_->get_logger(), "Motor %d (%s) still in fault (error code %d)", i + 1, name_str.c_str(),
                      joint_states[i].error);
          motor_fault_warn_last_time[i] = time;
        }
      }
    }
    else
    {
      motor_fault_warn_last_time[i] = rclcpp::Time(0);
    }
  }

  if (joint_first_read_)
  {
    int valid_motors = 0;
    for (int i = 0; i < static_cast<int>(mcs_->motorNum()); i++)
    {
      if (received_flags[i])
      {
        startup_received_once_[i] = true;
      }
      if (startup_received_once_[i])
      {
        valid_motors++;
      }
    }

    if (valid_motors == static_cast<int>(mcs_->motorNum()))
    {
      for (int i = 0; i < static_cast<int>(mcs_->motorNum()); i++)
      {
        jointData_[i].pos_ = joint_states[i].pos;
        jointData_[i].vel_ = joint_states[i].vel;
        jointData_[i].prev_pos_ = jointData_[i].pos_;
        jointData_[i].prev_vel_ = jointData_[i].vel_;
      }

      joint_first_read_ = false;

      // 发布电机就绪状态
      if (!motors_ready_published_)
      {
        std_msgs::msg::Bool ready_msg;
        ready_msg.data = true;
        motors_ready_pub_->publish(ready_msg);
        motors_ready_published_ = true;
        RCLCPP_INFO(nh_->get_logger(),
                    "\033[32m[MOTORS] Ready status published to "
                    "/hardware_status/motors_ready (received feedback from "
                    "%d/%d motors)\033[0m",
                    valid_motors, static_cast<int>(mcs_->motorNum()));
      }
    }
    else
    {
      if (valid_motors > 0)
      {
        std::string missing_motors_str;
        for (int i = 0; i < static_cast<int>(mcs_->motorNum()); i++)
        {
          if (!startup_received_once_[i])
          {
            const std::string name_str =
                (i < static_cast<int>(joint_names.size())) ? joint_names[i] : std::string("unknown");
            if (!missing_motors_str.empty()) missing_motors_str += ", ";
            missing_motors_str += "[" + std::to_string(i) + "]" + name_str;
          }
        }
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                             "Waiting for all motors to respond... (%d/%d "
                             "motors online). Missing: %s",
                             valid_motors, static_cast<int>(mcs_->motorNum()), missing_motors_str.c_str());
      }
    }
  }
  else
  {
    for (int i = 0; i < static_cast<int>(mcs_->motorNum()); i++)
    {
      // Check for missing motor feedback and apply safety measures
      if (!received_flags[i])
      {
        consecutive_missing_feedback_[i]++;

        const std::string name_str =
            (i < static_cast<int>(joint_names.size())) ? joint_names[i] : std::string("unknown");
        if (consecutive_missing_feedback_[i] >= max_consecutive_missing_)
        {
          if (!emergency_stop_triggered_)
          {
            RCLCPP_ERROR(nh_->get_logger(),
                         "EMERGENCY STOP: %s has %d consecutive missing "
                         "feedback frames! System entering safe mode.",
                         name_str.c_str(), consecutive_missing_feedback_[i]);
            emergency_stop_triggered_ = true;
          }
        }

        // For safety, keep previous values and update prev_vel to current vel
        // This prevents filter state inconsistency when feedback returns
        jointData_[i].prev_vel_ = jointData_[i].vel_;
        continue;
      }
      else
      {
        if (consecutive_missing_feedback_[i] > 0)
        {
          const std::string name_str =
              (i < static_cast<int>(joint_names.size())) ? joint_names[i] : std::string("unknown");
          if (consecutive_missing_feedback_[i] >= max_consecutive_missing_)
          {
            RCLCPP_INFO(nh_->get_logger(), "\033[35m%s feedback recovered after %d missing frames\033[0m",
                        name_str.c_str(), consecutive_missing_feedback_[i]);
          }
          consecutive_missing_feedback_[i] = 0;
        }
      }

      jointData_[i].pos_ = joint_states[i].pos;
      jointData_[i].vel_ =
          joint_filter_coeff_real_ * joint_states[i].vel + (1.0 - joint_filter_coeff_real_) * jointData_[i].prev_vel_;
      jointData_[i].tau_ = joint_states[i].tau;

      jointData_[i].prev_vel_ = jointData_[i].vel_;
      mcs_->setReceiveFlag(i, false);
    }
  }

  // 将 IMU 的四元数赋值给 body_xquat_，用于 BodyState 接口输出
  // 注意顺序差异：
  // - imuData_.ori 存储为 [x, y, z, w]（ROS Imu 消息约定）
  // - body_xquat_ 采用 [w, x, y, z]（以 {1,0,0,0} 为单位四元数可见）
  body_xquat_[0] = imuData_.ori[3];  // w
  body_xquat_[1] = imuData_.ori[0];  // x
  body_xquat_[2] = imuData_.ori[1];  // y
  body_xquat_[3] = imuData_.ori[2];  // z

  // Sync imuData_ -> LeggedHardwareInterface IMU fields so controller reads
  // real IMU via getters imuData_.ori: [x, y, z, w]  →
  // imu_qx_/imu_qy_/imu_qz_/imu_qw_
  imu_qx_ = imuData_.ori[0];
  imu_qy_ = imuData_.ori[1];
  imu_qz_ = imuData_.ori[2];
  imu_qw_ = imuData_.ori[3];
  // LeggedController fills LowState.body_xquat from getBodyOrientation() -> base_q* (xyzw).
  // Keep base quaternion in sync with IMU so rl_controller / imitation see a live base attitude.
  base_qx_ = imu_qx_;
  base_qy_ = imu_qy_;
  base_qz_ = imu_qz_;
  base_qw_ = imu_qw_;
  // angular velocity
  imu_gyx_ = imuData_.angular_vel[0];
  imu_gyy_ = imuData_.angular_vel[1];
  imu_gyz_ = imuData_.angular_vel[2];

  if (emergency_stop_triggered_)
  {
    bool all_feedback_ok = true;
    bool all_states_normal = true;

    for (int i = 0; i < static_cast<int>(mcs_->motorNum()); i++)
    {
      if (consecutive_missing_feedback_[i] >= max_consecutive_missing_)
      {
        all_feedback_ok = false;
        break;
      }

      if (received_flags[i] && (joint_states[i].error != 0))
      {
        all_states_normal = false;
      }
    }

    if (all_feedback_ok && all_states_normal)
    {
      RCLCPP_INFO(nh_->get_logger(),
                  "\033[35mAll motor feedback recovered and all motor states "
                  "normal\033[0m");
      emergency_stop_triggered_ = false;
    }
    else if (!all_states_normal)
    {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
                           "Some motors still have abnormal states, cannot "
                           "recover from emergency stop");
    }
  }

  // set timestamp as monotonic-increasing int64 (nanoseconds)
  timestamp_ns_ = static_cast<double>(time.nanoseconds());

  // Sync jointData_ -> joint_motor_states_ so controller reads real CAN data
  for (size_t i = 0; i < mcs_->motorNum() && i < joint_motor_states_.size(); ++i)
  {
    joint_motor_states_[i].q = jointData_[i].pos_;
    joint_motor_states_[i].dq = jointData_[i].vel_;
    joint_motor_states_[i].tau = jointData_[i].tau_;
  }

  return true;
}

void LeggedReal::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!imu_connection_status_)
  {
    imu_connection_status_ = true;
  }
  last_imu_received_time_ = nh_->now();

  // Apply first-order low-pass filter to IMU data
  if (imu_first_read_)
  {
    // Initialize filtered values with raw data on first read
    imuData_.angular_vel[0] = msg->angular_velocity.x;
    imuData_.angular_vel[1] = msg->angular_velocity.y;
    imuData_.angular_vel[2] = msg->angular_velocity.z;

    // Store previous values
    for (int i = 0; i < 3; i++)
    {
      imuData_.prev_angular_vel[i] = imuData_.angular_vel[i];
    }
    imu_first_read_ = false;
  }
  else
  {
    // Apply first-order low-pass filter: y[n] = α * x[n] + (1-α) * y[n-1]
    // Angular velocity filtering
    imuData_.angular_vel[0] = imu_filter_coeff_real_ * msg->angular_velocity.x +
                              (1.0 - imu_filter_coeff_real_) * imuData_.prev_angular_vel[0];
    imuData_.angular_vel[1] = imu_filter_coeff_real_ * msg->angular_velocity.y +
                              (1.0 - imu_filter_coeff_real_) * imuData_.prev_angular_vel[1];
    imuData_.angular_vel[2] = imu_filter_coeff_real_ * msg->angular_velocity.z +
                              (1.0 - imu_filter_coeff_real_) * imuData_.prev_angular_vel[2];

    // Update previous values for next iteration
    for (int i = 0; i < 3; i++)
    {
      imuData_.prev_angular_vel[i] = imuData_.angular_vel[i];
    }
  }

  // Update orientation (no filtering typically applied to quaternion)
  imuData_.ori[0] = msg->orientation.x;
  imuData_.ori[1] = msg->orientation.y;
  imuData_.ori[2] = msg->orientation.z;
  imuData_.ori[3] = msg->orientation.w;
}

bool LeggedReal::write(rclcpp::Time time, rclcpp::Duration period) {
  // Sync joint_motor_commands_ (written by controller) -> jointData_ for CAN
  // output
  for (size_t i = 0; i < joint_motor_commands_.size() && i < jointData_.size(); ++i)
  {
    jointData_[i].pos_des_ = joint_motor_commands_[i].q_des;
    jointData_[i].vel_des_ = joint_motor_commands_[i].dq_des;
    jointData_[i].kp_ = joint_motor_commands_[i].kp;
    jointData_[i].kd_ = joint_motor_commands_[i].kd;
    jointData_[i].ff_ = joint_motor_commands_[i].tau_ff;
  }

  bool imu_check_failed = false;
  rclcpp::Time current_time = nh_->now();

  // IMU启动连接超时检查
  if (imu_enabled_ && !imu_connection_status_)
  {
    double startup_elapsed = (current_time - imu_startup_time_).seconds();
    if (startup_elapsed >= imu_startup_timeout_)
    {
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                            "IMU connection status check failed! Startup timeout exceeded.");
      imu_check_failed = true;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
                           "IMU initializing... elapsed: %.1fs (timeout: %.1fs)", startup_elapsed,
                           imu_startup_timeout_);
    }
  }
  else
  {  // IMU断开连接超时检查
    double connection_elapsed = (current_time - last_imu_received_time_).seconds();
    if (connection_elapsed > imu_connection_timeout_)
    {
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                            "IMU connection timeout! Last received: %.1fs ago", connection_elapsed);
      imu_check_failed = true;
      imu_connection_status_ = false;
    }
  }

  // 发布emergency状态
  if (emergency_stop_triggered_ || imu_check_failed || joint_check_failed_)
  {
    if (!emergency_published_)
    {
      std_msgs::msg::Float32 emg;
      emg.data = 1.0f;
      emergency_pub_->publish(emg);
      emergency_published_ = true;
      if (imu_check_failed)
      {
        RCLCPP_ERROR(nh_->get_logger(), "IMU check failed - Emergency stop published");
      }
      else if (joint_check_failed_)
      {
        RCLCPP_ERROR(nh_->get_logger(), "Joint limit check failed - Emergency stop published");
      }
      else
      {
        RCLCPP_ERROR(nh_->get_logger(), "Motor fault detected - Emergency stop published");
      }
    }
  }
  else
  {
    // 清除emergency发布标志
    if (emergency_published_)
    {
      emergency_published_ = false;
      std_msgs::msg::Float32 emg;
      emg.data = 0.0f;
      emergency_pub_->publish(emg);
      RCLCPP_INFO(nh_->get_logger(), "Emergency cleared");
    }
  }

  // 传动解算 - 关节空间 -> 电机空间
  if (use_ankle_transmission_)
  {
    // 左脚踝
    {
      double theta_p_cmd_urdf = jointData_[left_ankle_pitch_idx_].pos_des_;
      double theta_r_cmd_urdf = jointData_[left_ankle_roll_idx_].pos_des_;

      double kp_p = jointData_[left_ankle_pitch_idx_].kp_;
      double kp_r = jointData_[left_ankle_roll_idx_].kp_;
      double kd_p = jointData_[left_ankle_pitch_idx_].kd_;
      double kd_r = jointData_[left_ankle_roll_idx_].kd_;

      bool calc_success =
          left_ankle_transmission_->calc_torque_cmd(theta_p_cmd_urdf, theta_r_cmd_urdf, kp_p, kp_r, kd_p, kd_r);

      if (calc_success)
      {
        Eigen::Vector2d torque_cmd = left_ankle_transmission_->get_torque_cmd();
        jointData_[left_ankle_upper_idx_].ff_ = torque_cmd(0);
        jointData_[left_ankle_lower_idx_].ff_ = torque_cmd(1);
      }
      else
      {
        double theta_p_urdf = left_ankle_transmission_->theta_p_urdf_;
        double theta_r_urdf = left_ankle_transmission_->theta_r_urdf_;
        double theta_u = left_ankle_transmission_->theta_u_;
        double theta_l = left_ankle_transmission_->theta_l_;
        RCLCPP_ERROR(
            nh_->get_logger(),
            "Left ankle calc_torque_cmd failed: theta_p_urdf: %.3f, theta_r_urdf: %.3f, theta_u: %.3f, theta_l: %.3f",
            theta_p_urdf, theta_r_urdf, theta_u, theta_l);
        jointData_[left_ankle_upper_idx_].ff_ = 0.0;
        jointData_[left_ankle_lower_idx_].ff_ = 0.0;
      }

      jointData_[left_ankle_upper_idx_].pos_des_ = 0.0;
      jointData_[left_ankle_upper_idx_].vel_des_ = 0.0;
      jointData_[left_ankle_upper_idx_].kp_ = 0.0;
      jointData_[left_ankle_upper_idx_].kd_ = 0.0;
      jointData_[left_ankle_lower_idx_].pos_des_ = 0.0;
      jointData_[left_ankle_lower_idx_].vel_des_ = 0.0;
      jointData_[left_ankle_lower_idx_].kp_ = 0.0;
      jointData_[left_ankle_lower_idx_].kd_ = 0.0;
    }

    // 右脚踝
    {
      double theta_p_cmd_urdf = jointData_[right_ankle_pitch_idx_].pos_des_;
      double theta_r_cmd_urdf = jointData_[right_ankle_roll_idx_].pos_des_;

      double kp_p = jointData_[right_ankle_pitch_idx_].kp_;
      double kp_r = jointData_[right_ankle_roll_idx_].kp_;
      double kd_p = jointData_[right_ankle_pitch_idx_].kd_;
      double kd_r = jointData_[right_ankle_roll_idx_].kd_;

      bool calc_success =
          right_ankle_transmission_->calc_torque_cmd(theta_p_cmd_urdf, theta_r_cmd_urdf, kp_p, kp_r, kd_p, kd_r);

      if (calc_success)
      {
        Eigen::Vector2d torque_cmd = right_ankle_transmission_->get_torque_cmd();
        jointData_[right_ankle_upper_idx_].ff_ = torque_cmd(0);
        jointData_[right_ankle_lower_idx_].ff_ = torque_cmd(1);
      }
      else
      {
        double theta_p_urdf = right_ankle_transmission_->theta_p_urdf_;
        double theta_r_urdf = right_ankle_transmission_->theta_r_urdf_;
        double theta_u = right_ankle_transmission_->theta_u_;
        double theta_l = right_ankle_transmission_->theta_l_;
        RCLCPP_ERROR(
            nh_->get_logger(),
            "Right ankle calc_torque_cmd failed: theta_p_urdf: %.3f, theta_r_urdf: %.3f, theta_u: %.3f, theta_l: %.3f",
            theta_p_urdf, theta_r_urdf, theta_u, theta_l);
        jointData_[right_ankle_upper_idx_].ff_ = 0.0;
        jointData_[right_ankle_lower_idx_].ff_ = 0.0;
      }

      jointData_[right_ankle_upper_idx_].pos_des_ = 0.0;
      jointData_[right_ankle_upper_idx_].vel_des_ = 0.0;
      jointData_[right_ankle_upper_idx_].kp_ = 0.0;
      jointData_[right_ankle_upper_idx_].kd_ = 0.0;
      jointData_[right_ankle_lower_idx_].pos_des_ = 0.0;
      jointData_[right_ankle_lower_idx_].vel_des_ = 0.0;
      jointData_[right_ankle_lower_idx_].kp_ = 0.0;
      jointData_[right_ankle_lower_idx_].kd_ = 0.0;
    }
  }

  // 转发控制命令到电机
  for (size_t i = 0; i < mcs_->motorNum(); ++i)
  {
    JointCmd jc;
    jc.pos_d = static_cast<float>(jointData_[i].pos_des_);
    jc.vel_d = static_cast<float>(jointData_[i].vel_des_);
    jc.tau_d = static_cast<float>(jointData_[i].ff_);
    jc.kp = static_cast<float>(jointData_[i].kp_);
    jc.kd = static_cast<float>(jointData_[i].kd_);
    mcs_->setCommand(i, jc);
  }

  // 使用桥接层统一刷新发布
  mcs_->flush();

  return true;
}

}  // namespace legged
