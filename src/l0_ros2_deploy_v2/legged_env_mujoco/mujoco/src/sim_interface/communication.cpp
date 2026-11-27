#include "sim_interface/communication.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iterator>
#include <utility>

#include "main.h"

void ExpireMotorCommand(int64_t now_steady_ns) {
  if (!commandWatchdogArmed || now_steady_ns - commandReceivedSteadyNs <= watchdogNs) return;
  bool fault = false;
  for (std::size_t i = 0; i < motorEnabled.size(); ++i)
  {
    if (motorEnabled[i])
    {
      motorCommandFault[i] = true;
      fault = true;
    }
    motorEnabled[i] = false;
    recvCmd[i] = SimJointCommand{};
  }
  commandReceived = false;
  commandWatchdogArmed = false;
  if (fault)
    RCLCPP_ERROR(rclcpp::get_logger("mujoco"),
                 "Motor command watchdog expired; virtual motors disabled until explicit enable");
}

void HandleMotorEnable(const luvbot_hardware_interfaces::msg::MotorEnableCommand::ConstSharedPtr msg) {
  if (!motorMapping || msg->motor_types.empty() || msg->motor_types.size() != msg->enabled.size())
  {
    RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected invalid MotorEnableCommand arrays");
    return;
  }
  std::vector<bool> seen(motorMapping->size(), false);
  for (const auto index : msg->motor_types)
  {
    if (index >= motorMapping->size() || !motorMapping->entries()[index].enabled || seen[index])
    {
      RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected unknown, disabled, or duplicate motor enable index");
      return;
    }
    seen[index] = true;
  }

  pthread_mutex_lock(&recvMutex_);
  const int64_t now_steady_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count();
  ExpireMotorCommand(now_steady_ns);
  for (std::size_t i = 0; i < msg->motor_types.size(); ++i)
  {
    const auto index = msg->motor_types[i];
    motorEnabled[index] = msg->enabled[i];
    recvCmd[index] = SimJointCommand{};
    if (msg->enabled[i])
    {
      motorCommandFault[index] = false;
      if (!commandWatchdogArmed)
      {
        commandReceivedSteadyNs = now_steady_ns;
        commandWatchdogArmed = true;
      }
    }
  }
  pthread_mutex_unlock(&recvMutex_);
}

void HandleMotorCommand(const luvbot_hardware_interfaces::msg::MotorCommand::ConstSharedPtr msg) {
  if (!motorMapping || msg->control_mode != luvbot_hardware_interfaces::msg::MotorCommand::CONTROL_MODE_MIT ||
      msg->commands.size() != motorMapping->enabledSize() || msg->stamp.sec < 0 || msg->stamp.nanosec >= 1000000000U)
  {
    RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected invalid MotorCommand mode, size, or stamp");
    return;
  }
  const rclcpp::Time current = rclcpp::Clock(RCL_ROS_TIME).now();
  const rclcpp::Time stamp(msg->stamp);
  const int64_t age_ns = (current - stamp).nanoseconds();
  if (age_ns > watchdogNs || age_ns < -100000000LL)
  {
    RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected invalid or expired MotorCommand");
    return;
  }

  std::vector<SimJointCommand> next(motorMapping->size());
  std::vector<bool> seen(motorMapping->size(), false);
  for (const auto& command : msg->commands)
  {
    const std::size_t index = command.route.motor_type;
    const double values[] = {command.q, command.dq, command.tau, command.kp, command.kd};
    if (index >= motorMapping->size() || !motorMapping->entries()[index].enabled || seen[index] ||
        !std::all_of(std::begin(values), std::end(values), [](double value) { return std::isfinite(value); }))
    {
      RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected MotorCommand index or numeric payload");
      return;
    }
    seen[index] = true;

    const auto& mapping = motorMapping->entries()[index];
    const double direction = mapping.motor_direction * mapping.limits.rotation_direction;
    const double reduction_squared = mapping.reduction * mapping.reduction;
    next[index].q = command.q * direction / mapping.reduction + mapping.zero_offset;
    next[index].dq = command.dq * direction / mapping.reduction;
    next[index].tau = command.tau * direction * mapping.reduction;
    next[index].kp = command.kp * reduction_squared;
    next[index].kd = command.kd * reduction_squared;
    const double mapped_values[] = {next[index].q, next[index].dq, next[index].tau, next[index].kp, next[index].kd};
    if (!std::all_of(std::begin(mapped_values), std::end(mapped_values),
                     [](double value) { return std::isfinite(value); }))
    {
      RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected overflowing MotorCommand conversion");
      return;
    }
  }

  pthread_mutex_lock(&recvMutex_);
  const int64_t now_steady_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count();
  ExpireMotorCommand(now_steady_ns);
  // Unsigned half-range comparison accepts forward progress across uint32 rollover.
  const uint32_t sequence_delta = msg->sequence_id - lastSequenceId;
  if (commandReceived && (sequence_delta == 0U || sequence_delta >= 0x80000000U))
  {
    pthread_mutex_unlock(&recvMutex_);
    RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected duplicate or out-of-order MotorCommand");
    return;
  }
  for (std::size_t i = 0; i < next.size(); ++i)
    if (!motorEnabled[i] || motorCommandFault[i]) next[i] = SimJointCommand{};
  recvCmd = std::move(next);
  lastSequenceId = msg->sequence_id;
  commandReceivedSteadyNs = now_steady_ns;
  commandReceived = true;
  commandWatchdogArmed = true;
  pthread_mutex_unlock(&recvMutex_);
}

void PublishState(const mjModel* m, const mjData* d, const char* body_name,
                  const rclcpp::Publisher<luvbot_hardware_interfaces::msg::MotorStateArray>::SharedPtr& motor_pub,
                  const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& imu_pub,
                  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& odom_pub) {
  pthread_mutex_lock(&sendMutex_);
  int body_id = mj_name2id(m, mjOBJ_BODY, body_name);
  if (body_id == -1)
  {
    std::cerr << "Body '" << (body_name ? body_name : "<null>") << "' not found in model" << std::endl;
    pthread_mutex_unlock(&sendMutex_);
    return;
  }

  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
  pthread_mutex_lock(&recvMutex_);
  ExpireMotorCommand(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count());
  const std::vector<bool> fault = motorCommandFault;
  const std::vector<bool> enabled = motorEnabled;
  pthread_mutex_unlock(&recvMutex_);
  luvbot_hardware_interfaces::msg::MotorStateArray motor_state;
  motor_state.stamp = now;
  motor_state.sequence_id = ++stateSequenceId;
  const int joint_count = std::min(m->nu, static_cast<int>(motorMapping ? motorMapping->size() : 0));
  motor_state.states.reserve(joint_count);
  for (int i = 0; i < joint_count; ++i)
  {
    const auto& mapping = motorMapping->entries()[static_cast<std::size_t>(i)];
    if (!mapping.enabled) continue;
    const auto& address = simJointAddresses[static_cast<std::size_t>(i)];
    const double direction = mapping.motor_direction * mapping.limits.rotation_direction;
    luvbot_hardware_interfaces::msg::MotorState state;
    state.route = mapping.route;
    state.error_code = 0U;
    using MotorState = luvbot_hardware_interfaces::msg::MotorState;
    state.operating_mode = MotorState::OPERATING_MODE_UNKNOWN;
    state.enable_state = enabled[i] ? MotorState::ENABLE_ENABLED : MotorState::ENABLE_DISABLED;
    state.q = (d->qpos[address.qpos] - mapping.zero_offset) * direction * mapping.reduction;
    state.dq = d->qvel[address.dof] * direction * mapping.reduction;
    state.tau = d->qfrc_actuator[address.dof] * direction / mapping.reduction;
    state.valid = !fault[i] && std::isfinite(state.q) && std::isfinite(state.dq) && std::isfinite(state.tau);
    if (state.valid) motor_state.feedback_mask |= uint32_t{1} << i;
    motor_state.states.push_back(state);
  }

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = now;
  imu.header.frame_id = body_name;
  imu.orientation.w = d->qpos[3];
  imu.orientation.x = d->qpos[4];
  imu.orientation.y = d->qpos[5];
  imu.orientation.z = d->qpos[6];
  imu.angular_velocity.x = d->qvel[3];
  imu.angular_velocity.y = d->qvel[4];
  imu.angular_velocity.z = d->qvel[5];

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = body_name;
  odom.pose.pose.position.x = d->xpos[3 * body_id + 0];
  odom.pose.pose.position.y = d->xpos[3 * body_id + 1];
  odom.pose.pose.position.z = d->xpos[3 * body_id + 2];
  odom.pose.pose.orientation.w = d->xquat[4 * body_id + 0];
  odom.pose.pose.orientation.x = d->xquat[4 * body_id + 1];
  odom.pose.pose.orientation.y = d->xquat[4 * body_id + 2];
  odom.pose.pose.orientation.z = d->xquat[4 * body_id + 3];
  odom.twist.twist.linear.x = d->qvel[0];
  odom.twist.twist.linear.y = d->qvel[1];
  odom.twist.twist.linear.z = d->qvel[2];
  odom.twist.twist.angular = imu.angular_velocity;

  motor_pub->publish(motor_state);
  imu_pub->publish(imu);
  odom_pub->publish(odom);
  pthread_mutex_unlock(&sendMutex_);
}
