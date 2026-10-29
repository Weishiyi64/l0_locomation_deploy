#include "sim_interface/communication.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <utility>

#include "main.h"

void HandleMotorCommand(const luvbot_hardware_interfaces::msg::MotorCommand::ConstSharedPtr msg) {
  const rclcpp::Time current = rclcpp::Clock(RCL_ROS_TIME).now();
  const rclcpp::Time stamp(msg->stamp);
  const rclcpp::Time valid_until(msg->valid_until);
  if (!motorMapping || msg->source_id.empty() ||
      msg->control_mode != luvbot_hardware_interfaces::msg::MotorCommand::CONTROL_MODE_HYBRID ||
      msg->commands.size() != motorMapping->size() || valid_until <= current || valid_until <= stamp ||
      stamp > current + rclcpp::Duration::from_seconds(0.1))
  {
    RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected invalid or expired MotorCommand");
    return;
  }

  pthread_mutex_lock(&recvMutex_);
  if (commandReceived && msg->sequence_id <= lastSequenceId)
  {
    pthread_mutex_unlock(&recvMutex_);
    RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected out-of-order MotorCommand");
    return;
  }

  std::vector<SimJointCommand> next(motorMapping->size());
  std::unordered_set<int> seen;
  bool valid = true;
  for (const auto& command : msg->commands)
  {
    const int index = motorMapping->indexOfRoute(command.route);
    const double values[] = {command.q, command.dq, command.tau, command.kp, command.kd};
    if (index < 0 || !motorEnabled[static_cast<std::size_t>(index)] || !seen.emplace(index).second)
    {
      valid = false;
      break;
    }
    for (const double value : values) valid = valid && std::isfinite(value);
    if (!valid) break;

    const auto& mapping = motorMapping->entries()[static_cast<std::size_t>(index)];
    const double direction = mapping.motor_direction * mapping.limits.rotation_direction;
    const double reduction_squared = mapping.reduction * mapping.reduction;
    next[index].q = command.q * direction / mapping.reduction + mapping.zero_offset;
    next[index].dq = command.dq * direction / mapping.reduction;
    next[index].tau = command.tau * direction * mapping.reduction;
    next[index].kp = command.kp * reduction_squared;
    next[index].kd = command.kd * reduction_squared;
  }
  if (valid && seen.size() == motorMapping->size())
  {
    recvCmd = std::move(next);
    lastSequenceId = msg->sequence_id;
    commandValidUntilNs = rclcpp::Time(msg->valid_until).nanoseconds();
    commandReceivedSteadyNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    commandReceived = true;
  }
  pthread_mutex_unlock(&recvMutex_);
  if (!valid) RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected MotorCommand route or numeric payload");
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
    const double direction = mapping.motor_direction * mapping.limits.rotation_direction;
    luvbot_hardware_interfaces::msg::MotorState state;
    state.route = mapping.route;
    state.valid = true;
    state.error_code = state.ERROR_NONE;
    state.operating_mode = enabled[i] ? state.OPERATING_MODE_HYBRID : state.OPERATING_MODE_DISABLED;
    state.q = static_cast<float>((d->qpos[7 + i] - mapping.zero_offset) * direction * mapping.reduction);
    state.dq = static_cast<float>(d->qvel[6 + i] * direction * mapping.reduction);
    state.tau = static_cast<float>(d->qfrc_actuator[6 + i] * direction / mapping.reduction);
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
