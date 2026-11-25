#include "sim_interface/communication.h"
#include "sim_interface/controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <utility>

#include "main.h"

void HandleMotorCommand(const robot_interfaces::msg::MotorCommand::ConstSharedPtr msg) {
  const rclcpp::Time current = rclcpp::Clock(RCL_ROS_TIME).now();
  const rclcpp::Time stamp(msg->stamp);
  const rclcpp::Time valid_until = stamp + rclcpp::Duration::from_nanoseconds(watchdogNs);
  if (!motorMapping ||
      msg->control_mode != robot_interfaces::msg::MotorCommand::CONTROL_MODE_MIT ||
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
    const int index = static_cast<int>(command.route.motor_type);
    const double values[] = {command.q, command.dq, command.tau, command.kp, command.kd};
    if (index < 0 || index >= static_cast<int>(motorMapping->size()) || !motorEnabled[static_cast<std::size_t>(index)] || !seen.emplace(index).second)
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
    commandValidUntilNs = valid_until.nanoseconds();
    commandReceivedSteadyNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    commandReceived = true;
  }
  pthread_mutex_unlock(&recvMutex_);
  if (!valid) RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected MotorCommand route or numeric payload");
}

void PublishState(const mjModel* m, const mjData* d, const char* body_name,
                  const rclcpp::Publisher<robot_interfaces::msg::MotorStateArray>::SharedPtr& motor_pub,
                  const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& imu_pub,
                  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& odom_pub,
                  const rclcpp::Publisher<luvbot_control_interfaces::msg::SimSupportState>::SharedPtr& support_pub) {
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
  robot_interfaces::msg::MotorStateArray motor_state;
  motor_state.stamp = now;
  motor_state.sequence_id = ++stateSequenceId;
  const int joint_count = std::min(m->nu, static_cast<int>(motorMapping ? motorMapping->size() : 0));
  motor_state.states.reserve(joint_count);
  for (int i = 0; i < joint_count; ++i)
  {
    const auto& mapping = motorMapping->entries()[static_cast<std::size_t>(i)];
    const double direction = mapping.motor_direction * mapping.limits.rotation_direction;
    robot_interfaces::msg::MotorState state;
    state.route.motor_type = static_cast<uint8_t>(i);
    motor_state.feedback_mask |= 1U << i;
    state.valid = true;
    state.error_code = 0;
    state.operating_mode = state.OPERATING_MODE_UNKNOWN;
    state.q = static_cast<float>((d->qpos[simQpos[i]] - mapping.zero_offset) * direction * mapping.reduction);
    state.dq = static_cast<float>(d->qvel[simDof[i]] * direction * mapping.reduction);
    state.tau = static_cast<float>(d->qfrc_actuator[simDof[i]] * direction / mapping.reduction);
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

  if (support_pub) {
    luvbot_control_interfaces::msg::SimSupportState support;
    support.header = odom.header;
    support.horizontal_speed_m_s = std::hypot(d->qvel[0], d->qvel[1]);
    const int floor = mj_name2id(m, mjOBJ_GEOM, "floor");
    const int left = mj_name2id(m, mjOBJ_BODY, "left_ankle_roll_link");
    const int right = mj_name2id(m, mjOBJ_BODY, "right_ankle_roll_link");
    for (int k = 0; k < d->ncon; ++k) {
      const auto& contact = d->contact[k];
      if (contact.geom1 != floor && contact.geom2 != floor) continue;
      const int foot = m->geom_bodyid[contact.geom1 == floor ? contact.geom2 : contact.geom1];
      if (foot != left && foot != right) continue;
      mjtNum wrench[6]{};
      mj_contactForce(m, d, k, wrench);
      (foot == left ? support.left_normal_force_n : support.right_normal_force_n) += wrench[0];
    }
    support.command_valid = trialAppliedTargets.valid;
    for (int i = 0; i < joint_count; ++i) {
      support.joint_names.push_back(motorMapping->entries()[i].joint_name);
      support.target_positions.push_back(trialAppliedTargets.positions[i]);
    }
    support_pub->publish(support);
  }
  motor_pub->publish(motor_state);
  imu_pub->publish(imu);
  odom_pub->publish(odom);
  pthread_mutex_unlock(&sendMutex_);
}
