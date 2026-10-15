#include "sim_interface/communication.h"

#include <iostream>

#include "main.h"

// void HandleLowCmd(const legged_msgs::msg::LowCmd::ConstPtr& msg) {
void HandleLowCmd(const legged_msgs::msg::LowCmd::ConstSharedPtr msg) {
  pthread_mutex_lock(&recvMutex_);
  // int64_t msg_timestamp = msg->shifted_time.toNSec();
  rclcpp::Time t(msg->shifted_time);
  int64_t msg_timestamp = static_cast<int64_t>(msg->shifted_time.sec) * 1000000000LL +
                          static_cast<int64_t>(msg->shifted_time.nanosec);  // t.nanoseconds();

  if (msg_timestamp > last_timestamp_)
  {
    recvCmd = *msg;
    last_timestamp_ = msg_timestamp;
  }
  else
  {
    // ("[Mujoco] Timestamp expired, msg_timestamp_: %ld,
    // last_timestamp_: %ld", msg_timestamp, last_timestamp_);
    RCLCPP_WARN(rclcpp::get_logger("mujoco"), "[Mujoco] Timestamp expired, msg_timestamp_: %ld, last_timestamp_: %ld",
                msg_timestamp, last_timestamp_);
  }
  pthread_mutex_unlock(&recvMutex_);
}

// ros::Publisher& statePublisher_) {
void PublishState(const mjModel* m, const mjData* d, const char* body_name,
                  rclcpp::Publisher<legged_msgs::msg::LowState>::SharedPtr statePublisher_) {
  pthread_mutex_lock(&sendMutex_);
  int body_id = mj_name2id(m, mjOBJ_BODY, body_name);
  if (body_id == -1)
  {
    std::cerr << "Body '" << (body_name ? body_name : "<null>") << "' not found in model" << std::endl;
    pthread_mutex_unlock(&sendMutex_);
    return;
  }

  // sendState_.shifted_time = ros::Time::now();
  rclcpp::Time shifted_time = rclcpp::Clock().now();
  sendState_.shifted_time = rclcpp::Clock().now();  // rclcpp::Duration::from_seconds(shifted_time.seconds());

  // Extract body pose
  sendState_.body_xpos[0] = d->xpos[3 * body_id + 0];
  sendState_.body_xpos[1] = d->xpos[3 * body_id + 1];
  sendState_.body_xpos[2] = d->xpos[3 * body_id + 2];

  sendState_.body_xquat[0] = d->xquat[4 * body_id + 0];
  sendState_.body_xquat[1] = d->xquat[4 * body_id + 1];
  sendState_.body_xquat[2] = d->xquat[4 * body_id + 2];
  sendState_.body_xquat[3] = d->xquat[4 * body_id + 3];

  // Extract IMU data
  sendState_.imu.quaternion[0] = d->qpos[3];
  sendState_.imu.quaternion[1] = d->qpos[4];
  sendState_.imu.quaternion[2] = d->qpos[5];
  sendState_.imu.quaternion[3] = d->qpos[6];

  sendState_.imu.gyroscope[0] = d->qvel[3];
  sendState_.imu.gyroscope[1] = d->qvel[4];
  sendState_.imu.gyroscope[2] = d->qvel[5];

  // Extract joint states
  int joint_count = m->nv - 6;
  sendState_.motor_state.resize(joint_count);
  for (int i = 0; i < joint_count; i++)
  {
    sendState_.motor_state[i].q = d->qpos[7 + i];
    sendState_.motor_state[i].dq = d->qvel[6 + i];
    sendState_.motor_state[i].tau = d->qfrc_actuator[6 + i];  // 2.28 发现的ros1 bug，同步修改到这里
  }

  statePublisher_->publish(sendState_);
  pthread_mutex_unlock(&sendMutex_);
}
