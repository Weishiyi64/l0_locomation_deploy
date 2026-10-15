#pragma once
#include <can_msgs/msg/frame.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace actuator_bridge {

struct JointCmd {
  float pos_d{0.0f};
  float vel_d{0.0f};
  float tau_d{0.0f};
  float kp{0.0f};
  float kd{0.0f};
};

struct JointState {
  float pos{0.0f};
  float vel{0.0f};
  float tau{0.0f};
  uint16_t state{0};
  uint16_t error{0};
};

struct MotorCmd {
  std::string name;
  std::string vendor;
  int32_t id{-1};

  float pos_d{0.0f};
  float vel_d{0.0f};
  float tau_d{0.0f};
  float kp{0.0f};
  float kd{0.0f};

  can_msgs::msg::Frame raw;
};

struct MotorState {
  std::string name;
  std::string vendor;
  int32_t id{0};

  float pos{0.0f};
  float vel{0.0f};
  float tau{0.0f};
  float mos_temp{0.0f};
  float motor_temp{0.0f};
  uint16_t state{0};
  uint16_t error{0};

  can_msgs::msg::Frame raw;
};

}  // namespace actuator_bridge
