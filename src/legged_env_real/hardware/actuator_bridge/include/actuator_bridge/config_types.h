#pragma once
#include <string>
#include <vector>

namespace actuator_bridge {

enum class ActuatorMode { ENABLE, DISABLE, FAKE };

// 限制参数
struct Limits {
  float p_min{0.0f}, p_max{0.0f};
  float v_min{0.0f}, v_max{0.0f};
  float t_min{0.0f}, t_max{0.0f};
  float kp_min{0.0f}, kp_max{0.0f};
  float kd_min{0.0f}, kd_max{0.0f};
  float cur_min{0.0f}, cur_max{0.0f};
  float rotation_direction{0.0f};
  float kt{0.0f};
};

// 执行器规格
struct ActuatorSpec {
  std::string name;
  std::string vendor;
  int id{0};
  std::string model;

  std::string can_device;
  float reduction{1.0f};
  float zero_offset{0.0f};
  int motor_direction{1};

  std::string tag;
  std::string alias;

  Limits limits;

  float joint_lower_limit{0.0f};
  float joint_upper_limit{0.0f};

  ActuatorMode mode{ActuatorMode::ENABLE};
};

// 总线配置
struct BusConfig {
  std::string vendor;
  std::string can_device;
  std::vector<ActuatorSpec> actuators;
};

}  // namespace actuator_bridge
