#pragma once

#include "actuator_bridge/driver_types.h"

namespace actuator_bridge {

class CoordinateTransformer {
 public:
  // 命令转换：关节坐标 -> 电机坐标
  static bool transformCommand(const JointCmd& joint_cmd, MotorCmd& motor_cmd, float zero_offset, float motor_direction,
                               float reduction, float rotation_direction, float wrap_offset);

  // 反馈位置转换：电机坐标 -> 关节原始位置
  static bool transformRawFeedbackPosition(float& joint_pos, const MotorState& motor_state, float zero_offset,
                                           float motor_direction, float reduction, float rotation_direction);

  // 反馈转换：电机坐标 -> 关节坐标
  static bool transformFeedback(JointState& joint_state, const MotorState& motor_state, float zero_offset,
                                float motor_direction, float reduction, float rotation_direction, float wrap_offset);
};

}  // namespace actuator_bridge
