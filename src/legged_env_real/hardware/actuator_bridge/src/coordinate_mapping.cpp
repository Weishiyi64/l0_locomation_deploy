#include "actuator_bridge/coordinate_mapping.h"

#include <cstdio>

namespace actuator_bridge {

// 关节 -> 电机
bool CoordinateTransformer::transformCommand(const JointCmd& joint_cmd, MotorCmd& motor_cmd, float zero_offset,
                                             float motor_direction, float reduction, float rotation_direction,
                                             float wrap_offset) {
  if (motor_direction == 0.0f || reduction == 0.0f || rotation_direction == 0.0f)
  {
    fprintf(stderr,
            "\033[31m[ERROR] CoordinateTransformer: invalid motor_direction, "
            "reduction, or rotation_direction\033[0m\n");
    return false;
  }

  const float total_dir = motor_direction * rotation_direction;

  motor_cmd.pos_d = (joint_cmd.pos_d - wrap_offset - zero_offset) * total_dir * reduction;
  motor_cmd.vel_d = joint_cmd.vel_d * total_dir * reduction;
  motor_cmd.tau_d = joint_cmd.tau_d * total_dir / reduction;

  const float r2 = reduction * reduction;
  motor_cmd.kp = joint_cmd.kp / r2;
  motor_cmd.kd = joint_cmd.kd / r2;

  return true;
}

bool CoordinateTransformer::transformRawFeedbackPosition(float& joint_pos, const MotorState& motor_state,
                                                         float zero_offset, float motor_direction, float reduction,
                                                         float rotation_direction) {
  if (motor_direction == 0.0f || reduction == 0.0f || rotation_direction == 0.0f)
  {
    fprintf(stderr,
            "\033[31m[ERROR] CoordinateTransformer: invalid motor_direction, "
            "reduction, or rotation_direction\033[0m\n");
    return false;
  }

  const float total_dir = motor_direction * rotation_direction;
  joint_pos = (motor_state.pos * total_dir / reduction) + zero_offset;
  return true;
}

// 电机 -> 关节
bool CoordinateTransformer::transformFeedback(JointState& joint_state, const MotorState& motor_state, float zero_offset,
                                              float motor_direction, float reduction, float rotation_direction,
                                              float wrap_offset) {
  if (!transformRawFeedbackPosition(joint_state.pos, motor_state, zero_offset, motor_direction, reduction,
                                    rotation_direction))
  {
    return false;
  }

  const float total_dir = motor_direction * rotation_direction;
  joint_state.pos += wrap_offset;
  joint_state.vel = motor_state.vel * total_dir / reduction;
  joint_state.tau = motor_state.tau * total_dir * reduction;

  joint_state.state = motor_state.state;
  joint_state.error = motor_state.error;

  return true;
}

}  // namespace actuator_bridge
