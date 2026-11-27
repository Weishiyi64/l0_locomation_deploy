#pragma once

#include <ArmSwingController.h>
#include <ControllerTypes.h>
#include <FaceFollow.h>
#include <IHardwareLayer.h>
#include <JointCommandTransition.h>

#include <array>
#include <vector>

namespace legged {

class JointCommandComposer {
 public:
  struct Context {
    IHardwareLayer* robot_hw{nullptr};
    FaceFollowHeadOutput* head_ctrl{nullptr};
    const ArmSwingController* arm_swing_ctrl{nullptr};
    JointCommandTransition* release_handoff{nullptr};

    PolicyInputs policy_inputs;
    JointGainProfile rl_gains;
    // Borrowed only for the synchronous composeAll() call on the control thread.
    const std::vector<JointMotorState>* motor_states{nullptr};
    std::array<double, 9>* upper_position_snapshot{nullptr};  // Optional composed positions, control-thread owned.
    const std::array<double, 3>* head_hold_positions{nullptr};

    bool blend_policy_gains{false};
    bool walking{false};
    bool allow_auxiliary_overlays{false};
    bool emergency{false};
    bool is_sim{false};
    double kp_standby{0.0};
    double emergency_damping_kd{5.0};
    double wall_sec{0.0};
  };

  bool composeAll(const Context& ctx);
  bool holdLastCommand(IHardwareLayer* robot_hw) const;
  const std::vector<JointMotorCommand>& lastCommands() const { return last_commands_; }

 private:
  std::vector<JointMotorCommand> next_commands_, last_commands_;
};

}  // namespace legged
