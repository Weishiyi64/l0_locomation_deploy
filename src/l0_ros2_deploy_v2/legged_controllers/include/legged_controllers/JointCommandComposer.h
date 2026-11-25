#pragma once

#include <legged_controllers/ArmSwingController.h>
#include <legged_controllers/ControllerTypes.h>
#include <legged_controllers/HeadScript.h>
#include <legged_controllers/IHardwareLayer.h>
#include <legged_controllers/JointCSV.h>
#include <legged_controllers/JointCommandTransition.h>

#include <vector>

namespace legged {

class JointCommandComposer {
 public:
  struct Context {
    IHardwareLayer* robot_hw{nullptr};
    HeadScriptController* head_ctrl{nullptr};
    JointCsvController* joint_csv{nullptr};
    const ArmSwingController* arm_swing_ctrl{nullptr};
    JointCommandTransition* release_handoff{nullptr};

    PolicyInputs policy_inputs;
    JointGainProfile rl_gains;
    std::vector<JointMotorState> motor_states;
    const std::vector<bool>* is_leg_joint{nullptr};

    bool blend_policy_gains{false};
    bool walking{false};
    bool allow_auxiliary_overlays{false};
    bool emergency{false};
    bool is_sim{false};
    bool action_sequence_full_csv{false};
    bool action_sequence_active{false};
    double kp_standby{0.0};
    double emergency_damping_kd{5.0};
    double wall_sec{0.0};
  };

  void composeAll(const Context& ctx) const;
};

}  // namespace legged
