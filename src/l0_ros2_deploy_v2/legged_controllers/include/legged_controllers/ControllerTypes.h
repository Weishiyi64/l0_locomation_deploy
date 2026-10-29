#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace legged {

/// Which policy node supplies validated joint targets while walking.
enum class PolicyCmdSource : std::uint8_t { kRl = 0, kImitation = 1, kWbc = 2 };

struct JointGainProfile {
  std::vector<double> kps;
  std::vector<double> kds;
};

struct PolicyInputs {
  std::vector<double> q_snapshot;
  JointGainProfile gains;
  bool using_imitation{false};
  bool valid{false};
};

struct RuntimeControlState {
  bool walking{false};
  bool emergency_control{false};
  bool action_full_csv{false};
  bool policy_drives_legs{false};
  bool allow_auxiliary_overlays{false};
  bool rl_inference_enabled{false};
};

}  // namespace legged
