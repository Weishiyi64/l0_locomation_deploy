#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace legged {

/// Which policy node's PosCmd is fed to leg PD while walking.
enum class PolicyCmdSource : std::uint8_t { kRl = 0, kImitation = 1 };

struct JointGainProfile {
  std::vector<double> kps;
  std::vector<double> kds;
};

struct PolicyInputs {
  std::vector<double> q_snapshot;
  JointGainProfile gains;
  bool using_imitation{false};
};

struct KeyboardActionBinding {
  std::string name;
  std::string key;
  std::string sequence_path;
  std::string audio_cmd;
  bool uses_rl_imitation{false};
  bool requires_walking_policy{false};
  std::string imitation_policy;
};

struct EmotionSequenceCandidate {
  std::string sequence_path;
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
