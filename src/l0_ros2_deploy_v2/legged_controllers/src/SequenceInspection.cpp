#include "legged_controllers/SequenceInspection.h"

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include "legged_controllers/ControllerFileUtils.h"

namespace legged {

bool inspectSequenceImitation(const std::string& path, rclcpp::Logger logger, const char* context, bool& uses,
                              bool& requires_walking_policy, std::string& imitation_policy) {
  uses = false;
  requires_walking_policy = false;
  imitation_policy.clear();
  YAML::Node root;
  try
  { root = YAML::LoadFile(path); } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "%s: failed to inspect sequence '%s': %s", context, path.c_str(), e.what());
    return false;
  }

  const YAML::Node sequence = root["sequence"];
  YAML::Node phases;
  if (sequence) phases = sequence["phases"];
  if (!phases || !phases.IsSequence())
  {
    RCLCPP_ERROR(logger, "%s: sequence '%s' is missing sequence.phases", context, path.c_str());
    return false;
  }

  for (const auto& phase : phases)
  {
    if (!phase || !phase.IsMap() || !phase["mode"]) continue;
    const std::string mode = phase["mode"].as<std::string>("");
    if (mode == "rl_imitation")
    {
      uses = true;
      requires_walking_policy = true;
    }
    else if (mode == "rl_with_csv")
    {
      requires_walking_policy = true;
    }
    else if (!mode.empty())
    {
      RCLCPP_ERROR(logger, "%s: sequence '%s' uses invalid mode '%s'", context, path.c_str(), mode.c_str());
      return false;
    }
  }

  if (sequence && sequence["imitation_policy"])
  {
    imitation_policy = trimCopy(sequence["imitation_policy"].as<std::string>(""));
  }
  if (imitation_policy.empty() && sequence && sequence["imitation_policy_name"])
  {
    imitation_policy = trimCopy(sequence["imitation_policy_name"].as<std::string>(""));
  }
  if (uses && imitation_policy.empty())
  {
    RCLCPP_ERROR(logger, "%s: sequence '%s' has rl_imitation phase(s) but sequence.imitation_policy is missing",
                 context, path.c_str());
    return false;
  }
  return true;
}

}  // namespace legged
