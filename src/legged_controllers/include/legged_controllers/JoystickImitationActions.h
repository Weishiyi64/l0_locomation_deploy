#pragma once

#include <cstddef>
#include <map>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <vector>

namespace legged {

enum class JoystickImitationFinishMode { kReturnRl, kReturnRlDirect, kEmergencyStop };

struct JoystickImitationBinding {
  std::string name;
  std::string label;
  std::string policy;
  std::string combo_description;
  JoystickImitationFinishMode on_finish{JoystickImitationFinishMode::kReturnRl};
};

class JoystickImitationActions {
 public:
  struct LoadOptions {
    std::string package_share_dir;
    std::string policy_root_path;
    std::string robot_type;
    bool validate_policy_assets{true};
  };

  struct ButtonRequirement {
    int index{-1};
    std::string name;
  };

  struct AxisRequirement {
    int index{-1};
    int sign{0};
    std::string name;
  };

  bool load(const std::string& path, const LoadOptions& options, rclcpp::Logger logger);
  const JoystickImitationBinding* consumeTriggered(const sensor_msgs::msg::Joy& joy);

  bool empty() const { return bindings_.empty(); }
  std::size_t bindingCount() const { return bindings_.size(); }
  std::size_t skippedBindingCount() const { return skipped_binding_count_; }

 private:
  struct BindingState {
    JoystickImitationBinding binding;
    std::vector<ButtonRequirement> buttons;
    std::vector<AxisRequirement> axes;
    std::string canonical_combo;
  };

  bool comboActive(const BindingState& binding, const sensor_msgs::msg::Joy& joy) const;

  std::vector<BindingState> bindings_;
  std::map<std::string, bool> last_combo_states_;
  std::size_t skipped_binding_count_{0};
  double axis_threshold_{0.5};
};

}  // namespace legged
