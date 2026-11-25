#pragma once

#include <cstddef>
#include <map>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <vector>

namespace legged {

struct JoystickSequenceBinding {
  std::string name;
  std::string label;
  std::string sequence_path;
  std::string combo_description;
};

class JoystickSequenceActions {
 public:
  struct LoadOptions {
    std::string package_share_dir;
    bool validate_sequence_files{true};
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
  const JoystickSequenceBinding* consumeTriggered(const sensor_msgs::msg::Joy& joy);

  bool empty() const { return bindings_.empty(); }
  std::size_t bindingCount() const { return bindings_.size(); }

 private:
  struct BindingState {
    JoystickSequenceBinding binding;
    std::vector<ButtonRequirement> buttons;
    std::vector<AxisRequirement> axes;
    std::string canonical_combo;
  };

  bool comboActive(const BindingState& binding, const sensor_msgs::msg::Joy& joy) const;

  std::vector<BindingState> bindings_;
  std::map<std::string, bool> last_combo_states_;
  double axis_threshold_{0.5};
};

}  // namespace legged
