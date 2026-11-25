#pragma once

#include <legged_controllers/ControllerTypes.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <condition_variable>
#include <luvbot_control_interfaces/msg/policy_command.hpp>
#include <luvbot_control_interfaces/srv/control_wbc_motion.hpp>
#include <luvbot_control_interfaces/srv/switch_policy.hpp>
#include <luvbot_control_interfaces/srv/seed_policy_history.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>

namespace legged {

class PolicyMux {
 public:
  PolicyMux() = default;

  void init(rclcpp::Node::SharedPtr node, const std::vector<std::string>& joint_names);
  void configureStartup(bool single_node_publishes_imitation_command, const JointGainProfile& rl_gains,
                        double policy_switch_timeout_sec, const std::string& policy_root_path,
                        const std::string& robot_type);
  void createRosInterfaces();

  bool loadWalkingGainProfile(const YAML::Node& rc, const std::string& profile_name, size_t expected_size,
                              JointGainProfile& out) const;
  bool loadImitationWalkingGains(const std::string& policy_name);
  bool switchImitationPolicy(const std::string& policy_name, const char* context);

  void setRlCommand(const luvbot_control_interfaces::msg::PolicyCommand& msg);
  void setImitationCommand(const luvbot_control_interfaces::msg::PolicyCommand& msg);
  bool loadWbcWalkingGains(const std::string& robot_cfg_path, const std::string& motion_id);
  std::shared_ptr<luvbot_control_interfaces::srv::ControlWbcMotion::Response> prepareWbcMotion(
      uint8_t command, const std::string& motion_id, double timeout_sec, uint64_t handoff_id = 0);
  uint64_t nextHandoffId() { return ++handoff_sequence_; }
  bool seedRlHistory(uint64_t handoff_id, double timeout_sec);
  void setWbcCommand(const luvbot_control_interfaces::msg::PolicyCommand& msg);
  uint64_t rlCommandRevision() const;
  uint64_t wbcCommandRevision() const;
  bool waitForRlCommandAfter(uint64_t revision, double timeout_sec, uint64_t handoff_id = 0);
  bool waitForWbcCommandAfter(uint64_t revision, double timeout_sec, uint64_t handoff_id = 0);
  void publishWbcActive(bool active, const char* reason = nullptr);
  bool wbcActiveCommanded() const { return wbc_active_commanded_.load(); }
  void clearCommands(double value);
  void resizeCommands(size_t joint_count);

  void setSource(PolicyCmdSource source, const char* reason = nullptr);
  PolicyCmdSource source() const { return policy_cmd_source_.load(); }
  PolicyCmdSource walkStartPolicySource() const;
  PolicyInputs selectInputs(bool walking) const;
  JointGainProfile rlGainsSnapshot() const;

  void publishRlObsReset();
  void publishImitationReset();
  void publishImitationActive(bool active, const char* reason = nullptr);
  void returnToRlWithoutGainRamp(const char* reason);

  bool singleNodePublishesImitationCommand() const { return single_node_publishes_imitation_command_; }
  bool imitationActiveCommanded() const { return imitation_active_commanded_.load(); }

 private:
  rclcpp::Logger logger() const;
  void invalidateImitationCommandLocked();
  bool validateAndOrderCommand(const luvbot_control_interfaces::msg::PolicyCommand& msg,
                               std::vector<double>& ordered_positions, uint64_t& last_sequence,
                               bool& sequence_initialized, const char* topic_label);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rl_imitation_reset_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rl_imitation_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rl_obs_history_reset_pub_;
  rclcpp::Client<luvbot_control_interfaces::srv::SwitchPolicy>::SharedPtr switch_imitation_policy_client_;
  rclcpp::CallbackGroup::SharedPtr imitation_policy_client_callback_group_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wbc_active_pub_;
  rclcpp::Client<luvbot_control_interfaces::srv::ControlWbcMotion>::SharedPtr prepare_wbc_motion_client_;
  rclcpp::CallbackGroup::SharedPtr wbc_client_callback_group_;
  rclcpp::Client<luvbot_control_interfaces::srv::SeedPolicyHistory>::SharedPtr seed_rl_history_client_;
  std::atomic<uint64_t> handoff_sequence_{0};
  luvbot_control_interfaces::msg::PolicyCommand pos_cmd_wbc_;
  uint64_t last_wbc_sequence_{0};
  uint64_t rl_command_revision_{0};
  uint64_t wbc_command_revision_{0};
  bool wbc_sequence_initialized_{false};
  bool pos_cmd_wbc_valid_{false};
  std::condition_variable command_cv_;
  JointGainProfile wbc_walking_gains_;
  std::atomic<bool> wbc_active_commanded_{false};
  mutable std::mutex pos_cmd_mutex_;
  luvbot_control_interfaces::msg::PolicyCommand pos_cmd_rl_;
  luvbot_control_interfaces::msg::PolicyCommand pos_cmd_imitation_;
  std::vector<std::string> joint_names_;
  uint64_t last_rl_sequence_{0};
  uint64_t last_imitation_sequence_{0};
  bool rl_sequence_initialized_{false};
  bool imitation_sequence_initialized_{false};
  bool pos_cmd_imitation_valid_{false};
  std::atomic<PolicyCmdSource> policy_cmd_source_{PolicyCmdSource::kRl};
  bool single_node_publishes_imitation_command_{false};

  mutable std::mutex gains_mutex_;
  JointGainProfile rl_walking_gains_;
  JointGainProfile imitation_walking_gains_;
  std::string current_imitation_policy_;
  std::string policy_root_path_;
  std::string robot_type_;
  bool imitation_policy_ready_{false};
  double imitation_policy_switch_timeout_sec_{120.0};
  std::atomic<bool> imitation_active_commanded_{false};
};

}  // namespace legged
