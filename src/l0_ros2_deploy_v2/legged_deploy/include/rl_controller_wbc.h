#pragma once

#ifndef USE_RKNN
#error "RLWbc is an RKNN-only deployment worker"
#endif

#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <vector>

#include "base/base_rl_controller.h"
#include "luvbot_control_interfaces/srv/control_wbc_motion.hpp"
#include "rknn_model.h"
#include "wbc_lora_assets.h"
#include "wbc_json_reference.h"

class RLWbc : public BaseRLController {
 public:
  explicit RLWbc(rclcpp::Node::SharedPtr node);
  ~RLWbc() override;

 private:
  struct Motion {
    std::string robot_cfg_path_;
    std::string config_dir_;
    std::vector<float> default_joint_positions_;
    std::vector<float> action_scale_;
    std::vector<int> history_steps_;
    std::vector<int> future_steps_;
    int previous_action_steps_{3};
    double control_dt_{0.02};
    double start_hold_sec_{1.0};
    std::string policy_input_name_{"policy"};
    std::string command_input_name_{"command_short"};
    std::string output_name_{"action"};
    int reference_num_frames_{0};
    int reference_root_body_index_{-1};
    std::string motion_id_;
    std::string adapter_id_;
    std::shared_ptr<const WbcJsonReference> json_reference_;
    bool json_consumed_{false};

    std::unique_ptr<RknnModel> model_;
    RknnModel* policy_{nullptr};
    std::unique_ptr<RknnModel> reference_model_;
    std::array<int, 6> reference_output_indices_{};
    int policy_input_index_{-1};
    int command_input_index_{-1};
    int output_index_{-1};
  };
  struct RobotFrame {
    std::array<float, 3> angular_velocity{};
    std::array<float, 3> projected_gravity{};
    std::vector<float> joint_position;
    std::vector<float> joint_velocity;
  };

  struct ReferenceFrame {
    std::array<double, 3> root_position{};
    std::array<double, 4> root_orientation{};
    std::vector<float> joint_position;
  };

  std::string rlCmdPublishTopic() const override;
  void loadConfig(Motion& motion, const std::string& path);
  void initializeModel(Motion& motion);
  void runLoop();
  bool inferAndPublish(double elapsed_sec, bool commit_action, std::string& error);
  bool captureFrame(RobotFrame& frame, std::array<double, 4>* orientation_wxyz, std::string& error,
                    int64_t* observation_stamp = nullptr);
  ReferenceFrame inferReferenceFrame(int index);
  ReferenceFrame inferReferenceFrame(const Motion& motion, int index);
  std::vector<float> makePolicyInput() const;
  std::vector<float> makeCommandInput(const std::array<double, 4>& robot_orientation, int reference_index,
                                    bool reset_anchor);
  void publishDone(bool success, const char* reason);
  void logTiming(const char* phase);

  void prepareCallback(const std::shared_ptr<luvbot_control_interfaces::srv::ControlWbcMotion::Request> request,
                       std::shared_ptr<luvbot_control_interfaces::srv::ControlWbcMotion::Response> response);
  void activeCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void initializeJsonReference();
  void jsonReferenceCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void jsonCancelCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  void publishJsonStatus(const std::string& id, const char* state, const std::string& detail);
  void stateCallback(const luvbot_control_interfaces::msg::PolicyObservation::ConstSharedPtr msg) override;

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, std::unique_ptr<Motion>> motions_;
  Motion* current_{nullptr};
  std::unique_ptr<WbcLoraAssets> lora_assets_;
  std::unique_ptr<RknnModel> shared_policy_;
  std::array<WbcLoraAssets::Buffer, 2> adapter_buffers_;
  int active_buffer_{0};
  // This mutex is only taken by non-realtime prepare callbacks, never by the control loop.
  std::mutex loading_mutex_;
  bool preparing_{false};
  uint64_t prepare_generation_{0};
  double prepare_timeout_sec_{0.5};
  rclcpp::CallbackGroup::SharedPtr prepare_callback_group_;
  std::vector<const float*> inputs_;
  std::vector<std::size_t> input_sizes_;
  std::vector<std::string> joint_names_;
  LeggedDeployParams deploy_params_;

  struct TimingStats {
    uint64_t samples{0};
    uint64_t timeouts{0};
    double reference_total{0.0};
    double reference_max{0.0};
    double policy_total{0.0};
    double policy_max{0.0};
    double cycle_total{0.0};
    double cycle_max{0.0};
    // Bounded 0.1 ms histograms; final bucket is overflow (>= 100 ms).
    std::array<uint64_t, 1001> reference_hist{}, policy_hist{}, cycle_hist{};
  } timing_;
  double reference_sec_{0.0};
  double policy_sec_{0.0};
  std::size_t reference_queries_{0};
  std::deque<RobotFrame> state_history_;
  std::deque<std::vector<float>> action_history_;
  std::array<double, 4> yaw_anchor_wxyz_{{1.0, 0.0, 0.0, 0.0}};
  std::chrono::steady_clock::time_point motion_start_{};
  bool prepared_{false};
  bool active_{false};
  bool done_published_{false};
  std::mutex observation_mutex_;
  bool observation_received_{false};
  std::chrono::steady_clock::time_point observation_received_at_{};
  int64_t observation_stamp_ns_{0};
  double sensor_timeout_sec_{0.1};

  mutable std::mutex mutex_;
  std::atomic<bool> loop_running_{true};
  std::thread loop_thread_;
  rclcpp::Service<luvbot_control_interfaces::srv::ControlWbcMotion>::SharedPtr prepare_service_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  std::string json_base_action_;
  ReferenceFrame json_base_reference_;
  std::vector<std::pair<double, double>> json_joint_limits_;
  std::deque<std::string> json_recent_ids_, json_cancelled_ids_;
  rclcpp::CallbackGroup::SharedPtr json_callback_group_, json_cancel_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_sub_, json_cancel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_status_pub_;
};
