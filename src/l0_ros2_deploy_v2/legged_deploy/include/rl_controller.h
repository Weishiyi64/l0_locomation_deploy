#ifndef RL_SDK_H
#define RL_SDK_H

#include "inference_model.h"

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>

#include "base/base_rl_controller.h"
#include "base/rl_controller_registrar.h"
#include "order_mapping.h"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "rl_data.h"

class RL : public BaseRLController {
 public:
  RL(rclcpp::Node::SharedPtr nh);
  ~RL();

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actor_input_pub_;
  void initTopic(rclcpp::Node::SharedPtr nh);
  void initModel();
  virtual bool getParam();
  void initData();

  bool updateObservation(const robot_interfaces::msg::PolicyObservation& observation);
  bool updateAction(Robot_Output& output);

  void update();
  void stateCallback(const robot_interfaces::msg::PolicyObservation::ConstSharedPtr msg) override;
  RL_ModelParams params;
  LeggedDeployParams deploy_params;

 private:
  rclcpp::Node::SharedPtr nh_;

 protected:
  std::string rlCmdPublishTopic() const override;

  std::atomic_bool loopRunning_{};
  std::thread loopThread_;

  // Set by control/policy/observation/reset and consumed in the policy thread.
  std::atomic_bool obs_reset_requested_{false};

  Robot_State rbt_state;
  Robot_Output rbt_output;
  Observations obs;

  std::unique_ptr<ObservationManager> obs_manager_;

  std::unique_ptr<InferenceModel> model_policy_;
  int model_policy_output_idx_ = 0;
  std::string policy_output_name_;
  order_mapping::OrderMapper order_mapper_;

  void resetObservationState();
  void requestObservationStateReset();
  void obsHistoryResetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subWbcActive_;
  std::atomic<bool> wbc_active_{false};
  std::string initialized_return_id_;  // Policy thread only.
  bool has_published_action_{false};  // Policy thread only.
  std::atomic_bool imitation_active_{false};
  bool previous_rl_inference_enabled_ = false;
  Eigen::Quaterniond initial_yaw_quat_inv_ = Eigen::Quaterniond::Identity();
  bool initial_yaw_valid_ = false;
  std::mutex observation_mutex_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subObsHistoryReset_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subImitationActive_;
};

#endif
