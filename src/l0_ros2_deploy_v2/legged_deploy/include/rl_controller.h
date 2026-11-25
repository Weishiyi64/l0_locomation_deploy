#ifndef RL_SDK_H
#define RL_SDK_H

#ifdef USE_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#elif defined(USE_RKNN)
#include "rknn_model.h"
#elif defined(USE_OPENVINO)
#include <openvino/openvino.hpp>
#endif

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
#ifdef USE_TENSORRT
#include "trt_logger.h"
#endif
#include "rl_data.h"
#include "luvbot_control_interfaces/srv/seed_policy_history.hpp"

class RL : public BaseRLController {
 public:
  RL(rclcpp::Node::SharedPtr nh);
  ~RL();

  void initTopic(rclcpp::Node::SharedPtr nh);
  void initModel();
  virtual bool getParam();
  void initData();

  // Made virtual so derived controllers can use different observation storage
  virtual bool updateObservation();
  // Model inference hooks. Made virtual so derived controllers
  // can override how they interpret model outputs (by name, dict, etc.).
  virtual bool updateAction();

  Eigen::Vector3d qr_ty_i(const Eigen::Vector4d& q, const Eigen::Vector3d& v);

  void update();
  void stateCallback(const luvbot_control_interfaces::msg::PolicyObservation::ConstSharedPtr msg) override;
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
  // Serialize callbacks with the 50 Hz policy thread; never acquired by the hardware loop.
  std::mutex state_mutex_;
  bool history_init_pending_{true};
  bool handoff_seed_pending_{false};
  uint64_t last_handoff_id_{0};
  Eigen::VectorXd handoff_seed_mj_;
  std::chrono::steady_clock::time_point handoff_seed_received_{};
  rclcpp::Service<luvbot_control_interfaces::srv::SeedPolicyHistory>::SharedPtr seed_history_service_;
  void seedHistoryCallback(
      const std::shared_ptr<luvbot_control_interfaces::srv::SeedPolicyHistory::Request> request,
      std::shared_ptr<luvbot_control_interfaces::srv::SeedPolicyHistory::Response> response);

  Robot_State rbt_state;
  Robot_Output rbt_output;
  Observations obs;

  std::unique_ptr<ObservationManager> obs_manager_;

  rclcpp::Time start_time;

#ifdef USE_TENSORRT
  // TensorRT inference objects
  TRTLogger trt_logger_;
  nvinfer1::IRuntime* trt_runtime_ = nullptr;
  nvinfer1::ICudaEngine* trt_engine_ = nullptr;
  nvinfer1::IExecutionContext* trt_context_ = nullptr;
  void* input_device_ = nullptr;
  void* output_device_ = nullptr;
  std::vector<float> output_host_;
  int input_binding_idx_ = 0;
  int output_binding_idx_ = 1;
#elif defined(USE_RKNN)
  std::unique_ptr<RknnModel> rknn_policy_;
  int rknn_policy_output_idx_ = 0;
#elif defined(USE_OPENVINO)
  // OpenVINO inference objects
  ov::Core ov_core_;
  ov::CompiledModel ov_compiled_model_;
  ov::InferRequest ov_infer_request_;
#endif
  std::string policy_output_name_;
  order_mapping::OrderMapper order_mapper_;

  void resetObservationState();
  void requestObservationStateReset();
  void obsHistoryResetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subWbcActive_;
  std::atomic<bool> wbc_active_{false};
  std::atomic_bool imitation_active_{false};
  bool previous_rl_inference_enabled_ = false;
  double initial_yaw_ = 0.0;
  bool initial_yaw_valid_ = false;
  std::mutex policy_command_mutex_;
  Eigen::Vector3d policy_command_ = Eigen::Vector3d::Zero();

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subObsHistoryReset_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subImitationActive_;
};

#endif
