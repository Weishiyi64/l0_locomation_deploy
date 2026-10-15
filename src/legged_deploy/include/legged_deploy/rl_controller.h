#ifndef RL_SDK_H
#define RL_SDK_H

#ifdef USE_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#elif defined(USE_OPENVINO)
#include <openvino/openvino.hpp>
#endif

#include <Eigen/Dense>
#include <atomic>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <thread>

#include "legged_deploy/base/base_rl_controller.h"
#include "legged_deploy/base/rl_controller_registrar.h"
#include "legged_deploy/order_mapping.h"
#ifdef USE_TENSORRT
#include "legged_deploy/trt_logger.h"
#endif
#include "legged_msgs/msg/low_cmd.h"
#include "legged_msgs/msg/low_state.hpp"
#include "legged_msgs/msg/pos_cmd.h"
#include "rl_data.h"

class RL : public BaseRLController {
 public:
  RL(rclcpp::Node::SharedPtr nh);
  ~RL();

  void initTopic(rclcpp::Node::SharedPtr nh);
  void initModel();
  virtual bool getParam();
  void initData();

  // Made virtual so derived controllers can use different observation storage
  virtual void updateObservation();
  // Model inference hooks. Made virtual so derived controllers
  // can override how they interpret model outputs (by name, dict, etc.).
  virtual void updateAction();

  Eigen::Vector3d qr_ty_i(const Eigen::Vector4d& q, const Eigen::Vector3d& v);

  void update();
  void stateCallback(const legged_msgs::msg::LowState::ConstSharedPtr msg);

  RL_ModelParams params;
  LeggedDeployParams deploy_params;

 private:
  rclcpp::Node::SharedPtr nh_;

 protected:
  std::string rlCmdPublishTopic() const override;

  std::atomic_bool loopRunning_{};
  std::thread loopThread_;

  // Set by /rl_obs_history_reset and consumed in the policy thread.
  std::atomic_bool obs_reset_requested_{false};

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
#elif defined(USE_OPENVINO)
  // OpenVINO inference objects
  ov::Core ov_core_;
  ov::CompiledModel ov_compiled_model_;
  ov::InferRequest ov_infer_request_;
#endif
  std::string policy_output_name_;
  order_mapping::OrderMapper order_mapper_;

  void processBehaviorInput();
  bool isInputRisingEdge(const std::string& input_name, bool current_state);
  void resetObservationState();
  void requestObservationStateReset();
  void obsHistoryResetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void policyCommandCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);

  std::map<std::string, bool> lastInputStates_;
  std::atomic_bool imitation_active_{false};
  bool previous_rl_inference_enabled_ = false;
  double initial_yaw_ = 0.0;
  bool initial_yaw_valid_ = false;
  std::mutex policy_command_mutex_;
  Eigen::Vector3d policy_command_ = Eigen::Vector3d::Zero();
  rclcpp::Time policy_command_stamp_{0, 0, RCL_ROS_TIME};
  bool external_policy_command_received_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subObsHistoryReset_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subImitationActive_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subPolicyCommand_;
};

#endif
