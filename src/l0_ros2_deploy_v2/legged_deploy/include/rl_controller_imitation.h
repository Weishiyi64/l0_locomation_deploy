#ifndef RL_SDK_IMITATION_H
#define RL_SDK_IMITATION_H

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
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>
#include <vector>

#include "base/base_rl_controller.h"
#include "base/rl_controller_registrar.h"
#include "order_mapping.h"
#ifdef USE_TENSORRT
#include "trt_logger.h"
#endif
#include "luvbot_control_interfaces/srv/switch_policy.hpp"
#include "rl_data.h"
#include "rl_data_imitation.h"

// Per-output buffer for the imitation (motion) model.
// Holds the TRT binding index, a GPU device pointer, and a CPU readback buffer.
struct ImitationOutputBuffer {
  int binding_idx = -1;
  void* device_ptr = nullptr;
  std::vector<float> host_buf;
};

class RL_imitation : public BaseRLController {
 public:
  RL_imitation(rclcpp::Node::SharedPtr nh);
  ~RL_imitation();

  void initTopic(rclcpp::Node::SharedPtr nh, bool create_switch_service = true);
  void initModel();
  bool getParam();
  void initData();

  bool updateObservation();
  bool updateAction();
  void updateImitationData();

  // Fallback observation builder used when no imitation model is provided
  void updateObservationFallback();

  Eigen::Vector3d qr_ty_i(const Eigen::Vector4d& q, const Eigen::Vector3d& v);

  void update();
  void stateCallback(const luvbot_control_interfaces::msg::PolicyObservation::ConstSharedPtr msg) override;
  void imitationResetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void imitationActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  RL_IMITATION_ModelParams params;
  LeggedDeployParams deploy_params;

 protected:
  std::string rlCmdPublishTopic() const override;

  std::atomic_bool loopRunning_{};
  std::thread loopThread_;

 private:
  void set_anchor_ori(const Eigen::Quaterniond& robot_initial_quat, const Eigen::Quaterniond& q_robot,
                      const Eigen::Quaterniond& imitation_initial_quat, const Eigen::Quaterniond& q_imitation);

  Eigen::Quaterniond get_imitation_anchor_ori_b_cpp(const Eigen::Quaterniond& robot_initial_quat,
                                                    const Eigen::Quaterniond& q_robot,
                                                    const Eigen::Quaterniond& imitation_initial_quat,
                                                    const Eigen::Quaterniond& q_imitation);

  struct MotionData {
    Eigen::Quaterniond body_quat;
    Eigen::VectorXd joint_pos;
    Eigen::VectorXd joint_vel;
    Eigen::VectorXd contact_mask;
    Eigen::VectorXd root_pos;
    Eigen::VectorXd root_quat;
    Eigen::VectorXd joint_pos_policy;
  };

  struct CachedPolicyRuntime {
    CachedPolicyRuntime(rclcpp::Logger logger, rcl_clock_type_t clock_type)
        : rbt_state(logger),
          rbt_output(logger),
          obs(Observations_Imitation(1, 1)),
          imitation_start_time(0, 0, clock_type),
          last_update_time(0, 0, RCL_SYSTEM_TIME) {}

    std::string policy_name;
    std::string robot_cfg_path;
    RL_IMITATION_ModelParams params;
    bool pub_imitationref_joint_states = false;
    Robot_Imitation_State rbt_state;
    Robot_Imitation_Output rbt_output;
    Observations_Imitation obs;
    std::unique_ptr<ObservationManager_Imitation> obs_manager;
    std::unique_ptr<ObservationManager> fallback_obs_manager;
    RL_ModelParams fallback_params;
    rclcpp::Time imitation_start_time;
    bool previous_rl_inference_enabled = false;
    bool initial_quats_logged = false;
    double initial_yaw = 0.0;
    bool initial_yaw_valid = false;
    Eigen::Quaterniond robot_initial_quat;
    Eigen::Quaterniond imitation_initial_quat;
    int time_step_counter = 0;
    bool imitation_model_enabled = true;
    std::string policy_output_name;
    bool motion_done_sent = false;
    std::vector<MotionData> precomputed_motion_data;
    int motion_data_size = 0;
    bool has_body_quat_output = false;
    rclcpp::Time last_update_time;
    bool update_frequency_initialized = false;

#ifdef USE_TENSORRT
    nvinfer1::IRuntime* trt_runtime_policy = nullptr;
    nvinfer1::ICudaEngine* trt_engine_policy = nullptr;
    nvinfer1::IExecutionContext* trt_context_policy = nullptr;
    void* policy_input_device = nullptr;
    void* policy_output_device = nullptr;
    std::vector<float> policy_output_host;
    int policy_input_binding_idx = 0;
    int policy_output_binding_idx = 1;
    nvinfer1::IRuntime* trt_runtime_imitation = nullptr;
    nvinfer1::ICudaEngine* trt_engine_imitation = nullptr;
    nvinfer1::IExecutionContext* trt_context_imitation = nullptr;
    void* imitation_input_device = nullptr;
    int imitation_input_binding_idx = 0;
#elif defined(USE_RKNN)
    std::unique_ptr<RknnModel> rknn_policy;
    std::unique_ptr<RknnModel> rknn_imitation;
    int rknn_policy_output_idx = 0;
#elif defined(USE_OPENVINO)
    ov::CompiledModel ov_compiled_policy;
    ov::InferRequest ov_infer_policy;
    ov::CompiledModel ov_compiled_imitation;
    ov::InferRequest ov_infer_imitation;
#endif
    std::map<std::string, ImitationOutputBuffer> imitation_outputs;
  };

  // Auto-detect motion duration by checking if 5 consecutive frames are
  // identical
  int detectMaxTimeSteps(const std::map<std::string, size_t>& output_dims);

  void createSwitchPolicyService();
  void resetImitationState();
  void resetObservationState();
  void releaseModelResourcesLocked();
  void releaseCachedRuntime(CachedPolicyRuntime& runtime);
  CachedPolicyRuntime takeCurrentRuntimeLocked();
  void storeCurrentRuntimeLocked();
  void restoreRuntimeLocked(CachedPolicyRuntime&& runtime);
  bool loadPolicyIntoCurrentLocked(const std::string& policy_name, std::string& robot_cfg_path, std::string& message);
  bool loadPolicyLocked(const std::string& policy_name, std::string& robot_cfg_path, std::string& message);
  void preloadPoliciesLocked(const std::vector<std::string>& policies);
  void refreshInferenceEnabledLocked();
  void switchImitationPolicyCallback(
      const std::shared_ptr<luvbot_control_interfaces::srv::SwitchPolicy::Request> request,
      std::shared_ptr<luvbot_control_interfaces::srv::SwitchPolicy::Response> response);

  // Run one imitation model inference step.
  // Copies timestep to device, calls executeV2, copies all outputs to host.
  // Returns false on CUDA or TRT error.
  bool runImitationInference(float timestep);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subResetCmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subImitationActive_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubImitationJointStates_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubImitationMotionDone_;
  rclcpp::Service<luvbot_control_interfaces::srv::SwitchPolicy>::SharedPtr switchPolicySrv_;

  bool motion_done_sent_ = false;

  bool pub_imitationref_joint_states_ = false;

  Robot_Imitation_State rbt_state;
  Robot_Imitation_Output rbt_output;
  Observations_Imitation obs;

  std::unique_ptr<ObservationManager_Imitation> obs_manager_;
  // Fallback (non-v3) observation manager when motion model is disabled
  std::unique_ptr<ObservationManager> fallback_obs_manager_;
  // Parameters mirrored for the non-v3 fallback ObservationManager
  RL_ModelParams fallback_params_;

  rclcpp::Time imitation_start_time;
  bool previous_rl_inference_enabled_ = false;
  bool initial_quats_logged_ = false;
  double initial_yaw_ = 0.0;
  bool initial_yaw_valid_ = false;

  Eigen::Quaterniond robot_initial_quat, imitation_initial_quat;

  // Order mapper instance
  order_mapping::OrderMapper order_mapper_;

  // time step counter and motion flag
  int time_step_counter_ = 0;
  // whether a separate imitation model (imitation_name) was provided and loaded
  bool imitation_model_enabled_ = true;

  // optional override for the policy model output name (lowercased)
  std::string policy_output_name_;
  std::string policy_root_path_;
  std::string robot_type_;
  std::string current_policy_name_;
  std::string current_robot_cfg_path_;
  bool dynamic_policy_switch_enabled_ = false;
  bool imitation_active_ = false;
  std::atomic_bool model_loaded_{false};
  std::atomic<int> loop_frequency_hz_{100};
  mutable std::mutex model_mutex_;
  std::vector<std::string> available_imitation_policies_;
  std::vector<std::string> preload_imitation_policies_;
  std::map<std::string, CachedPolicyRuntime> cached_policies_;
  rclcpp::Time last_update_time_;
  bool update_frequency_initialized_ = false;

  std::vector<MotionData> precomputed_motion_data_;
  int motion_data_size_ = 0;
  bool has_body_quat_output_ = false;

#ifdef USE_TENSORRT
  // ---- Shared TRT logger ----
  TRTLogger trt_logger_;

  // ---- Policy model TRT objects ----
  nvinfer1::IRuntime* trt_runtime_policy_ = nullptr;
  nvinfer1::ICudaEngine* trt_engine_policy_ = nullptr;
  nvinfer1::IExecutionContext* trt_context_policy_ = nullptr;
  void* policy_input_device_ = nullptr;
  void* policy_output_device_ = nullptr;
  std::vector<float> policy_output_host_;
  int policy_input_binding_idx_ = 0;
  int policy_output_binding_idx_ = 1;

  // ---- Imitation (motion) model TRT objects ----
  nvinfer1::IRuntime* trt_runtime_imitation_ = nullptr;
  nvinfer1::ICudaEngine* trt_engine_imitation_ = nullptr;
  nvinfer1::IExecutionContext* trt_context_imitation_ = nullptr;
  void* imitation_input_device_ = nullptr;
  int imitation_input_binding_idx_ = 0;
  std::vector<void*> imitation_bindings_;
#elif defined(USE_RKNN)
  std::unique_ptr<RknnModel> rknn_policy_;
  std::unique_ptr<RknnModel> rknn_imitation_;
  int rknn_policy_output_idx_ = 0;
#elif defined(USE_OPENVINO)
  // ---- OpenVINO inference objects ----
  ov::Core ov_core_;
  ov::CompiledModel ov_compiled_policy_;
  ov::InferRequest ov_infer_policy_;
  ov::CompiledModel ov_compiled_imitation_;
  ov::InferRequest ov_infer_imitation_;
#endif

  // Named output buffers (host side) – used by both TRT and OpenVINO.
  // TRT also stores a device_ptr; OpenVINO leaves device_ptr nullptr.
  std::map<std::string, ImitationOutputBuffer> imitation_outputs_;

  rclcpp::Node::SharedPtr nh_;
};

#endif
