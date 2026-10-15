#include "legged_deploy/rl_controller_imitation.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <utility>

#include "legged_deploy/base/rl_controller_registrar.h"
#include "legged_deploy/order_mapping.h"

static RLControllerRegistrar registrar_rl_imitation("rl_imitation", [](rclcpp::Node::SharedPtr nh) {
  return std::unique_ptr<ControllerType>(new RL_imitation(nh));
});

// RL_imitation::RL_imitation(ros::NodeHandle& node_handle) :
// BaseRLController(node_handle), loopRunning_(true) {
RL_imitation::RL_imitation(rclcpp::Node::SharedPtr node_handle)
    : BaseRLController(node_handle),
      loopRunning_(true),
      rbt_state(node_handle->get_logger()),
      rbt_output(node_handle->get_logger()),
      obs(Observations_Imitation(1, 1)),
      order_mapper_(node_handle->get_logger()),
      nh_(node_handle) {
  imitation_start_time = rclcpp::Time(0, 0, node_handle->get_clock()->get_clock_type());
  last_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  node_handle->get_parameter_or("dynamic_policy_switch_enabled", dynamic_policy_switch_enabled_, false);
  node_handle->get_parameter_or("policy_root_path", policy_root_path_, std::string(LEGGED_POLICY_PATH));
  node_handle->get_parameter_or("text_motion_enabled", text_motion_enabled_, false);
  node_handle->get_parameter_or("text_motion_policy", text_motion_policy_, std::string(""));
  node_handle->get_parameter_or("text_motion_json_topic", text_motion_json_topic_, std::string("/text_motion/json"));
  node_handle->get_parameter_or("text_motion_ready_topic", text_motion_ready_topic_,
                                std::string("/text_motion/ready"));
  node_handle->get_parameter_or("text_motion_external_packager", text_motion_external_packager_, false);
  node_handle->get_parameter_or("text_motion_amp_stand_profile", text_motion_amp_stand_profile_, std::string(""));
  int text_motion_max_payload_bytes = static_cast<int>(text_motion_max_payload_bytes_);
  node_handle->get_parameter_or("text_motion_max_payload_bytes", text_motion_max_payload_bytes,
                                text_motion_max_payload_bytes);
  node_handle->get_parameter_or("text_motion_max_frames", text_motion_max_frames_, text_motion_max_frames_);
  node_handle->get_parameter_or("text_motion_hold_frames", text_motion_hold_frames_, text_motion_hold_frames_);
  node_handle->get_parameter_or("text_motion_transition_frames", text_motion_transition_frames_,
                                text_motion_transition_frames_);
  node_handle->get_parameter_or("text_motion_return_frames", text_motion_return_frames_,
                                text_motion_return_frames_);
  node_handle->get_parameter_or("text_motion_return_settle_frames", text_motion_return_settle_frames_,
                                text_motion_return_settle_frames_);
  node_handle->get_parameter_or("text_motion_lowpass_cutoff_hz", text_motion_lowpass_cutoff_hz_,
                                text_motion_lowpass_cutoff_hz_);
  text_motion_max_payload_bytes_ = static_cast<size_t>(std::max(1024, text_motion_max_payload_bytes));
  text_motion_max_frames_ = std::max(2, text_motion_max_frames_);
  text_motion_hold_frames_ = std::max(0, text_motion_hold_frames_);
  text_motion_transition_frames_ = std::max(2, text_motion_transition_frames_);
  text_motion_return_frames_ = std::max(2, text_motion_return_frames_);
  text_motion_return_settle_frames_ = std::max(1, text_motion_return_settle_frames_);
  text_motion_lowpass_cutoff_hz_ = std::max(0.0, text_motion_lowpass_cutoff_hz_);
  if (text_motion_enabled_ &&
      (!dynamic_policy_switch_enabled_ || text_motion_policy_.empty() ||
       (!text_motion_external_packager_ && text_motion_amp_stand_profile_.empty())))
  {
    throw std::runtime_error(
        "text_motion_enabled requires dynamic policy switching and the configured text-motion packaging inputs");
  }
  int idle_frequency = 100;
  node_handle->get_parameter_or("dynamic_imitation_idle_frequency", idle_frequency, 100);
  loop_frequency_hz_.store(std::max(1, idle_frequency));
  if (node_handle->has_parameter("available_imitation_policies"))
  {
    node_handle->get_parameter("available_imitation_policies", available_imitation_policies_);
  }
  if (node_handle->has_parameter("preload_imitation_policies"))
  {
    node_handle->get_parameter("preload_imitation_policies", preload_imitation_policies_);
  }

  try
  {
    if (dynamic_policy_switch_enabled_)
    {
      if (!node_handle->get_parameter("robot_type", robot_type_) || robot_type_.empty())
      {
        throw std::runtime_error("Required parameter 'robot_type' not found for dynamic imitation worker");
      }
      params.robot_type = robot_type_;
      if (!readDeployParams(deploy_params))
      {
        throw std::runtime_error("Failed to read deployment parameters");
      }
      if (!preload_imitation_policies_.empty())
      {
        std::lock_guard<std::mutex> lock(model_mutex_);
        preloadPoliciesLocked(preload_imitation_policies_);
      }
      initTopic(nh);
      quit.store(false);
      model_loaded_.store(false);
    }
    else
    {
      // Get parameters
      if (!getParam())
      {
        throw std::runtime_error("Failed to get parameters");
      }
      robot_type_ = params.robot_type;
      // ros2
      std::string robot_cfg_path("");
      if (!node_handle->get_parameter("robot_cfg_path", robot_cfg_path))
      {
        throw std::runtime_error("Required parameter 'robot_cfg_path' not found");
      }
      YAML::Node root = YAML::LoadFile(robot_cfg_path);
      YAML::Node rc = root["RobotCfg"];
      requireNode(rc, "RobotCfg");
      policy_output_name_ = rc["policy_output_name"].as<std::string>("");
      std::transform(policy_output_name_.begin(), policy_output_name_.end(), policy_output_name_.begin(), ::tolower);

      pub_imitationref_joint_states_ = rc["pub_imitationref_joint_states"].as<bool>(false);

      // Initialize
      order_mapper_.initialize(params.joint_order_mj, params.joint_order_isaac);

      initData();
      initTopic(nh);
      RCLCPP_INFO(node_handle->get_logger(), "RL_imitation: loading policy '%s'", params.subpolicy.c_str());
      initModel();
      current_policy_name_ = params.subpolicy;
      current_robot_cfg_path_ = robot_cfg_path;
      model_loaded_.store(true);
      loop_frequency_hz_.store(std::max(1, params.policy_frequency));

      // Set quit flag
      quit.store(false);

      RCLCPP_INFO(node_handle->get_logger(), "RL_imitation: policy '%s' loaded OK", params.subpolicy.c_str());
    }
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_handle->get_logger(), "Failed to initialize RL controller: %s", e.what());
    throw;
  }

  // Setup loop thread
  loopThread_ = std::thread([this]() {
    int last_frequency = 0;
    std::unique_ptr<rclcpp::Rate> rate;
    while (loopRunning_ && rclcpp::ok())
    {
      const int frequency = std::max(1, loop_frequency_hz_.load());
      if (!rate || frequency != last_frequency)
      {
        rate = std::make_unique<rclcpp::Rate>(frequency);
        last_frequency = frequency;
      }
      update();
      rate->sleep();
    }
  });

  // Set realtime priority
  if (deploy_params.set_realtime_priority)
  {
    sched_param sched{};
    sched.sched_priority = deploy_params.thread_priority;
    int policy = getSchedulerPolicy(deploy_params.thread_scheduling_method);
    if (pthread_setschedparam(loopThread_.native_handle(), policy, &sched) != 0)
    {
      // ("RL_imitation: Failed to set realtime priority");
      RCLCPP_WARN(node_handle->get_logger(), "RL_imitation: Failed to set realtime priority");
    }
    else
    {
      printThreadSchedulerInfo(loopThread_.native_handle(), "RL_imitation");
    }
  }
}

std::string RL_imitation::rlCmdPublishTopic() const { return "/rlCmd_imitation"; }

RL_imitation::~RL_imitation() {
  {
    std::lock_guard<std::mutex> lock(text_motion_queue_mutex_);
    text_motion_worker_stop_ = true;
  }
  text_motion_queue_cv_.notify_all();
  if (text_motion_worker_thread_.joinable()) text_motion_worker_thread_.join();

  loopRunning_ = false;
  if (loopThread_.joinable())
  {
    loopThread_.join();
  }

  std::lock_guard<std::mutex> lock(model_mutex_);
  releaseModelResourcesLocked();
  for (auto& kv : cached_policies_)
  {
    releaseCachedRuntime(kv.second);
  }
  cached_policies_.clear();
}

void RL_imitation::releaseModelResourcesLocked() {
  model_loaded_.store(false);
  rl_inference_enabled_.store(false);
  current_policy_name_.clear();
  current_robot_cfg_path_.clear();
  previous_rl_inference_enabled_ = false;
  update_frequency_initialized_ = false;
  motion_done_sent_ = false;
  precomputed_motion_data_.clear();
  motion_data_size_ = 0;
  has_body_quat_output_ = false;
  native_state_history_.clear();
  native_policy_input_.clear();
  native_command_input_.clear();

#ifdef USE_TENSORRT
  if (policy_input_device_)
  {
    cudaFree(policy_input_device_);
    policy_input_device_ = nullptr;
  }
  if (policy_command_device_)
  {
    cudaFree(policy_command_device_);
    policy_command_device_ = nullptr;
  }
  if (policy_output_device_)
  {
    cudaFree(policy_output_device_);
    policy_output_device_ = nullptr;
  }
  if (trt_context_policy_)
  {
    delete trt_context_policy_;
    trt_context_policy_ = nullptr;
  }
  if (trt_engine_policy_)
  {
    delete trt_engine_policy_;
    trt_engine_policy_ = nullptr;
  }
  if (trt_runtime_policy_)
  {
    delete trt_runtime_policy_;
    trt_runtime_policy_ = nullptr;
  }

  if (imitation_input_device_)
  {
    cudaFree(imitation_input_device_);
    imitation_input_device_ = nullptr;
  }
  for (auto& kv : imitation_outputs_)
  {
    if (kv.second.device_ptr)
    {
      cudaFree(kv.second.device_ptr);
      kv.second.device_ptr = nullptr;
    }
  }
  if (trt_context_imitation_)
  {
    delete trt_context_imitation_;
    trt_context_imitation_ = nullptr;
  }
  if (trt_engine_imitation_)
  {
    delete trt_engine_imitation_;
    trt_engine_imitation_ = nullptr;
  }
  if (trt_runtime_imitation_)
  {
    delete trt_runtime_imitation_;
    trt_runtime_imitation_ = nullptr;
  }
#elif defined(USE_OPENVINO)
  ov_compiled_policy_ = ov::CompiledModel();
  ov_infer_policy_ = ov::InferRequest();
  ov_compiled_imitation_ = ov::CompiledModel();
  ov_infer_imitation_ = ov::InferRequest();
#endif

  imitation_outputs_.clear();
  obs_manager_.reset();
  fallback_obs_manager_.reset();
#ifdef USE_TENSORRT
  policy_output_host_.clear();
#endif
  params = RL_IMITATION_ModelParams();
  params.robot_type = robot_type_;
}

void RL_imitation::releaseCachedRuntime(CachedPolicyRuntime& runtime) {
#ifdef USE_TENSORRT
  if (runtime.policy_input_device)
  {
    cudaFree(runtime.policy_input_device);
    runtime.policy_input_device = nullptr;
  }
  if (runtime.policy_command_device)
  {
    cudaFree(runtime.policy_command_device);
    runtime.policy_command_device = nullptr;
  }
  if (runtime.policy_output_device)
  {
    cudaFree(runtime.policy_output_device);
    runtime.policy_output_device = nullptr;
  }
  if (runtime.trt_context_policy)
  {
    delete runtime.trt_context_policy;
    runtime.trt_context_policy = nullptr;
  }
  if (runtime.trt_engine_policy)
  {
    delete runtime.trt_engine_policy;
    runtime.trt_engine_policy = nullptr;
  }
  if (runtime.trt_runtime_policy)
  {
    delete runtime.trt_runtime_policy;
    runtime.trt_runtime_policy = nullptr;
  }

  if (runtime.imitation_input_device)
  {
    cudaFree(runtime.imitation_input_device);
    runtime.imitation_input_device = nullptr;
  }
  for (auto& kv : runtime.imitation_outputs)
  {
    if (kv.second.device_ptr)
    {
      cudaFree(kv.second.device_ptr);
      kv.second.device_ptr = nullptr;
    }
  }
  if (runtime.trt_context_imitation)
  {
    delete runtime.trt_context_imitation;
    runtime.trt_context_imitation = nullptr;
  }
  if (runtime.trt_engine_imitation)
  {
    delete runtime.trt_engine_imitation;
    runtime.trt_engine_imitation = nullptr;
  }
  if (runtime.trt_runtime_imitation)
  {
    delete runtime.trt_runtime_imitation;
    runtime.trt_runtime_imitation = nullptr;
  }
  runtime.policy_output_host.clear();
#elif defined(USE_OPENVINO)
  runtime.ov_compiled_policy = ov::CompiledModel();
  runtime.ov_infer_policy = ov::InferRequest();
  runtime.ov_compiled_imitation = ov::CompiledModel();
  runtime.ov_infer_imitation = ov::InferRequest();
#endif

  runtime.imitation_outputs.clear();
  runtime.obs_manager.reset();
  runtime.fallback_obs_manager.reset();
  runtime.precomputed_motion_data.clear();
}

RL_imitation::CachedPolicyRuntime RL_imitation::takeCurrentRuntimeLocked() {
  CachedPolicyRuntime runtime(nh_->get_logger(), nh_->get_clock()->get_clock_type());
  runtime.policy_name = current_policy_name_;
  runtime.robot_cfg_path = current_robot_cfg_path_;
  runtime.params = std::move(params);
  runtime.pub_imitationref_joint_states = pub_imitationref_joint_states_;
  runtime.rbt_state = std::move(rbt_state);
  runtime.rbt_output = std::move(rbt_output);
  runtime.obs = std::move(obs);
  runtime.obs_manager = std::move(obs_manager_);
  runtime.fallback_obs_manager = std::move(fallback_obs_manager_);
  runtime.fallback_params = std::move(fallback_params_);
  runtime.imitation_start_time = imitation_start_time;
  runtime.previous_rl_inference_enabled = previous_rl_inference_enabled_;
  runtime.initial_quats_logged = initial_quats_logged_;
  runtime.initial_yaw = initial_yaw_;
  runtime.initial_yaw_valid = initial_yaw_valid_;
  runtime.robot_initial_quat = robot_initial_quat;
  runtime.imitation_initial_quat = imitation_initial_quat;
  runtime.time_step_counter = time_step_counter_;
  runtime.imitation_model_enabled = imitation_model_enabled_;
  runtime.policy_output_name = policy_output_name_;
  runtime.motion_done_sent = motion_done_sent_;
  runtime.precomputed_motion_data = std::move(precomputed_motion_data_);
  runtime.motion_data_size = motion_data_size_;
  runtime.has_body_quat_output = has_body_quat_output_;
  runtime.last_update_time = last_update_time_;
  runtime.update_frequency_initialized = update_frequency_initialized_;

#ifdef USE_TENSORRT
  runtime.trt_runtime_policy = trt_runtime_policy_;
  runtime.trt_engine_policy = trt_engine_policy_;
  runtime.trt_context_policy = trt_context_policy_;
  runtime.policy_input_device = policy_input_device_;
  runtime.policy_command_device = policy_command_device_;
  runtime.policy_output_device = policy_output_device_;
  runtime.policy_output_host = std::move(policy_output_host_);
  runtime.policy_input_binding_idx = policy_input_binding_idx_;
  runtime.policy_command_binding_idx = policy_command_binding_idx_;
  runtime.policy_output_binding_idx = policy_output_binding_idx_;
  runtime.trt_runtime_imitation = trt_runtime_imitation_;
  runtime.trt_engine_imitation = trt_engine_imitation_;
  runtime.trt_context_imitation = trt_context_imitation_;
  runtime.imitation_input_device = imitation_input_device_;
  runtime.imitation_input_binding_idx = imitation_input_binding_idx_;
  trt_runtime_policy_ = nullptr;
  trt_engine_policy_ = nullptr;
  trt_context_policy_ = nullptr;
  policy_input_device_ = nullptr;
  policy_command_device_ = nullptr;
  policy_output_device_ = nullptr;
  policy_input_binding_idx_ = 0;
  policy_command_binding_idx_ = -1;
  policy_output_binding_idx_ = 1;
  trt_runtime_imitation_ = nullptr;
  trt_engine_imitation_ = nullptr;
  trt_context_imitation_ = nullptr;
  imitation_input_device_ = nullptr;
  imitation_input_binding_idx_ = 0;
#elif defined(USE_OPENVINO)
  runtime.ov_compiled_policy = std::move(ov_compiled_policy_);
  runtime.ov_infer_policy = std::move(ov_infer_policy_);
  runtime.ov_compiled_imitation = std::move(ov_compiled_imitation_);
  runtime.ov_infer_imitation = std::move(ov_infer_imitation_);
  ov_compiled_policy_ = ov::CompiledModel();
  ov_infer_policy_ = ov::InferRequest();
  ov_compiled_imitation_ = ov::CompiledModel();
  ov_infer_imitation_ = ov::InferRequest();
#endif
  runtime.imitation_outputs = std::move(imitation_outputs_);

  model_loaded_.store(false);
  rl_inference_enabled_.store(false);
  current_policy_name_.clear();
  current_robot_cfg_path_.clear();
  params = RL_IMITATION_ModelParams();
  params.robot_type = robot_type_;
  pub_imitationref_joint_states_ = false;
  rbt_state = Robot_Imitation_State(nh_->get_logger());
  rbt_output = Robot_Imitation_Output(nh_->get_logger());
  obs = Observations_Imitation(1, 1);
  fallback_params_ = RL_ModelParams();
  imitation_start_time = rclcpp::Time(0, 0, nh_->get_clock()->get_clock_type());
  previous_rl_inference_enabled_ = false;
  initial_quats_logged_ = false;
  initial_yaw_ = 0.0;
  initial_yaw_valid_ = false;
  robot_initial_quat = Eigen::Quaterniond::Identity();
  imitation_initial_quat = Eigen::Quaterniond::Identity();
  order_mapper_ = order_mapping::OrderMapper(nh_->get_logger());
  time_step_counter_ = 0;
  imitation_model_enabled_ = true;
  policy_output_name_.clear();
  motion_done_sent_ = false;
  motion_data_size_ = 0;
  has_body_quat_output_ = false;
  last_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  update_frequency_initialized_ = false;
  return runtime;
}

void RL_imitation::storeCurrentRuntimeLocked() {
  if (!model_loaded_.load() || current_policy_name_.empty()) return;

  const bool keep_cached = std::find(preload_imitation_policies_.begin(), preload_imitation_policies_.end(),
                                     current_policy_name_) != preload_imitation_policies_.end();
  if (!keep_cached)
  {
    releaseModelResourcesLocked();
    return;
  }

  CachedPolicyRuntime runtime = takeCurrentRuntimeLocked();
  if (runtime.policy_name.empty())
  {
    releaseCachedRuntime(runtime);
    return;
  }

  const std::string policy_name = runtime.policy_name;
  auto existing = cached_policies_.find(policy_name);
  if (existing != cached_policies_.end())
  {
    releaseCachedRuntime(existing->second);
    cached_policies_.erase(existing);
  }
  cached_policies_.emplace(policy_name, std::move(runtime));
}

void RL_imitation::restoreRuntimeLocked(CachedPolicyRuntime&& runtime) {
  releaseModelResourcesLocked();

  params = std::move(runtime.params);
  current_policy_name_ = std::move(runtime.policy_name);
  current_robot_cfg_path_ = std::move(runtime.robot_cfg_path);
  pub_imitationref_joint_states_ = runtime.pub_imitationref_joint_states;
  rbt_state = std::move(runtime.rbt_state);
  rbt_output = std::move(runtime.rbt_output);
  obs = std::move(runtime.obs);
  obs_manager_ = std::move(runtime.obs_manager);
  fallback_obs_manager_ = std::move(runtime.fallback_obs_manager);
  fallback_params_ = std::move(runtime.fallback_params);
  imitation_start_time = runtime.imitation_start_time;
  previous_rl_inference_enabled_ = runtime.previous_rl_inference_enabled;
  initial_quats_logged_ = runtime.initial_quats_logged;
  initial_yaw_ = runtime.initial_yaw;
  initial_yaw_valid_ = runtime.initial_yaw_valid;
  robot_initial_quat = runtime.robot_initial_quat;
  imitation_initial_quat = runtime.imitation_initial_quat;
  time_step_counter_ = runtime.time_step_counter;
  imitation_model_enabled_ = runtime.imitation_model_enabled;
  policy_output_name_ = std::move(runtime.policy_output_name);
  motion_done_sent_ = runtime.motion_done_sent;
  precomputed_motion_data_ = std::move(runtime.precomputed_motion_data);
  motion_data_size_ = runtime.motion_data_size;
  has_body_quat_output_ = runtime.has_body_quat_output;
  last_update_time_ = runtime.last_update_time;
  update_frequency_initialized_ = runtime.update_frequency_initialized;

#ifdef USE_TENSORRT
  trt_runtime_policy_ = runtime.trt_runtime_policy;
  trt_engine_policy_ = runtime.trt_engine_policy;
  trt_context_policy_ = runtime.trt_context_policy;
  policy_input_device_ = runtime.policy_input_device;
  policy_command_device_ = runtime.policy_command_device;
  policy_output_device_ = runtime.policy_output_device;
  policy_output_host_ = std::move(runtime.policy_output_host);
  policy_input_binding_idx_ = runtime.policy_input_binding_idx;
  policy_command_binding_idx_ = runtime.policy_command_binding_idx;
  policy_output_binding_idx_ = runtime.policy_output_binding_idx;
  trt_runtime_imitation_ = runtime.trt_runtime_imitation;
  trt_engine_imitation_ = runtime.trt_engine_imitation;
  trt_context_imitation_ = runtime.trt_context_imitation;
  imitation_input_device_ = runtime.imitation_input_device;
  imitation_input_binding_idx_ = runtime.imitation_input_binding_idx;
  runtime.trt_runtime_policy = nullptr;
  runtime.trt_engine_policy = nullptr;
  runtime.trt_context_policy = nullptr;
  runtime.policy_input_device = nullptr;
  runtime.policy_command_device = nullptr;
  runtime.policy_output_device = nullptr;
  runtime.trt_runtime_imitation = nullptr;
  runtime.trt_engine_imitation = nullptr;
  runtime.trt_context_imitation = nullptr;
  runtime.imitation_input_device = nullptr;
#elif defined(USE_OPENVINO)
  ov_compiled_policy_ = std::move(runtime.ov_compiled_policy);
  ov_infer_policy_ = std::move(runtime.ov_infer_policy);
  ov_compiled_imitation_ = std::move(runtime.ov_compiled_imitation);
  ov_infer_imitation_ = std::move(runtime.ov_infer_imitation);
#endif
  imitation_outputs_ = std::move(runtime.imitation_outputs);

  if (!params.robot_type.empty())
  {
    robot_type_ = params.robot_type;
  }
  else
  {
    params.robot_type = robot_type_;
  }
  lowState_.motor_state.resize(params.num_actions);
  posCmd_.q.assign(params.num_actions, 0.0);
  order_mapper_.initialize(params.joint_order_mj, params.joint_order_isaac);
  model_loaded_.store(!current_policy_name_.empty());
  loop_frequency_hz_.store(std::max(1, params.policy_frequency));
}

void RL_imitation::update() {
  std::lock_guard<std::mutex> lock(model_mutex_);
  if (!model_loaded_.load())
  {
    previous_rl_inference_enabled_ = false;
    update_frequency_initialized_ = false;
    return;
  }

  // const auto currentTime = ros::Time::now();
  const auto currentTime = rclcpp::Clock().now();
  bool overrun = false;

  if (!update_frequency_initialized_)
  {
    last_update_time_ = currentTime;
    update_frequency_initialized_ = true;
  }
  else
  {
    // ros::Duration elapsedTime = currentTime - last_call_time;
    rclcpp::Duration elapsedTime = currentTime - last_update_time_;
    last_update_time_ = currentTime;
    // double actual_frequency = 1.0 / elapsedTime.toSec();
    double actual_frequency = 1.0 / elapsedTime.seconds();
    double frequency_diff = std::abs(actual_frequency - params.policy_frequency);
    double frequency_threshold = params.policy_frequency * 0.2;

    if (frequency_diff > frequency_threshold)
    {
      overrun = true;
      // ("[RL Frequency] Expected: %.2f Hz, Actual: %.2f Hz,
      // Difference: %.2f Hz",
      RCLCPP_WARN(nh_->get_logger(),
                  "[RL Frequency] Expected: %.2f Hz, Actual: %.2f Hz, "
                  "Difference: %.2f Hz",
                  static_cast<double>(params.policy_frequency), actual_frequency, frequency_diff);
    }
  }

  // 检测rl_inference_enabled_从false到true的转换
  bool current_rl_inference_enabled = rl_inference_enabled_.load();
  if (current_rl_inference_enabled && !previous_rl_inference_enabled_)
  {
    // 从false转为true时，重置开始时间和时间步计数器
    resetImitationState();
  }
  previous_rl_inference_enabled_ = current_rl_inference_enabled;

  if (current_rl_inference_enabled)
  {
    if (!overrun || !deploy_params.skip_infer_on_overrun)
    {
      updateObservation();
      updateAction();
      // pubCmd_ov.publish(posCmd_);
      pubCmd_ov->publish(posCmd_);
    }
    else
    {
      RCLCPP_WARN(nh_->get_logger(), "Skipping inference this cycle due to overrun by configuration");
    }

    if (motion_data_size_ > 0 && pubImitationMotionDone_)
    {
      const int last_idx = motion_data_size_ - 1;
      if (time_step_counter_ >= last_idx && !motion_done_sent_)
      {
        std_msgs::msg::Bool done_msg;
        done_msg.data = true;
        pubImitationMotionDone_->publish(done_msg);
        motion_done_sent_ = true;
        RCLCPP_INFO(nh_->get_logger(), "RL_imitation: published /imitation_motion_done (reached frame %d / %d)",
                    time_step_counter_, motion_data_size_);
      }
    }
  }
}

// void RL_imitation::initTopic(ros::NodeHandle& nh_) {
void RL_imitation::initTopic(rclcpp::Node::SharedPtr nh_, bool create_switch_service) {
  // initialize common topics (subscribe /rlState, advertise /rlCmd_imitation via rlCmdPublishTopic)
  initBaseTopic(nh_);
  // initialize special topics
  // subResetCmd_ = nh_.subscribe<std_msgs::Bool>("/rlImitationReset", 100,
  // &RL_imitation::imitationResetCallback, this);
  subResetCmd_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "/rlImitationReset", rclcpp::QoS(100),
      std::bind(&RL_imitation::imitationResetCallback, this, std::placeholders::_1));

  subImitationActive_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "/rlImitationActive", rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&RL_imitation::imitationActiveCallback, this, std::placeholders::_1));

  // Created unconditionally; each loaded policy still controls whether reference states are published.
  pubImitationJointStates_ =
      nh_->create_publisher<sensor_msgs::msg::JointState>("/imitationref_joint_states", rclcpp::QoS(100));
  pubImitationMotionDone_ = nh_->create_publisher<std_msgs::msg::Bool>("/imitation_motion_done", rclcpp::QoS(10));
  if (create_switch_service)
  {
    createSwitchPolicyService();
  }

  if (text_motion_enabled_ && !text_motion_external_packager_)
  {
    subTextMotionJson_ = nh_->create_subscription<std_msgs::msg::String>(
        text_motion_json_topic_, rclcpp::QoS(1).reliable(),
        std::bind(&RL_imitation::textMotionJsonCallback, this, std::placeholders::_1));
    pubTextMotionReady_ = nh_->create_publisher<std_msgs::msg::String>(text_motion_ready_topic_,
                                                                      rclcpp::QoS(1).reliable());
    pubTextMotionStatus_ = nh_->create_publisher<std_msgs::msg::String>("/text_motion/status",
                                                                       rclcpp::QoS(10).reliable());
    text_motion_worker_thread_ = std::thread(&RL_imitation::textMotionWorkerLoop, this);
    RCLCPP_INFO(nh_->get_logger(),
                "Text motion receiver enabled: json=%s ready=%s tracking_policy=%s (background preparation)",
                text_motion_json_topic_.c_str(), text_motion_ready_topic_.c_str(), text_motion_policy_.c_str());
  }
}

void RL_imitation::createSwitchPolicyService() {
  switchPolicySrv_ = nh_->create_service<legged_msgs::srv::SwitchImitationPolicy>(
      "/switch_imitation_policy",
      std::bind(&RL_imitation::switchImitationPolicyCallback, this, std::placeholders::_1, std::placeholders::_2));
}

#ifdef USE_TENSORRT
// Helper: compute total number of elements in a TRT binding tensor.
static size_t trtBindingElements(nvinfer1::ICudaEngine* engine, int idx) {
  auto dims = engine->getTensorShape(engine->getIOTensorName(idx));
  size_t count = 1;
  for (int d = 0; d < dims.nbDims; ++d)
  {
    count *= static_cast<size_t>(dims.d[d]);
  }
  return count;
}
// Helper: load a pre-built .trt engine file directly.
// Derives the .trt path from onnx_path by replacing the .onnx extension.
// Returns a deserialized ICudaEngine, or throws on failure.
static nvinfer1::ICudaEngine* loadTrtEngine(const std::string& onnx_path, nvinfer1::IRuntime* runtime) {
  // Derive .trt path
  std::string trt_path = onnx_path;
  const std::string ext = ".onnx";
  if (trt_path.size() >= ext.size() && trt_path.compare(trt_path.size() - ext.size(), ext.size(), ext) == 0)
  {
    trt_path.replace(trt_path.size() - ext.size(), ext.size(), ".trt");
  }
  else
  {
    trt_path += ".trt";
  }

  std::ifstream trt_file(trt_path, std::ios::binary);
  if (!trt_file.good())
  {
    throw std::runtime_error("TRT engine file not found: " + trt_path);
  }
  trt_file.seekg(0, std::ios::end);
  size_t size = trt_file.tellg();
  trt_file.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  trt_file.read(data.data(), size);
  trt_file.close();

  auto* engine = runtime->deserializeCudaEngine(data.data(), size);
  if (!engine)
  {
    throw std::runtime_error("TRT: deserializeCudaEngine failed for " + trt_path);
  }
  return engine;
}
#endif  // USE_TENSORRT

void RL_imitation::initModel() {
  rbt_state =
      Robot_Imitation_State(nh_->get_logger(), params.num_actions, params.command_dim, params.imitation_anchor_dim);
  rbt_output = Robot_Imitation_Output(nh_->get_logger(), params.num_actions);
  obs = Observations_Imitation(params.num_single_obs, params.frame_stack_indices);

#ifdef USE_TENSORRT
  trt_logger_.ros_logger = nh_->get_logger();
#endif

  try
  {
#ifdef USE_TENSORRT
    // ---- Policy model (TRT) ----
    trt_runtime_policy_ = nvinfer1::createInferRuntime(trt_logger_);
    if (!trt_runtime_policy_) throw std::runtime_error("TRT: createInferRuntime failed (policy)");

    trt_engine_policy_ = loadTrtEngine(params.policy_path, trt_runtime_policy_);
    trt_context_policy_ = trt_engine_policy_->createExecutionContext();
    if (!trt_context_policy_) throw std::runtime_error("TRT: createExecutionContext failed (policy)");

    {
      const int nb_policy = static_cast<int>(trt_engine_policy_->getNbIOTensors());
      policy_input_binding_idx_ = -1;
      policy_command_binding_idx_ = -1;
      policy_output_binding_idx_ = -1;
      for (int i = 0; i < nb_policy; ++i)
      {
        const char* tensor_name = trt_engine_policy_->getIOTensorName(i);
        const bool is_input = trt_engine_policy_->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT;
        if (is_input)
        {
          const std::string name(tensor_name);
          if (params.native_dynamic_reference && (name == "command" || name == "command_short"))
            policy_command_binding_idx_ = i;
          else
            policy_input_binding_idx_ = i;
        }
        else
        {
          if (!policy_output_name_.empty())
          {
            std::string bname = tensor_name;
            std::transform(bname.begin(), bname.end(), bname.begin(), ::tolower);
            if (bname == policy_output_name_) policy_output_binding_idx_ = i;
          }
          if (policy_output_binding_idx_ == -1) policy_output_binding_idx_ = i;
        }
      }
      if (policy_input_binding_idx_ < 0 || policy_output_binding_idx_ < 0 ||
          (params.native_dynamic_reference && policy_command_binding_idx_ < 0))
        throw std::runtime_error("TRT: failed to resolve policy bindings");
    }
    {
      const size_t policy_in_sz = params.native_dynamic_reference
                                      ? 399U
                                      : static_cast<size_t>(params.num_single_obs) * params.frame_stack;
      const size_t policy_out_sz = static_cast<size_t>(params.num_actions);
      cudaError_t err = cudaMalloc(&policy_input_device_, policy_in_sz * sizeof(float));
      if (err != cudaSuccess)
        throw std::runtime_error(std::string("cudaMalloc policy input: ") + cudaGetErrorString(err));
      if (params.native_dynamic_reference)
      {
        err = cudaMalloc(&policy_command_device_, 240U * sizeof(float));
        if (err != cudaSuccess)
          throw std::runtime_error(std::string("cudaMalloc policy command: ") + cudaGetErrorString(err));
      }
      err = cudaMalloc(&policy_output_device_, policy_out_sz * sizeof(float));
      if (err != cudaSuccess)
        throw std::runtime_error(std::string("cudaMalloc policy output: ") + cudaGetErrorString(err));
      policy_output_host_.resize(policy_out_sz, 0.0f);
    }
    // ---- Imitation model (TRT); absent for native dynamic references ----
    if (!params.native_dynamic_reference)
    {
    trt_runtime_imitation_ = nvinfer1::createInferRuntime(trt_logger_);
    if (!trt_runtime_imitation_) throw std::runtime_error("TRT: createInferRuntime failed (imitation)");
    trt_engine_imitation_ = loadTrtEngine(params.imitation_path, trt_runtime_imitation_);
    trt_context_imitation_ = trt_engine_imitation_->createExecutionContext();
    if (!trt_context_imitation_) throw std::runtime_error("TRT: createExecutionContext failed (imitation)");

    {
      const int nb_imit = static_cast<int>(trt_engine_imitation_->getNbIOTensors());
      imitation_input_binding_idx_ = -1;
      cudaError_t err = cudaSuccess;
      for (int i = 0; i < nb_imit; ++i)
      {
        const char* tensor_name = trt_engine_imitation_->getIOTensorName(i);
        const std::string bname = std::string(tensor_name);
        const size_t num_elem = trtBindingElements(trt_engine_imitation_, i);
        const bool is_input = trt_engine_imitation_->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT;
        if (is_input)
        {
          imitation_input_binding_idx_ = i;
          err = cudaMalloc(&imitation_input_device_, num_elem * sizeof(float));
          if (err != cudaSuccess)
            throw std::runtime_error(std::string("cudaMalloc imitation input: ") + cudaGetErrorString(err));
        }
        else
        {
          ImitationOutputBuffer buf;
          buf.binding_idx = i;
          buf.host_buf.assign(num_elem, 0.0f);
          err = cudaMalloc(&buf.device_ptr, num_elem * sizeof(float));
          if (err != cudaSuccess)
            throw std::runtime_error(std::string("cudaMalloc imitation output '") + bname +
                                     "': " + cudaGetErrorString(err));
          imitation_outputs_[bname] = std::move(buf);
        }
      }
      if (imitation_input_binding_idx_ < 0) throw std::runtime_error("TRT: no input binding in imitation engine");
    }
    }
#elif defined(USE_OPENVINO)
    // ---- Policy model (OpenVINO) ----
    ov_compiled_policy_ = ov_core_.compile_model(params.policy_path, "CPU");
    ov_infer_policy_ = ov_compiled_policy_.create_infer_request();

    if (!params.native_dynamic_reference)
    {
      // ---- Imitation model (OpenVINO) ----
      ov_compiled_imitation_ = ov_core_.compile_model(params.imitation_path, "CPU");
      ov_infer_imitation_ = ov_compiled_imitation_.create_infer_request();

    // Warmup to resolve output shapes, then build imitation_outputs_ map
    ov_infer_imitation_.get_input_tensor().data<float>()[0] = 0.0f;
    ov_infer_imitation_.infer();
    for (const auto& out : ov_compiled_imitation_.outputs())
    {
      const std::string name = out.get_any_name();
      auto tensor = ov_infer_imitation_.get_tensor(name);
      ImitationOutputBuffer buf;
      buf.binding_idx = -1;
      buf.host_buf.assign(tensor.get_size(), 0.0f);
      buf.device_ptr = nullptr;
      imitation_outputs_[name] = std::move(buf);
    }
    }
#endif

    has_body_quat_output_ = imitation_outputs_.find("body_quat_w") != imitation_outputs_.end();

    // ---- Common: validate required outputs exist ----
    std::map<std::string, size_t> output_dims;
    if (!params.native_dynamic_reference)
    {
    for (const auto& cmd_item : params.command_items)
    {
      auto it = imitation_outputs_.find(cmd_item.type);
      if (it == imitation_outputs_.end())
      {
        RCLCPP_ERROR(nh_->get_logger(), "TRT: Imitation model missing required output '%s'", cmd_item.type.c_str());
        throw std::runtime_error("TRT: Motion model missing required output: " + cmd_item.type);
      }
      output_dims[cmd_item.type] = it->second.host_buf.size();
    }

    // Warmup inference (also validates model runs correctly)
    if (!runImitationInference(0.0f)) throw std::runtime_error("TRT: Warmup inference failed for imitation");

    motion_data_size_ = detectMaxTimeSteps(output_dims);

    // Pre-allocate and precompute all time steps
    precomputed_motion_data_.clear();
    precomputed_motion_data_.reserve(motion_data_size_);

    for (int t = 0; t < motion_data_size_; ++t)
    {
      if (!runImitationInference(static_cast<float>(t)))
      {
        RCLCPP_ERROR(nh_->get_logger(), "RL_imitation: TRT inference failed at timestep %d", t);
        throw std::runtime_error("TRT: imitation inference failed during precomputation");
      }

      precomputed_motion_data_.emplace_back();
      MotionData& motion_data = precomputed_motion_data_.back();
      motion_data.body_quat = Eigen::Quaterniond::Identity();
      if (has_body_quat_output_)
      {
        const float* body_quat_data = imitation_outputs_.at("body_quat_w").host_buf.data();
        motion_data.body_quat.w() = body_quat_data[0];
        motion_data.body_quat.x() = body_quat_data[1];
        motion_data.body_quat.y() = body_quat_data[2];
        motion_data.body_quat.z() = body_quat_data[3];
      }

      for (const auto& item_type_dim : output_dims)
      {
        const std::string& output_name = item_type_dim.first;
        size_t output_dim = item_type_dim.second;
        const float* output_data = imitation_outputs_.at(output_name).host_buf.data();

        if (output_name == "joint_pos")
        {
          motion_data.joint_pos.resize(output_dim);
          for (size_t i = 0; i < output_dim; ++i) motion_data.joint_pos(i) = output_data[i];
        }
        else if (output_name == "joint_vel")
        {
          motion_data.joint_vel.resize(output_dim);
          for (size_t i = 0; i < output_dim; ++i) motion_data.joint_vel(i) = output_data[i];
        }
        else if (output_name == "contact_mask")
        {
          motion_data.contact_mask.resize(output_dim);
          for (size_t i = 0; i < output_dim; ++i) motion_data.contact_mask(i) = output_data[i];
        }
        else if (output_name == "root_pos")
        {
          motion_data.root_pos.resize(output_dim);
          for (size_t i = 0; i < output_dim; ++i) motion_data.root_pos(i) = output_data[i];
        }
        else if (output_name == "root_quat")
        {
          motion_data.root_quat.resize(output_dim);
          for (size_t i = 0; i < output_dim; ++i) motion_data.root_quat(i) = output_data[i];
        }
        else if (output_name == "joint_pos_policy")
        {
          motion_data.joint_pos_policy.resize(output_dim);
          for (size_t i = 0; i < output_dim; ++i) motion_data.joint_pos_policy(i) = output_data[i];
        }
      }
    }
    }

    if (params.native_dynamic_reference)
    {
      native_policy_input_.assign(399, 0.0f);
      native_command_input_.assign(240, 0.0f);
      native_state_history_.clear();
      imitation_model_enabled_ = false;
      motion_data_size_ = 0;
      RCLCPP_INFO(nh_->get_logger(),
                  "RL_imitation: native dynamic reference ABI enabled (policy[399] + command[240])");
    }

  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to initialize model: %s", e.what());
    throw;
  }
}

bool RL_imitation::runImitationInference(float timestep) {
#ifdef USE_TENSORRT
  cudaError_t err = cudaMemcpy(imitation_input_device_, &timestep, sizeof(float), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    RCLCPP_ERROR(nh_->get_logger(), "TRT imitation: H2D failed: %s", cudaGetErrorString(err));
    return false;
  }
  {
    const char* input_name = trt_engine_imitation_->getIOTensorName(imitation_input_binding_idx_);
    if (!trt_context_imitation_->setTensorAddress(input_name, imitation_input_device_))
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT imitation: setTensorAddress failed for input");
      return false;
    }
    for (const auto& kv : imitation_outputs_)
    {
      if (!trt_context_imitation_->setTensorAddress(kv.first.c_str(), kv.second.device_ptr))
      {
        RCLCPP_ERROR(nh_->get_logger(), "TRT imitation: setTensorAddress failed for '%s'", kv.first.c_str());
        return false;
      }
    }
    if (!trt_context_imitation_->enqueueV3(0))
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT imitation: enqueueV3 failed");
      return false;
    }
  }
  for (auto& kv : imitation_outputs_)
  {
    err = cudaMemcpy(kv.second.host_buf.data(), kv.second.device_ptr, kv.second.host_buf.size() * sizeof(float),
                     cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT imitation: D2H failed '%s': %s", kv.first.c_str(), cudaGetErrorString(err));
      return false;
    }
  }

#elif defined(USE_OPENVINO)
  ov_infer_imitation_.get_input_tensor().data<float>()[0] = timestep;
  ov_infer_imitation_.infer();
  for (auto& kv : imitation_outputs_)
  {
    auto tensor = ov_infer_imitation_.get_tensor(kv.first);
    const float* src = tensor.data<const float>();
    std::copy(src, src + kv.second.host_buf.size(), kv.second.host_buf.begin());
  }
#endif
  return true;
}

int RL_imitation::detectMaxTimeSteps(const std::map<std::string, size_t>& output_dims) {
  const int MAX_DETECTION_STEPS = 100000;  // Safety limit
  const int STABILITY_FRAMES = 5000;       // Number of consecutive identical frames to detect
  const double EPSILON = 1e-6;             // Threshold for comparing floating point values

  std::vector<MotionData> recent_frames(STABILITY_FRAMES);
  int frame_count = 0;  // Total frames processed

  for (int t = 0; t < MAX_DETECTION_STEPS; ++t)
  {
    if (!runImitationInference(static_cast<float>(t)))
    {
      RCLCPP_WARN(nh_->get_logger(),
                  "RL_imitation: TRT inference failed at timestep %d. Using %d "
                  "as duration.",
                  t, t);
      return t;
    }

    // Use circular buffer index
    int buffer_idx = frame_count % STABILITY_FRAMES;
    MotionData& current_frame = recent_frames.at(buffer_idx);
    current_frame.body_quat = Eigen::Quaterniond::Identity();
    if (has_body_quat_output_)
    {
      const float* body_quat_data = imitation_outputs_.at("body_quat_w").host_buf.data();
      current_frame.body_quat.w() = body_quat_data[0];
      current_frame.body_quat.x() = body_quat_data[1];
      current_frame.body_quat.y() = body_quat_data[2];
      current_frame.body_quat.z() = body_quat_data[3];
    }

    // Get only the outputs specified in command_config
    for (const auto& item_type_dim : output_dims)
    {
      const std::string& output_name = item_type_dim.first;
      size_t output_dim = item_type_dim.second;
      const float* output_data = imitation_outputs_.at(output_name).host_buf.data();

      if (output_name == "joint_pos")
      {
        current_frame.joint_pos.resize(output_dim);
        for (size_t i = 0; i < output_dim; ++i) current_frame.joint_pos(i) = output_data[i];
      }
      else if (output_name == "joint_vel")
      {
        current_frame.joint_vel.resize(output_dim);
        for (size_t i = 0; i < output_dim; ++i) current_frame.joint_vel(i) = output_data[i];
      }
      else if (output_name == "contact_mask")
      {
        current_frame.contact_mask.resize(output_dim);
        for (size_t i = 0; i < output_dim; ++i) current_frame.contact_mask(i) = output_data[i];
      }
      else if (output_name == "root_pos")
      {
        current_frame.root_pos.resize(output_dim);
        for (size_t i = 0; i < output_dim; ++i) current_frame.root_pos(i) = output_data[i];
      }
      else if (output_name == "root_quat")
      {
        current_frame.root_quat.resize(output_dim);
        for (size_t i = 0; i < output_dim; ++i) current_frame.root_quat(i) = output_data[i];
      }
      else if (output_name == "joint_pos_policy")
      {
        current_frame.joint_pos_policy.resize(output_dim);
        for (size_t i = 0; i < output_dim; ++i) current_frame.joint_pos_policy(i) = output_data[i];
      }
    }
    frame_count++;

    // Check if we have enough frames and if they are all identical
    if (frame_count >= STABILITY_FRAMES)
    {
      bool all_identical = true;

      // Get reference frame (oldest in buffer)
      int ref_idx = (frame_count - STABILITY_FRAMES) % STABILITY_FRAMES;
      const MotionData& reference = recent_frames.at(ref_idx);

      for (int i = 1; i < STABILITY_FRAMES; ++i)
      {
        int frame_idx = (frame_count - STABILITY_FRAMES + i) % STABILITY_FRAMES;
        const MotionData& frame = recent_frames.at(frame_idx);

        // Check quaternion (using quaternion distance)
        if (has_body_quat_output_)
        {
          double quat_diff = std::abs(reference.body_quat.w() - frame.body_quat.w()) +
                             std::abs(reference.body_quat.x() - frame.body_quat.x()) +
                             std::abs(reference.body_quat.y() - frame.body_quat.y()) +
                             std::abs(reference.body_quat.z() - frame.body_quat.z());
          if (quat_diff > EPSILON)
          {
            all_identical = false;
            break;
          }
        }

        // Check only the outputs that exist based on command_config
        if (reference.joint_pos.size() > 0 && (reference.joint_pos - frame.joint_pos).norm() > EPSILON)
        {
          all_identical = false;
          break;
        }

        if (reference.joint_vel.size() > 0 && (reference.joint_vel - frame.joint_vel).norm() > EPSILON)
        {
          all_identical = false;
          break;
        }

        if (reference.contact_mask.size() > 0 && (reference.contact_mask - frame.contact_mask).norm() > EPSILON)
        {
          all_identical = false;
          break;
        }

        if (reference.root_pos.size() > 0 && (reference.root_pos - frame.root_pos).norm() > EPSILON)
        {
          all_identical = false;
          break;
        }

        if (reference.root_quat.size() > 0 && (reference.root_quat - frame.root_quat).norm() > EPSILON)
        {
          all_identical = false;
          break;
        }

        if (reference.joint_pos_policy.size() > 0 &&
            (reference.joint_pos_policy - frame.joint_pos_policy).norm() > EPSILON)
        {
          all_identical = false;
          break;
        }
      }

      if (all_identical)
      {
        int stable_start = t - STABILITY_FRAMES + 1;
        int duration = stable_start + 1;  // Include stable start frame
        return duration;
      }
    }
  }

  RCLCPP_WARN(nh_->get_logger(),
              "RL_imitation: No stability detected within %d steps. Using %d "
              "as duration.",
              MAX_DETECTION_STEPS, MAX_DETECTION_STEPS);
  return MAX_DETECTION_STEPS;
}

void RL_imitation::initData() {
  lowState_.motor_state.resize(params.num_actions);
  posCmd_.q.resize(params.num_actions);

  obs_manager_ = std::make_unique<ObservationManager_Imitation>(nh_->get_logger(), params.observation_items, params);

  // Check observation manager dimension
  int total_obs_dim = obs_manager_->getTotalDimension();
  if (total_obs_dim != params.num_single_obs)
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "RL_imitation: observation dimension mismatch! Detected: %d, "
                 "Expected (num_single_obs): %d",
                 total_obs_dim, params.num_single_obs);
    throw std::runtime_error("Observation dimension mismatch in configuration");
  }
}

bool RL_imitation::getParam() {
  // for ros2
  // try 获取rl_config_path变量，catch 无文件，文件无法解析返回false
  std::string rl_config_path_ = "";
  try
  { /* code */
  } catch (const std::exception& e)
  { std::cerr << e.what() << '\n'; }

  try
  {
    // read common parameters first
    if (!readCommonParams(static_cast<RLBaseModelParams&>(params), params.observation_items))
    {
      throw std::runtime_error("Failed to read common parameters");
    }

    // read deployment configuration parameters
    if (!readDeployParams(deploy_params))
    {
      throw std::runtime_error("Failed to read deployment parameters");
    }
    nh_->get_parameter_or("policy_root_path", policy_root_path_, std::string(LEGGED_POLICY_PATH));

    // v3-specific params
    int error = 0;
    // error += static_cast<int>(!nh.getParam("/RobotCfg/motion_name",
    // params.imitation_name)); error +=
    // static_cast<int>(!nh.getParam("/RobotCfg/joint_order_mj",
    // params.joint_order_mj)); error +=
    // static_cast<int>(!nh.getParam("/RobotCfg/joint_order_isaac",
    // params.joint_order_isaac)); error +=
    // static_cast<int>(!nh.getParam("/RobotCfg/action_scale",
    // params.action_scale)); for ros2 read yaml
    std::string robot_cfg_path("");
    if (!nh_->get_parameter("robot_cfg_path", robot_cfg_path))
    {
      throw std::runtime_error("Required parameter 'robot_cfg_path' not found");
    }
    YAML::Node root = YAML::LoadFile(robot_cfg_path);
    YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");

    params.native_dynamic_reference = rc["native_dynamic_reference"].as<bool>(false);
    params.imitation_name = rc["motion_name"].as<std::string>("");
    params.joint_order_mj = asStringList(rc["joint_order_mj"], "RobotCfg.joint_order_mj");
    params.joint_order_isaac = asStringList(rc["joint_order_isaac"], "RobotCfg.joint_order_isaac");
    params.action_scale = asDoubleList(rc["action_scale"], "RobotCfg.action_scale");

    // Extract command_dim and imitation_anchor_dim from observation_items
    params.command_dim = 0;
    params.imitation_anchor_dim = 0;
    for (const auto& item : params.observation_items)
    {
      if (item.type == "command")
      {
        params.command_dim = item.dim;
      }
      else if (item.type == "anchor_ori")
      {
        params.imitation_anchor_dim = item.dim;
      }
    }

    if (params.native_dynamic_reference)
    {
      params.command_dim = 240;
      params.imitation_anchor_dim = 0;
    }
    else if (params.command_dim == 0)
    {
      RCLCPP_ERROR(nh_->get_logger(), "Failed to extract command_dim (%d) from observation_config", params.command_dim);
      throw std::runtime_error("Missing command in observation_config");
    }

    // Read command_config
    // XmlRpc::XmlRpcValue command_config;
    // if (!nh.getParam("/RobotCfg/command_config/items", command_config)) {
    //    RCLCPP_ERROR(nh_->get_logger(), "RL_imitation: command_config/items is
    //    required but not found in configuration"); throw
    //    std::runtime_error("Missing required command_config in rl_imitation
    //    configuration");
    //}

    // params.command_items.clear();
    // for (int i = 0; i < command_config.size(); ++i) {
    //     std::string type = static_cast<std::string>(command_config[i].type);
    //     int dim = static_cast<int>(command_config[i].dim);
    //     params.command_items.emplace_back(type, dim);
    // }
    // RCLCPP_INFO(nh_->get_logger(), "RL_imitation: Loaded %zu command items
    // from command_config", params.command_items.size());

    params.command_indices.clear();
    if (rc["command_config"] && rc["command_config"]["index"])
    {
      const YAML::Node index_node = rc["command_config"]["index"];
      if (!index_node.IsSequence() || index_node.size() == 0)
      {
        throw std::runtime_error("Invalid RobotCfg.command_config.index");
      }
      for (const auto& index_item : index_node)
      {
        const int index = index_item.as<int>();
        if (index < 0 && !params.native_dynamic_reference)
        {
          throw std::runtime_error("RobotCfg.command_config.index only supports non-negative indices");
        }
        params.command_indices.push_back(index);
      }
    }
    if (params.command_indices.empty())
    {
      params.command_indices.push_back(0);
    }

    if (rc["command_config"] && rc["command_config"]["items"])
    {
      params.command_items.clear();
      for (const auto& item : rc["command_config"]["items"])
      {
        std::string item_type = item["type"].as<std::string>();
        int item_dim = item["dim"].as<int>();
        params.command_items.emplace_back(item_type, item_dim);
      }
    }
    else
    {
      return false;
    }

    int command_dim_per_index = 0;
    for (const auto& item : params.command_items)
    {
      command_dim_per_index += item.dim;
    }
    const int total_dim = command_dim_per_index * static_cast<int>(params.command_indices.size());
    if (total_dim != params.command_dim)
    {
      RCLCPP_ERROR(nh_->get_logger(),
                   "Command config dimension mismatch: command_items total per index=%d, index_count=%zu, total=%d, "
                   "command_dim=%d",
                   command_dim_per_index, params.command_indices.size(), total_dim, params.command_dim);
      throw std::runtime_error("Command config dimension mismatch");
    }

    // build policy/motion paths
    params.policy_path = policy_root_path_ + std::string("/") + params.robot_type + std::string("/") +
                         params.subpolicy + std::string("/") + params.policy_name + std::string(".onnx");
    params.imitation_path = policy_root_path_ + std::string("/") + params.robot_type + std::string("/") +
                            params.subpolicy + std::string("/") + params.imitation_name + std::string(".onnx");

    if (error > 0)
    {
      RCLCPP_ERROR(nh_->get_logger(), "Failed to get RL_imitation-specific parameters");
      throw std::runtime_error("Failed to get RL_imitation-specific parameters");
    }
    return true;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Error in the config: %s", e.what());
    return false;
  }
}

void RL_imitation::refreshInferenceEnabledLocked() {
  const bool enabled = lowState_.rl_inference_signal && imitation_active_ && model_loaded_.load();
  rl_inference_enabled_.store(enabled);
}

bool RL_imitation::loadPolicyIntoCurrentLocked(const std::string& policy_name, std::string& robot_cfg_path,
                                               std::string& message) {
  const std::string requested_policy = policy_name;
  const std::string robot_type = !robot_type_.empty() ? robot_type_ : params.robot_type;
  if (robot_type.empty())
  {
    message = "empty robot_type";
    return false;
  }

  robot_cfg_path = policy_root_path_ + "/" + robot_type + "/" + requested_policy + "/rl_config.yaml";
  {
    std::ifstream f(robot_cfg_path);
    if (!f.good())
    {
      message = "robot_cfg_path not found: " + robot_cfg_path;
      return false;
    }
  }

  releaseModelResourcesLocked();
  params = RL_IMITATION_ModelParams();
  params.robot_type = robot_type;
  robot_type_ = robot_type;
  imitation_active_ = false;
  refreshInferenceEnabledLocked();

  try
  {
    nh_->set_parameter(rclcpp::Parameter("subpolicy", requested_policy));
    nh_->set_parameter(rclcpp::Parameter("robot_cfg_path", robot_cfg_path));

    if (!getParam())
    {
      throw std::runtime_error("Failed to read parameters for policy '" + requested_policy + "'");
    }

    YAML::Node root = YAML::LoadFile(robot_cfg_path);
    YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");
    policy_output_name_ = rc["policy_output_name"].as<std::string>("");
    std::transform(policy_output_name_.begin(), policy_output_name_.end(), policy_output_name_.begin(), ::tolower);
    pub_imitationref_joint_states_ = rc["pub_imitationref_joint_states"].as<bool>(false);

    order_mapper_.initialize(params.joint_order_mj, params.joint_order_isaac);
    initData();
    initModel();
    resetImitationState();

    current_policy_name_ = requested_policy;
    current_robot_cfg_path_ = robot_cfg_path;
    model_loaded_.store(true);
    loop_frequency_hz_.store(std::max(1, params.policy_frequency));
    refreshInferenceEnabledLocked();

    message = "loaded";
    return true;
  } catch (const std::exception& e)
  {
    releaseModelResourcesLocked();
    params.robot_type = robot_type;
    robot_type_ = robot_type;
    message = e.what();
    return false;
  }
}

void RL_imitation::preloadPoliciesLocked(const std::vector<std::string>& policies) {
  if (policies.empty()) return;

  for (const auto& policy : policies)
  {
    if (policy.empty()) continue;
    if (cached_policies_.find(policy) != cached_policies_.end()) continue;

    RCLCPP_INFO(nh_->get_logger(), "RL_imitation: loading policy '%s'", policy.c_str());
    std::string robot_cfg_path;
    std::string message;
    if (!loadPolicyIntoCurrentLocked(policy, robot_cfg_path, message))
    {
      RCLCPP_ERROR(nh_->get_logger(), "RL_imitation: policy '%s' load FAILED: %s", policy.c_str(), message.c_str());
      throw std::runtime_error("Failed to preload imitation policy '" + policy + "': " + message);
    }

    RCLCPP_INFO(nh_->get_logger(), "RL_imitation: policy '%s' loaded OK", policy.c_str());
    storeCurrentRuntimeLocked();
  }

  imitation_active_ = false;
  refreshInferenceEnabledLocked();
}

bool RL_imitation::loadPolicyLocked(const std::string& policy_name, std::string& robot_cfg_path, std::string& message) {
  const std::string requested_policy = policy_name;
  if (requested_policy.empty())
  {
    message = "empty policy_name";
    RCLCPP_ERROR(nh_->get_logger(), "RL_imitation: policy '<empty>' load FAILED: %s", message.c_str());
    return false;
  }
  if (!available_imitation_policies_.empty() &&
      std::find(available_imitation_policies_.begin(), available_imitation_policies_.end(), requested_policy) ==
          available_imitation_policies_.end())
  {
    message = "policy is not listed in available_imitation_policies";
    RCLCPP_ERROR(nh_->get_logger(), "RL_imitation: policy '%s' load FAILED: %s", requested_policy.c_str(),
                 message.c_str());
    return false;
  }

  const bool force_text_motion_reload = text_motion_external_packager_ &&
      requested_policy.rfind(text_motion_policy_ + "_", 0) == 0;
  if (model_loaded_.load() && requested_policy == current_policy_name_ && !force_text_motion_reload)
  {
    robot_cfg_path = current_robot_cfg_path_.empty()
                         ? policy_root_path_ + "/" + params.robot_type + "/" + requested_policy + "/rl_config.yaml"
                         : current_robot_cfg_path_;
    message = "policy already loaded";
    RCLCPP_INFO(nh_->get_logger(), "RL_imitation: policy '%s' loaded OK (already active)", requested_policy.c_str());
    return true;
  }

  RCLCPP_INFO(nh_->get_logger(), "RL_imitation: loading policy '%s'", requested_policy.c_str());
  auto cached = cached_policies_.find(requested_policy);
  if (cached != cached_policies_.end())
  {
    storeCurrentRuntimeLocked();
    CachedPolicyRuntime runtime = std::move(cached->second);
    cached_policies_.erase(cached);
    robot_cfg_path = runtime.robot_cfg_path;
    restoreRuntimeLocked(std::move(runtime));
    resetImitationState();
    refreshInferenceEnabledLocked();
    message = "policy restored from preload cache";
    RCLCPP_INFO(nh_->get_logger(), "RL_imitation: policy '%s' loaded OK (cached)", current_policy_name_.c_str());
    return true;
  }

  storeCurrentRuntimeLocked();
  const bool ok = loadPolicyIntoCurrentLocked(requested_policy, robot_cfg_path, message);
  if (ok)
  {
    RCLCPP_INFO(nh_->get_logger(), "RL_imitation: policy '%s' loaded OK", requested_policy.c_str());
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "RL_imitation: policy '%s' load FAILED: %s", requested_policy.c_str(),
                 message.c_str());
  }
  return ok;
}

void RL_imitation::switchImitationPolicyCallback(
    const std::shared_ptr<legged_msgs::srv::SwitchImitationPolicy::Request> request,
    std::shared_ptr<legged_msgs::srv::SwitchImitationPolicy::Response> response) {
  if (!dynamic_policy_switch_enabled_)
  {
    response->success = false;
    response->message = "dynamic_policy_switch_enabled is false";
    response->policy_name = current_policy_name_;
    response->robot_cfg_path = "";
    return;
  }

  std::lock_guard<std::mutex> lock(model_mutex_);
  imitation_active_ = false;
  refreshInferenceEnabledLocked();

  std::string robot_cfg_path;
  std::string message;
  const bool ok = loadPolicyLocked(request->policy_name, robot_cfg_path, message);
  response->success = ok;
  response->message = message;
  response->policy_name = ok ? request->policy_name : current_policy_name_;
  response->robot_cfg_path = ok ? robot_cfg_path : "";
}

void RL_imitation::imitationResetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (msg->data)
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    if (!model_loaded_.load()) return;
    resetImitationState();
  }
}

void RL_imitation::imitationActiveCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(model_mutex_);
  imitation_active_ = msg->data;
  refreshInferenceEnabledLocked();
}

void RL_imitation::resetImitationState() {
  // 重置开始时间，这样时间步计数器会从0重新开始
  imitation_start_time = nh_->get_clock()->now();
  time_step_counter_ = 0;
  initial_quats_logged_ = false;
  initial_yaw_ = 0.0;
  initial_yaw_valid_ = false;
  motion_done_sent_ = false;
  resetObservationState();
}

void RL_imitation::resetObservationState() {
  obs = Observations_Imitation(params.num_single_obs, params.frame_stack_indices);
  native_state_history_.clear();
  if (params.native_dynamic_reference)
  {
    native_policy_input_.assign(399, 0.0f);
    native_command_input_.assign(240, 0.0f);
  }

  if (rbt_output.target_q.size() > 0)
  {
    rbt_output.target_q.setZero();
  }
  if (rbt_output.action.size() > 0)
  {
    rbt_output.action.setZero();
  }
}

void RL_imitation::stateCallback(const legged_msgs::msg::LowState::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(model_mutex_);
  lowState_.rl_inference_signal = msg->rl_inference_signal;
  if (!model_loaded_.load())
  {
    refreshInferenceEnabledLocked();
    return;
  }

  // copy body_xpos and body_xquat
  for (size_t i = 0; i < 3; i++)
  {
    lowState_.body_xpos[i] = msg->body_xpos[i];
  }
  for (size_t i = 0; i < 4; i++)
  {
    lowState_.body_xquat[i] = msg->body_xquat[i];
  }

  const size_t n = std::min(static_cast<size_t>(params.num_actions), msg->motor_state.size());
  for (size_t i = 0; i < n; ++i)
  {
    lowState_.motor_state[i] = std::move(msg->motor_state[i]);
  }
  lowState_.imu = std::move(msg->imu);
  lowState_.cmd_vel = std::move(msg->cmd_vel);
  lowState_.shifted_time = std::move(msg->shifted_time);

  refreshInferenceEnabledLocked();
}

Eigen::Quaterniond RL_imitation::get_imitation_anchor_ori_b_cpp(const Eigen::Quaterniond& robot_initial_quat,
                                                                const Eigen::Quaterniond& q_robot,
                                                                const Eigen::Quaterniond& imitation_initial_quat,
                                                                const Eigen::Quaterniond& q_imitation) {
  auto yaw_from_quat = [](const Eigen::Quaterniond& q) -> double {
    const double w = q.w();
    const double x = q.x();
    const double y = q.y();
    const double z = q.z();
    const double t3 = 2.0 * (w * z + x * y);
    const double t4 = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(t3, t4);
  };

  auto quat_from_yaw_eigen = [](double yaw) -> Eigen::Quaterniond {
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    // Eigen::Quaterniond ctor is (w, x, y, z)
    return Eigen::Quaterniond(cy, 0.0, 0.0, sy);
  };

  const double robot_yaw = yaw_from_quat(robot_initial_quat);

  const double imitation_yaw = yaw_from_quat(imitation_initial_quat);

  const Eigen::Quaterniond robot_yaw_quat_init = quat_from_yaw_eigen(robot_yaw);
  const Eigen::Quaterniond imitation_yaw_quat_init = quat_from_yaw_eigen(imitation_yaw);

  // 去除各自的 yaw 分量
  const Eigen::Quaterniond robot_quat_no_yaw = robot_yaw_quat_init.conjugate() * q_robot;
  const Eigen::Quaterniond imitation_quat_no_yaw = imitation_yaw_quat_init.conjugate() * q_imitation;

  // 相对姿态（与原实现等价：conj(robot_no_yaw) ⊗ motion_no_yaw）
  Eigen::Quaterniond res = robot_quat_no_yaw.conjugate() * imitation_quat_no_yaw;
  res.normalize();
  return res;
}

void RL_imitation::updateImitationData() {
  if (!precomputed_motion_data_.empty())
  {
    int current_timestep_index = time_step_counter_;
    if (current_timestep_index >= motion_data_size_ && motion_data_size_ > 0)
    {
      current_timestep_index = motion_data_size_ - 1;
    }

    const MotionData& current_motion_data = precomputed_motion_data_[current_timestep_index];

    // Copy precomputed data to rbt_state
    rbt_state.command_quat = has_body_quat_output_ ? current_motion_data.body_quat : Eigen::Quaterniond::Identity();

    if (pub_imitationref_joint_states_)
    {  // publish imitation joint states
      sensor_msgs::msg::JointState imitation_joint_states_msg;
      // imitation_joint_states_msg.header.stamp = ros::Time::now();
      imitation_joint_states_msg.header.stamp = rclcpp::Clock().now();
      imitation_joint_states_msg.name.resize(current_motion_data.joint_pos.size());
      imitation_joint_states_msg.position.resize(current_motion_data.joint_pos.size());
      imitation_joint_states_msg.velocity.resize(current_motion_data.joint_vel.size());
      for (Eigen::Index i = 0; i < current_motion_data.joint_pos.size(); ++i)
      {
        const size_t idx = static_cast<size_t>(i);
        imitation_joint_states_msg.name[idx] = params.joint_order_isaac[idx];
        imitation_joint_states_msg.position[idx] = current_motion_data.joint_pos(i);
        imitation_joint_states_msg.velocity[idx] = current_motion_data.joint_vel(i);
      }
      pubImitationJointStates_->publish(imitation_joint_states_msg);
    }

    int current_idx = 0;
    for (int relative_index : params.command_indices)
    {
      int timestep_index = time_step_counter_ + relative_index;
      if (timestep_index >= motion_data_size_ && motion_data_size_ > 0)
      {
        timestep_index = motion_data_size_ - 1;
      }

      const MotionData& motion_data = precomputed_motion_data_[timestep_index];
      for (const auto& cmd_item : params.command_items)
      {
        if (cmd_item.type == "joint_pos")
        {
          int copy_dim = std::min(cmd_item.dim, static_cast<int>(motion_data.joint_pos.size()));
          for (int i = 0; i < copy_dim; ++i)
          {
            rbt_state.command(current_idx + i) = motion_data.joint_pos(i);
          }
          for (int i = copy_dim; i < cmd_item.dim; ++i)
          {
            rbt_state.command(current_idx + i) = 0.0;
          }
        }
        else if (cmd_item.type == "joint_vel")
        {
          int copy_dim = std::min(cmd_item.dim, static_cast<int>(motion_data.joint_vel.size()));
          for (int i = 0; i < copy_dim; ++i)
          {
            rbt_state.command(current_idx + i) = motion_data.joint_vel(i);
          }
          for (int i = copy_dim; i < cmd_item.dim; ++i)
          {
            rbt_state.command(current_idx + i) = 0.0;
          }
        }
        else if (cmd_item.type == "contact_mask")
        {
          int copy_dim = std::min(cmd_item.dim, static_cast<int>(motion_data.contact_mask.size()));
          for (int i = 0; i < copy_dim; ++i)
          {
            rbt_state.command(current_idx + i) = motion_data.contact_mask(i);
          }
          for (int i = copy_dim; i < cmd_item.dim; ++i)
          {
            rbt_state.command(current_idx + i) = 0.0;
          }
        }
        else if (cmd_item.type == "root_pos")
        {
          int copy_dim = std::min(cmd_item.dim, static_cast<int>(motion_data.root_pos.size()));
          for (int i = 0; i < copy_dim; ++i)
          {
            rbt_state.command(current_idx + i) = motion_data.root_pos(i);
          }
          for (int i = copy_dim; i < cmd_item.dim; ++i)
          {
            rbt_state.command(current_idx + i) = 0.0;
          }
        }
        else if (cmd_item.type == "root_quat")
        {
          int copy_dim = std::min(cmd_item.dim, static_cast<int>(motion_data.root_quat.size()));
          for (int i = 0; i < copy_dim; ++i)
          {
            rbt_state.command(current_idx + i) = motion_data.root_quat(i);
          }
          for (int i = copy_dim; i < cmd_item.dim; ++i)
          {
            rbt_state.command(current_idx + i) = 0.0;
          }
        }
        else if (cmd_item.type == "joint_pos_policy")
        {
          int copy_dim = std::min(cmd_item.dim, static_cast<int>(motion_data.joint_pos_policy.size()));
          for (int i = 0; i < copy_dim; ++i)
          {
            rbt_state.command(current_idx + i) = motion_data.joint_pos_policy(i);
          }
          for (int i = copy_dim; i < cmd_item.dim; ++i)
          {
            rbt_state.command(current_idx + i) = 0.0;
          }
        }
        else
        {
          throw std::runtime_error("Unknown command item type");
        }
        current_idx += cmd_item.dim;
      }
    }

    if (current_idx != rbt_state.command.size())
    {
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000 /* ms */,
                            "Command dimension mismatch: configured items total %d, expected %d", current_idx,
                            static_cast<int>(rbt_state.command.size()));
    }
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "RL_imitation: No precomputed motion data available.");
  }
}

void RL_imitation::set_anchor_ori(const Eigen::Quaterniond& robot_initial_quat, const Eigen::Quaterniond& q_robot,
                                  const Eigen::Quaterniond& imitation_initial_quat,
                                  const Eigen::Quaterniond& q_imitation) {
  if (rbt_state.anchor_ori.size() < 6)
  {
    if (rbt_state.anchor_ori.size() > 0)
    {
      rbt_state.anchor_ori.setZero();
    }
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000 /* ms */,
                         "RL_imitation: anchor_ori requires at least 6 values, but configured dim is %ld",
                         rbt_state.anchor_ori.size());
    return;
  }

  auto imitation_anchor_ori_b =
      get_imitation_anchor_ori_b_cpp(robot_initial_quat, q_robot, imitation_initial_quat, q_imitation);
  const Eigen::Matrix3d imitation_anchor_ori_b_mat = imitation_anchor_ori_b.toRotationMatrix();
  rbt_state.anchor_ori(0) = imitation_anchor_ori_b_mat(0, 0);
  rbt_state.anchor_ori(1) = imitation_anchor_ori_b_mat(0, 1);
  rbt_state.anchor_ori(2) = imitation_anchor_ori_b_mat(1, 0);
  rbt_state.anchor_ori(3) = imitation_anchor_ori_b_mat(1, 1);
  rbt_state.anchor_ori(4) = imitation_anchor_ori_b_mat(2, 0);
  rbt_state.anchor_ori(5) = imitation_anchor_ori_b_mat(2, 1);
}

void RL_imitation::updateObservation() {
  // ROS_INFO_THROTTLE(1.0, "Updating observation at time step %d",
  // time_step_counter_);
  try
  {
    // clean rbt_state
    rbt_state.clean();

    rbt_state.body_xpos(0) = lowState_.body_xpos[0];
    rbt_state.body_xpos(1) = lowState_.body_xpos[1];
    rbt_state.body_xpos(2) = lowState_.body_xpos[2];

    // LowState.body_xquat follows LeggedHardwareInterface::getBodyOrientation(): [x, y, z, w]
    // (same as imu.quaternion in LeggedController). Eigen::Quaterniond is (w, x, y, z).
    rbt_state.body_xquat.w() = lowState_.body_xquat[3];
    rbt_state.body_xquat.x() = lowState_.body_xquat[0];
    rbt_state.body_xquat.y() = lowState_.body_xquat[1];
    rbt_state.body_xquat.z() = lowState_.body_xquat[2];

    // Legacy policies read a precomputed motion model here. Native dynamic
    // policies consume the C++ trajectory directly below.
    if (params.native_dynamic_reference)
    {
      if (precomputed_motion_data_.empty()) throw std::runtime_error("dynamic reference trajectory is empty");
      const int index = std::clamp(time_step_counter_, 0, static_cast<int>(precomputed_motion_data_.size()) - 1);
      rbt_state.command_quat = precomputed_motion_data_[static_cast<size_t>(index)].body_quat;
    }
    else
    {
      updateImitationData();
    }

    if (time_step_counter_ == 0 && !initial_quats_logged_)
    {
      // on the first time step, store the robot initial quat and imitation
      // initial quat
      robot_initial_quat = rbt_state.body_xquat;
      imitation_initial_quat = rbt_state.command_quat;
      initial_quats_logged_ = true;
    }

    // Build MJ-ordered vectors from hardware state
    Eigen::VectorXd mj_q(params.num_actions);
    Eigen::VectorXd mj_dq(params.num_actions);

    for (int i = 0; i < params.num_actions; ++i)
    {
      mj_q[i] = lowState_.motor_state[i].q - params.default_joint_angles[i];
      mj_dq[i] = lowState_.motor_state[i].dq;
    }

    // Transform to Isaac ordering
    order_mapper_.transformMjToIsaac(mj_q, rbt_state.q);
    order_mapper_.transformMjToIsaac(mj_dq, rbt_state.dq);

    for (size_t i = 0; i < 3; ++i)
    {
      rbt_state.base_ang_vel(i) = lowState_.imu.gyroscope[i];
    }
    rbt_state.quat.x() = lowState_.imu.quaternion[0];
    rbt_state.quat.y() = lowState_.imu.quaternion[1];
    rbt_state.quat.z() = lowState_.imu.quaternion[2];
    rbt_state.quat.w() = lowState_.imu.quaternion[3];
    rbt_state.quat.normalize();

    auto yaw_from_quat = [](const Eigen::Quaterniond& q) -> double {
      const double w = q.w();
      const double x = q.x();
      const double y = q.y();
      const double z = q.z();
      const double t3 = 2.0 * (w * z + x * y);
      const double t4 = 1.0 - 2.0 * (y * y + z * z);
      return std::atan2(t3, t4);
    };
    auto quat_from_yaw = [](double yaw) -> Eigen::Quaterniond {
      const double cy = std::cos(yaw * 0.5);
      const double sy = std::sin(yaw * 0.5);
      return Eigen::Quaterniond(cy, 0.0, 0.0, sy);
    };

    const double current_yaw = yaw_from_quat(rbt_state.quat);
    if (!initial_yaw_valid_)
    {
      initial_yaw_ = current_yaw;
      initial_yaw_valid_ = true;
    }
    const Eigen::Quaterniond initial_yaw_quat_inv = quat_from_yaw(initial_yaw_).conjugate();
    rbt_state.quat_obs = initial_yaw_quat_inv * rbt_state.quat;
    rbt_state.quat_obs.normalize();

    if (params.imitation_anchor_dim > 0 && has_body_quat_output_)
    {
      set_anchor_ori(robot_initial_quat, rbt_state.quat, imitation_initial_quat, rbt_state.command_quat);
    }
    else if (params.imitation_anchor_dim > 0)
    {
      rbt_state.anchor_ori.setZero();
    }

    // {
    //     // print rbt_state for debug
    //     ("time_step_counter_: %d", time_step_counter_);
    //     obs_manager_->showInfoInOrder(rbt_state, rbt_output);
    // }
    if (params.native_dynamic_reference)
    {
      // One state frame: angular velocity, projected gravity, joint position,
      // joint velocity and previous action. Training uses histories grouped by
      // signal (not seven concatenated 69-D frames).
      Eigen::VectorXd state_frame(69);
      state_frame.segment<3>(0) = rbt_state.base_ang_vel;
      Eigen::Vector4d quat_xyzw;
      quat_xyzw << rbt_state.quat.x(), rbt_state.quat.y(), rbt_state.quat.z(), rbt_state.quat.w();
      state_frame.segment<3>(3) = qr_ty_i(quat_xyzw, Eigen::Vector3d(0.0, 0.0, -1.0));
      state_frame.segment(6, 21) = rbt_state.q;
      state_frame.segment(27, 21) = rbt_state.dq;
      state_frame.segment(48, 21) = rbt_output.action;
      state_frame = state_frame.cwiseMax(-params.clip_obs).cwiseMin(params.clip_obs);
      if (native_state_history_.empty())
      {
        for (int i = 0; i < 17; ++i) native_state_history_.push_back(state_frame);
      }
      else
      {
        native_state_history_.push_back(state_frame);
        while (native_state_history_.size() > 17) native_state_history_.pop_front();
      }

      static const int history_offsets[] = {0, 1, 2, 3, 4, 8, 16};
      size_t cursor = 0;
      auto append_history = [&](int start, int width, int count) {
        for (int i = 0; i < count; ++i)
        {
          const int offset = history_offsets[i];
          const size_t history_index = native_state_history_.size() - 1 - static_cast<size_t>(offset);
          for (int j = 0; j < width; ++j)
            native_policy_input_[cursor++] = static_cast<float>(native_state_history_[history_index](start + j));
        }
      };
      append_history(0, 3, 7);
      append_history(3, 3, 7);
      append_history(6, 21, 7);
      append_history(27, 21, 7);
      append_history(48, 21, 3);
      if (cursor != 399) throw std::runtime_error("internal native policy input dimension mismatch");

      static const int future_offsets[] = {-8, -4, -2, 0, 1, 2, 3, 4};
      const int current_index = std::clamp(time_step_counter_, 0, motion_data_size_ - 1);
      const MotionData& current = precomputed_motion_data_[static_cast<size_t>(current_index)];
      const Eigen::Matrix3d anchor =
          get_imitation_anchor_ori_b_cpp(robot_initial_quat, rbt_state.quat,
                                         imitation_initial_quat, current.body_quat).toRotationMatrix();
      const Eigen::Matrix3d current_motion = current.body_quat.toRotationMatrix();
      std::vector<int> future_indices;
      future_indices.reserve(8);
      for (int offset : future_offsets)
        future_indices.push_back(std::clamp(current_index + offset, 0, motion_data_size_ - 1));

      // command_short is signal-major, matching MimicLite export order:
      //   all 8 root positions, then all 8 orientation-6D values, then all
      //   8 joint-position vectors.  Do not interleave the three signals per
      //   future frame; that makes the network interpret joint angles as root
      //   orientation/position and produces violent whole-body commands.
      cursor = 0;
      for (int future_index : future_indices)
      {
        const MotionData& future = precomputed_motion_data_[static_cast<size_t>(future_index)];
        Eigen::Vector3d current_pos = current.root_pos.head<3>();
        current_pos.z() = 0.0;
        Eigen::Vector3d delta = future.root_pos.head<3>() - current_pos;
        const double ref_yaw = std::atan2(2.0 * (current.body_quat.w() * current.body_quat.z() +
                                                 current.body_quat.x() * current.body_quat.y()),
                                          1.0 - 2.0 * (current.body_quat.y() * current.body_quat.y() +
                                                       current.body_quat.z() * current.body_quat.z()));
        const double c = std::cos(ref_yaw), s = std::sin(ref_yaw);
        native_command_input_[cursor++] = static_cast<float>(c * delta.x() + s * delta.y());
        native_command_input_[cursor++] = static_cast<float>(-s * delta.x() + c * delta.y());
        native_command_input_[cursor++] = static_cast<float>(delta.z());
      }
      for (int future_index : future_indices)
      {
        const MotionData& future = precomputed_motion_data_[static_cast<size_t>(future_index)];
        const Eigen::Matrix3d relative = anchor * current_motion.transpose() * future.body_quat.toRotationMatrix();
        for (int row = 0; row < 2; ++row)
          for (int col = 0; col < 3; ++col)
            native_command_input_[cursor++] = static_cast<float>(relative(row, col));
      }
      for (int future_index : future_indices)
      {
        const MotionData& future = precomputed_motion_data_[static_cast<size_t>(future_index)];
        for (int joint = 0; joint < 21; ++joint)
          native_command_input_[cursor++] = static_cast<float>(future.joint_pos(joint));
      }
      if (cursor != 240) throw std::runtime_error("internal native command dimension mismatch");
    }
    else
    {
      double shifted_time_toSec = lowState_.shifted_time.sec + 1e-9 * lowState_.shifted_time.nanosec;
      obs_manager_->assembleObservations(obs.observations, rbt_state, rbt_output, params, shifted_time_toSec);
      obs.observations = obs.observations.cwiseMax(-params.clip_obs).cwiseMin(params.clip_obs);
      if (!obs.hist_obs.empty()) obs.hist_obs.pop_front();
      obs.hist_obs.push_back(obs.observations);
    }

    // 基于实际运行时间计算时间步数，向下取整（与 ROS1 一致：已 reset 后才推进）
    const rclcpp::Time time_unset(0, 0, nh_->get_clock()->get_clock_type());
    if (imitation_start_time == time_unset)
    {
      time_step_counter_ = 0;
    }
    else
    {
      rclcpp::Duration elapsed_time = nh_->get_clock()->now() - imitation_start_time;
      double elapsed_seconds = elapsed_time.seconds();
      double time_step_float = elapsed_seconds * params.policy_frequency;
      time_step_counter_ = static_cast<int>(std::floor(time_step_float));
      if (motion_data_size_ > 0)
      {
        const int last_frame = motion_data_size_ - 1;
        if (time_step_counter_ > last_frame)
        {
          time_step_counter_ = last_frame;
        }
      }
    }

  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "updateObservation exception: %s", e.what());
    // 不抛出，防止回调崩溃；保留现场以便后续检查
  }
}

void RL_imitation::updateAction() {
  static std::mutex inference_mutex;
  std::lock_guard<std::mutex> lock(inference_mutex);

  if (!params.native_dynamic_reference)
  {
    const int hist_size = static_cast<int>(obs.hist_obs.size());
    for (int i = 0; i < params.frame_stack; ++i)
    {
      const int hist_index = hist_size - 1 + params.frame_stack_indices[i];
      const double* src_data = obs.hist_obs[hist_index].data();
      float* dst_data = obs.input_flat.data() + i * params.num_single_obs;
      for (int j = 0; j < params.num_single_obs; ++j) dst_data[j] = static_cast<float>(src_data[j]);
    }
  }

  const std::vector<float>& policy_input = params.native_dynamic_reference ? native_policy_input_ : obs.input_flat;
  const size_t input_size = params.native_dynamic_reference
                                ? 399U
                                : static_cast<size_t>(params.num_single_obs) * params.frame_stack;
  const size_t output_size = static_cast<size_t>(params.num_actions);

  if (policy_input.size() != input_size)
  {
    RCLCPP_ERROR(nh_->get_logger(), "TRT: Input flat size mismatch");
    return;
  }

  // -- backend-specific inference --
  const float* raw_output = nullptr;
  std::vector<float> ov_out_buf;

#ifdef USE_TENSORRT
  {
    cudaError_t err =
        cudaMemcpy(policy_input_device_, policy_input.data(), input_size * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT policy: H2D failed: %s", cudaGetErrorString(err));
      return;
    }
    if (params.native_dynamic_reference)
    {
      err = cudaMemcpy(policy_command_device_, native_command_input_.data(), 240U * sizeof(float),
                       cudaMemcpyHostToDevice);
      if (err != cudaSuccess)
      {
        RCLCPP_ERROR(nh_->get_logger(), "TRT policy command: H2D failed: %s", cudaGetErrorString(err));
        return;
      }
    }
    const char* input_name = trt_engine_policy_->getIOTensorName(policy_input_binding_idx_);
    const char* output_name = trt_engine_policy_->getIOTensorName(policy_output_binding_idx_);
    const bool command_ok = !params.native_dynamic_reference ||
        trt_context_policy_->setTensorAddress(
            trt_engine_policy_->getIOTensorName(policy_command_binding_idx_), policy_command_device_);
    if (!trt_context_policy_->setTensorAddress(input_name, policy_input_device_) || !command_ok ||
        !trt_context_policy_->setTensorAddress(output_name, policy_output_device_))
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT policy: setTensorAddress failed");
      return;
    }
    if (!trt_context_policy_->enqueueV3(0))
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT policy: enqueueV3 failed");
      return;
    }
    err = cudaMemcpy(policy_output_host_.data(), policy_output_device_, output_size * sizeof(float),
                     cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT policy: D2H failed: %s", cudaGetErrorString(err));
      return;
    }
    raw_output = policy_output_host_.data();
  }

#elif defined(USE_OPENVINO)
  {
    auto in_tensor = params.native_dynamic_reference ? ov_infer_policy_.get_tensor("policy")
                                                     : ov_infer_policy_.get_input_tensor();
    std::copy(policy_input.begin(), policy_input.end(), in_tensor.data<float>());
    if (params.native_dynamic_reference)
    {
      ov::Tensor command_tensor;
      try
      { command_tensor = ov_infer_policy_.get_tensor("command"); } catch (...)
      { command_tensor = ov_infer_policy_.get_tensor("command_short"); }
      std::copy(native_command_input_.begin(), native_command_input_.end(), command_tensor.data<float>());
    }
    ov_infer_policy_.infer();
    ov::Tensor out_tensor;
    if (!policy_output_name_.empty())
    {
      try
      { out_tensor = ov_infer_policy_.get_tensor(policy_output_name_); } catch (...)
      { out_tensor = ov_infer_policy_.get_output_tensor(); }
    }
    else
    {
      out_tensor = ov_infer_policy_.get_output_tensor();
    }
    const float* p = out_tensor.data<const float>();
    ov_out_buf.assign(p, p + output_size);
    raw_output = ov_out_buf.data();
  }
#endif

  if (!raw_output) return;

  // Read Isaac-ordered actions
  for (int isaac_i = 0; isaac_i < params.num_actions; ++isaac_i)
  {
    rbt_output.action[isaac_i] =
        std::clamp(static_cast<double>(raw_output[isaac_i]), -params.clip_actions, params.clip_actions);
  }

  // Transform to MJ ordering
  Eigen::VectorXd mj_actions;
  order_mapper_.transformIsaacToMj(rbt_output.action, mj_actions);

  // Apply scaling and default angles in MJ ordering
  for (int mj_i = 0; mj_i < params.num_actions; ++mj_i)
  {
    double scale = params.action_scale[mj_i];
    rbt_output.target_q[mj_i] = mj_actions[mj_i] * scale + params.default_joint_angles[mj_i];
    posCmd_.q[mj_i] = rbt_output.target_q[mj_i];
  }
}

Eigen::Vector3d RL_imitation::qr_ty_i(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
  double q_w = q[3];
  Eigen::Vector3d q_vec = q.head<3>();
  Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
  Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
  Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);
  return (a - b + c);
}
