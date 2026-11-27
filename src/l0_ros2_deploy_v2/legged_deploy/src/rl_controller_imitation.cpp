#include "rl_controller_imitation.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <utility>

#include "base/rl_controller_registrar.h"
#include "order_mapping.h"

static RLControllerRegistrar registrar_rl_imitation("rl_imitation", [](rclcpp::Node::SharedPtr nh) {
  return std::unique_ptr<ControllerType>(new RL_imitation(nh));
});

RL_imitation::RL_imitation(rclcpp::Node::SharedPtr node_handle)
    : BaseRLController(node_handle),
      loopRunning_(true),
      rbt_state(node_handle->get_logger()),
      rbt_output(node_handle->get_logger()),
      obs(Observations_Imitation(1, 1)),
      order_mapper_(node_handle->get_logger()),
      nh_(node_handle) {
  imitation_start_time = rclcpp::Time(0, 0, node_handle->get_clock()->get_clock_type());
  node_handle->get_parameter_or("dynamic_policy_switch_enabled", dynamic_policy_switch_enabled_, false);
  node_handle->get_parameter_or("policy_root_path", policy_root_path_, std::string(LEGGED_POLICY_PATH));
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

      RCLCPP_INFO(node_handle->get_logger(), "RL_imitation: policy '%s' loaded OK", params.subpolicy.c_str());
    }
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_handle->get_logger(), "Failed to initialize RL controller: %s", e.what());
    throw;
  }

  loopThread_ = std::thread([this]() {
    while (loopRunning_ && rclcpp::ok())
    {
      const auto started = SteadyClock::now();
      update();
      finishPolicyCycle(started, loop_frequency_hz_.load());
    }
  });

  configureThreadScheduler(loopThread_.native_handle(), deploy_params, "RL_imitation");
}

std::string RL_imitation::rlCmdPublishTopic() const { return "control/policy/command/imitation"; }

RL_imitation::~RL_imitation() {
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
  policy_observation_received_at_ = SteadyClock::time_point{};
  rl_inference_enabled_.store(false);
  current_policy_name_.clear();
  current_robot_cfg_path_.clear();
  previous_rl_inference_enabled_ = false;
  motion_done_sent_ = false;
  precomputed_motion_data_.clear();
  motion_data_size_ = 0;
  has_body_quat_output_ = false;

  model_policy_.reset();
  model_imitation_.reset();
  model_policy_output_idx_ = 0;

  imitation_outputs_.clear();
  obs_manager_.reset();
  params = RL_IMITATION_ModelParams();
  params.robot_type = robot_type_;
}

void RL_imitation::releaseCachedRuntime(CachedPolicyRuntime& runtime) {
  runtime.model_policy.reset();
  runtime.model_imitation.reset();
  runtime.model_policy_output_idx = 0;

  runtime.imitation_outputs.clear();
  runtime.obs_manager.reset();
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
  runtime.imitation_start_time = imitation_start_time;
  runtime.previous_rl_inference_enabled = previous_rl_inference_enabled_;
  runtime.initial_quats_logged = initial_quats_logged_;
  runtime.robot_initial_quat = robot_initial_quat;
  runtime.imitation_initial_quat = imitation_initial_quat;
  runtime.time_step_counter = time_step_counter_;
  runtime.policy_output_name = policy_output_name_;
  runtime.motion_done_sent = motion_done_sent_;
  runtime.precomputed_motion_data = std::move(precomputed_motion_data_);
  runtime.motion_data_size = motion_data_size_;
  runtime.has_body_quat_output = has_body_quat_output_;

  runtime.model_policy = std::move(model_policy_);
  runtime.model_imitation = std::move(model_imitation_);
  runtime.model_policy_output_idx = model_policy_output_idx_;
  model_policy_output_idx_ = 0;
  runtime.imitation_outputs = std::move(imitation_outputs_);

  model_loaded_.store(false);
  policy_observation_received_at_ = SteadyClock::time_point{};
  rl_inference_enabled_.store(false);
  current_policy_name_.clear();
  current_robot_cfg_path_.clear();
  params = RL_IMITATION_ModelParams();
  params.robot_type = robot_type_;
  pub_imitationref_joint_states_ = false;
  rbt_state = Robot_Imitation_State(nh_->get_logger());
  rbt_output = Robot_Imitation_Output(nh_->get_logger());
  obs = Observations_Imitation(1, 1);
  imitation_start_time = rclcpp::Time(0, 0, nh_->get_clock()->get_clock_type());
  previous_rl_inference_enabled_ = false;
  initial_quats_logged_ = false;
  robot_initial_quat = Eigen::Quaterniond::Identity();
  imitation_initial_quat = Eigen::Quaterniond::Identity();
  order_mapper_ = order_mapping::OrderMapper(nh_->get_logger());
  time_step_counter_ = 0;
  policy_output_name_.clear();
  motion_done_sent_ = false;
  motion_data_size_ = 0;
  has_body_quat_output_ = false;
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
  imitation_start_time = runtime.imitation_start_time;
  previous_rl_inference_enabled_ = runtime.previous_rl_inference_enabled;
  initial_quats_logged_ = runtime.initial_quats_logged;
  robot_initial_quat = runtime.robot_initial_quat;
  imitation_initial_quat = runtime.imitation_initial_quat;
  time_step_counter_ = runtime.time_step_counter;
  policy_output_name_ = std::move(runtime.policy_output_name);
  motion_done_sent_ = runtime.motion_done_sent;
  precomputed_motion_data_ = std::move(runtime.precomputed_motion_data);
  motion_data_size_ = runtime.motion_data_size;
  has_body_quat_output_ = runtime.has_body_quat_output;

  model_policy_ = std::move(runtime.model_policy);
  model_imitation_ = std::move(runtime.model_imitation);
  model_policy_output_idx_ = runtime.model_policy_output_idx;
  runtime.model_policy_output_idx = 0;
  imitation_outputs_ = std::move(runtime.imitation_outputs);

  if (!params.robot_type.empty())
  {
    robot_type_ = params.robot_type;
  }
  else
  {
    params.robot_type = robot_type_;
  }
  policy_observation_received_at_ = SteadyClock::time_point{};
  policy_observation_.joints.name = params.joint_order_mj;
  policy_observation_.joints.position.assign(params.num_actions, 0.0);
  policy_observation_.joints.velocity.assign(params.num_actions, 0.0);
  policy_observation_.joints.effort.assign(params.num_actions, 0.0);
  posCmd_.joint_names.assign(params.joint_order_mj.begin(), params.joint_order_mj.end());
  posCmd_.target_positions.assign(params.num_actions, 0.0);
  order_mapper_.initialize(params.joint_order_mj, params.joint_order_isaac);
  model_loaded_.store(!current_policy_name_.empty());
  loop_frequency_hz_.store(std::max(1, params.policy_frequency));
}

void RL_imitation::update() {
  const auto started = SteadyClock::now();
  std::lock_guard<std::mutex> lock(model_mutex_);
  const bool enabled = model_loaded_.load() && rl_inference_enabled_.load();
  if (!enabled)
  {
    previous_rl_inference_enabled_ = false;
    return;
  }
  if (!isPolicyInputFresh(policy_observation_, policy_observation_received_at_)) return;
  if (!previous_rl_inference_enabled_) resetImitationState();
  previous_rl_inference_enabled_ = true;

  if (!updateObservation()) return;
  auto next_output = rbt_output;
  if (!updateAction(next_output) || !isPolicyInputFresh(policy_observation_, policy_observation_received_at_) ||
      !isPolicyResultTimely(started, params.policy_frequency)) return;
  if (!publishPolicyCommand(rclcpp::Time(policy_observation_.header.stamp, nh_->get_clock()->get_clock_type()))) return;
  rbt_output = std::move(next_output);

  if (motion_data_size_ > 0 && pubImitationMotionDone_ &&
      time_step_counter_ >= motion_data_size_ - 1 && !motion_done_sent_)
  {
    std_msgs::msg::Bool done_msg;
    done_msg.data = true;
    pubImitationMotionDone_->publish(done_msg);
    motion_done_sent_ = true;
    RCLCPP_INFO(nh_->get_logger(), "RL_imitation: published /imitation_motion_done (reached frame %d / %d)",
                time_step_counter_, motion_data_size_);
  }
}

void RL_imitation::initTopic(rclcpp::Node::SharedPtr nh_, bool create_switch_service) {
  // Initialize the common PolicyObservation and PolicyCommand topics.
  initBaseTopic(nh_);
  // initialize special topics
  // subResetCmd_ = nh_.subscribe<std_msgs::Bool>("/rlImitationReset", 100,
  // &RL_imitation::imitationResetCallback, this);
  subResetCmd_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/imitation/reset", rclcpp::QoS(100),
      std::bind(&RL_imitation::imitationResetCallback, this, std::placeholders::_1));

  subImitationActive_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/imitation/active", rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&RL_imitation::imitationActiveCallback, this, std::placeholders::_1));

  // Created unconditionally; each loaded policy still controls whether reference states are published.
  pubImitationJointStates_ =
      nh_->create_publisher<sensor_msgs::msg::JointState>("/imitationref_joint_states", rclcpp::QoS(100));
  pubImitationMotionDone_ = nh_->create_publisher<std_msgs::msg::Bool>("/imitation_motion_done", rclcpp::QoS(10));
  if (create_switch_service)
  {
    createSwitchPolicyService();
  }
}

void RL_imitation::createSwitchPolicyService() {
  switchPolicySrv_ = nh_->create_service<robot_interfaces::srv::SwitchPolicy>(
      "control/policy/switch",
      std::bind(&RL_imitation::switchImitationPolicyCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void RL_imitation::initModel() {
  rbt_state =
      Robot_Imitation_State(nh_->get_logger(), params.num_actions, params.command_dim, params.imitation_anchor_dim);
  rbt_output = Robot_Imitation_Output(nh_->get_logger(), params.num_actions);
  obs = Observations_Imitation(params.num_single_obs, params.frame_stack_indices);

  try
  {
    // ---- Policy model (Inference) ----
    model_policy_ = std::make_unique<InferenceModel>(inferenceModelPath(params.policy_path));
    const size_t policy_in_sz = static_cast<size_t>(params.num_single_obs) * params.frame_stack;
    const size_t policy_out_sz = static_cast<size_t>(params.num_actions);
    if (model_policy_->inputCount() != 1 || model_policy_->inputElements() != policy_in_sz)
      throw std::runtime_error(
          "Inference policy input size mismatch: model=" + std::to_string(model_policy_->inputElements()) +
          ", config=" + std::to_string(policy_in_sz));
    model_policy_output_idx_ = model_policy_->outputIndex(policy_output_name_);
    if (model_policy_output_idx_ < 0) throw std::runtime_error("Inference policy output not found: " + policy_output_name_);
    if (model_policy_->outputElements(static_cast<size_t>(model_policy_output_idx_)) != policy_out_sz)
      throw std::runtime_error("Inference policy output size mismatch: model=" +
                               std::to_string(model_policy_->outputElements(model_policy_output_idx_)) +
                               ", config=" + std::to_string(policy_out_sz));

    // ---- Imitation model (Inference) ----
    model_imitation_ = std::make_unique<InferenceModel>(inferenceModelPath(params.imitation_path));
    if (model_imitation_->inputCount() != 1 || model_imitation_->inputElements() != 1)
      throw std::runtime_error("Inference imitation model input must contain exactly one timestep value");
    for (size_t i = 0; i < model_imitation_->outputCount(); ++i)
    {
      ImitationOutputBuffer buf;
      buf.output_index = static_cast<int>(i);
      buf.host_buf.assign(model_imitation_->outputElements(i), 0.0f);
      imitation_outputs_[model_imitation_->outputName(i)] = std::move(buf);
    }

    has_body_quat_output_ = imitation_outputs_.find("body_quat_w") != imitation_outputs_.end();

    // ---- Common: validate required outputs exist ----
    std::map<std::string, size_t> output_dims;
    for (const auto& cmd_item : params.command_items)
    {
      auto it = imitation_outputs_.find(cmd_item.type);
      if (it == imitation_outputs_.end())
      {
        RCLCPP_ERROR(nh_->get_logger(), "Imitation model missing required output '%s'", cmd_item.type.c_str());
        throw std::runtime_error("Motion model missing required output: " + cmd_item.type);
      }
      output_dims[cmd_item.type] = it->second.host_buf.size();
    }

    // Warmup inference (also validates model runs correctly)
    if (!runImitationInference(0.0f)) throw std::runtime_error("Warmup inference failed for imitation");

    motion_data_size_ = detectMaxTimeSteps(output_dims);

    // Pre-allocate and precompute all time steps
    precomputed_motion_data_.clear();
    precomputed_motion_data_.reserve(motion_data_size_);

    for (int t = 0; t < motion_data_size_; ++t)
    {
      if (!runImitationInference(static_cast<float>(t)))
      {
        RCLCPP_ERROR(nh_->get_logger(), "RL_imitation: inference failed at timestep %d", t);
        throw std::runtime_error("Imitation inference failed during precomputation");
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

  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to initialize model: %s", e.what());
    throw;
  }
}

bool RL_imitation::runImitationInference(float timestep) {
  if (!model_imitation_->run(&timestep, 1))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Inference imitation inference failed: %s", model_imitation_->lastError().c_str());
    return false;
  }
  for (auto& kv : imitation_outputs_)
  {
    const auto& output = model_imitation_->output(static_cast<size_t>(kv.second.output_index));
    std::copy(output.begin(), output.end(), kv.second.host_buf.begin());
  }

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
                  "RL_imitation: inference failed at timestep %d. Using %d "
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
  policy_observation_received_at_ = SteadyClock::time_point{};
  policy_observation_.joints.name = params.joint_order_mj;
  policy_observation_.joints.position.resize(params.num_actions);
  policy_observation_.joints.velocity.resize(params.num_actions);
  policy_observation_.joints.effort.resize(params.num_actions);
  posCmd_.joint_names.assign(params.joint_order_mj.begin(), params.joint_order_mj.end());
  posCmd_.target_positions.resize(params.num_actions);

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

    if (params.command_dim == 0)
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
        if (index < 0)
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
  const bool enabled = policy_observation_.inference_enabled && imitation_active_ && model_loaded_.load() &&
                       policy_observation_received_at_ != SteadyClock::time_point{};
  rl_inference_enabled_.store(enabled);
  if (!enabled) previous_rl_inference_enabled_ = false;
}

void RL_imitation::stateCallback(const robot_interfaces::msg::PolicyObservation::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(model_mutex_);
  if (model_loaded_.load() && validatePolicyObservation(*msg, params.joint_order_mj, "RL_imitation"))
  {
    policy_observation_ = *msg;
    policy_observation_received_at_ = SteadyClock::now();
  }
  else
  {
    policy_observation_.inference_enabled = false;
    policy_observation_received_at_ = SteadyClock::time_point{};
  }
  refreshInferenceEnabledLocked();
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

  if (model_loaded_.load() && requested_policy == current_policy_name_)
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
    const std::shared_ptr<robot_interfaces::srv::SwitchPolicy::Request> request,
    std::shared_ptr<robot_interfaces::srv::SwitchPolicy::Response> response) {
  if (!dynamic_policy_switch_enabled_)
  {
    response->success = false;
    response->error_code = robot_interfaces::srv::SwitchPolicy::Response::ERROR_INVALID_CONFIGURATION;
    response->message = "dynamic_policy_switch_enabled is false";
    response->active_policy = current_policy_name_;
    return;
  }

  std::lock_guard<std::mutex> lock(model_mutex_);
  imitation_active_ = false;
  refreshInferenceEnabledLocked();

  std::string robot_cfg_path;
  std::string message;
  const bool ok = loadPolicyLocked(request->policy_name, robot_cfg_path, message);
  response->success = ok;
  response->error_code = ok ? robot_interfaces::srv::SwitchPolicy::Response::ERROR_NONE
                            : robot_interfaces::srv::SwitchPolicy::Response::ERROR_LOAD_FAILED;
  response->message = message;
  response->active_policy = current_policy_name_;
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
  motion_done_sent_ = false;
  resetObservationState();
}

void RL_imitation::resetObservationState() {
  obs = Observations_Imitation(params.num_single_obs, params.frame_stack_indices);

  if (rbt_output.target_q.size() > 0)
  {
    rbt_output.target_q.setZero();
  }
  if (rbt_output.action.size() > 0)
  {
    rbt_output.action.setZero();
  }
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

bool RL_imitation::updateObservation() {
  // ROS_INFO_THROTTLE(1.0, "Updating observation at time step %d",
  // time_step_counter_);
  try
  {
    // clean rbt_state
    rbt_state.clean();

    rbt_state.imu_quat.w() = policy_observation_.imu.orientation.w;
    rbt_state.imu_quat.x() = policy_observation_.imu.orientation.x;
    rbt_state.imu_quat.y() = policy_observation_.imu.orientation.y;
    rbt_state.imu_quat.z() = policy_observation_.imu.orientation.z;

    // Select the reference for this cycle's time, rather than the previous cycle's frame.
    const double elapsed = std::max(0.0, (nh_->get_clock()->now() - imitation_start_time).seconds());
    time_step_counter_ = motion_data_size_ > 0 ? static_cast<int>(std::min(
        static_cast<double>(motion_data_size_ - 1), std::floor(elapsed * params.policy_frequency))) : 0;
    updateImitationData();

    if (!initial_quats_logged_)
    {
      // on the first time step, store the robot initial quat and imitation
      // initial quat
      robot_initial_quat = rbt_state.imu_quat;
      imitation_initial_quat = rbt_state.command_quat;
      initial_quats_logged_ = true;
    }

    order_mapper_.mapJointObservation(policy_observation_.joints.position, policy_observation_.joints.velocity,
                                      params.default_joint_angles, rbt_state.q, rbt_state.dq);

    rbt_state.base_ang_vel << policy_observation_.imu.angular_velocity.x, policy_observation_.imu.angular_velocity.y,
        policy_observation_.imu.angular_velocity.z;
    rbt_state.quat.x() = policy_observation_.imu.orientation.x;
    rbt_state.quat.y() = policy_observation_.imu.orientation.y;
    rbt_state.quat.z() = policy_observation_.imu.orientation.z;
    rbt_state.quat.w() = policy_observation_.imu.orientation.w;
    rbt_state.quat.normalize();

    // Imitation observations use anchor_ori, not the AMP yaw-relative quaternion.

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
    obs_manager_->assembleObservations(obs.observations, rbt_state, rbt_output, params);

    obs.observations = obs.observations.cwiseMax(-params.clip_obs).cwiseMin(params.clip_obs);
    if (!obs.hist_obs.empty())
    {
      obs.hist_obs.pop_front();
    }
    obs.hist_obs.push_back(obs.observations);

    return true;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "updateObservation exception: %s", e.what());
    return false;
  }
}

bool RL_imitation::updateAction(Robot_Imitation_Output& output) {
  const int hist_size = static_cast<int>(obs.hist_obs.size());
  for (int i = 0; i < params.frame_stack; ++i)
  {
    const int hist_index = hist_size - 1 + params.frame_stack_indices[i];
    const double* src_data = obs.hist_obs[hist_index].data();
    float* dst_data = obs.input_flat.data() + i * params.num_single_obs;
    for (int j = 0; j < params.num_single_obs; ++j)
    {
      dst_data[j] = static_cast<float>(src_data[j]);
    }
  }

  const size_t input_size = static_cast<size_t>(params.num_single_obs) * params.frame_stack;
  [[maybe_unused]] const size_t output_size = static_cast<size_t>(params.num_actions);

  if (obs.input_flat.size() != input_size)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Policy input flat size mismatch");
    return false;
  }

  const float* raw_output = nullptr;

  if (!model_policy_->run(obs.input_flat.data(), input_size))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Inference policy inference failed: %s", model_policy_->lastError().c_str());
    return false;
  }
  raw_output = model_policy_->output(static_cast<size_t>(model_policy_output_idx_)).data();

  if (!raw_output || !std::all_of(raw_output, raw_output + output_size,
                                  [](float value) { return std::isfinite(value); }))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Policy returned an invalid action");
    return false;
  }

  // Read Isaac-ordered actions
  for (int isaac_i = 0; isaac_i < params.num_actions; ++isaac_i)
  {
    output.action[isaac_i] =
        std::clamp(static_cast<double>(raw_output[isaac_i]), -params.clip_actions, params.clip_actions);
  }

  // Transform to MJ ordering
  Eigen::VectorXd mj_actions;
  order_mapper_.transformIsaacToMj(output.action, mj_actions);

  // Apply scaling and default angles in MJ ordering
  for (int mj_i = 0; mj_i < params.num_actions; ++mj_i)
  {
    double scale = params.action_scale[mj_i];
    output.target_q[mj_i] = mj_actions[mj_i] * scale + params.default_joint_angles[mj_i];
    posCmd_.target_positions[mj_i] = output.target_q[mj_i];
  }
  return true;
}
