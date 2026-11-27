#include "rl_controller.h"
#include "policy_action_history.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "order_mapping.h"
#include "base/rl_controller_registrar.h"

#ifndef LEGGED_POLICY_PATH
#define LEGGED_POLICY_PATH "/tmp/policies"  // 只给 VSCode 看，编译时会被 CMake 覆盖
#endif

static RLControllerRegistrar registrar_rl("rl", [](rclcpp::Node::SharedPtr nh) {
  return std::unique_ptr<ControllerType>(new RL(nh));
});

std::string RL::rlCmdPublishTopic() const { return "control/policy/command/rl"; }

RL::RL(rclcpp::Node::SharedPtr node_handle)
    : BaseRLController(node_handle),
      nh_(node_handle),
      loopRunning_(true),
      rbt_state(node_handle->get_logger()),
      rbt_output(node_handle->get_logger()),
      obs(ObservationsGeneric(1, 1)),
      order_mapper_(node_handle->get_logger()) {
  try
  {
    // Get parameters
    if (!getParam())
    {
      throw std::runtime_error("Failed to get parameters");
    }
    std::string robot_cfg_path("");
    if (!node_handle->get_parameter("robot_cfg_path", robot_cfg_path))
    {
      throw std::runtime_error("Required parameter 'robot_cfg_path' not found");
    }
    YAML::Node root = YAML::LoadFile(robot_cfg_path);
    YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");
    policy_output_name_ = rc["policy_output_name"].as<std::string>("");
    RCLCPP_INFO(node_handle->get_logger(), "RL: using policy_output_name '%s'", policy_output_name_.c_str());
    std::transform(policy_output_name_.begin(), policy_output_name_.end(), policy_output_name_.begin(), ::tolower);

    // Initialize
    order_mapper_.initialize(params.joint_order_mj, params.joint_order_isaac);

    initData();
    initTopic(nh);
    initModel();

    RCLCPP_INFO(nh_->get_logger(), "RL Policy initialized successfully");
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to initialize RL policy: %s", e.what());
    throw;
  }

  loopThread_ = std::thread([this]() {
    while (loopRunning_ && rclcpp::ok())
    {
      const auto started = SteadyClock::now();
      update();
      finishPolicyCycle(started, params.policy_frequency);
    }
  });
  configureThreadScheduler(loopThread_.native_handle(), deploy_params, "RL");
}

RL::~RL() {
  loopRunning_ = false;
  if (loopThread_.joinable())
  {
    loopThread_.join();
  }

  RCLCPP_DEBUG(nh_->get_logger(), "RL destruction");
}

void RL::update() {
  const auto started = SteadyClock::now();
  robot_interfaces::msg::PolicyObservation observation;
  SteadyClock::time_point received_at;
  bool reset_observation_state;
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    const bool enabled = rl_inference_enabled_.load();
    const bool just_enabled = enabled && !previous_rl_inference_enabled_;
    previous_rl_inference_enabled_ = enabled;
    if (!enabled) return;
    // The active topic can arrive before the snapshot carrying this return's target seed.
    if (policy_observation_.source_policy == robot_interfaces::msg::PolicyObservation::SOURCE_WBC &&
        policy_observation_.amp_return_id.empty()) return;
    const bool reset_requested = obs_reset_requested_.exchange(false);
    reset_observation_state = reset_requested || just_enabled;
    observation = policy_observation_;
    received_at = policy_observation_received_at_;
  }
  if (!isPolicyInputFresh(observation, received_at))
  {
    if (reset_observation_state) requestObservationStateReset();
    return;
  }
  const bool new_return = !observation.amp_return_id.empty() &&
                          observation.amp_return_id != initialized_return_id_;
  const bool seed_measured = observation.amp_return_id.empty() &&
                             (reset_observation_state || !has_published_action_);
  if (new_return || seed_measured)
  {
    std::vector<double> actions;
    bool seeded_from_command = false;
    if (new_return)
    {
      std::string error;
      seeded_from_command = observation.source_policy == robot_interfaces::msg::PolicyObservation::SOURCE_WBC &&
          observation.source_command.source_id == observation.amp_return_id &&
          actionHistoryFromCommand(observation.source_command, params.joint_order_mj,
                                    params.default_joint_angles, params.action_scale, nh_->get_clock()->now(),
                                    actions, error);
      if (!seeded_from_command)
      {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
            "AMP return seed unavailable; initializing from fresh measured joints: %s",
            error.empty() ? "source/return mismatch or expired seed" : error.c_str());
      }
    }
    if (!seeded_from_command)
    {
      actions.resize(params.num_actions);
      for (int i = 0; i < params.num_actions; ++i)
        actions[i] = params.action_scale[i] == 0.0 ? 0.0 :
            (observation.joints.position[i] - params.default_joint_angles[i]) / params.action_scale[i];
    }

    Eigen::VectorXd mj_actions(params.num_actions);
    std::size_t clipped = 0;
    for (int i = 0; i < params.num_actions; ++i)
    {
      if (!std::isfinite(actions[i]))
      {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                             "AMP history seed rejected: non-finite action at joint %d", i);
        requestObservationStateReset();
        return;
      }
      mj_actions[i] = std::clamp(actions[i], -params.clip_actions, params.clip_actions);
      if (mj_actions[i] != actions[i]) ++clipped;
      rbt_output.target_q[i] = mj_actions[i] * params.action_scale[i] + params.default_joint_angles[i];
    }
    order_mapper_.transformMjToIsaac(mj_actions, rbt_output.action);
    reset_observation_state = true;
    if (new_return && seeded_from_command)
      RCLCPP_DEBUG(nh_->get_logger(), "[AMP history] return=%s seeded %d observation frames from WBC targets; clipped=%zu",
                  observation.amp_return_id.c_str(), params.frame_history_length, clipped);
  }
  if (reset_observation_state) resetObservationState();
  if (!updateObservation(observation))
  {
    requestObservationStateReset();
    return;
  }
  if (reset_observation_state)
    for (auto& h : obs.hist_obs) h = obs.observations;
  initialized_return_id_ = observation.amp_return_id;
  posCmd_.amp_return_id = initialized_return_id_;

  auto next_output = rbt_output;
  if (!updateAction(next_output)) return;
  std::lock_guard<std::mutex> lock(observation_mutex_);
  if (!rl_inference_enabled_.load() || obs_reset_requested_.load() ||
      observation.amp_return_id != policy_observation_.amp_return_id ||
      !isPolicyInputFresh(observation, received_at) ||
      !isPolicyInputFresh(policy_observation_, policy_observation_received_at_) ||
      !isPolicyResultTimely(started, params.policy_frequency)) return;
  if (publishPolicyCommand(rclcpp::Time(observation.header.stamp, nh_->get_clock()->get_clock_type())))
  {
    rbt_output = std::move(next_output);
    has_published_action_ = true;
  }
}

void RL::initTopic(rclcpp::Node::SharedPtr nh_) {
  initBaseTopic(nh_);
  actor_input_pub_ = nh_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "control/policy/amp/actor_input", rclcpp::QoS(32).best_effort());
  subObsHistoryReset_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/observation/reset", rclcpp::QoS(10),
      std::bind(&RL::obsHistoryResetCallback, this, std::placeholders::_1));
  subImitationActive_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/imitation/active", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        imitation_active_.store(msg->data);
        if (msg->data)
        {
          rl_inference_enabled_.store(false);
        }
        else
        {
          if (policy_observation_.amp_return_id.empty()) requestObservationStateReset();
          rl_inference_enabled_.store(policy_observation_.inference_enabled && !wbc_active_.load());
        }
      });
  subWbcActive_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/wbc/active", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        const bool previous = wbc_active_.exchange(msg->data);
        const bool has_return = !policy_observation_.amp_return_id.empty();
        const bool preview = has_return && policy_observation_.source_policy ==
            robot_interfaces::msg::PolicyObservation::SOURCE_WBC;
        if (previous != msg->data && !has_return) requestObservationStateReset();
        rl_inference_enabled_.store(policy_observation_.inference_enabled && !imitation_active_.load() &&
                                    (!msg->data || preview));
      });
  RCLCPP_INFO(nh_->get_logger(), "RL: consuming controller PolicyObservation");
}

void RL::initModel() {
  rbt_state = Robot_State(nh_->get_logger(), params.num_actions);
  rbt_output = Robot_Output(nh_->get_logger(), params.num_actions);
  obs = Observations(params.num_single_obs, params.frame_stack_indices);

  try
  {
    const std::string model_path = inferenceModelPath(params.policy_path);
    model_policy_ = std::make_unique<InferenceModel>(model_path);
    const size_t input_size = static_cast<size_t>(params.num_single_obs) * params.frame_stack;
    const size_t output_size = static_cast<size_t>(params.num_actions);
    if (model_policy_->inputCount() != 1 || model_policy_->inputElements() != input_size)
      throw std::runtime_error(
          "Inference policy input size mismatch: model=" + std::to_string(model_policy_->inputElements()) +
          ", config=" + std::to_string(input_size));
    model_policy_output_idx_ = model_policy_->outputIndex(policy_output_name_);
    if (model_policy_output_idx_ < 0) throw std::runtime_error("Inference policy output not found: " + policy_output_name_);
    if (model_policy_->outputElements(static_cast<size_t>(model_policy_output_idx_)) != output_size)
      throw std::runtime_error("Inference policy output size mismatch: model=" +
                               std::to_string(model_policy_->outputElements(model_policy_output_idx_)) +
                               ", config=" + std::to_string(output_size));
    RCLCPP_INFO(nh_->get_logger(), "Inference: Loaded model from %s", model_path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to init Inference model: %s", e.what());
    throw;
  }

}

void RL::initData() {
  policy_observation_.joints.name = params.joint_order_mj;
  policy_observation_.joints.position.resize(params.num_actions);
  policy_observation_.joints.velocity.resize(params.num_actions);
  policy_observation_.joints.effort.resize(params.num_actions);
  posCmd_.joint_names.assign(params.joint_order_mj.begin(), params.joint_order_mj.end());
  posCmd_.target_positions.resize(params.num_actions);

  obs_manager_ = std::make_unique<ObservationManager>(params.observation_items, params);

  // Check observation manager dimension
  int total_obs_dim = obs_manager_->getTotalDimension();
  if (total_obs_dim != params.num_single_obs)
  {
    RCLCPP_ERROR(nh_->get_logger(),
                 "RL: observation dimension mismatch! Detected: %d, Expected "
                 "(num_single_obs): %d",
                 total_obs_dim, params.num_single_obs);
    throw std::runtime_error("Observation dimension mismatch in configuration");
  }
}

bool RL::getParam() {
  try
  {
    // use centralized reader for shared params
    if (!readCommonParams(static_cast<RLBaseModelParams&>(params), params.observation_items))
    {
      throw std::runtime_error("Failed to read common parameters");
    }

    // read deployment configuration parameters
    if (!readDeployParams(deploy_params))
    {
      throw std::runtime_error("Failed to read deployment parameters");
    }

    // policy path depends on build flags
    params.policy_path = LEGGED_POLICY_PATH + std::string("/") + params.robot_type + std::string("/") +
                         params.subpolicy + std::string("/") + params.policy_name + std::string(".onnx");

    std::string robot_cfg_path("");
    if (!nh_->get_parameter("robot_cfg_path", robot_cfg_path))
    {
      throw std::runtime_error("Required parameter 'robot_cfg_path' not found");
    }
    YAML::Node root = YAML::LoadFile(robot_cfg_path);
    YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");

    params.joint_order_mj = asStringList(rc["joint_order_mj"], "RobotCfg.joint_order_mj");
    params.joint_order_isaac = asStringList(rc["joint_order_isaac"], "RobotCfg.joint_order_isaac");
    params.action_scale = asDoubleList(rc["action_scale"], "RobotCfg.action_scale");

    // Load gait_config from YAML when present
    if (rc["gait_config"])
    {
      YAML::Node gc = rc["gait_config"];
      params.gait_config.type = gc["type"].as<std::string>("fixed gait");
      if (gc["params"])
      {
        params.gait_config.gait_frequency = gc["params"]["gait_frequency"].as<double>(params.gait_frequency);
        params.gait_config.swing_height = gc["params"]["swing_height"].as<double>(0.0);
      }
      else
      {
        params.gait_config.gait_frequency = params.gait_frequency;
      }
      if (gc["items"])
      {
        params.gait_config.items.clear();
        for (const auto& item : gc["items"])
        {
          std::string t = item["type"].as<std::string>();
          int d = item["dim"].as<int>();
          double s = item["scale"].as<double>(1.0);
          params.gait_config.items.emplace_back(t, d, s);
        }
      }
      RCLCPP_INFO(nh_->get_logger(), "RL: gait_config loaded type=%s freq=%.2f swing=%.3f items=%zu",
                  params.gait_config.type.c_str(), params.gait_config.gait_frequency, params.gait_config.swing_height,
                  params.gait_config.items.size());
    }
    else
    {
      params.gait_config.gait_frequency = params.gait_frequency;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "RL: read action_scale for %zu joints", params.action_scale.size());
    RCLCPP_DEBUG(nh_->get_logger(), "RL: read joint_order_mj with %zu joints", params.joint_order_mj.size());
    RCLCPP_DEBUG(nh_->get_logger(), "RL: read joint_order_isaac with %zu joints", params.joint_order_isaac.size());

    return true;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "RL: error in the config: %s", e.what());
    return false;
  }
}

void RL::stateCallback(const robot_interfaces::msg::PolicyObservation::ConstSharedPtr msg) {
  const bool valid = validatePolicyObservation(*msg, params.joint_order_mj, "RL");
  std::lock_guard<std::mutex> lock(observation_mutex_);
  if (valid && policy_observation_received_at_ != SteadyClock::time_point{} &&
      rclcpp::Time(msg->header.stamp) <= rclcpp::Time(policy_observation_.header.stamp)) return;
  if (!valid)
  {
    policy_observation_.inference_enabled = false;
    policy_observation_received_at_ = SteadyClock::time_point{};
    rl_inference_enabled_.store(false);
    requestObservationStateReset();
    return;
  }
  if (policy_observation_.inference_enabled != msg->inference_enabled) requestObservationStateReset();
  policy_observation_ = *msg;
  policy_observation_received_at_ = SteadyClock::now();
  rl_inference_enabled_.store(msg->inference_enabled && !imitation_active_.load() &&
                              (!wbc_active_.load() || (!msg->amp_return_id.empty() &&
                                  msg->source_policy == robot_interfaces::msg::PolicyObservation::SOURCE_WBC)));
}

bool RL::updateObservation(const robot_interfaces::msg::PolicyObservation& observation) {
  order_mapper_.mapJointObservation(observation.joints.position, observation.joints.velocity,
                                    params.default_joint_angles, rbt_state.q, rbt_state.dq);
  rbt_state.base_ang_vel << observation.imu.angular_velocity.x, observation.imu.angular_velocity.y,
      observation.imu.angular_velocity.z;
  rbt_state.quat.x() = observation.imu.orientation.x;
  rbt_state.quat.y() = observation.imu.orientation.y;
  rbt_state.quat.z() = observation.imu.orientation.z;
  rbt_state.quat.w() = observation.imu.orientation.w;

  rbt_state.quat.normalize();

  if (!initial_yaw_valid_)
  {
    const auto& q = rbt_state.quat;
    const double initial_yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    initial_yaw_quat_inv_ = Eigen::Quaterniond(std::cos(initial_yaw * 0.5), 0.0, 0.0,
                                              -std::sin(initial_yaw * 0.5));
    initial_yaw_valid_ = true;
    RCLCPP_DEBUG(nh_->get_logger(), "RL: captured initial yaw %.6f rad for quat observation", initial_yaw);
  }
  rbt_state.quat_obs = initial_yaw_quat_inv_ * rbt_state.quat;
  rbt_state.quat_obs.normalize();

  rbt_state.policy_command << observation.velocity_command.linear.x, observation.velocity_command.linear.y,
      observation.velocity_command.angular.z;

  obs_manager_->assembleObservations(obs.observations, rbt_state, rbt_output, params);

  obs.observations = obs.observations.cwiseMax(-params.clip_obs).cwiseMin(params.clip_obs);

  if (!obs.hist_obs.empty())
  {
    obs.hist_obs.pop_front();
  }
  obs.hist_obs.push_back(obs.observations);
  return true;
}

void RL::resetObservationState() {
  obs = Observations(params.num_single_obs, params.frame_stack_indices);
  initial_yaw_quat_inv_ = Eigen::Quaterniond::Identity();
  initial_yaw_valid_ = false;
}

void RL::requestObservationStateReset() { obs_reset_requested_.store(true); }

void RL::obsHistoryResetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (!msg->data) return;
  std::lock_guard<std::mutex> lock(observation_mutex_);
  // A seeded return is reset by its ID, not by a late legacy Bool notification.
  if (!policy_observation_.amp_return_id.empty()) return;
  requestObservationStateReset();
}

bool RL::updateAction(Robot_Output& output) {
  const int hist_size = static_cast<int>(obs.hist_obs.size());
  for (int i = 0; i < params.frame_stack; ++i)
  {
    const int hist_index = hist_size - 1 + params.frame_stack_indices[i];
    const double* src_data = obs.hist_obs[hist_index].data();
    float* dst_data = obs.input_flat.data() + i * params.num_single_obs;
    for (int j = 0; j < params.num_single_obs; ++j) dst_data[j] = static_cast<float>(src_data[j]);
  }

  const size_t input_size = static_cast<size_t>(params.num_single_obs) * params.frame_stack;
  [[maybe_unused]] const size_t output_size = static_cast<size_t>(params.num_actions);

  if (obs.input_flat.size() != input_size)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Input flat size mismatch");
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

  if (actor_input_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Float32MultiArray sample;
    sample.data.assign(obs.input_flat.data(), obs.input_flat.data() + input_size);
    actor_input_pub_->publish(sample);
  }
  // Read Isaac-ordered actions
  for (int i = 0; i < params.num_actions; ++i)
    output.action[i] = std::clamp(static_cast<double>(raw_output[i]), -params.clip_actions, params.clip_actions);

  // Transform to MJ ordering and apply scaling
  Eigen::VectorXd mj_actions;
  order_mapper_.transformIsaacToMj(output.action, mj_actions);
  for (int mj_i = 0; mj_i < params.num_actions; ++mj_i)
  {
    output.target_q[mj_i] = mj_actions[mj_i] * params.action_scale[mj_i] + params.default_joint_angles[mj_i];
    posCmd_.target_positions[mj_i] = output.target_q[mj_i];
  }
  return true;
}
