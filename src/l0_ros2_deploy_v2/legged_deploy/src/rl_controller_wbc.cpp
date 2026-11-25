#include "rl_controller_wbc.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "base/rl_controller_registrar.h"

namespace {

constexpr std::size_t kJointCount = 21;
constexpr std::size_t kPolicyInputSize = 399;
constexpr std::size_t kCommandInputSize = 240;
constexpr std::size_t kReferenceBodyCount = 23;
const std::array<const char*, 6> kReferenceOutputs =
    {{"joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"}};
const std::array<std::size_t, 6> kReferenceOutputSizes = {{21, 21, 69, 92, 69, 69}};

std::string joinPath(const std::string& base, const std::string& child) {
  if (child.empty() || child.front() == '/') return child;
  return base + "/" + child;
}

std::string wbcModelPath(const std::string& path) {
#ifdef USE_RKNN
  return rknnModelPath(path);
#else
  return path;
#endif
}

bool finiteVector(const std::vector<float>& values) {
  return std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); });
}

std::array<double, 4> normalize(const std::array<double, 4>& q) {
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!std::isfinite(norm) || norm < 1e-9) throw std::runtime_error("invalid robot orientation quaternion");
  return {{q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm}};
}

std::array<double, 4> conjugate(const std::array<double, 4>& q) { return {{q[0], -q[1], -q[2], -q[3]}}; }

std::array<double, 4> multiply(const std::array<double, 4>& a, const std::array<double, 4>& b) {
  return {{a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
           a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
           a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]}};
}

std::array<double, 3> rotateInverse(const std::array<double, 4>& orientation, const std::array<double, 3>& vector) {
  const auto q = normalize(orientation);
  const auto rotated = multiply(multiply(conjugate(q), {{0.0, vector[0], vector[1], vector[2]}}), q);
  return {{rotated[1], rotated[2], rotated[3]}};
}

std::array<double, 4> yawQuaternion(const std::array<double, 4>& orientation) {
  const auto q = normalize(orientation);
  const double yaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  return {{std::cos(yaw * 0.5), 0.0, 0.0, std::sin(yaw * 0.5)}};
}

void appendRotationFirstTwoRows(const std::array<double, 4>& input, std::vector<float>& out) {
  const auto q = normalize(input);
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  out.push_back(static_cast<float>(1.0 - 2.0 * (y * y + z * z)));
  out.push_back(static_cast<float>(2.0 * (x * y - z * w)));
  out.push_back(static_cast<float>(2.0 * (x * z + y * w)));
  out.push_back(static_cast<float>(2.0 * (x * y + z * w)));
  out.push_back(static_cast<float>(1.0 - 2.0 * (x * x + z * z)));
  out.push_back(static_cast<float>(2.0 * (y * z - x * w)));
}

RLControllerRegistrar registrar_wbc("rl_wbc", [](rclcpp::Node::SharedPtr node) {
  return std::unique_ptr<ControllerType>(new RLWbc(std::move(node)));
});

}  // namespace

std::string RLWbc::rlCmdPublishTopic() const { return "control/policy/command/wbc"; }

RLWbc::RLWbc(rclcpp::Node::SharedPtr node) : BaseRLController(node), node_(std::move(node)) {
  const auto preload_started = std::chrono::steady_clock::now();
  std::string policy_root;
  std::string robot_type;
  std::vector<std::string> policies;
  if (!node_->get_parameter("policy_root_path", policy_root) || policy_root.empty() ||
      !node_->get_parameter("robot_type", robot_type) || robot_type.empty() ||
      !node_->get_parameter("available_wbc_policies", policies) || policies.empty())
    throw std::runtime_error("WBC requires policy_root_path, robot_type and available_wbc_policies");
  const auto valid_directory = [](const std::string& value) {
    return !value.empty() && std::all_of(value.begin(), value.end(), [](char c) {
      return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
    });
  };
  if (!valid_directory(robot_type) || std::set<std::string>(policies.begin(), policies.end()).size() != policies.size())
    throw std::runtime_error("Invalid WBC robot_type or duplicate policy directories");
  std::string mode = "static", manifest;
  node_->get_parameter("wbc_mode", mode);
  node_->get_parameter("wbc_lora_manifest", manifest);
  if (mode != "static" && mode != "lora") throw std::runtime_error("wbc_mode must be static or lora");
  lora_mode_ = mode == "lora";
  if (lora_mode_) {
#ifdef USE_RKNN
    if (manifest.empty()) throw std::runtime_error("LoRA requires a WBC manifest, never an AMP subpolicy");
    lora_assets_ = std::make_unique<WbcLoraAssets>(manifest, policies);
    shared_policy_ = std::make_unique<RknnModel>(lora_assets_->modelPath(), RknnModel::InputMode::ResidentWbc);
    lora_assets_->bindModel(*shared_policy_);
    for (auto& buffer : adapter_buffers_) lora_assets_->allocate(buffer);
    RCLCPP_INFO(node_->get_logger(), "WBC shared Base initialized once: %s; checkpoint=%s; %s",
                lora_assets_->modelPath().c_str(), lora_assets_->baseHash().c_str(), shared_policy_->runtimeVersion().c_str());
    RCLCPP_INFO(node_->get_logger(), "WBC resident I/O (explicit cache sync, FP16/FP32 native):\n%s",
                shared_policy_->residentLayout().c_str());
#else
    throw std::runtime_error("LoRA resident inference requires RKNN; use static ONNX for OpenVINO simulation");
#endif
  }
  inputs_.resize(lora_mode_ ? 0 : 2, nullptr);
  input_sizes_.resize(inputs_.size(), 0);
  for (const auto& policy : policies)
  {
    if (!valid_directory(policy)) throw std::runtime_error("Invalid WBC policy directory: " + policy);
    const auto started = std::chrono::steady_clock::now();
    auto motion = std::make_unique<Motion>();
    motion->config_dir_ = joinPath(joinPath(policy_root, robot_type), policy);
    if (!std::ifstream(joinPath(motion->config_dir_, "rl_config.yaml")))
      motion->config_dir_ = joinPath(joinPath(joinPath(policy_root, robot_type), "wbc"), policy);
    motion->robot_cfg_path_ = joinPath(motion->config_dir_, "rl_config.yaml");
#ifdef USE_RKNN
    if (lora_assets_) motion->robot_cfg_path_ = lora_assets_->action(policy).config;
#endif
    try
    {
      loadConfig(*motion, motion->robot_cfg_path_);
      if (lora_mode_ && motion->motion_id_ != policy) throw std::runtime_error("WBC action/config identity mismatch");
      initializeModel(*motion);
      const std::string id = motion->motion_id_;
      if (!motions_.emplace(id, std::move(motion)).second)
        throw std::runtime_error("Duplicate WBC motion_id: " + id);
      RCLCPP_INFO(node_->get_logger(), "WBC preloaded %s: %.2f ms", id.c_str(),
                  std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count());
    } catch (const std::exception& exception)
    {
      throw std::runtime_error("WBC preload failed for " + policy + ": " + exception.what());
    }
  }
  if (!readDeployParams(deploy_params_)) throw std::runtime_error("Failed to read WBC deployment parameters");
  if (!node_->has_parameter("sensor_timeout")) node_->declare_parameter("sensor_timeout", 0.1);
  node_->get_parameter("sensor_timeout", sensor_timeout_sec_);
  if (!std::isfinite(sensor_timeout_sec_) || sensor_timeout_sec_ <= 0.0)
    throw std::runtime_error("sensor_timeout must be finite and positive");
  initBaseTopic(node_);
  posCmd_.source_id = "wbc";
  posCmd_.target_positions.resize(kJointCount);
  posCmd_.joint_names.assign(joint_names_.begin(), joint_names_.end());

  done_pub_ = node_->create_publisher<std_msgs::msg::Bool>("control/policy/wbc/done", rclcpp::QoS(10));
  active_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/wbc/active", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&RLWbc::activeCallback, this, std::placeholders::_1));
  prepare_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  prepare_service_ = node_->create_service<luvbot_control_interfaces::srv::ControlWbcMotion>(
      "control/policy/wbc/prepare",
      std::bind(&RLWbc::prepareCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, prepare_callback_group_);

  loop_thread_ = std::thread(&RLWbc::runLoop, this);
  if (deploy_params_.set_realtime_priority)
  {
    sched_param scheduler{};
    scheduler.sched_priority = deploy_params_.thread_priority;
    const int policy = getSchedulerPolicy(deploy_params_.thread_scheduling_method);
    if (pthread_setschedparam(loop_thread_.native_handle(), policy, &scheduler) != 0)
      RCLCPP_WARN(node_->get_logger(), "WBC: failed to set realtime priority");
    else
      printThreadSchedulerInfo(loop_thread_.native_handle(), "WBC");
  }
  RCLCPP_INFO(node_->get_logger(), "WBC worker ready: 50 Hz, %zu motions preloaded in %.2f ms; idle inference disabled",
              motions_.size(),
              std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - preload_started).count());
}

RLWbc::~RLWbc() {
  loop_running_.store(false);
  if (loop_thread_.joinable()) loop_thread_.join();
  // The owning executor is stopped before controller destruction. Also retire
  // any loader already holding the spare slot before model memory is released.
  std::lock_guard<std::mutex> loading_lock(loading_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);
  ++prepare_generation_;
  preparing_ = prepared_ = active_ = false;
}

void RLWbc::loadConfig(Motion& motion, const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node robot = root["RobotCfg"];
  const YAML::Node wbc = root["WbcCfg"];
  requireNode(robot, "RobotCfg");
  requireNode(wbc, "WbcCfg");
  const auto joints = asStringList(robot["joint_order_mj"], "RobotCfg.joint_order_mj");
  if (!joint_names_.empty() && joints != joint_names_)
    throw std::runtime_error("All WBC motions must have identical observation joint names/order");
  joint_names_ = joints;
  for (const char* key : {"kps", "kds"})
  {
    const auto gains = asDoubleList(robot[key], std::string("RobotCfg.") + key);
    if (gains.size() != kJointCount || !std::all_of(gains.begin(), gains.end(), [](double value) {
          return std::isfinite(value) && value >= 0.0;
        }))
      throw std::runtime_error(std::string("Invalid WBC gains: ") + key);
  }
  const auto defaults = asDoubleList(robot["default_joint_angles"], "RobotCfg.default_joint_angles");
  const auto scales = asDoubleList(robot["action_scale"], "RobotCfg.action_scale");
  if (joint_names_.size() != kJointCount || defaults.size() != kJointCount || scales.size() != kJointCount)
    throw std::runtime_error("WBC joint/default/action_scale arrays must each contain 21 values");
  if (std::set<std::string>(joint_names_.begin(), joint_names_.end()).size() != joint_names_.size())
    throw std::runtime_error("WBC policy joint names must be unique");
  motion.default_joint_positions_.assign(defaults.begin(), defaults.end());
  motion.action_scale_.assign(scales.begin(), scales.end());
  if (!finiteVector(motion.default_joint_positions_) || !finiteVector(motion.action_scale_) ||
      std::any_of(motion.action_scale_.begin(), motion.action_scale_.end(), [](float value) { return value <= 0.0f; }))
    throw std::runtime_error("WBC defaults must be finite and action scales finite and positive");

  motion.clip_actions_ = robot["clip_actions"].as<float>();
  motion.control_dt_ = wbc["control_dt"].as<double>();
  motion.start_hold_sec_ = wbc["start_hold_sec"].as<double>();
  motion.history_steps_ = wbc["history_steps"].as<std::vector<int>>();
  motion.future_steps_ = wbc["future_steps"].as<std::vector<int>>();
  motion.previous_action_steps_ = wbc["previous_action_steps"].as<int>();
  motion.policy_input_name_ = wbc["policy_input_name"].as<std::string>();
  motion.command_input_name_ = wbc["command_input_name"].as<std::string>();
  motion.output_name_ = wbc["output_name"].as<std::string>();
  if (!std::isfinite(motion.control_dt_) || !std::isfinite(motion.start_hold_sec_) || !std::isfinite(motion.clip_actions_) ||
      std::abs(motion.control_dt_ - 0.02) > 1e-9 || motion.start_hold_sec_ < 0.0 || motion.clip_actions_ <= 0.0f ||
      robot["policy_frequency"].as<int>() != 50 || robot["num_actions"].as<int>() != 21 ||
      motion.policy_input_name_ != "policy" ||
      (motion.command_input_name_ != "command_short" && motion.command_input_name_ != "command") ||
      motion.output_name_ != "action" ||
      motion.history_steps_ != std::vector<int>({0, 1, 2, 3, 4, 8, 16}) ||
      motion.future_steps_ != std::vector<int>({-8, -4, -2, 0, 1, 2, 3, 4}) || motion.previous_action_steps_ != 3)
    throw std::runtime_error("WBC config does not match the exported 399/240 model ABI");

  motion.motion_id_ = wbc["motion_id"].as<std::string>();
  if (motion.motion_id_.empty() || motion.motion_id_.size() > 32 ||
      !std::all_of(motion.motion_id_.begin(), motion.motion_id_.end(), [](char c) {
        return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
      }))
    throw std::runtime_error("Invalid WBC motion id");
  motion.reference_num_frames_ = wbc["reference_num_frames"].as<int>();
  motion.reference_root_body_index_ = wbc["reference_root_body_index"].as<int>();
  if (motion.reference_num_frames_ <= 0 || motion.reference_num_frames_ > 100000 || motion.reference_root_body_index_ < 0 ||
      motion.reference_root_body_index_ >= static_cast<int>(kReferenceBodyCount))
    throw std::runtime_error("Invalid WBC reference frame count or root body index");
}

void RLWbc::initializeModel(Motion& motion) {
#ifdef USE_RKNN
  if (lora_assets_) motion.policy_ = shared_policy_.get();
  else
#endif
  {
    motion.model_ = std::make_unique<WbcModel>(wbcModelPath(joinPath(motion.config_dir_, "policy.onnx")));
    motion.policy_ = motion.model_.get();
  }
  if (motion.policy_->inputCount() != (lora_mode_ ? 33U : 2U) || motion.policy_->outputCount() != 1)
    throw std::runtime_error("WBC RKNN model input/output count does not match selected mode");
  motion.policy_input_index_ = motion.policy_->inputIndex(motion.policy_input_name_);
  motion.command_input_index_ = motion.policy_->inputIndex(motion.command_input_name_);
  if (motion.command_input_index_ < 0)
    motion.command_input_index_ =
        motion.policy_->inputIndex(motion.command_input_name_ == "command_short" ? "command" : "command_short");
  motion.output_index_ = motion.policy_->outputIndex(motion.output_name_);
  if (motion.policy_input_index_ < 0 || motion.command_input_index_ < 0 ||
      motion.policy_input_index_ == motion.command_input_index_ || motion.output_index_ < 0)
    throw std::runtime_error("WBC RKNN input/output names do not match policy, command_short, action");
  if (motion.policy_->inputElements(motion.policy_input_index_) != kPolicyInputSize ||
      motion.policy_->inputElements(motion.command_input_index_) != kCommandInputSize ||
      motion.policy_->outputElements(motion.output_index_) != kJointCount)
    throw std::runtime_error("WBC RKNN tensor sizes do not match policy[399], command_short[240], action[21]");

  auto reference_path = wbcModelPath(joinPath(motion.config_dir_, "reference_motion.onnx"));
#ifdef USE_RKNN
  if (lora_assets_) reference_path = lora_assets_->action(motion.motion_id_).reference;
#endif
  motion.reference_model_ = std::make_unique<WbcModel>(reference_path);
  if (motion.reference_model_->inputCount() != 1 || motion.reference_model_->inputIndex("time_step") != 0 ||
      motion.reference_model_->inputElements() != 1 || motion.reference_model_->outputCount() != kReferenceOutputs.size())
    throw std::runtime_error("WBC reference RKNN must have time_step[1,1] and six reference outputs");
  for (std::size_t i = 0; i < kReferenceOutputs.size(); ++i)
  {
    const int index = motion.reference_model_->outputIndex(kReferenceOutputs[i]);
    if (index < 0 || motion.reference_model_->outputElements(index) != kReferenceOutputSizes[i])
      throw std::runtime_error(std::string("WBC reference RKNN output size/name mismatch: ") + kReferenceOutputs[i]);
    motion.reference_output_indices_[i] = index;
  }
}

bool RLWbc::captureFrame(RobotFrame& frame, std::array<double, 4>* orientation_wxyz, std::string& error,
                         int64_t* observation_stamp) {
  luvbot_control_interfaces::msg::PolicyObservation sensor;
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    const double age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - observation_received_at_).count();
    const double stamp_age = (node_->now().nanoseconds() - observation_stamp_ns_) * 1e-9;
    if (!observation_received_ || age > sensor_timeout_sec_ || stamp_age < 0.0 || stamp_age > sensor_timeout_sec_ ||
        !policy_observation_.inference_enabled)
    {
      error = "fresh, inference-enabled PolicyObservation is required";
      return false;
    }
    sensor = policy_observation_;
    if (observation_stamp) *observation_stamp = observation_stamp_ns_;
  }
  try
  {
    const std::array<double, 4> orientation = normalize(
        {{sensor.imu.orientation.w, sensor.imu.orientation.x, sensor.imu.orientation.y, sensor.imu.orientation.z}});
    const auto gravity = rotateInverse(orientation, {{0.0, 0.0, -1.0}});
    const double angular_velocity[] = {sensor.imu.angular_velocity.x, sensor.imu.angular_velocity.y,
                                       sensor.imu.angular_velocity.z};
    for (std::size_t i = 0; i < 3; ++i)
    {
      frame.angular_velocity[i] = static_cast<float>(angular_velocity[i]);
      frame.projected_gravity[i] = static_cast<float>(gravity[i]);
    }
    frame.joint_position.resize(kJointCount);
    frame.joint_velocity.resize(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      frame.joint_position[i] = static_cast<float>(sensor.joints.position[i]) - current_->default_joint_positions_[i];
      frame.joint_velocity[i] = static_cast<float>(sensor.joints.velocity[i]);
    }
    if (orientation_wxyz) *orientation_wxyz = orientation;
  } catch (const std::exception& exception)
  {
    error = exception.what();
    return false;
  }
  if (!finiteVector(frame.joint_position) || !finiteVector(frame.joint_velocity))
  {
    error = "non-finite WBC sensor input";
    return false;
  }
  return true;
}

RLWbc::ReferenceFrame RLWbc::inferReferenceFrame(int index) {
  const float time_step = static_cast<float>(std::max(0, std::min(index, current_->reference_num_frames_ - 1)));
  const auto started = std::chrono::steady_clock::now();
  ++reference_queries_;
  const bool inferred = current_->reference_model_->run(&time_step, 1);
  reference_sec_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  if (!inferred)
    throw std::runtime_error("WBC reference RKNN inference failed: " + current_->reference_model_->lastError());
  ReferenceFrame frame;
  frame.joint_position = current_->reference_model_->output(current_->reference_output_indices_[0]);
  const auto& positions = current_->reference_model_->output(current_->reference_output_indices_[2]);
  const auto& orientations = current_->reference_model_->output(current_->reference_output_indices_[3]);
  for (std::size_t i = 0; i < 3; ++i) frame.root_position[i] = positions[current_->reference_root_body_index_ * 3 + i];
  double norm_squared = 0.0;
  for (std::size_t i = 0; i < 4; ++i)
  {
    frame.root_orientation[i] = orientations[current_->reference_root_body_index_ * 4 + i];
    norm_squared += frame.root_orientation[i] * frame.root_orientation[i];
  }
  if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1.0) > 0.1)
    throw std::runtime_error("WBC reference RKNN returned an invalid root quaternion");
  frame.root_orientation = normalize(frame.root_orientation);
  return frame;
}

std::vector<float> RLWbc::makePolicyInput() const {
  std::vector<float> input;
  input.reserve(kPolicyInputSize);
  for (int field = 0; field < 4; ++field)
  {
    for (int offset : current_->history_steps_)
    {
      const RobotFrame& frame = state_history_.at(static_cast<std::size_t>(offset));
      if (field == 0)
        input.insert(input.end(), frame.angular_velocity.begin(), frame.angular_velocity.end());
      else if (field == 1)
        input.insert(input.end(), frame.projected_gravity.begin(), frame.projected_gravity.end());
      else if (field == 2)
        input.insert(input.end(), frame.joint_position.begin(), frame.joint_position.end());
      else
        input.insert(input.end(), frame.joint_velocity.begin(), frame.joint_velocity.end());
    }
  }
  for (const auto& action : action_history_) input.insert(input.end(), action.begin(), action.end());
  if (input.size() != kPolicyInputSize) throw std::runtime_error("constructed WBC policy input is not 399 elements");
  return input;
}

std::vector<float> RLWbc::makeCommandInput(const std::array<double, 4>& robot_orientation, int reference_index,
                                         bool reset_anchor) {
  std::vector<int> indices;
  std::vector<ReferenceFrame> references;
  indices.reserve(current_->future_steps_.size());
  references.reserve(current_->future_steps_.size());
  for (int offset : current_->future_steps_)
  {
    const int index = std::max(0, std::min(reference_index + offset, current_->reference_num_frames_ - 1));
    const auto previous = std::find(indices.begin(), indices.end(), index);
    // Reuse only duplicate clamped frames within this control cycle.
    if (previous == indices.end())
      references.push_back(inferReferenceFrame(index));
    else
      references.push_back(references[static_cast<std::size_t>(previous - indices.begin())]);
    indices.push_back(index);
  }
  std::vector<float> input;
  input.reserve(kCommandInputSize);
  const auto& current = references[3];  // future_steps[3] is the current reference frame.
  if (reset_anchor)
    yaw_anchor_wxyz_ = multiply(yawQuaternion(robot_orientation), conjugate(yawQuaternion(current.root_orientation)));
  const auto reference_yaw = yawQuaternion(current.root_orientation);
  // Signal-major layout: 8 positions, 8 rotation matrices' first two rows, 8 joint vectors.
  for (const auto& future : references)
  {
    const auto local = rotateInverse(reference_yaw,
        {{future.root_position[0] - current.root_position[0],
          future.root_position[1] - current.root_position[1], future.root_position[2]}});
    for (double value : local) input.push_back(static_cast<float>(value));
  }
  const auto robot_inverse = conjugate(normalize(robot_orientation));
  for (const auto& future : references)
  {
    appendRotationFirstTwoRows(multiply(multiply(robot_inverse, yaw_anchor_wxyz_), future.root_orientation), input);
  }
  for (const auto& future : references)
  {
    const auto& joints = future.joint_position;
    input.insert(input.end(), joints.begin(), joints.end());
  }
  if (input.size() != kCommandInputSize) throw std::runtime_error("constructed WBC command input is not 240 elements");
  return input;
}

bool RLWbc::inferAndPublish(double elapsed_sec, bool commit_action, std::string& error) {
  const auto inference_started = std::chrono::steady_clock::now();
  reference_sec_ = 0.0;
  policy_sec_ = 0.0;
  reference_queries_ = 0;
  int64_t input_stamp = 0;
  const bool success = [&]() {
    RobotFrame frame;
    std::array<double, 4> orientation;
    if (!captureFrame(frame, &orientation, error, &input_stamp)) return false;
    state_history_.push_front(std::move(frame));
    while (state_history_.size() > 17) state_history_.pop_back();

    try
    {
      const std::vector<float> policy_input = makePolicyInput();
      const int reference_index = std::min(current_->reference_num_frames_ - 1,
          static_cast<int>(std::max(0.0, elapsed_sec - current_->start_hold_sec_) / current_->control_dt_));
      const std::vector<float> command_input = makeCommandInput(orientation, reference_index, !commit_action);
      const auto policy_started = std::chrono::steady_clock::now();
      bool inferred;
#ifdef USE_RKNN
      if (lora_assets_) {
        inferred = shared_policy_->runResident(policy_input.data(), policy_input.size(),
                                               command_input.data(), command_input.size());
      } else
#endif
      {
        inputs_[current_->policy_input_index_] = policy_input.data();
        input_sizes_[current_->policy_input_index_] = policy_input.size();
        inputs_[current_->command_input_index_] = command_input.data();
        input_sizes_[current_->command_input_index_] = command_input.size();
        inferred = current_->policy_->run(inputs_, input_sizes_);
      }
      policy_sec_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - policy_started).count();
      if (!inferred)
      {
        error = current_->policy_->lastError();
        return false;
      }
      auto action = current_->policy_->output(current_->output_index_);
      if (action.size() != kJointCount || !finiteVector(action))
      {
        error = "WBC policy returned an invalid action";
        return false;
      }
      // Reject late results and observations invalidated during inference.
      RobotFrame latest_frame;
      if (!captureFrame(latest_frame, nullptr, error)) return false;
      const double inference_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - inference_started).count();
      const double input_age = (node_->now().nanoseconds() - input_stamp) * 1e-9;
      if (input_age < 0.0 || input_age > sensor_timeout_sec_ ||
          inference_sec > std::min(current_->control_dt_, std::min(sensor_timeout_sec_, command_validity_ms_ / 1000.0)))
      {
        error = input_age < 0.0 || input_age > sensor_timeout_sec_ ?
            "WBC input timestamp expired/invalid" : "WBC inference exceeded command validity budget";
        return false;
      }
      for (float& value : action) value = std::max(-current_->clip_actions_, std::min(value, current_->clip_actions_));
      posCmd_.target_positions.resize(kJointCount);
      for (std::size_t i = 0; i < kJointCount; ++i)
        posCmd_.target_positions[i] =
            static_cast<double>(action[i]) * current_->action_scale_[i] + current_->default_joint_positions_[i];
      publishPolicyCommand();
      if (commit_action)
      {
        action_history_.push_front(action);
        while (action_history_.size() > static_cast<std::size_t>(current_->previous_action_steps_))
          action_history_.pop_back();
      }
      return true;
    } catch (const std::exception& exception)
    {
      error = exception.what();
      return false;
    }
  }();
  const double cycle_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - inference_started).count();
  ++timing_.samples;
  if (cycle_sec > std::min(current_->control_dt_, std::min(sensor_timeout_sec_, command_validity_ms_ / 1000.0)))
    ++timing_.timeouts;
  timing_.reference_total += reference_sec_;
  timing_.reference_max = std::max(timing_.reference_max, reference_sec_);
  timing_.policy_total += policy_sec_;
  timing_.policy_max = std::max(timing_.policy_max, policy_sec_);
  timing_.cycle_total += cycle_sec;
  timing_.cycle_max = std::max(timing_.cycle_max, cycle_sec);
  const auto bucket = [](double seconds) {
    return static_cast<std::size_t>(std::min(1000.0, std::max(0.0, seconds * 10000.0)));
  };
  ++timing_.reference_hist[bucket(reference_sec_)];
  ++timing_.policy_hist[bucket(policy_sec_)];
  ++timing_.cycle_hist[bucket(cycle_sec)];
  if (!success) {
    const double budget = std::min(current_->control_dt_, std::min(sensor_timeout_sec_, command_validity_ms_ / 1000.0));
    error += "; cycle_ms=" + std::to_string(cycle_sec * 1000.0) + " budget_ms=" + std::to_string(budget * 1000.0) +
             " input_age_ms=" + (input_stamp > 0 ? std::to_string((node_->now().nanoseconds() - input_stamp) * 1e-6) : "unavailable") +
             " reference_queries=" + std::to_string(reference_queries_);
  }
  return success;
}

void RLWbc::prepareCallback(const std::shared_ptr<luvbot_control_interfaces::srv::ControlWbcMotion::Request> request,
                            std::shared_ptr<luvbot_control_interfaces::srv::ControlWbcMotion::Response> response) {
  const auto started = std::chrono::steady_clock::now();
  response->motion_id = request->motion_id;
  if (request->command == luvbot_control_interfaces::srv::ControlWbcMotion::Request::CANCEL) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++prepare_generation_;
    preparing_ = prepared_ = active_ = false;
    response->success = true;
    response->message = "WBC cancelled; in-flight loading result invalidated";
    return;
  }
  if (request->command != luvbot_control_interfaces::srv::ControlWbcMotion::Request::START) {
    response->message = "unsupported WBC command";
    return;
  }
  // The executor callback is a non-realtime loader. Observation/active callbacks remain runnable.
  // A cancelled read can finish, but a new request cannot write the same spare slot until it does.
  std::unique_lock<std::mutex> loading_lock(loading_mutex_, std::try_to_lock);
  if (!loading_lock.owns_lock()) {
    response->message = "previous adapter load is still retiring";
    return;
  }
  Motion* next = nullptr;
  int slot;
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prepared_ && rclcpp::Time(posCmd_.valid_until) <= node_->now()) prepared_ = false;
    if (active_ || prepared_ || preparing_) {
      response->message = "a WBC motion is preparing, prepared or active";
      return;
    }
    const auto motion = motions_.find(request->motion_id);
    if (motion == motions_.end()) {
      response->message = "unknown WBC motion_id";
      return;
    }
    next = motion->second.get();
    slot = 1 - active_buffer_;
    generation = ++prepare_generation_;
    preparing_ = true;
  }
  std::string error;
  const auto load_started = std::chrono::steady_clock::now();
  try {
#ifdef USE_RKNN
    if (lora_assets_) {
      lora_assets_->loadAdapter(lora_assets_->action(request->motion_id), adapter_buffers_[slot]);
      shared_policy_->stageResidentParameters(slot, lora_assets_->parameterInputs(adapter_buffers_[slot]));
    }
#endif
  } catch (const std::exception& exception) {
    error = exception.what();
  }
  const double load_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - load_started).count();
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != prepare_generation_ || !preparing_) {
    response->message = "WBC preparation was cancelled";
    return;
  }
  preparing_ = false;
  const auto elapsed = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(); };
  if (!error.empty() || elapsed() >= prepare_timeout_sec_) {
    response->message = error.empty() ? "adapter load exceeded preparation budget" : error;
    RCLCPP_WARN(node_->get_logger(), "WBC prepare %s rejected: %s", request->motion_id.c_str(), response->message.c_str());
    return;
  }
  // No inference is in flight while mutex_ is held. Never mutate the active adapter buffer.
  Motion* previous = current_;
  const int previous_slot = active_buffer_;
#ifdef USE_RKNN
  const int previous_resident_slot = shared_policy_ ? shared_policy_->residentSlot() : -1;
#endif
  // Do allocating snapshots before committing the new device bindings.
  const auto previous_command = posCmd_;
#ifdef USE_RKNN
  if (shared_policy_ && !shared_policy_->commitResidentParameters(slot)) {
    response->message = shared_policy_->lastError();
    RCLCPP_ERROR(node_->get_logger(), "WBC prepare %s rejected: %s", request->motion_id.c_str(), response->message.c_str());
    return;
  }
#endif
  // Keep the complete inactive motion state for rollback after first-frame failure.
  const auto previous_anchor = yaw_anchor_wxyz_;
  auto previous_states = std::move(state_history_);
  auto previous_actions = std::move(action_history_);
  const auto previous_timing = timing_;
  const bool previous_done = done_published_;
  current_ = next;
  active_buffer_ = slot;
  timing_ = TimingStats{};
  yaw_anchor_wxyz_ = {{1.0, 0.0, 0.0, 0.0}};
  RobotFrame initial_frame;
  try {
    if (elapsed() < prepare_timeout_sec_ && captureFrame(initial_frame, nullptr, error)) {
      state_history_.assign(17, initial_frame);
      action_history_.assign(static_cast<std::size_t>(current_->previous_action_steps_), std::vector<float>(kJointCount, 0.0f));
      done_published_ = false;
      if (inferAndPublish(0.0, false, error) && elapsed() < prepare_timeout_sec_) {
        response->robot_cfg_path = current_->robot_cfg_path_;
        response->duration_sec = current_->start_hold_sec_ + current_->reference_num_frames_ * current_->control_dt_;
        response->message = "WBC motion prepared";
        logTiming("prepare");
        if (elapsed() < prepare_timeout_sec_) {
          timing_ = TimingStats{};
          prepared_ = true;
          response->success = true;
          RCLCPP_INFO(node_->get_logger(), "WBC prepared action=%s generation=%llu slot=%d shared_context=%s %.2f ms load_pack=%.2f ms",
                      request->motion_id.c_str(), static_cast<unsigned long long>(generation), slot,
                      lora_mode_ ? "reused" : "static", elapsed() * 1000.0, load_ms);
          return;
        }
        error = "preparation deadline after diagnostics";
      }
    }
  } catch (const std::exception& exception) {
    error = exception.what();
  }
  logTiming("prepare_failed");
#ifdef USE_RKNN
  if (shared_policy_ && !shared_policy_->restoreResidentParameters(previous_resident_slot))
    error += "; " + shared_policy_->lastError();
#endif
  current_ = previous;
  active_buffer_ = previous_slot;
  yaw_anchor_wxyz_ = previous_anchor;
  state_history_ = std::move(previous_states);
  action_history_ = std::move(previous_actions);
  timing_ = previous_timing;
  done_published_ = previous_done;
  posCmd_ = previous_command;
  response->message = "WBC pre-inference failed: " + (error.empty() ? std::string("preparation deadline") : error);
  RCLCPP_WARN(node_->get_logger(), "%s", response->message.c_str());
}

void RLWbc::activeCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg->data)
  {
    ++prepare_generation_;
    preparing_ = false;
    if (active_) logTiming("deactivated");
    active_ = false;
    prepared_ = false;
    return;
  }
  // A late activation must not revive a cancelled or expired preparation.
  if (!prepared_ || rclcpp::Time(posCmd_.valid_until) <= node_->now())
  {
    prepared_ = false;
    RCLCPP_ERROR(node_->get_logger(), "Ignoring WBC activation without a fresh prepared command");
    return;
  }
  motion_start_ = std::chrono::steady_clock::now();
  active_ = true;
  prepared_ = false;
}

void RLWbc::logTiming(const char* phase) {
#ifdef USE_RKNN
  if (shared_policy_) {
    const auto stats = shared_policy_->residentStats();
    const double runs = static_cast<double>(std::max<std::uint64_t>(1, stats.runs));
    RCLCPP_INFO(node_->get_logger(),
        "WBC resident cumulative [%s]: contexts=%llu allocated_bytes=%llu layout_bytes=%llu "
        "prepares=%llu parameter_write_bytes=%llu converted_elements=%llu parameter_binds=%llu parameter_syncs=%llu "
        "commits=%llu rollbacks=%llu runs=%llu observation_syncs=%llu output_syncs=%llu "
        "pack_total_ms=%.3f commit_total_ms=%.3f; observation avg/max=%.3f/%.3f ms "
        "run_wait avg/max=%.3f/%.3f ms output avg/max=%.3f/%.3f ms",
        phase, static_cast<unsigned long long>(stats.context_initializations),
        static_cast<unsigned long long>(stats.allocated_bytes), static_cast<unsigned long long>(stats.layout_bytes),
        static_cast<unsigned long long>(stats.parameter_prepares), static_cast<unsigned long long>(stats.parameter_write_bytes),
        static_cast<unsigned long long>(stats.parameter_converted_elements),
        static_cast<unsigned long long>(stats.parameter_bind_calls), static_cast<unsigned long long>(stats.parameter_sync_calls),
        static_cast<unsigned long long>(stats.commits), static_cast<unsigned long long>(stats.rollbacks),
        static_cast<unsigned long long>(stats.runs), static_cast<unsigned long long>(stats.observation_sync_calls),
        static_cast<unsigned long long>(stats.output_sync_calls), stats.pack_ms, stats.commit_ms,
        stats.observation_ms / runs, stats.observation_max_ms, stats.execution_ms / runs, stats.execution_max_ms,
        stats.output_ms / runs, stats.output_max_ms);
    RCLCPP_INFO(node_->get_logger(),
        "WBC resident startup SDK memory: query_ret=%d weight_bytes=%llu internal_bytes=%llu dma_bytes=%llu "
        "(SDK DMA may include persistent I/O; do not add these totals)",
        stats.sdk_memory_query, static_cast<unsigned long long>(stats.sdk_weight_bytes),
        static_cast<unsigned long long>(stats.sdk_internal_bytes), static_cast<unsigned long long>(stats.sdk_dma_bytes));
  }
#endif
  if (!current_ || timing_.samples == 0) return;
  const double mean_ms = 1000.0 / static_cast<double>(timing_.samples);
  RCLCPP_INFO(node_->get_logger(),
              "WBC timing %s [%s]: samples=%llu, timeouts=%llu; "
              "reference avg/max=%.3f/%.3f ms, policy avg/max=%.3f/%.3f ms, cycle avg/max=%.3f/%.3f ms",
              current_->motion_id_.c_str(), phase,
              static_cast<unsigned long long>(timing_.samples), static_cast<unsigned long long>(timing_.timeouts),
              timing_.reference_total * mean_ms, timing_.reference_max * 1000.0,
              timing_.policy_total * mean_ms, timing_.policy_max * 1000.0,
              timing_.cycle_total * mean_ms, timing_.cycle_max * 1000.0);
  const auto percentile = [&](const std::array<uint64_t, 1001>& histogram, double fraction) {
    const auto rank = static_cast<uint64_t>(std::ceil(fraction * static_cast<double>(timing_.samples)));
    uint64_t cumulative = 0;
    for (std::size_t i = 0; i < histogram.size(); ++i) {
      cumulative += histogram[i];
      if (cumulative >= rank)
        return i == histogram.size() - 1 ? std::numeric_limits<double>::infinity() : (i + 1) * 0.1;
    }
    return std::numeric_limits<double>::infinity();
  };
  RCLCPP_INFO(node_->get_logger(),
      "WBC timing %s [%s]: histogram P95/P99 upper bounds (0.1 ms bins, inf means >=100 ms): "
      "reference=%.1f/%.1f policy=%.1f/%.1f cycle=%.1f/%.1f ms",
      current_->motion_id_.c_str(), phase,
      percentile(timing_.reference_hist, .95), percentile(timing_.reference_hist, .99),
      percentile(timing_.policy_hist, .95), percentile(timing_.policy_hist, .99),
      percentile(timing_.cycle_hist, .95), percentile(timing_.cycle_hist, .99));
}

void RLWbc::stateCallback(const luvbot_control_interfaces::msg::PolicyObservation::ConstSharedPtr msg) {
  const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  const double age = (node_->now().nanoseconds() - stamp_ns) * 1e-9;
  const bool valid = validatePolicyObservation(*msg, joint_names_, "WBC");
  std::lock_guard<std::mutex> lock(observation_mutex_);
  if (!valid || stamp_ns <= 0 || age < 0.0 || age > sensor_timeout_sec_ ||
      (observation_received_ && stamp_ns <= observation_stamp_ns_))
  {
    observation_received_ = false;
    return;
  }
  policy_observation_ = *msg;
  observation_received_at_ = std::chrono::steady_clock::now();
  observation_stamp_ns_ = stamp_ns;
  observation_received_ = true;
}

void RLWbc::publishDone(bool success, const char* reason) {
  if (done_published_) return;
  done_published_ = true;
  logTiming(reason);
  std_msgs::msg::Bool done;
  done.data = success;
  done_pub_->publish(done);
  RCLCPP_INFO(node_->get_logger(), "WBC motion '%s' stopped: %s", current_->motion_id_.c_str(), reason);
}

void RLWbc::runLoop() {
  rclcpp::WallRate rate(50.0);
  while (loop_running_.load() && rclcpp::ok())
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_)
      {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - motion_start_).count();
        const double duration = current_->start_hold_sec_ + current_->reference_num_frames_ * current_->control_dt_;
        RobotFrame current_frame;
        std::string state_error;
        if (!captureFrame(current_frame, nullptr, state_error))
        {
          active_ = false;
          publishDone(false, state_error.c_str());
        }
        else if (elapsed >= duration)
        {
          active_ = false;
          publishDone(true, "completed");
        }
        else
        {
          std::string error;
          if (!inferAndPublish(elapsed, true, error))
          {
            active_ = false;
            RCLCPP_ERROR(node_->get_logger(), "WBC inference failed: %s", error.c_str());
            publishDone(false, "runtime failure");
          }
        }
      }
    }
    rate.sleep();
  }
}
