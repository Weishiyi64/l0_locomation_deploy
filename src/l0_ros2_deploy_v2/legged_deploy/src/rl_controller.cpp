#include "rl_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iomanip>
#include <mutex>
#include <stdexcept>

#include "order_mapping.h"
// #include <XmlRpcValue.h>
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
    // Read optional parameters
    /*
    if (nh.getParam("/RobotCfg/policy_output_name", policy_output_name_)) {
        RCLCPP_INFO(nh_->get_logger(), "RL: using policy_output_name '%s'",
    policy_output_name_.c_str());
        // normalize to lower for matching convenience
        std::transform(policy_output_name_.begin(), policy_output_name_.end(),
    policy_output_name_.begin(), ::tolower);
    }
    */
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

    // Set quit flag
    quit.store(false);

    RCLCPP_INFO(nh_->get_logger(), "RL Policy initialized successfully");
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to initialize RL policy: %s", e.what());
    throw;
  }

  // Setup loop thread
  loopThread_ = std::thread([&]() {
    // ros::Rate rate(params.policy_frequency);
    rclcpp::Rate rate(params.policy_frequency);

    // while (loopRunning_ && ros::ok()) {
    while (loopRunning_ && rclcpp::ok())
    {
      update();
      rate.sleep();
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
      RCLCPP_WARN(nh_->get_logger(), "RL: Failed to set realtime priority");
    }
    else
    {
      printThreadSchedulerInfo(loopThread_.native_handle(), "RL");
    }
  }

  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m======================================================"
              "========\033[0m");
  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m======================================================"
              "========\033[0m");
  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m======================================================"
              "========\033[0m");
  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m======================================================"
              "========\033[0m");
  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m             Controller started successfully!         "
              "        \033[0m");
  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m======================================================"
              "========\033[0m");
  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m======================================================"
              "========\033[0m");
  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m======================================================"
              "========\033[0m");
  RCLCPP_INFO(nh_->get_logger(),
              "\033[1;32m======================================================"
              "========\033[0m");
}

RL::~RL() {
  loopRunning_ = false;
  if (loopThread_.joinable())
  {
    loopThread_.join();
  }

#ifdef USE_TENSORRT
  if (input_device_)
  {
    cudaFree(input_device_);
    input_device_ = nullptr;
  }
  if (output_device_)
  {
    cudaFree(output_device_);
    output_device_ = nullptr;
  }
  if (trt_context_)
  {
    delete trt_context_;
    trt_context_ = nullptr;
  }
  if (trt_engine_)
  {
    delete trt_engine_;
    trt_engine_ = nullptr;
  }
  if (trt_runtime_)
  {
    delete trt_runtime_;
    trt_runtime_ = nullptr;
  }
#endif

  RCLCPP_INFO(nh_->get_logger(), "RL destruction");
}

void RL::update() {
  // static ros::Time last_call_time = ros::Time::now();
  static rclcpp::Time last_call_time = rclcpp::Clock().now();
  static bool first_call = true;

  // const auto currentTime = ros::Time::now();
  const auto currentTime = rclcpp::Clock().now();
  // ros::Duration elapsedTime = currentTime - last_call_time;
  rclcpp::Duration elapsedTime = currentTime - last_call_time;
  last_call_time = currentTime;
  bool overrun = false;

  if (first_call)
    first_call = false;
  else
  {
    // double actual_frequency = 1.0 / elapsedTime.toSec();
    double actual_frequency = 1.0 / elapsedTime.seconds();
    double frequency_diff = std::abs(actual_frequency - params.policy_frequency);
    double frequency_threshold = params.policy_frequency * 0.2;
    double frequency_print_check = params.policy_frequency * 0.1;

    if (frequency_diff > frequency_threshold)
    {
      overrun = true;
      RCLCPP_WARN(nh_->get_logger(),
                  "[RL Frequency] Expected: %.2f Hz, Actual: %.2f Hz, "
                  "Difference: %.2f Hz",
                  static_cast<double>(params.policy_frequency), actual_frequency, frequency_diff);
    }
    else if (frequency_diff > frequency_print_check)
    {
      RCLCPP_INFO(nh_->get_logger(),
                  "\033[34m[RL Frequency] Expected: %.2f Hz, Actual: %.2f Hz, "
                  "Difference: %.2f Hz\033[0m",
                  static_cast<double>(params.policy_frequency), actual_frequency, frequency_diff);
    }
  }

  const bool current_rl_inference_enabled = rl_inference_enabled_.load();
  const bool reset_requested = obs_reset_requested_.exchange(false);
  const bool inference_started = current_rl_inference_enabled && !previous_rl_inference_enabled_;
  const bool reset_observation_state = reset_requested || inference_started;
  if (reset_observation_state)
  {
    resetObservationState();
  }
  previous_rl_inference_enabled_ = current_rl_inference_enabled;

  if (current_rl_inference_enabled)
  {
    if (!overrun || !deploy_params.skip_infer_on_overrun)
    {
      if (!updateObservation()) return;
      if (reset_observation_state)
      {
        for (auto& h : obs.hist_obs)
        {
          h = obs.observations;
        }
        RCLCPP_DEBUG(nh_->get_logger(), "[RL] Observation history reset for fresh walk start");
      }

      if (updateAction())
      {
        publishPolicyCommand();
      }
    }
    else
    {
      RCLCPP_WARN(nh_->get_logger(), "Skipping inference this cycle due to overrun by configuration");
    }
  }
}

void RL::initTopic(rclcpp::Node::SharedPtr nh_) {
  initBaseTopic(nh_);
  subObsHistoryReset_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/observation/reset", rclcpp::QoS(10),
      std::bind(&RL::obsHistoryResetCallback, this, std::placeholders::_1));
  subImitationActive_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/imitation/active", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        imitation_active_.store(msg->data);
        if (msg->data)
        {
          rl_inference_enabled_.store(false);
        }
        else
        {
          requestObservationStateReset();
          rl_inference_enabled_.store(policy_observation_.inference_enabled && !wbc_active_.load());
        }
      });
  subWbcActive_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "control/policy/wbc/active", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        wbc_active_.store(msg->data);
        if (!msg->data) requestObservationStateReset();
        rl_inference_enabled_.store(policy_observation_.inference_enabled && !imitation_active_.load() && !msg->data);
      });
  RCLCPP_INFO(nh_->get_logger(), "RL: consuming controller PolicyObservation");
}

void RL::initModel() {
  rbt_state = Robot_State(nh_->get_logger(), params.num_actions);
  rbt_output = Robot_Output(nh_->get_logger(), params.num_actions);
  obs = Observations(params.num_single_obs, params.frame_stack_indices);

#ifdef USE_TENSORRT
  trt_logger_.ros_logger = nh_->get_logger();

  // Derive .trt path (replace .onnx extension or append)
  std::string trt_path = params.policy_path;
  {
    const std::string ext = ".onnx";
    if (trt_path.size() >= ext.size() && trt_path.compare(trt_path.size() - ext.size(), ext.size(), ext) == 0)
      trt_path.replace(trt_path.size() - ext.size(), ext.size(), ".trt");
    else
      trt_path += ".trt";
  }

  try
  {
    trt_runtime_ = nvinfer1::createInferRuntime(trt_logger_);
    if (!trt_runtime_) throw std::runtime_error("Failed to create TensorRT runtime");

    std::ifstream trt_file(trt_path, std::ios::binary);
    if (!trt_file.good()) throw std::runtime_error("TRT engine file not found: " + trt_path);
    trt_file.seekg(0, std::ios::end);
    size_t trt_size = trt_file.tellg();
    trt_file.seekg(0, std::ios::beg);
    std::vector<char> trt_data(trt_size);
    trt_file.read(trt_data.data(), trt_size);
    trt_file.close();

    trt_engine_ = trt_runtime_->deserializeCudaEngine(trt_data.data(), trt_size);
    if (!trt_engine_) throw std::runtime_error("Failed to deserialize TRT engine: " + trt_path);

    trt_context_ = trt_engine_->createExecutionContext();
    if (!trt_context_) throw std::runtime_error("Failed to create TRT execution context");

    const int nb = static_cast<int>(trt_engine_->getNbIOTensors());
    input_binding_idx_ = -1;
    output_binding_idx_ = -1;
    for (int i = 0; i < nb; ++i)
    {
      const char* tensor_name = trt_engine_->getIOTensorName(i);
      const bool is_input = trt_engine_->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT;
      if (is_input)
      {
        input_binding_idx_ = i;
      }
      else
      {
        if (!policy_output_name_.empty())
        {
          std::string bname = tensor_name;
          std::transform(bname.begin(), bname.end(), bname.begin(), ::tolower);
          if (bname == policy_output_name_) output_binding_idx_ = i;
        }
        if (output_binding_idx_ == -1) output_binding_idx_ = i;
      }
    }
    if (input_binding_idx_ < 0 || output_binding_idx_ < 0)
      throw std::runtime_error("TRT: Could not resolve input/output bindings");

    const size_t input_size = static_cast<size_t>(params.num_single_obs) * params.frame_stack;
    const size_t output_size = static_cast<size_t>(params.num_actions);

    cudaError_t err = cudaMalloc(&input_device_, input_size * sizeof(float));
    if (err != cudaSuccess) throw std::runtime_error(std::string("cudaMalloc input: ") + cudaGetErrorString(err));
    err = cudaMalloc(&output_device_, output_size * sizeof(float));
    if (err != cudaSuccess)
    {
      cudaFree(input_device_);
      input_device_ = nullptr;
      throw std::runtime_error(std::string("cudaMalloc output: ") + cudaGetErrorString(err));
    }
    output_host_.resize(output_size, 0.0f);

  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to init TRT model: %s", e.what());
    throw;
  }

#elif defined(USE_RKNN)
  try
  {
    const std::string rknn_path = rknnModelPath(params.policy_path);
    rknn_policy_ = std::make_unique<RknnModel>(rknn_path);
    const size_t input_size = static_cast<size_t>(params.num_single_obs) * params.frame_stack;
    const size_t output_size = static_cast<size_t>(params.num_actions);
    if (rknn_policy_->inputElements() != input_size)
      throw std::runtime_error(
          "RKNN policy input size mismatch: model=" + std::to_string(rknn_policy_->inputElements()) +
          ", config=" + std::to_string(input_size));
    rknn_policy_output_idx_ = rknn_policy_->outputIndex(policy_output_name_);
    if (rknn_policy_output_idx_ < 0) throw std::runtime_error("RKNN policy output not found: " + policy_output_name_);
    if (rknn_policy_->outputElements(static_cast<size_t>(rknn_policy_output_idx_)) != output_size)
      throw std::runtime_error("RKNN policy output size mismatch: model=" +
                               std::to_string(rknn_policy_->outputElements(rknn_policy_output_idx_)) +
                               ", config=" + std::to_string(output_size));
    RCLCPP_INFO(nh_->get_logger(), "RKNN: Loaded RK3588 model from %s", rknn_path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to init RKNN model: %s", e.what());
    throw;
  }

#elif defined(USE_OPENVINO)
  try
  {
    RCLCPP_INFO(nh_->get_logger(), "OpenVINO: Loading model from %s", params.policy_path.c_str());
    ov_compiled_model_ = ov_core_.compile_model(params.policy_path, "CPU");
    ov_infer_request_ = ov_compiled_model_.create_infer_request();
    RCLCPP_INFO(nh_->get_logger(), "OpenVINO: Model initialized successfully");
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to init OpenVINO model: %s", e.what());
    throw;
  }
#endif
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
    // RL-specific params
    /*
    int error = 0;
    error += static_cast<int>(!nh.getParam("/RobotCfg/joint_order_mj",
    params.joint_order_mj)); error +=
    static_cast<int>(!nh.getParam("/RobotCfg/joint_order_isaac",
    params.joint_order_isaac)); error +=
    static_cast<int>(!nh.getParam("/RobotCfg/action_scale",
    params.action_scale)); if (error > 0) { RCLCPP_ERROR(nh_->get_logger(), "RL:
    Failed to get required parameters from RobotCfg"); throw
    std::runtime_error("RL: Failed to get required parameters from RobotCfg");
    }
    */

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

    RCLCPP_INFO(nh_->get_logger(), "RL: read action_scale for %zu joints", params.action_scale.size());
    RCLCPP_INFO(nh_->get_logger(), "RL: read joint_order_mj with %zu joints", params.joint_order_mj.size());
    RCLCPP_INFO(nh_->get_logger(), "RL: read joint_order_isaac with %zu joints", params.joint_order_isaac.size());

    return true;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh_->get_logger(), "RL: error in the config: %s", e.what());
    return false;
  }
}

void RL::stateCallback(const luvbot_control_interfaces::msg::PolicyObservation::ConstSharedPtr msg) {
  if (!validatePolicyObservation(*msg, params.joint_order_mj, "RL")) return;
  policy_observation_ = *msg;
  {
    std::lock_guard<std::mutex> lock(policy_command_mutex_);
    policy_command_ << msg->velocity_command.linear.x, msg->velocity_command.linear.y, msg->velocity_command.angular.z;
  }
  rl_inference_enabled_.store(msg->inference_enabled && !imitation_active_.load() && !wbc_active_.load());
}

bool RL::updateObservation() {
  // Build MJ-ordered vectors from hardware state
  Eigen::VectorXd mj_q(params.num_actions);
  Eigen::VectorXd mj_dq(params.num_actions);

  for (int mj_i = 0; mj_i < params.num_actions; ++mj_i)
  {
    mj_q[mj_i] = policy_observation_.joints.position[mj_i] - params.default_joint_angles[mj_i];
    mj_dq[mj_i] = policy_observation_.joints.velocity[mj_i];
  }

  // Transform to Isaac ordering
  order_mapper_.transformMjToIsaac(mj_q, rbt_state.q);
  order_mapper_.transformMjToIsaac(mj_dq, rbt_state.dq);
  // 在这里给rbt_state.q的一些数据强制写0
  rbt_state.base_ang_vel << policy_observation_.imu.angular_velocity.x, policy_observation_.imu.angular_velocity.y,
      policy_observation_.imu.angular_velocity.z;
  rbt_state.quat.x() = policy_observation_.imu.orientation.x;
  rbt_state.quat.y() = policy_observation_.imu.orientation.y;
  rbt_state.quat.z() = policy_observation_.imu.orientation.z;
  rbt_state.quat.w() = policy_observation_.imu.orientation.w;

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
    RCLCPP_DEBUG(nh_->get_logger(), "RL: captured initial yaw %.6f rad for quat observation", initial_yaw_);
  }
  const Eigen::Quaterniond initial_yaw_quat_inv = quat_from_yaw(initial_yaw_).conjugate();
  rbt_state.quat_obs = initial_yaw_quat_inv * rbt_state.quat;
  rbt_state.quat_obs.normalize();

  {
    std::lock_guard<std::mutex> lock(policy_command_mutex_);
    rbt_state.policy_command = policy_command_;
  }

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
  initial_yaw_ = 0.0;
  initial_yaw_valid_ = false;

  if (rbt_output.target_q.size() > 0)
  {
    rbt_output.target_q.setZero();
  }
  if (rbt_output.action.size() > 0)
  {
    rbt_output.action.setZero();
  }
}

void RL::requestObservationStateReset() { obs_reset_requested_.store(true); }

void RL::obsHistoryResetCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (!msg->data) return;
  requestObservationStateReset();
}

bool RL::updateAction() {
  static std::mutex inference_mutex;
  std::lock_guard<std::mutex> lock(inference_mutex);

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

  // -- backend-specific inference --
  const float* raw_output = nullptr;
  std::vector<float> ov_out_buf;  // used only in OV branch

#ifdef USE_TENSORRT
  cudaError_t err =
      cudaMemcpy(input_device_, obs.input_flat.data(), input_size * sizeof(float), cudaMemcpyHostToDevice);
  if (err != cudaSuccess)
  {
    RCLCPP_ERROR(nh_->get_logger(), "TRT: H2D failed: %s", cudaGetErrorString(err));
    return false;
  }
  {
    const char* input_name = trt_engine_->getIOTensorName(input_binding_idx_);
    const char* output_name = trt_engine_->getIOTensorName(output_binding_idx_);
    if (!trt_context_->setTensorAddress(input_name, input_device_) ||
        !trt_context_->setTensorAddress(output_name, output_device_))
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT: setTensorAddress failed");
      return false;
    }
    if (!trt_context_->enqueueV3(0))
    {
      RCLCPP_ERROR(nh_->get_logger(), "TRT: enqueueV3 failed");
      return false;
    }
  }
  err = cudaMemcpy(output_host_.data(), output_device_, output_size * sizeof(float), cudaMemcpyDeviceToHost);
  if (err != cudaSuccess)
  {
    RCLCPP_ERROR(nh_->get_logger(), "TRT: D2H failed: %s", cudaGetErrorString(err));
    return false;
  }
  raw_output = output_host_.data();

#elif defined(USE_RKNN)
  if (!rknn_policy_->run(obs.input_flat.data(), input_size))
  {
    RCLCPP_ERROR(nh_->get_logger(), "RKNN policy inference failed: %s", rknn_policy_->lastError().c_str());
    return false;
  }
  raw_output = rknn_policy_->output(static_cast<size_t>(rknn_policy_output_idx_)).data();

#elif defined(USE_OPENVINO)
  {
    auto in_tensor = ov_infer_request_.get_input_tensor();
    std::copy(obs.input_flat.begin(), obs.input_flat.end(), in_tensor.data<float>());
  }
  ov_infer_request_.infer();
  {
    ov::Tensor out_tensor;
    if (!policy_output_name_.empty())
    {
      try
      { out_tensor = ov_infer_request_.get_tensor(policy_output_name_); } catch (...)
      { out_tensor = ov_infer_request_.get_output_tensor(); }
    }
    else
    {
      out_tensor = ov_infer_request_.get_output_tensor();
    }
    const float* p = out_tensor.data<const float>();
    ov_out_buf.assign(p, p + output_size);
  }
  raw_output = ov_out_buf.data();
#endif

  if (!raw_output) return false;

  // Read Isaac-ordered actions
  for (int i = 0; i < params.num_actions; ++i)
    rbt_output.action[i] = std::clamp(static_cast<double>(raw_output[i]), -params.clip_actions, params.clip_actions);

  // Transform to MJ ordering and apply scaling
  Eigen::VectorXd mj_actions;
  order_mapper_.transformIsaacToMj(rbt_output.action, mj_actions);
  for (int mj_i = 0; mj_i < params.num_actions; ++mj_i)
  {
    rbt_output.target_q[mj_i] = mj_actions[mj_i] * params.action_scale[mj_i] + params.default_joint_angles[mj_i];
    posCmd_.target_positions[mj_i] = rbt_output.target_q[mj_i];
  }
  return true;
}

Eigen::Vector3d RL::qr_ty_i(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
  double q_w = q[3];
  Eigen::Vector3d q_vec = q.head<3>();
  Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
  Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
  Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);
  return (a - b + c);
}
