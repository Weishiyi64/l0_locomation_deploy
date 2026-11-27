#include "base/base_rl_controller.h"

#include <signal.h>
#ifdef __linux__
#include <sys/resource.h>
#endif

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iterator>

namespace {
bool validateObservationItemConfig(const rclcpp::Logger& logger, const ObservationItem& item) {
  if (item.dim <= 0)
  {
    RCLCPP_ERROR(logger, "observation_config item '%s' has invalid dim=%d, dim must be positive", item.type.c_str(),
                 item.dim);
    return false;
  }

  if (item.type == "quat" && item.dim != 4)
  {
    RCLCPP_ERROR(logger, "observation_config item 'quat' must use dim=4, but got dim=%d", item.dim);
    return false;
  }

  if (item.type == "gravity_proj" && item.dim != 3)
  {
    RCLCPP_ERROR(logger, "observation_config item 'gravity_proj' must use dim=3, but got dim=%d", item.dim);
    return false;
  }

  return true;
}

bool parseFrameStackConfig(const rclcpp::Logger& logger, const YAML::Node& frame_stack_config,
                           RLBaseModelParams& out_params) {
  if (!frame_stack_config)
  {
    RCLCPP_ERROR(logger, "Failed to load RobotCfg.frame_stack");
    return false;
  }

  out_params.frame_stack_indices.clear();

  if (frame_stack_config.IsScalar())
  {
    out_params.frame_stack = frame_stack_config.as<int>(0);
    if (out_params.frame_stack <= 0)
    {
      RCLCPP_ERROR(logger, "frame_stack must be positive when configured as an integer, got %d",
                   out_params.frame_stack);
      return false;
    }
    out_params.frame_stack_indices.reserve(out_params.frame_stack);
    for (int i = out_params.frame_stack - 1; i >= 0; --i)
    {
      out_params.frame_stack_indices.push_back(-i);
    }
  }
  else if (frame_stack_config.IsSequence())
  {
    if (frame_stack_config.size() == 0)
    {
      RCLCPP_ERROR(logger, "frame_stack list must not be empty");
      return false;
    }
    out_params.frame_stack_indices.reserve(frame_stack_config.size());
    for (std::size_t i = 0; i < frame_stack_config.size(); ++i)
    {
      const int frame_idx = frame_stack_config[i].as<int>(0);
      if (frame_idx > 0)
      {
        RCLCPP_ERROR(logger, "frame_stack list only supports current/past frames. Invalid index: %d", frame_idx);
        return false;
      }
      out_params.frame_stack_indices.push_back(frame_idx);
    }
    out_params.frame_stack = static_cast<int>(out_params.frame_stack_indices.size());
  }
  else
  {
    RCLCPP_ERROR(logger, "frame_stack must be configured as an integer or integer list");
    return false;
  }

  out_params.frame_history_length = 0;
  for (int frame_idx : out_params.frame_stack_indices)
  {
    out_params.frame_history_length = std::max(out_params.frame_history_length, 1 - frame_idx);
  }

  return true;
}
}  // namespace

void BaseRLController::spinUntilShutdown() {
  rclcpp::WallRate rate(10.0);
  while (rclcpp::ok() && !g_shutdown_request)
  {
    rate.sleep();
  }
}

void BaseRLController::finishPolicyCycle(SteadyClock::time_point started, int frequency) {
  const auto finished = SteadyClock::now();
  const double period_ms = 1000.0 / frequency;
  const double work_ms = std::chrono::duration<double, std::milli>(finished - started).count();
  auto& timing = cycle_timing_;
  if (timing.frequency != frequency)
  {
    timing = CycleTiming{};
    timing.frequency = frequency;
    timing.window_start = started;
  }
  const double interval_ms = timing.previous_start == SteadyClock::time_point{} ? 0.0 :
      std::chrono::duration<double, std::milli>(started - timing.previous_start).count();
  timing.previous_start = started;
  ++timing.samples;
  if (work_ms > period_ms || interval_ms > period_ms * 1.2) ++timing.late;
  timing.interval_max_ms = std::max(timing.interval_max_ms, interval_ms);
  timing.work_max_ms = std::max(timing.work_max_ms, work_ms);
  if (finished - timing.window_start >= std::chrono::seconds(5))
  {
    if (timing.late > 0)
      RCLCPP_WARN(nh->get_logger(), "[Policy Timing] target=%d Hz samples=%llu late=%llu "
                   "interval_max=%.3f ms work_max=%.3f ms; missed cycles are not replayed",
                   frequency, static_cast<unsigned long long>(timing.samples),
                   static_cast<unsigned long long>(timing.late), timing.interval_max_ms, timing.work_max_ms);
    timing.window_start = finished;
    timing.samples = timing.late = 0;
    timing.interval_max_ms = timing.work_max_ms = 0.0;
  }
  // Anchor each deadline to the actual start, so a late wakeup never causes catch-up cycles.
  std::this_thread::sleep_until(started +
      std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(1.0 / frequency)));
}

bool BaseRLController::isPolicyInputFresh(const robot_interfaces::msg::PolicyObservation& observation,
                                         SteadyClock::time_point received_at) const {
  const auto now = nh->get_clock()->now();
  const rclcpp::Time stamp(observation.header.stamp, now.get_clock_type());
  const double stamp_age_ms = (now - stamp).seconds() * 1000.0;
  const double receive_age_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - received_at).count();
  if (!observation.inference_enabled || received_at == SteadyClock::time_point{} || stamp.nanoseconds() <= 0 ||
      stamp_age_ms < 0.0 || stamp_age_ms >= command_validity_ms_ || receive_age_ms >= command_validity_ms_)
  {
    RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000,
                         "Policy observation is disabled or stale: stamp_age=%.3f ms receive_age=%.3f ms",
                         stamp_age_ms, receive_age_ms);
    return false;
  }
  return true;
}

bool BaseRLController::isPolicyResultTimely(SteadyClock::time_point started, int frequency) const {
  const double elapsed_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count();
  const double budget_ms = std::min(1000.0 / frequency, command_validity_ms_);
  if (elapsed_ms > budget_ms)
  {
    RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000,
                         "Discarding late policy result: work=%.3f ms budget=%.3f ms", elapsed_ms, budget_ms);
    return false;
  }
  return true;
}

void BaseRLController::configureThreadScheduler(pthread_t thread, const LeggedDeployParams& params,
                                                const char* thread_name) {
#ifdef __linux__
  const int name_result = pthread_setname_np(thread, thread_name);
  if (name_result != 0)
    RCLCPP_WARN(nh->get_logger(), "%s: failed to name loop thread: %s (%d)", thread_name,
                std::strerror(name_result), name_result);
#endif

  if (params.set_realtime_priority)
  {
    sched_param scheduler{};
    scheduler.sched_priority = params.thread_priority;
    const int policy = getSchedulerPolicy(params.thread_scheduling_method);
    // pthread functions return the error number directly, rather than setting errno.
    const int result = pthread_setschedparam(thread, policy, &scheduler);
    if (result != 0)
    {
      RCLCPP_ERROR(nh->get_logger(), "%s: failed to set scheduler=%s priority=%d: %s (%d); "
                   "continuing with the existing scheduler", thread_name,
                   params.thread_scheduling_method.c_str(), params.thread_priority, std::strerror(result), result);
#ifdef __linux__
      if (result == EPERM)
      {
        struct rlimit limit{};
        if (getrlimit(RLIMIT_RTPRIO, &limit) == 0)
        {
          const std::string soft = limit.rlim_cur == RLIM_INFINITY ? "unlimited" : std::to_string(limit.rlim_cur);
          const std::string hard = limit.rlim_max == RLIM_INFINITY ? "unlimited" : std::to_string(limit.rlim_max);
          RCLCPP_ERROR(nh->get_logger(), "%s: RLIMIT_RTPRIO soft=%s hard=%s; check the launching session's "
                       "rtprio limit or CAP_SYS_NICE. Updated login limits require a new login and node restart.",
                       thread_name, soft.c_str(), hard.c_str());
        }
      }
#endif
    }
  }
  else
  {
    RCLCPP_INFO(nh->get_logger(), "%s: realtime scheduling disabled by configuration", thread_name);
  }
  printThreadSchedulerInfo(thread, thread_name);
}

void BaseRLController::initBaseTopic(rclcpp::Node::SharedPtr nh_) {
  const std::string cmd_topic = rlCmdPublishTopic();

  if (!nh_->has_parameter("policy_command_validity_ms"))
    nh_->declare_parameter("policy_command_validity_ms", command_validity_ms_);
  nh_->get_parameter("policy_command_validity_ms", command_validity_ms_);
  if (!std::isfinite(command_validity_ms_) || command_validity_ms_ <= 0.0)
    throw std::runtime_error("policy_command_validity_ms must be finite and positive");
  subState_ = nh_->create_subscription<robot_interfaces::msg::PolicyObservation>(
      "control/policy/observation", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&BaseRLController::stateCallback, this, std::placeholders::_1));
  pubCmd_ov = nh_->create_publisher<robot_interfaces::msg::PolicyCommand>(
      cmd_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  posCmd_.source_id = cmd_topic.find("imitation") == std::string::npos ? "rl" : "imitation";
}

bool BaseRLController::publishPolicyCommand(const rclcpp::Time& input_time) {
  const auto now = nh->get_clock()->now();
  const auto valid_until = input_time + rclcpp::Duration::from_seconds(command_validity_ms_ / 1000.0);
  if (input_time > now || valid_until <= now) return false;
  posCmd_.header.stamp = now;
  posCmd_.sequence_id = command_sequence_id_++;
  // Inference and queueing consume the input's validity budget; publishing must not renew it.
  posCmd_.valid_until = valid_until;
  pubCmd_ov->publish(posCmd_);
  return true;
}

bool BaseRLController::validatePolicyObservation(const robot_interfaces::msg::PolicyObservation& message,
                                                 const std::vector<std::string>& expected_joint_names,
                                                 const char* consumer_name) const {
  const auto& joints = message.joints;
  if (message.header.frame_id.empty() || joints.name != expected_joint_names || joints.name.empty() ||
      joints.name.size() > 64U || joints.position.size() != joints.name.size() ||
      joints.velocity.size() != joints.name.size() || joints.effort.size() != joints.name.size())
  {
    RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000,
                         "%s: rejected observation with invalid frame or joint arrays", consumer_name);
    return false;
  }

  const auto finite_range = [](const auto& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
  };
  const auto valid_quaternion = [](const geometry_msgs::msg::Quaternion& q) {
    const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(norm_squared) && norm_squared > 1e-12 && std::abs(norm_squared - 1.0) <= 0.1;
  };
  const double scalar_values[] = {
      message.base_pose.position.x,       message.base_pose.position.y,       message.base_pose.position.z,
      message.imu.angular_velocity.x,     message.imu.angular_velocity.y,     message.imu.angular_velocity.z,
      message.velocity_command.linear.x,  message.velocity_command.linear.y,  message.velocity_command.linear.z,
      message.velocity_command.angular.x, message.velocity_command.angular.y, message.velocity_command.angular.z,
      message.behavior_up_down,           message.behavior_left_right};
  const bool finite_scalars = std::all_of(std::begin(scalar_values), std::end(scalar_values),
                                          [](double value) { return std::isfinite(value); });
  if (!finite_range(joints.position) || !finite_range(joints.velocity) || !finite_range(joints.effort) ||
      !finite_scalars || !valid_quaternion(message.base_pose.orientation) || !valid_quaternion(message.imu.orientation))
  {
    RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000,
                         "%s: rejected observation with non-finite values or invalid quaternion", consumer_name);
    return false;
  }
  return true;
}

void requireNode(const YAML::Node& n, const std::string& key_path) {
  if (!n) throw std::runtime_error("Missing required YAML node: " + key_path);
}

std::vector<double> asDoubleList(const YAML::Node& n, const std::string& path) {
  requireNode(n, path);
  std::vector<double> out;
  for (const auto& it : n) out.push_back(it.as<double>());
  return out;
}

std::vector<std::string> asStringList(const YAML::Node& n, const std::string& path) {
  requireNode(n, path);
  std::vector<std::string> out;
  for (const auto& it : n) out.push_back(it.as<std::string>());
  return out;
}

bool BaseRLController::loadRobotCfgFromYaml(const std::string& yaml_path, RLBaseModelParams& out_params,
                                            std::vector<ObservationItem>& out_obs_items) {
  try
  {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");

    out_params.policy_type = rc["policy_type"].as<std::string>("");
    out_params.num_actions = rc["num_actions"].as<int>(0);
    out_params.policy_name = rc["policy_name"].as<std::string>("");
    out_params.policy_frequency = rc["policy_frequency"].as<int>(0);
    if (out_params.policy_frequency <= 0)
      throw std::runtime_error("RobotCfg.policy_frequency must be positive");
    out_params.lowlevel_frequency = rc["lowlevel_frequency"].as<int>(0);
    out_params.gait_frequency = rc["gait_frequency"].as<double>(0);
    if (out_params.gait_frequency == 0.0 && rc["gait_config"] && rc["gait_config"]["params"])
    {
      out_params.gait_frequency = rc["gait_config"]["params"]["gait_frequency"].as<double>(0);
    }
    out_params.num_single_obs = rc["num_single_obs"].as<int>(0);
    if (!parseFrameStackConfig(nh->get_logger(), rc["frame_stack"], out_params))
    {
      return false;
    }

    out_params.clip_obs = rc["clip_obs"].as<double>(0.0);
    out_params.clip_actions = rc["clip_actions"].as<double>(0.0);
    out_params.default_joint_angles = asDoubleList(rc["default_joint_angles"], "RobotCfg.default_joint_angles");

    if (rc["observation_config"] && rc["observation_config"]["items"])
    {
      out_obs_items.clear();
      for (const auto& item : rc["observation_config"]["items"])
      {
        std::string item_type = item["type"].as<std::string>();
        int item_dim = item["dim"].as<int>();
        double item_scale = item["scale"].as<double>(1.0);
        ObservationItem obs_item(item_type, item_dim, item_scale);
        if (!validateObservationItemConfig(nh->get_logger(), obs_item))
        {
          return false;
        }
        out_obs_items.emplace_back(std::move(obs_item));
      }
    }
    else
    {
      RCLCPP_ERROR(nh->get_logger(), "RobotCfg.observation_config.items is missing");
      return false;
    }
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh->get_logger(), "Failed to load RobotCfg YAML '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  return true;
}

bool BaseRLController::readCommonParams(RLBaseModelParams& out_params, std::vector<ObservationItem>& out_obs_items) {
  try
  {
    int error = 0;

    error += static_cast<int>(!nh->get_parameter("robot_type", out_params.robot_type));
    error += static_cast<int>(!nh->get_parameter("subpolicy", out_params.subpolicy));
    std::string yaml_path = "";
    error += static_cast<int>(!nh->get_parameter("robot_cfg_path", yaml_path));
    if (!loadRobotCfgFromYaml(yaml_path, out_params, out_obs_items))
    {
      error++;
    }

    if (error > 0)
    {
      RCLCPP_ERROR(nh->get_logger(), "Failed to get all common parameters");
      return false;
    }
    return true;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh->get_logger(), "Error in readCommonParams: %s", e.what());
    return false;
  }
}

bool BaseRLController::readDeployParams(LeggedDeployParams& out_params) {
  try
  {
    std::string yaml_path = "";
    bool get_path_error = nh->get_parameter("rl_controller_config_path", yaml_path);
    if (get_path_error == false)
    {
      RCLCPP_ERROR(nh->get_logger(), "readDeployParams, rl_controller_config_path get error");
      return false;
    }
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node rc = root["RLController"];
    requireNode(rc, "RLController");

    out_params.set_realtime_priority = rc["set_realtime_priority"].as<bool>(false);
    out_params.thread_scheduling_method = rc["thread_scheduling_method"].as<std::string>("");
    out_params.thread_priority = rc["thread_priority"].as<int>(0);

    RCLCPP_INFO(nh->get_logger(), "RL deployment config: %s; set_realtime_priority=%s scheduler=%s priority=%d",
                yaml_path.c_str(), out_params.set_realtime_priority ? "true" : "false",
                out_params.thread_scheduling_method.c_str(), out_params.thread_priority);
    if (out_params.set_realtime_priority)
    {
      const int policy = getSchedulerPolicy(out_params.thread_scheduling_method);
      if (getSchedulerPolicyName(policy) != out_params.thread_scheduling_method)
        throw std::runtime_error("Unknown thread_scheduling_method: " + out_params.thread_scheduling_method);
      const int min_priority = sched_get_priority_min(policy);
      const int max_priority = sched_get_priority_max(policy);
      if (min_priority < 0 || max_priority < 0 || out_params.thread_priority < min_priority ||
          out_params.thread_priority > max_priority)
        throw std::runtime_error("Invalid thread_priority=" + std::to_string(out_params.thread_priority) +
                                 " for " + out_params.thread_scheduling_method);
    }

    return true;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh->get_logger(), "Error in readDeployParams: %s", e.what());
    return false;
  }
}
