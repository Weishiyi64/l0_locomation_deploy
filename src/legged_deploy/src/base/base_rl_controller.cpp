#include "legged_deploy/base/base_rl_controller.h"

#include <signal.h>

#include <algorithm>

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

BaseRLController::~BaseRLController() {
  // signal any running thread to stop and join
  try
  { quit.store(true); } catch (...)
  {
    // swallow exceptions in destructor
  }
}

void BaseRLController::spinUntilShutdown() {
  RCLCPP_INFO(this->nh->get_logger(), "BaseRLController spinUntilShutdown start");
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok() && !g_shutdown_request)
  {
    /* code */
    rate.sleep();
  }

  quit.store(true);
  RCLCPP_INFO(this->nh->get_logger(), "BaseRLController spinUntilShutdown exiting");
}

void BaseRLController::initBaseTopic(rclcpp::Node::SharedPtr nh_) {
  const std::string cmd_topic = rlCmdPublishTopic();

  subState_ = nh_->create_subscription<legged_msgs::msg::LowState>(
      "/rlState", rclcpp::QoS(100), std::bind(&BaseRLController::stateCallback, this, std::placeholders::_1));
  pubCmd_ov = nh_->create_publisher<legged_msgs::msg::PosCmd>(cmd_topic, rclcpp::QoS(100));
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
    // int error = 0;
    //  Read RLController configuration parameters
    // error +=
    // static_cast<int>(!nh.getParam("/RLController/set_realtime_priority",
    // out_params.set_realtime_priority)); error +=
    // static_cast<int>(!nh.getParam("/RLController/thread_scheduling_method",
    // out_params.thread_scheduling_method)); error +=
    // static_cast<int>(!nh.getParam("/RLController/thread_priority",
    // out_params.thread_priority)); error +=
    // static_cast<int>(!nh.getParam("/RLController/skip_infer_on_overrun",
    // out_params.skip_infer_on_overrun));

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
    out_params.skip_infer_on_overrun = rc["skip_infer_on_overrun"].as<bool>(false);

    // if (error > 0) {
    //     ("Failed to get all deployment parameters");
    //     return false;
    // }

    return true;
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(nh->get_logger(), "Error in readDeployParams: %s", e.what());
    return false;
  }
}
