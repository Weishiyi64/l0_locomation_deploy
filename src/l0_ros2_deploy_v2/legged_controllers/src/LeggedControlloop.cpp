// Copyright information
// Copyright (c) 2026, yuan (Original author)
// All rights reserved.

#include "legged_controllers/LeggedControlloop.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>

#include <cstring>

#include "legged_controllers/LeggedController.h"

namespace legged {

// =============================================================================
// 构造函数：仅执行一次性初始化，不包含循环逻辑
// =============================================================================

LeggedControlloop::LeggedControlloop(rclcpp::Node::SharedPtr nh, std::shared_ptr<IHardwareLayer> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  // ---------- 一次性初始化 1：控制器创建与启动 ----------
  controller_ = std::make_shared<LeggedController>(nh_, hardwareInterface_);
  if (!controller_->init(nh_))
  {
    throw std::runtime_error("Failed to initialize LeggedController");
  }
  controller_->starting(rclcpp::Clock().now());
  RCLCPP_INFO(nh_->get_logger(), "LeggedController initialized and started successfully");

  // ---------- 一次性初始化 2：从 ROS 参数获取配置文件路径 ----------
  std::string robot_cfg_path;
  std::string legged_controller_config_path;
  if (!nh_->get_parameter("legged_controller_config_path", legged_controller_config_path))
  {
    throw std::runtime_error("get params[legged_controller_config_path] error");
  }
  RCLCPP_INFO_STREAM(nh_->get_logger(), "get params[legged_controller_config_path]:" << legged_controller_config_path);
  if (legged_controller_config_path.empty())
  {
    throw std::runtime_error("get params[legged_controller_config_path] is empty");
  }

  if (!nh_->get_parameter("robot_cfg_path", robot_cfg_path))
  {
    throw std::runtime_error("get params[robot_cfg_path] error");
  }
  RCLCPP_INFO_STREAM(nh_->get_logger(), "get params[robot_cfg_path]:" << robot_cfg_path);
  if (robot_cfg_path.empty())
  {
    throw std::runtime_error("get params[robot_cfg_path] is empty");
  }

  // ---------- 一次性初始化 3：从 YAML 解析机器人配置（如控制频率） ----------
  int error = 0;
  YAML::Node root = YAML::LoadFile(robot_cfg_path);
  YAML::Node rc = root["RobotCfg"];
  requireNode(rc, "RobotCfg");
  lowlevel_frequency = rc["lowlevel_frequency"].as<double>(-1.0);
  if (lowlevel_frequency < 0)
  {
    error += 1;
  }

  // ---------- 一次性初始化 4：从 YAML 解析控制器配置（线程调度等） ----------
  YAML::Node root1 = YAML::LoadFile(legged_controller_config_path);
  YAML::Node rc1 = root1["LeggedController"];
  requireNode(rc1, "LeggedController");
  threadPriority_ = rc1["thread_priority"].as<int>(-1);
  if (threadPriority_ < 0)
  {
    error += 1;
  }
  threadSchedulingMethod_ = rc1["thread_scheduling_method"].as<std::string>("");
  if (threadSchedulingMethod_.empty())
  {
    error += 1;
  }
  skipControlOnOverrun_ = rc1["skip_control_on_overrun"].as<bool>(false);
  setRealtimePriority_ = rc1["set_realtime_priority"].as<bool>(false);

  if (error > 0)
  {
    std::string error_message = "could not retrieve the required parameters.";
    RCLCPP_ERROR_STREAM(nh_->get_logger(), error_message);
    throw std::runtime_error(error_message);
  }

  // ---------- 一次性：仅「启动」控制循环线程 ----------
  // 下面 lambda 内 while 循环才是每周期执行：update() -> rate.sleep()
  loopThread_ = std::thread([&]() {
    rclcpp::Rate rate(lowlevel_frequency);
    while (loopRunning_ && rclcpp::ok())
    {
      update();
      rate.sleep();
    }
  });

  // ---------- 一次性初始化 6：可选地设置实时调度与优先级 ----------
  if (setRealtimePriority_)
  {
    sched_param sched{};
    sched.sched_priority = threadPriority_;
    int policy = getSchedulerPolicy(threadSchedulingMethod_);
    if (pthread_setschedparam(loopThread_.native_handle(), policy, &sched) != 0)
    {
      RCLCPP_WARN(nh_->get_logger(), "LeggedControlloop: Failed to set realtime priority");
    }
    else
    {
      printThreadSchedulerInfo(loopThread_.native_handle(), "LeggedControlloop");
    }
  }
}

// =============================================================================
// 析构：停止循环并等待线程结束（一次性收尾）
// =============================================================================

LeggedControlloop::~LeggedControlloop() {
  loopRunning_ = false;
  if (loopThread_.joinable())
  {
    loopThread_.join();
  }
}

// =============================================================================
// 控制循环体：由 loopThread_ 按 lowlevel_frequency 周期调用
// 顺序：read -> update -> write，同一线程内顺序执行故 cmd 无需额外加锁
// =============================================================================

void LeggedControlloop::update() {
  static rclcpp::Time last_call_time = rclcpp::Clock().now();

  const auto currentTime = rclcpp::Clock().now();
  rclcpp::Duration elapsedTime = currentTime - last_call_time;
  last_call_time = currentTime;
  bool overrun = false;

  try
  {
    // --- 循环步骤 1：从硬件读取状态 ---
    hardwareInterface_->read(rclcpp::Clock().now(), elapsedTime);

    // --- 循环步骤 2 & 3：控制律更新 + 写回硬件（超时时可按配置跳过） ---
    if (!overrun || !skipControlOnOverrun_)
    {
      controller_->update(rclcpp::Clock().now(), elapsedTime);
      hardwareInterface_->write(rclcpp::Clock().now(), elapsedTime);
    }
    else
    {
      RCLCPP_WARN(nh_->get_logger(), "Skipping update & write this cycle due to overrun by configuration");
    }
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                 "\033[1;31mLeggedControlloop::update exception: " << e.what() << "\033[0m");
  } catch (...)
  {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                          "\033[1;31mLeggedControlloop::update unknown exception\033[0m");
  }
}

}  // namespace legged
