#include <signal.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <thread>
#include <iostream>

#include "base/rl_controller_registrar.h"
#ifdef USE_RKNN
#include "wbc_io_probe.h"
#endif

volatile sig_atomic_t g_shutdown_request = 0;

void signalHandler(int /* sig */) { g_shutdown_request = 1; }

static void requireNode(const YAML::Node& n, const std::string& key_path) {
  if (!n) throw std::runtime_error("Missing required YAML node: " + key_path);
}

int main(int argc, char** argv) {
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  if (argc > 1 && std::string(argv[1]) == "--wbc-io-probe") {
#ifdef USE_RKNN
    return runWbcIoProbe(argc, argv, g_shutdown_request);
#else
    std::cerr << "--wbc-io-probe requires the RKNN build of rl_controller\n";
    return 2;
#endif
  }
  /*
  ros::init(argc, argv, "rl_controller");
  if (!ros::ok()) {
      ("Failed to initialize ROS");
      return -1;
  }
  ros::NodeHandle nh;
  */
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<rclcpp::Node>("rl_controller", rclcpp::NodeOptions()
                                                          // 允许从 launch / yaml 声明外的参数进来（否则 get_parameter
                                                          // 会拿不到）
                                                          .allow_undeclared_parameters(true)
                                                          .automatically_declare_parameters_from_overrides(true));

  auto logger = node->get_logger();

  // Print available controllers registered in ControllerFactory
  {
    auto names = RLControllerFactory::instance().registeredControllers();
    if (names.empty())
    {
      // ("No policies registered in RLControllerFactory");
      RCLCPP_WARN(logger, "No policies registered in RLControllerFactory");
    }
    else
    {
      // ("Available policies (%zu):", names.size());
      RCLCPP_INFO(logger, "Available policies (%zu):", names.size());
      for (const auto& n : names)
      {
        // ("  - %s", n.c_str());
        RCLCPP_INFO(logger, "  - %s", n.c_str());
      }
    }
  }

  try
  {
    std::string robot_type;
    // if (!nh.getParam("/robot_type", robot_type)) {
    if (!node->get_parameter("robot_type", robot_type))
    {
      throw std::runtime_error("Required parameter '/robot_type' not found");
    }

    std::string policy_type("");
    // ros2
    // path需要从launch中读取出来
    std::string robot_cfg_path("");
    if (!node->get_parameter("robot_cfg_path", robot_cfg_path))
    {
      throw std::runtime_error("Required parameter 'robot_cfg_path' not found");
    }
    YAML::Node root = YAML::LoadFile(robot_cfg_path);
    YAML::Node rc = root["RobotCfg"];
    requireNode(rc, "RobotCfg");
    policy_type = rc["policy_type"].as<std::string>("");
    std::string policy_type_override;
    if (node->get_parameter("policy_type", policy_type_override) && !policy_type_override.empty())
    {
      policy_type = policy_type_override;
      RCLCPP_INFO(logger, "policy_type overridden from parameter: '%s'", policy_type.c_str());
    }

    // ROS1 AsyncSpinner(0) == 多线程回调
    // ROS2 对应：MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor exec;

    // 工厂 create：ROS1 传 NodeHandle；ROS2 传 node shared_ptr
    RCLCPP_INFO(logger, "Creating policy for robot_type='%s', policy_type='%s'...", robot_type.c_str(),
                policy_type.c_str());

    auto policy = RLControllerFactory::instance().create(policy_type, node);
    if (!policy)
    {
      throw std::runtime_error("No policy registered for type: " + policy_type);
    }

    // 把节点加入 executor，让订阅/定时器等回调能跑
    exec.add_node(node);

    // 如果你的 policy 内部还会创建自己节点（不建议），需要把它也 add_node
    // 如果 policy 是“纯逻辑类”，内部用 node 创建 pub/sub/timer，那只要
    // add_node(node) 就够了

    // 让 executor 在后台跑，等价于 AsyncSpinner.start()
    std::thread spin_thread([&exec]() { exec.spin(); });

    RCLCPP_INFO(logger, "Policy created successfully, entering spinUntilShutdown");
    policy->spinUntilShutdown();  // 你已有的 shutdown loop（可保留）
    RCLCPP_INFO(logger, "Policy spin terminated, shutting down");

    // 停 executor
    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();

    /*
    if (!node->get_parameter("/RobotCfg/policy_type", policy_type)) {
        throw std::runtime_error("Required parameter '/RobotCfg/policy_type' not
    found");
    }
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ("Creating policy for robot_type='%s', policy_type='%s'...",
    robot_type.c_str(), policy_type.c_str()); auto policy =
    RLControllerFactory::instance().create(policy_type, nh); if (!policy) {
        throw std::runtime_error("No policy registered for type: " +
    policy_type);
    }

    ("Policy created successfully, entering spinUntilShutdown");
    policy->spinUntilShutdown();
    ("Policy spin terminated, shutting down");
    */

  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Failed to initialize RL policy: %s", e.what());
    return -1;
  } catch (...)
  {
    RCLCPP_ERROR(logger, "Unknown error occurred during initialization");
    return -1;
  }

  // ("rl_controller shutdown");
  RCLCPP_ERROR(logger, "rl_controller shutdown");
  rclcpp::shutdown();
  return 0;
}
