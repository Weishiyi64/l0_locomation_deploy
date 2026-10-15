#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include "legged_deploy/base/rl_controller_registrar.h"
#include "legged_deploy/rl_data.h"
// #include <ros/ros.h>
#include <atomic>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "legged_msgs/msg/low_state.hpp"
#include "legged_msgs/msg/pos_cmd.hpp"

void requireNode(const YAML::Node& n, const std::string& key_path);
std::vector<double> asDoubleList(const YAML::Node& n, const std::string& path);
std::vector<std::string> asStringList(const YAML::Node& n, const std::string& path);

class BaseRLController : public ControllerType {
 public:
  // explicit BaseRLController(ros::NodeHandle& nh) : nh(nh), quit(false) {}
  explicit BaseRLController(rclcpp::Node::SharedPtr nh) : nh(nh), quit(false) {}
  virtual ~BaseRLController();

  // Blocks until shutdown using global flag
  void spinUntilShutdown() override;

 protected:
  rclcpp::Node::SharedPtr nh;
  // ros::NodeHandle nh;
  std::atomic<bool> quit;

  // shared message state and common pub/sub used by RL policies
  legged_msgs::msg::LowState lowState_;
  legged_msgs::msg::PosCmd posCmd_;
  // ros::Subscriber subState_;
  // ros::Publisher pubCmd_ov;
  rclcpp::Subscription<legged_msgs::msg::LowState>::SharedPtr subState_;
  rclcpp::Publisher<legged_msgs::msg::PosCmd>::SharedPtr pubCmd_ov;

  // Flag to control RL inference
  std::atomic<bool> rl_inference_enabled_{false};

  // Initialize common topics (subscribe /rlState and advertise PosCmd on rlCmdPublishTopic())
  // The subscriber will call the virtual stateCallback implemented by derived
  // classes
  // void initBaseTopic(ros::NodeHandle& nh_);
  void initBaseTopic(rclcpp::Node::SharedPtr nh_);

  /// PosCmd publisher topic (e.g. /rlCmd_rl vs /rlCmd_imitation). Each concrete policy must implement.
  virtual std::string rlCmdPublishTopic() const = 0;

  // Derived policies must implement how to handle /rlState messages
  virtual void stateCallback(const legged_msgs::msg::LowState::ConstSharedPtr msg) = 0;

  // Read common RobotCfg params into RLBaseModelParams and observation items.
  // Returns true if parameters loaded successfully.
  bool readCommonParams(RLBaseModelParams& out_params, std::vector<ObservationItem>& out_obs_items);

  // Read deployment configuration parameters from RLController section.
  // Returns true if parameters loaded successfully.
  bool readDeployParams(LeggedDeployParams& out_params);

  // 从yaml文件中读取RL参数
  bool loadRobotCfgFromYaml(const std::string& yaml_path, RLBaseModelParams& out_params,
                            std::vector<ObservationItem>& out_obs_items);

  inline std::string getSchedulerPolicyName(int policy) {
    switch (policy)
    {
      case SCHED_OTHER:
        return "SCHED_OTHER";
      case SCHED_FIFO:
        return "SCHED_FIFO";
      case SCHED_RR:
        return "SCHED_RR";
#ifdef SCHED_BATCH
      case SCHED_BATCH:
        return "SCHED_BATCH";
#endif
#ifdef SCHED_IDLE
      case SCHED_IDLE:
        return "SCHED_IDLE";
#endif
      default:
        return "UNKNOWN";
    }
  }

  inline int getSchedulerPolicy(const std::string& method) {
    if (method == "SCHED_OTHER") return SCHED_OTHER;
    if (method == "SCHED_FIFO") return SCHED_FIFO;
    if (method == "SCHED_RR") return SCHED_RR;
#ifdef SCHED_BATCH
    if (method == "SCHED_BATCH") return SCHED_BATCH;
#endif
#ifdef SCHED_IDLE
    if (method == "SCHED_IDLE") return SCHED_IDLE;
#endif
    return SCHED_OTHER;  // default fallback
  }

  inline void printThreadSchedulerInfo(pthread_t thread, const std::string& threadName) {
    int policy;
    struct sched_param param;

    if (pthread_getschedparam(thread, &policy, &param) == 0)
    {
      RCLCPP_INFO(nh->get_logger(), "\033[34m%s current scheduler: %s, priority: %d\033[0m", threadName.c_str(),
                  getSchedulerPolicyName(policy).c_str(), param.sched_priority);
      // ("\033[34m%s current scheduler: %s, priority: %d\033[0m",
      // threadName.c_str(), getSchedulerPolicyName(policy).c_str(),
      // param.sched_priority);

      if (policy == SCHED_RR)
      {
        struct timespec ts;
        if (sched_rr_get_interval(0, &ts) == 0)
        {
          RCLCPP_INFO(nh->get_logger(), "\033[34m%s RR time slice: %ld.%03ld ms\033[0m", threadName.c_str(),
                      ts.tv_sec * 1000 + ts.tv_nsec / 1000000, (ts.tv_nsec % 1000000) / 1000);

          // ("\033[34m%s RR time slice: %ld.%03ld ms\033[0m",
          //          threadName.c_str(), ts.tv_sec * 1000 + ts.tv_nsec /
          //          1000000, (ts.tv_nsec % 1000000) / 1000);
        }
      }
    }
  }
};

#endif
