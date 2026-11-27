#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <atomic>
#include <chrono>
#include <fstream>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sched.h>
#include <string>
#include <thread>
#include <vector>

#include "base/rl_controller_registrar.h"
#include "robot_interfaces/msg/policy_command.hpp"
#include "robot_interfaces/msg/policy_observation.hpp"
#include "rl_data.h"

void requireNode(const YAML::Node& n, const std::string& key_path);
std::vector<double> asDoubleList(const YAML::Node& n, const std::string& path);
std::vector<std::string> asStringList(const YAML::Node& n, const std::string& path);

class BaseRLController : public ControllerType {
 public:
  explicit BaseRLController(rclcpp::Node::SharedPtr nh) : nh(nh) {}
  virtual ~BaseRLController() = default;

  // Blocks until shutdown using global flag
  void spinUntilShutdown() override;

 protected:
  rclcpp::Node::SharedPtr nh;
  using SteadyClock = std::chrono::steady_clock;

  robot_interfaces::msg::PolicyObservation policy_observation_;
  SteadyClock::time_point policy_observation_received_at_{};
  robot_interfaces::msg::PolicyCommand posCmd_;
  rclcpp::Subscription<robot_interfaces::msg::PolicyObservation>::SharedPtr subState_;
  rclcpp::Publisher<robot_interfaces::msg::PolicyCommand>::SharedPtr pubCmd_ov;
  uint64_t command_sequence_id_{0};
  double command_validity_ms_{100.0};

  // Flag to control RL inference
  std::atomic<bool> rl_inference_enabled_{false};

  void initBaseTopic(rclcpp::Node::SharedPtr nh_);
  bool publishPolicyCommand(const rclcpp::Time& input_time);
  void finishPolicyCycle(SteadyClock::time_point started, int frequency);
  bool isPolicyInputFresh(const robot_interfaces::msg::PolicyObservation& observation,
                          SteadyClock::time_point received_at) const;
  bool isPolicyResultTimely(SteadyClock::time_point started, int frequency) const;
  bool validatePolicyObservation(const robot_interfaces::msg::PolicyObservation& message,
                                 const std::vector<std::string>& expected_joint_names, const char* consumer_name) const;

  /// PolicyCommand publisher topic. Each concrete policy must implement.
  virtual std::string rlCmdPublishTopic() const = 0;
  virtual void stateCallback(const robot_interfaces::msg::PolicyObservation::ConstSharedPtr msg) = 0;

  // Read common RobotCfg params into RLBaseModelParams and observation items.
  // Returns true if parameters loaded successfully.
  bool readCommonParams(RLBaseModelParams& out_params, std::vector<ObservationItem>& out_obs_items);

  // Read deployment configuration parameters from RLController section.
  // Returns true if parameters loaded successfully.
  bool readDeployParams(LeggedDeployParams& out_params);

  void configureThreadScheduler(pthread_t thread, const LeggedDeployParams& params, const char* thread_name);

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
    }
  }

 private:
  struct CycleTiming {
    SteadyClock::time_point window_start{};
    SteadyClock::time_point previous_start{};
    int frequency{0};
    uint64_t samples{0};
    uint64_t late{0};
    double interval_max_ms{0.0};
    double work_max_ms{0.0};
  } cycle_timing_;
};

#endif
