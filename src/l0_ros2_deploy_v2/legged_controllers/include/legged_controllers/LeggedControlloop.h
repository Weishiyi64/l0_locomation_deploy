// Copyright information
//
// Copyright (c) 2022, Qiayuan (Original author)
// Copyright (c) 2024, Shibo Mu (Modifications)
// All rights reserved.

#pragma once

#include <legged_controllers/IHardwareLayer.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// Forward declaration
namespace legged {
class LeggedController;
}

namespace legged {
class LeggedControlloop {  // NOLINT(cppcoreguidelines-special-member-functions)
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;

 public:
  LeggedControlloop(rclcpp::Node::SharedPtr nh, std::shared_ptr<IHardwareLayer> hardware_interface);
  ~LeggedControlloop();

  void update();

 private:
  // Startup and shutdown of the internal node inside a roscpp program
  // ros::NodeHandle nh_;
  rclcpp::Node::SharedPtr nh_;

  std::thread loopThread_;

  // Timing
  double lowlevel_frequency{};
  int threadPriority_{};
  std::string threadSchedulingMethod_{};
  // Behavior configuration
  bool skipControlOnOverrun_{true};  // keep legacy behavior by default
  bool setRealtimePriority_{false};  // disabled by default
  std::shared_ptr<LeggedController> controller_;

  // Abstract Hardware Interface for your robot
  std::shared_ptr<IHardwareLayer> hardwareInterface_;

  std::atomic_bool loopRunning_{};

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
      // ("\033[32m%s current scheduler: %s, priority: %d\033[0m",
      RCLCPP_INFO(nh_->get_logger(), "\033[32m%s current scheduler: %s, priority: %d\033[0m", threadName.c_str(),
                  getSchedulerPolicyName(policy).c_str(), param.sched_priority);

      if (policy == SCHED_RR)
      {
        struct timespec ts;
        if (sched_rr_get_interval(0, &ts) == 0)
        {
          // ("\033[32m%s RR time slice: %ld.%03ld ms\033[0m",
          RCLCPP_INFO(nh_->get_logger(), "\033[32m%s RR time slice: %ld.%03ld ms\033[0m", threadName.c_str(),
                      ts.tv_sec * 1000 + ts.tv_nsec / 1000000, (ts.tv_nsec % 1000000) / 1000);
        }
      }
    }
  }
};

}  // namespace legged
