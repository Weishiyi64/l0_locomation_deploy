#pragma once

#include <mujoco/mujoco.h>
// #include <ros/ros.h>
#include <pthread.h>

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
// #include "legged_msgs/LowCmd.h"
// #include "legged_msgs/LowState.h"
#include "legged_msgs/msg/low_cmd.hpp"
#include "legged_msgs/msg/low_state.hpp"

// Forward declarations
namespace mujoco {
class Simulate;
}

// Global variables
extern mjModel* m;
extern mjData* d;

extern legged_msgs::msg::LowCmd recvCmd;
extern legged_msgs::msg::LowState sendState_;
extern int64_t last_timestamp_;
extern pthread_mutex_t recvMutex_;
extern pthread_mutex_t sendMutex_;

// Constants
extern const double syncMisalign;
extern const double simRefreshFraction;
extern const int kErrorLength;

// Function declarations are now in sim_interface modules
