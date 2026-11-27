#pragma once

#include <mujoco/mujoco.h>
// #include <ros/ros.h>
#include <pthread.h>

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <motor_mapping.hpp>
#include <vector>

// Forward declarations
namespace mujoco {
class Simulate;
}

// Global variables
extern mjModel* m;
extern mjData* d;

struct SimJointCommand {
  double q{};
  double dq{};
  double tau{};
  double kp{};
  double kd{};
};

struct SimJointAddress {
  int actuator{-1};
  int qpos{-1};
  int dof{-1};
};

// Indexed by deployment mapping; MuJoCo joint/actuator ordering may differ.
extern std::vector<SimJointAddress> simJointAddresses;
extern std::vector<SimJointCommand> recvCmd;
extern std::vector<bool> motorEnabled;
extern std::vector<bool> motorCommandFault;
extern std::unique_ptr<system_command_arbiter::MotorMapping> motorMapping;
extern int64_t commandReceivedSteadyNs;
extern int64_t watchdogNs;
extern uint32_t lastSequenceId;
extern bool commandReceived;
extern bool commandWatchdogArmed;
extern uint32_t stateSequenceId;
extern pthread_mutex_t recvMutex_;
extern pthread_mutex_t sendMutex_;

// Constants
extern const double syncMisalign;
extern const double simRefreshFraction;
extern const int kErrorLength;

// Function declarations are now in sim_interface modules
