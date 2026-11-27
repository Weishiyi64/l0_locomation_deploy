#ifndef RL_DATA_BASE_H
#define RL_DATA_BASE_H

#include <Eigen/Dense>
// #include <ros/ros.h>
#include <algorithm>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

// Minimal common types used by RL and RL_v3
struct ObservationItem {
  std::string type;
  int dim;
  double scale_value;
  ObservationItem(const std::string& t = "", int d = 0, double s = 1.0) : type(t), dim(d), scale_value(s) {}
};

// Base model params shared by RL and RL_v3. Keep this minimal and stable.
struct RLBaseModelParams {
  std::string robot_type;
  std::string subpolicy;
  std::string policy_name;
  std::string policy_path;
  std::string policy_type;

  int policy_frequency = 0;
  int lowlevel_frequency = 0;
  double gait_frequency = 0.0;

  int num_actions = 0;
  int num_single_obs = 0;
  int frame_stack = 0;
  int frame_history_length = 0;
  std::vector<int> frame_stack_indices;

  double clip_obs = 0.0;
  double clip_actions = 0.0;

  std::vector<double> default_joint_angles;
};

// Deployment configuration parameters
struct LeggedDeployParams {
  bool set_realtime_priority = false;
  std::string thread_scheduling_method = "SCHED_OTHER";
  int thread_priority = 0;
};

// Base robot state used by different versions
struct RobotBaseState {
  Eigen::Vector3d base_ang_vel;
  Eigen::Quaterniond quat;
  Eigen::Quaterniond quat_obs;
  Eigen::VectorXd q;
  Eigen::VectorXd dq;
  Eigen::VectorXd tau;
  Eigen::Vector3d policy_command;

  rclcpp::Logger logger_;

  // Default constructor for member initialization (order matches declaration)
  RobotBaseState(rclcpp::Logger logger)
      : base_ang_vel(Eigen::Vector3d::Zero()),
        quat(Eigen::Quaterniond::Identity()),
        quat_obs(Eigen::Quaterniond::Identity()),
        policy_command(Eigen::Vector3d::Zero()),
        logger_(logger) {}

  RobotBaseState(rclcpp::Logger logger, int num_actions)
      : base_ang_vel(Eigen::Vector3d::Zero()),
        quat(Eigen::Quaterniond::Identity()),
        quat_obs(Eigen::Quaterniond::Identity()),
        policy_command(Eigen::Vector3d::Zero()),
        logger_(logger) {
    if (num_actions > 0)
    {
      q = Eigen::VectorXd::Zero(num_actions);
      dq = Eigen::VectorXd::Zero(num_actions);
      tau = Eigen::VectorXd::Zero(num_actions);
    }
    else
    {
      RCLCPP_ERROR(logger_, "RobotBaseState: num_actions input error");
    }
  }
};

// Base output to allow reuse in v3
struct RobotOutputBase {
  Eigen::VectorXd target_q;
  Eigen::VectorXd action;

  rclcpp::Logger logger_;
  // Default constructor for member initialization
  RobotOutputBase(rclcpp::Logger logger) : logger_(logger) {}

  RobotOutputBase(rclcpp::Logger logger, int num_actions) : logger_(logger) {
    if (num_actions > 0)
    {
      target_q = Eigen::VectorXd::Zero(num_actions);
      action = Eigen::VectorXd::Zero(num_actions);
    }
    else
    {
      RCLCPP_ERROR(logger_, "RobotOutputBase: num_actions input error");
    }
  }
};

template <int DefaultObs = 1, int DefaultStack = 1>
struct ObservationsGeneric {
  Eigen::VectorXd observations;
  std::deque<Eigen::VectorXd> hist_obs;
  std::vector<int> frame_indices;
  int history_length = 0;

  std::vector<float> input_flat;

  ObservationsGeneric(int num_single_obs = DefaultObs, int frame_stack = DefaultStack) {
    std::vector<int> default_frame_indices;
    if (frame_stack > 0)
    {
      default_frame_indices.reserve(frame_stack);
      for (int i = frame_stack - 1; i >= 0; --i)
      {
        default_frame_indices.push_back(-i);
      }
    }
    initialize(num_single_obs, default_frame_indices);
  }

  ObservationsGeneric(int num_single_obs, const std::vector<int>& frame_stack_indices) {
    initialize(num_single_obs, frame_stack_indices);
  }

 private:
  void initialize(int num_single_obs, const std::vector<int>& frame_stack_indices) {
    observations = Eigen::VectorXd();
    hist_obs.clear();
    frame_indices = frame_stack_indices;
    history_length = 0;
    input_flat.clear();

    if (num_single_obs <= 0)
    {
      return;
    }

    observations = Eigen::VectorXd::Zero(num_single_obs);
    for (int idx : frame_indices)
    {
      history_length = std::max(history_length, 1 - idx);
    }
    if (history_length <= 0 && !frame_indices.empty())
    {
      history_length = 1;
    }

    for (int i = 0; i < history_length; ++i)
    {
      hist_obs.push_back(Eigen::VectorXd::Zero(num_single_obs));
    }

    if (!frame_indices.empty())
    {
      input_flat.resize(num_single_obs * frame_indices.size());
    }
  }
};

#endif
