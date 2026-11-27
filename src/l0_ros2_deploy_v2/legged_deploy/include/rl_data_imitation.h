#ifndef RL_IMITATION_DATA_H
#define RL_IMITATION_DATA_H

// reuse common definitions and includes from rl_data.h
#include <iomanip>

#include "rl_data.h"

// Command item structure for motion.onnx outputs
struct CommandItem {
  std::string type;
  int dim;
  CommandItem(const std::string& t = "", int d = 0) : type(t), dim(d) {}
};

// reuse ObservationItem and RLBaseModelParams from rl_data.h
struct RL_IMITATION_ModelParams : public RLBaseModelParams {
  std::string imitation_name;
  std::string imitation_path;

  std::vector<std::string> joint_order_mj;
  std::vector<std::string> joint_order_isaac;

  // v3 uses per-joint action scale
  std::vector<double> action_scale;

  // observation items reuse ObservationItem
  std::vector<ObservationItem> observation_items;

  // dimensions for command and anchor_ori
  int command_dim = 0;
  int imitation_anchor_dim = 0;

  // command configuration: specifies which motion.onnx outputs to use and in
  // what order
  std::vector<CommandItem> command_items;
  std::vector<int> command_indices;
};

struct Robot_Imitation_State : public RobotBaseState {
  Eigen::Quaterniond imu_quat;
  Eigen::VectorXd command;
  Eigen::Quaterniond command_quat;
  Eigen::VectorXd anchor_ori;

  // Default constructor for member initialization
  Robot_Imitation_State(rclcpp::Logger logger)
      : RobotBaseState(logger),
        imu_quat(Eigen::Quaterniond::Identity()),
        command_quat(Eigen::Quaterniond::Identity()) {}

  Robot_Imitation_State(rclcpp::Logger logger, int num_actions, int command_dim, int imitation_anchor_dim)
      : RobotBaseState(logger, num_actions),
        imu_quat(Eigen::Quaterniond::Identity()),
        command_quat(Eigen::Quaterniond::Identity()) {
    if (num_actions <= 0)
    {
      RCLCPP_ERROR(logger_, "Robot_Imitation_State: num_actions input error");
    }
    if (command_dim > 0)
    {
      command = Eigen::VectorXd::Zero(command_dim);
    }
    else
    {
      RCLCPP_ERROR(logger_, "Robot_Imitation_State: command_dim input error");
    }
    if (imitation_anchor_dim > 0)
    {
      anchor_ori = Eigen::VectorXd::Zero(imitation_anchor_dim);
    }
    else
    {
      anchor_ori = Eigen::VectorXd::Zero(0);
    }
  }

  void clean() {
    base_ang_vel.setZero();
    quat = Eigen::Quaterniond::Identity();
    quat_obs = Eigen::Quaterniond::Identity();
    imu_quat = Eigen::Quaterniond::Identity();
    if (q.size() > 0) q.setZero();
    if (dq.size() > 0) dq.setZero();
    if (tau.size() > 0) tau.setZero();
    if (command.size() > 0) command.setZero();
    command_quat = Eigen::Quaterniond::Identity();
    if (anchor_ori.size() > 0) anchor_ori.setZero();
  }
};

struct Robot_Imitation_Output : public RobotOutputBase {
  // Default constructor for member initialization
  Robot_Imitation_Output(rclcpp::Logger logger) : RobotOutputBase(logger) {}

  Robot_Imitation_Output(rclcpp::Logger logger, int num_actions) : RobotOutputBase(logger, num_actions) {
    if (num_actions <= 0)
    {
      RCLCPP_ERROR(logger, "Robot_Imitation_Output: num_actions input error");
    }
  }
};

class ObservationManager_Imitation {
 private:
  std::vector<ObservationItem> items_;
  int total_obs_dim_;
  rclcpp::Logger logger_;

 public:
  ObservationManager_Imitation(rclcpp::Logger logger, const std::vector<ObservationItem>& items,
                               const RL_IMITATION_ModelParams& /*params*/)
      : items_(items), total_obs_dim_(0), logger_(logger) {
    for (const auto& item : items_)
    {
      total_obs_dim_ += item.dim;
    }
  }

  void showInfoInOrder(const Robot_Imitation_State& state, const Robot_Imitation_Output& output) {
    // the print precision is 6 digits after decimal point
    for (const auto& item : items_)
    {
      if (item.type == "base_ang_vel")
      {
        RCLCPP_INFO_STREAM(logger_,
                           std::fixed << std::setprecision(6) << "base_ang_vel: " << state.base_ang_vel.transpose());
      }
      else if (item.type == "joint_pos")
      {
        RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(6) << "joint_pos: " << state.q.transpose());
      }
      else if (item.type == "joint_vel")
      {
        RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(6) << "joint_vel: " << state.dq.transpose());
      }
      else if (item.type == "actions")
      {
        RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(6) << "actions: " << output.action.transpose());
      }
      else if (item.type == "command")
      {
        RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(6) << "command: " << state.command.transpose());
      }
      else if (item.type == "anchor_ori")
      {
        RCLCPP_INFO_STREAM(logger_,
                           std::fixed << std::setprecision(6) << "anchor_ori: " << state.anchor_ori.transpose());
      }
    }
  }

  void assembleObservations(Eigen::VectorXd& observations, const Robot_Imitation_State& state,
                            const Robot_Imitation_Output& output, const RL_IMITATION_ModelParams& /* params */) {
    int current_idx = 0;
    for (const auto& item : items_)
    {
      if (item.type == "base_ang_vel")
      {
        observations.segment(current_idx, item.dim) = state.base_ang_vel.head(std::min(item.dim, 3)) * item.scale_value;
      }
      else if (item.type == "joint_pos")
      {
        int copy_dim = std::min(item.dim, static_cast<int>(state.q.size()));
        observations.segment(current_idx, copy_dim) = state.q.head(copy_dim) * item.scale_value;
        if (item.dim > copy_dim) observations.segment(current_idx + copy_dim, item.dim - copy_dim).setZero();
      }
      else if (item.type == "joint_vel")
      {
        int copy_dim = std::min(item.dim, static_cast<int>(state.dq.size()));
        observations.segment(current_idx, copy_dim) = state.dq.head(copy_dim) * item.scale_value;
        if (item.dim > copy_dim) observations.segment(current_idx + copy_dim, item.dim - copy_dim).setZero();
      }
      else if (item.type == "actions")
      {
        int copy_dim = std::min(item.dim, static_cast<int>(output.action.size()));
        observations.segment(current_idx, copy_dim) = output.action.head(copy_dim) * item.scale_value;
        if (item.dim > copy_dim) observations.segment(current_idx + copy_dim, item.dim - copy_dim).setZero();
      }
      else if (item.type == "command")
      {
        int copy_dim = std::min(item.dim, static_cast<int>(state.command.size()));
        observations.segment(current_idx, copy_dim) = state.command.head(copy_dim) * item.scale_value;
        if (item.dim > copy_dim) observations.segment(current_idx + copy_dim, item.dim - copy_dim).setZero();
      }
      else if (item.type == "anchor_ori")
      {
        // Copy from state.anchor_ori with scaling; truncate or pad zeros as
        // needed
        int copy_dim = std::min(item.dim, static_cast<int>(state.anchor_ori.size()));
        if (copy_dim > 0)
        {
          observations.segment(current_idx, copy_dim) = state.anchor_ori.head(copy_dim) * item.scale_value;
        }
        if (item.dim > copy_dim)
        {
          observations.segment(current_idx + copy_dim, item.dim - copy_dim).setZero();
        }
      }
      else if (item.type == "placeholder")
      {
        observations.segment(current_idx, item.dim).setConstant(item.scale_value);
      }
      else
      {
        // Unknown item types are zero-padded to avoid runtime errors
        observations.segment(current_idx, item.dim).setZero();
      }
      current_idx += item.dim;
    }
  }

  int getTotalDimension() const { return total_obs_dim_; }

};

using Observations_Imitation = ObservationsGeneric<1, 1>;

#endif
