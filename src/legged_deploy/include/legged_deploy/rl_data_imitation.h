#ifndef RL_IMITATION_DATA_H
#define RL_IMITATION_DATA_H

// reuse common definitions and includes from rl_data.h
#include <iomanip>

#include "legged_deploy/rl_data.h"

// Command item structure for motion.onnx outputs
struct CommandItem {
  std::string type;
  int dim;
  CommandItem(const std::string& t = "", int d = 0) : type(t), dim(d) {}
};

// reuse ObservationItem and RLBaseModelParams from rl_data.h
struct RL_IMITATION_ModelParams : public RLBaseModelParams {
  // Native MimicLite ABI: policy[399] + command[240] -> action[21].
  // In this mode the reference trajectory is supplied by C++ at runtime and
  // no per-motion motion.onnx/monolithic policy is loaded.
  bool native_dynamic_reference = false;
  std::string imitation_name;
  std::string imitation_path;

  std::vector<std::string> joint_order_mj;
  std::vector<std::string> joint_order_isaac;

  // v3 uses per-joint action scale
  std::vector<double> action_scale;

  // observation items reuse ObservationItem
  std::vector<ObservationItem> observation_items;

  // dimensions for command and anchor_ori
  int command_dim;
  int imitation_anchor_dim;

  // command configuration: specifies which motion.onnx outputs to use and in
  // what order
  std::vector<CommandItem> command_items;
  std::vector<int> command_indices;
};

struct Robot_Imitation_State : public RobotBaseState {
  Eigen::Vector3d body_xpos;
  Eigen::Quaterniond body_xquat;
  Eigen::VectorXd command;
  Eigen::Quaterniond command_quat;
  Eigen::VectorXd anchor_ori;
  rclcpp::Logger logger_;

  // Default constructor for member initialization
  Robot_Imitation_State(rclcpp::Logger logger)
      : RobotBaseState(logger),
        body_xpos(Eigen::Vector3d::Zero()),
        body_xquat(Eigen::Quaterniond::Identity()),
        command_quat(Eigen::Quaterniond::Identity()),
        logger_(logger) {}

  Robot_Imitation_State(rclcpp::Logger logger, int num_actions, int command_dim, int imitation_anchor_dim)
      : RobotBaseState(logger, num_actions),
        body_xpos(Eigen::Vector3d::Zero()),
        body_xquat(Eigen::Quaterniond::Identity()),
        command_quat(Eigen::Quaterniond::Identity()),
        logger_(logger) {
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
    RobotBaseState::q.setZero();
    eu_ang.setZero();
    base_ang_vel.setZero();
    quat = Eigen::Quaterniond::Identity();
    quat_obs = Eigen::Quaterniond::Identity();
    body_xpos.setZero();
    body_xquat = Eigen::Quaterniond::Identity();
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
        // ROS_INFO_STREAM(std::fixed << std::setprecision(6) << "base_ang_vel:
        // " << state.base_ang_vel.transpose());
        RCLCPP_INFO_STREAM(logger_,
                           std::fixed << std::setprecision(6) << "base_ang_vel: " << state.base_ang_vel.transpose());
      }
      else if (item.type == "joint_pos")
      {
        // ROS_INFO_STREAM(std::fixed << std::setprecision(6) << "joint_pos: "
        // << state.q.transpose());
        RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(6) << "joint_pos: " << state.q.transpose());
      }
      else if (item.type == "joint_vel")
      {
        // ROS_INFO_STREAM(std::fixed << std::setprecision(6) << "joint_vel: "
        // << state.dq.transpose());
        RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(6) << "joint_vel: " << state.dq.transpose());
      }
      else if (item.type == "actions")
      {
        // ROS_INFO_STREAM(std::fixed << std::setprecision(6) << "actions: " <<
        // output.action.transpose());
        RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(6) << "actions: " << output.action.transpose());
      }
      else if (item.type == "command")
      {
        // ROS_INFO_STREAM(std::fixed << std::setprecision(6) << "command: " <<
        // state.command.transpose());
        RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(6) << "command: " << state.command.transpose());
      }
      else if (item.type == "anchor_ori")
      {
        // ROS_INFO_STREAM(std::fixed << std::setprecision(6) << "anchor_ori: "
        // << state.anchor_ori.transpose());
        RCLCPP_INFO_STREAM(logger_,
                           std::fixed << std::setprecision(6) << "anchor_ori: " << state.anchor_ori.transpose());
      }
    }
  }

  void assembleObservations(Eigen::VectorXd& observations, const Robot_Imitation_State& state,
                            const Robot_Imitation_Output& output, const RL_IMITATION_ModelParams& /* params */,
                            double /*shifted_time*/) {
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

 private:
  Eigen::Vector3d qr_ty_i(const Eigen::Vector4d& q, const Eigen::Vector3d& v) const {
    double q_w = q[3];
    Eigen::Vector3d q_vec = q.head<3>();
    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
    Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
    Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);
    return (a - b + c);
  }
};

using Observations_Imitation = ObservationsGeneric<1, 1>;

#endif
