#ifndef RL_DATA_H
#define RL_DATA_H

#include <rcl/time.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <functional>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include "legged_deploy/base/rl_data_base.h"

// Gait configuration item
struct GaitConfigItem {
  std::string type;
  int dim;
  double scale_value;
  GaitConfigItem(const std::string& t = "", int d = 0, double s = 0.0) : type(t), dim(d), scale_value(s) {}
};

// Unified gait configuration
struct GaitConfig {
  // determines computation logic
  std::string type;

  // Configuration parameters
  double gait_frequency;
  double swing_height;

  // Observation items
  std::vector<GaitConfigItem> items;

  GaitConfig() : type("fixed gait"), gait_frequency(0.0), swing_height(0.0) {}
};

struct RL_ModelParams : public RLBaseModelParams {
  std::vector<double> action_scale;  // per-joint action scale

  std::vector<std::string> joint_order_mj;
  std::vector<std::string> joint_order_isaac;

  std::vector<ObservationItem> observation_items;
  GaitConfig gait_config;
};

struct Robot_State : public RobotBaseState {
  using RobotBaseState::RobotBaseState;
};

// Robot_Output is provided below as an alias of RobotOutputBase
struct Robot_Output : public RobotOutputBase {
  using RobotOutputBase::RobotOutputBase;
};

class ObservationManager {
 private:
  std::vector<ObservationItem> items_;
  int total_obs_dim_;

 public:
  ObservationManager(const std::vector<ObservationItem>& items, const RL_ModelParams& /* params */)
      : items_(items), total_obs_dim_(0) {
    for (const auto& item : items_)
    {
      total_obs_dim_ += item.dim;
    }
  }

  void assembleObservations(Eigen::VectorXd& observations, const Robot_State& state, const Robot_Output& output,
                            const RL_ModelParams& params, double shifted_time) {
    int current_idx = 0;

    for (const auto& item : items_)
    {
      if (item.type == "base_ang_vel")
      {
        observations.segment(current_idx, item.dim) = state.base_ang_vel * item.scale_value;
      }
      else if (item.type == "quat")
      {
        const int copy_dim = std::min(item.dim, 4);
        Eigen::Vector4d quat_wxyz(state.quat_obs.w(), state.quat_obs.x(), state.quat_obs.y(), state.quat_obs.z());
        observations.segment(current_idx, copy_dim) = quat_wxyz.head(copy_dim) * item.scale_value;
        if (item.dim > copy_dim)
        {
          observations.segment(current_idx + copy_dim, item.dim - copy_dim).setZero();
        }
      }
      else if (item.type == "gravity_proj")
      {
        Eigen::Vector3d v(0.0, 0.0, -1.0);
        Eigen::Vector4d qt(state.quat.x(), state.quat.y(), state.quat.z(), state.quat.w());
        observations.segment(current_idx, item.dim) = qr_ty_i(qt, v) * item.scale_value;
      }
      else if (item.type == "lin_vel_cmd")
      {
        observations[current_idx] = state.policy_command(0) * item.scale_value;
      }
      else if (item.type == "lat_vel_cmd")
      {
        observations[current_idx] = state.policy_command(1) * item.scale_value;
      }
      else if (item.type == "ang_vel_cmd")
      {
        observations[current_idx] = state.policy_command(2) * item.scale_value;
      }
      else if (item.type == "joint_pos")
      {
        observations.segment(current_idx, item.dim) = state.q * item.scale_value;
      }
      else if (item.type == "joint_vel")
      {
        observations.segment(current_idx, item.dim) = state.dq * item.scale_value;
      }
      else if (item.type == "last_actions")
      {
        int action_dim = std::min(item.dim, static_cast<int>(output.action.size()));
        observations.segment(current_idx, action_dim) = output.action.head(action_dim) * item.scale_value;
        if (item.dim > action_dim)
        {
          observations.segment(current_idx + action_dim, item.dim - action_dim).setZero();
        }
      }
      else if (item.type == "placeholder")
      {
        observations.segment(current_idx, item.dim).setConstant(item.scale_value);
      }
      else if (item.type == "gait")
      {
        assembleGaitObservations(observations, current_idx, state, params, shifted_time, item.scale_value);
      }

      current_idx += item.dim;
    }
  }

  int getTotalDimension() const { return total_obs_dim_; }

 private:
  void assembleGaitObservations(Eigen::VectorXd& observations, int start_idx, const Robot_State& state,
                                const RL_ModelParams& params, double /* shifted_time */, double obs_scale) {
    if (params.gait_config.type == "variable gait")
    {
      assembleVariableGait(observations, start_idx, state, params, obs_scale);
    }
    else if (params.gait_config.type == "fixed gait")
    {
      assembleFixedGait(observations, start_idx, state, params, obs_scale);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ObservationManager"), "Unknown gait type: %s", params.gait_config.type.c_str());
      throw std::runtime_error("Unknown gait type: " + params.gait_config.type);
    }
  }

  void assembleVariableGait(Eigen::VectorXd& observations, int start_idx, const Robot_State& state,
                            const RL_ModelParams& params, double obs_scale) {
    static rclcpp::Time start_moving_time;
    static bool is_first_moving = true;
    static double gait_indices = 0.0;
    static double swing_height_indices = 0.0;
    static std::array<double, 2> desired_contact_states = {0.0, 0.0};
    static constexpr double command_threshold = 0.2;

    double lin_vel_norm = std::sqrt(state.policy_command(0) * state.policy_command(0) +
                                    state.policy_command(1) * state.policy_command(1));
    double ang_vel_abs = std::abs(state.policy_command(2));
    bool is_moving = (lin_vel_norm + ang_vel_abs) > command_threshold;

    if (is_moving)
    {
      if (is_first_moving)
      {
        start_moving_time = rclcpp::Clock(RCL_ROS_TIME).now();
        is_first_moving = false;
      }

      double elapsed_time = (rclcpp::Clock(RCL_ROS_TIME).now() - start_moving_time).seconds();

      gait_indices = std::fmod(elapsed_time * params.gait_config.gait_frequency, 1.0);
      swing_height_indices = std::fmod(elapsed_time * params.gait_config.gait_frequency * 2.0, 1.0);

      desired_contact_states[0] = std::fmod(gait_indices + 0.5 + 1.0, 1.0) - 0.5;
      desired_contact_states[1] = std::fmod(gait_indices, 1.0) - 0.5;
    }
    else
    {
      // Reset time when stopped
      is_first_moving = true;
      gait_indices = 0.0;
      swing_height_indices = 0.0;
      desired_contact_states[0] = 0.0;
      desired_contact_states[1] = 0.0;
    }

    double swing_phase_pos = (std::sin(2.0 * M_PI * swing_height_indices - M_PI / 2.0) / 2.0 + 0.5);
    double first_foot_swing_height_target =
        params.gait_config.swing_height * swing_phase_pos * (desired_contact_states[0] > 0 ? 1.0 : 0.0);
    double second_foot_swing_height_target =
        params.gait_config.swing_height * swing_phase_pos * (desired_contact_states[1] > 0 ? 1.0 : 0.0);

    // Assemble observations based on gait_config items
    int current_idx = start_idx;
    for (const auto& gait_item : params.gait_config.items)
    {
      double scale = gait_item.scale_value * obs_scale;

      if (gait_item.type == "gait_frequency")
      {
        observations[current_idx] = params.gait_config.gait_frequency * scale;
      }
      else if (gait_item.type == "swing_height")
      {
        observations[current_idx] = params.gait_config.swing_height * scale;
      }
      else if (gait_item.type == "first_foot_swing_height")
      {
        observations[current_idx] = first_foot_swing_height_target * scale;
      }
      else if (gait_item.type == "second_foot_swing_height")
      {
        observations[current_idx] = second_foot_swing_height_target * scale;
      }
      else if (gait_item.type == "gait_indices")
      {
        observations[current_idx] = gait_indices * scale;
      }
      else if (gait_item.type == "swing_height_indices")
      {
        observations[current_idx] = swing_height_indices * scale;
      }

      current_idx += gait_item.dim;
    }
  }

  void assembleFixedGait(Eigen::VectorXd& observations, int start_idx, const Robot_State& /* state */,
                         const RL_ModelParams& params, double obs_scale) {
    double current_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();

    // Assemble observations based on gait_config items
    int current_idx = start_idx;
    for (const auto& gait_item : params.gait_config.items)
    {
      double scale = gait_item.scale_value * obs_scale;

      if (gait_item.type == "gait_phase_sin")
      {
        observations[current_idx] = std::sin(2 * M_PI * current_time / params.gait_config.gait_frequency) * scale;
      }
      else if (gait_item.type == "gait_phase_cos")
      {
        observations[current_idx] = std::cos(2 * M_PI * current_time / params.gait_config.gait_frequency) * scale;
      }

      current_idx += gait_item.dim;
    }
  }

  Eigen::Vector3d qr_ty_i(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
    double q_w = q[3];
    Eigen::Vector3d q_vec = q.head<3>();
    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
    Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
    Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);
    return (a - b + c);
  }
};

using Observations = ObservationsGeneric<1, 1>;

#endif
