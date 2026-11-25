// Optional soft-stop experiment. Only the OpenVINO MuJoCo build links this file.
#include "rl_controller_wbc.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace {
constexpr double kDt = 0.02;
constexpr double kStationaryLegPoseTolerance = 0.12;  // rad from measured AMP stand
constexpr double kStationaryLegSpeed = 0.02;          // rad/s in the reference

Eigen::Quaterniond orientation(const std::array<double, 4>& wxyz) {
  return Eigen::Quaterniond(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
}

double heading(const Eigen::Quaterniond& q) {
  return std::atan2(2. * (q.w()*q.z() + q.x()*q.y()), 1. - 2. * (q.y()*q.y() + q.z()*q.z()));
}
}

void RLWbc::initStopTrial(const std::string& policy_root, const std::string& robot_type) {
  const char* profile = std::getenv("L0_MUJOCO_STOP_PROFILE");
  if (!profile || !*profile) return;
  const char* emergency = std::getenv("L0_MUJOCO_FINISH_EMERGENCY");
  if (!emergency) throw std::runtime_error("soft STOP requires the sim2mujoco finish-action configuration");
  stop_emergency_actions_ = YAML::Load(emergency).as<std::vector<std::string>>();
  const auto stand = YAML::LoadFile(profile);
  const char* amp = std::getenv("L0_MUJOCO_STOP_AMP");
  if (!amp || std::string(amp) != "l0_v5_amp_1124" || robot_type != "l0_v5" ||
      stand["policy_name"].as<std::string>() != amp || stand["joint_names"].as<std::vector<std::string>>() != joint_names_)
    throw std::runtime_error("stop trial requires the L0 V5 AMP1124 capture and matching joint order");
  const std::string dir = policy_root + "/" + robot_type + "/" + amp;
  const std::string amp_model = std::string(LEGGED_POLICY_PATH) + "/" + robot_type + "/" + amp + "/policy.onnx";
  if (WbcLoraAssets::fileSha256(amp_model) != stand["policy_sha256"].as<std::string>() ||
      WbcLoraAssets::fileSha256(dir + "/rl_config.yaml") != stand["rl_config_sha256"].as<std::string>())
    throw std::runtime_error("AMP model/config no longer matches the supplied stand capture");
  const auto config = YAML::LoadFile(dir + "/rl_config.yaml")["RobotCfg"];
  stop_stand_ = stand["representative_frame"]["q_stand_rad"].as<std::vector<double>>();
  const auto pelvis = stand["representative_frame"];
  stop_stand_height_ = pelvis["pelvis_pos_m"].as<std::array<double, 3>>()[2];
  const auto xyzw = pelvis["pelvis_quat_xyzw"].as<std::array<double, 4>>();
  const Eigen::Quaterniond stand_q(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  if (!std::isfinite(stop_stand_height_) || stop_stand_height_ <= 0. ||
      !std::isfinite(stand_q.norm()) || std::abs(stand_q.norm()-1.) > 1e-3)
    throw std::runtime_error("invalid pelvis pose in soft STOP stand capture");
  const auto q = stand_q.normalized();
  stop_stand_orientation_ = {q.w(), q.x(), q.y(), q.z()};
  stop_amp_scale_ = config["action_scale"].as<std::vector<double>>();
  stop_amp_offset_ = config["default_joint_angles"].as<std::vector<double>>();
  for (const auto* values : {&stop_stand_, &stop_amp_scale_, &stop_amp_offset_})
    if (values->size() != 21 || !std::all_of(values->begin(), values->end(), [](double v) { return std::isfinite(v); }))
      throw std::runtime_error("invalid stop trial joint data");
  if (config["joint_order_isaac"].as<std::vector<std::string>>() != joint_names_ ||
      config["joint_order_mj"].as<std::vector<std::string>>() != joint_names_)
    throw std::runtime_error("stop trial AMP joint order mismatch");
  for (const auto& name : joint_names_)
    stop_upper_.push_back(name.find("shoulder") != std::string::npos || name.find("elbow") != std::string::npos ||
                          name.find("neck_") == 0);

  stop_amp_ = std::make_unique<WbcModel>(amp_model, WbcModel::InputMode::AmpPreview);
  if (stop_amp_->inputCount() != 1 || stop_amp_->inputIndex("obs") != 0 || stop_amp_->inputElements() != 360 ||
      stop_amp_->outputCount() != 1 || stop_amp_->outputIndex("actions") != 0 || stop_amp_->outputElements(0) != 21)
    throw std::runtime_error("stop trial AMP ABI mismatch");

  support_sub_ = node_->create_subscription<luvbot_control_interfaces::msg::SimSupportState>(
      "experiment/stop_switch/support", rclcpp::SensorDataQoS(),
      [this](luvbot_control_interfaces::msg::SimSupportState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        stop_support_ = msg;
      });
  soft_stop_sub_ = node_->create_subscription<std_msgs::msg::UInt64>("experiment/stop_switch/request", 1,
      [this](std_msgs::msg::UInt64::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || stop_requested_ || !msg->data || msg->data != posCmd_.handoff_id) return;
        if (std::find(stop_emergency_actions_.begin(), stop_emergency_actions_.end(), current_->motion_id_) !=
            stop_emergency_actions_.end()) {
          RCLCPP_WARN(node_->get_logger(), "Soft STOP unavailable for finish-to-emergency action %s", current_->motion_id_.c_str());
          return;
        }
        stop_requested_ = true;
        RCLCPP_INFO(node_->get_logger(), "MuJoCo soft STOP requested for %s", current_->motion_id_.c_str());
      });
}

RLWbc::ReferenceFrame RLWbc::stopReference(int index) const {
  ReferenceFrame out = stop_start_;
  const double u = std::min(1., (index - stop_frame_) / static_cast<double>(kStopBlendTicks));
  const double smooth = u*u*(3.-2.*u);
  for (size_t j = 0; j < stop_upper_.size(); ++j) {
    if (stop_move_legs_ || stop_upper_[j])
      out.joint_position[j] += smooth * (stop_stand_[j] - stop_start_.joint_position[j]) +
          u*(u-1.)*(u-1.) * (kStopBlendTicks*kDt) * stop_velocity_[j];
  }
  if (stop_move_legs_) {
    out.root_position[2] += smooth * (stop_stand_height_ - out.root_position[2]);
    const auto q = orientation(stop_start_.root_orientation).slerp(smooth, orientation(stop_end_orientation_));
    out.root_orientation = {q.w(), q.x(), q.y(), q.z()};
  }
  return out;
}

void RLWbc::updateStopTrial(int index, const RobotFrame& state) {
  luvbot_control_interfaces::msg::SimSupportState::ConstSharedPtr support;
  bool zero_command;
  int64_t observation_stamp;
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    support = stop_support_;
    observation_stamp = observation_stamp_ns_;
    const auto& v = policy_observation_.velocity_command;
    zero_command = v.linear.x == 0. && v.linear.y == 0. && v.angular.z == 0.;
  }
  if (!support) { stable_ticks_ = 0; return; }
  const int64_t stamp = rclcpp::Time(support->header.stamp).nanoseconds();
  const int64_t now = node_->now().nanoseconds();
  const double omega = std::hypot(state.angular_velocity[0], state.angular_velocity[1], state.angular_velocity[2]);
  if (!support->command_valid || !zero_command || now < stamp || now - stamp > 20000000 ||
      std::abs(stamp - observation_stamp) > 20000000 || stamp <= last_gate_stamp_ ||
      !(support->left_normal_force_n > 2. && support->right_normal_force_n > 2. &&
        support->horizontal_speed_m_s < 0.12 && omega < 0.6 && -state.projected_gravity[2] > 0.9659258)) {
    stable_ticks_ = 0;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "MuJoCo soft STOP waiting for fresh zero-command double support: feet %.1f/%.1f N, speed %.3f m/s, omega %.3f rad/s",
        support->left_normal_force_n, support->right_normal_force_n, support->horizontal_speed_m_s, omega);
    return;
  }
  if (last_gate_stamp_ && stamp - last_gate_stamp_ > 30000000) stable_ticks_ = 0;
  last_gate_stamp_ = stamp;
  if (stop_frame_ < 0) {
    stop_frame_ = index;
    stop_tick_ = 0;
    stop_start_ = inferReferenceFrame(index);
    const auto previous = inferReferenceFrame(index - 1);
    const auto next = inferReferenceFrame(index + 1);
    stop_velocity_.resize(21);
    stop_move_legs_ = false;
    for (size_t j = 0; j < 21; ++j) {
      stop_velocity_[j] = (next.joint_position[j] - previous.joint_position[j]) / (2*kDt);
      if (!stop_upper_[j] && (std::abs(stop_start_.joint_position[j] - stop_stand_[j]) > kStationaryLegPoseTolerance ||
                             std::abs(stop_velocity_[j]) > kStationaryLegSpeed))
        stop_move_legs_ = true;
    }
    if (stop_move_legs_) {
      const auto stand_q = orientation(stop_stand_orientation_);
      const double yaw_delta = heading(orientation(stop_start_.root_orientation)) - heading(stand_q);
      const auto target = Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()) * stand_q;
      stop_end_orientation_ = {target.w(), target.x(), target.y(), target.z()};
    }
    RCLCPP_INFO(node_->get_logger(), "MuJoCo soft STOP blend starts at reference frame %d (%s)", index,
                stop_move_legs_ ? "whole body to AMP stance" : "upper body, stationary legs");
    return;
  }
  if (index < stop_frame_ + kStopBlendTicks) return;
  if (support->joint_names != posCmd_.joint_names || support->target_positions.size() != 21)
    throw std::runtime_error("soft STOP motor target ordering mismatch");
  // The deployed AMP repeats the first complete seeded observation in all five history slots.
  std::array<float, 360> input{};
  std::copy(state.angular_velocity.begin(), state.angular_velocity.end(), input.begin());
  std::copy(state.projected_gravity.begin(), state.projected_gravity.end(), input.begin()+3);
  for (size_t j = 0; j < 21; ++j) {
    input[9+j] = state.joint_position[j] + current_->default_joint_positions_[j] - stop_amp_offset_[j];
    input[30+j] = state.joint_velocity[j];
    input[51+j] = stop_amp_scale_[j] == 0 ? 0 : (posCmd_.target_positions[j]-stop_amp_offset_[j])/stop_amp_scale_[j];
  }
  for (int h = 1; h < 5; ++h) std::copy_n(input.begin(), 72, input.begin()+72*h);
  if (!stop_amp_->run(input.data(), input.size())) throw std::runtime_error("soft STOP AMP preview failed");
  double leg_gap = 0., upper_gap = 0.;
  for (size_t j = 0; j < 21; ++j) {
    const double sent = support->target_positions[j];
    if (!std::isfinite(sent)) throw std::runtime_error("invalid motor target in soft STOP feedback");
    const double delta = std::abs(stop_amp_->output(0)[j]*stop_amp_scale_[j]+stop_amp_offset_[j]-sent);
    double& peak = stop_upper_[j] ? upper_gap : leg_gap;
    peak = std::max(peak, delta);
  }
  stable_ticks_ = leg_gap < 0.45 && upper_gap < 1.5 ? stable_ticks_+1 : 0;
  if (stable_ticks_ == 0)
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "MuJoCo soft STOP waiting for AMP targets: max delta legs %.3f / 0.45, upper %.3f / 1.5 rad",
        leg_gap, upper_gap);
  if (stable_ticks_ == 5)
    RCLCPP_INFO(node_->get_logger(), "MuJoCo soft STOP gate at frame %d: max delta legs %.5f upper %.5f rad",
                index, leg_gap, upper_gap);
}
