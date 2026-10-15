#include "legged_deploy/rl_controller_imitation.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace {

const std::unordered_map<std::string, std::pair<double, double>> kL0V3TextJointLimits = {
    {"left_shoulder_pitch_joint", {-2.618, 2.618}}, {"left_shoulder_roll_joint", {-1.571, 1.571}},
    {"left_elbow_roll_joint", {-1.571, 1.571}},     {"right_shoulder_pitch_joint", {-2.618, 2.618}},
    {"right_shoulder_roll_joint", {-1.571, 1.571}}, {"right_elbow_roll_joint", {-1.571, 1.571}},
    {"neck_yaw_joint", {-1.571, 1.571}},            {"neck_pitch_joint", {-0.436, 0.279}},
    {"neck_roll_joint", {-0.297, 0.297}},
};

template <typename T>
T requiredScalar(const YAML::Node& node, const char* key) {
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) throw std::runtime_error(std::string("missing scalar '") + key + "'");
  return value.as<T>();
}

}  // namespace

void RL_imitation::textMotionJsonCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  if (!text_motion_enabled_) return;
  if (msg->data.empty() || msg->data.size() > text_motion_max_payload_bytes_)
  {
    publishTextMotionStatus("rejected: JSON payload is empty or exceeds text_motion_max_payload_bytes");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(text_motion_queue_mutex_);
    // Latest command wins.  The ROS callback never parses, loads a model, or waits on the control loop.
    pending_text_motion_json_ = msg->data;
  }
  text_motion_queue_cv_.notify_one();
  publishTextMotionStatus("received: complete JSON queued for background preparation");
}

void RL_imitation::publishTextMotionStatus(const std::string& status) {
  if (!pubTextMotionStatus_) return;
  std_msgs::msg::String msg;
  msg.data = status;
  pubTextMotionStatus_->publish(msg);
}

void RL_imitation::textMotionWorkerLoop() {
  while (true)
  {
    std::string payload;
    {
      std::unique_lock<std::mutex> lock(text_motion_queue_mutex_);
      text_motion_queue_cv_.wait(lock, [this]() {
        return text_motion_worker_stop_ || !pending_text_motion_json_.empty();
      });
      if (text_motion_worker_stop_) return;
      payload.swap(pending_text_motion_json_);
    }

    std::string description;
    std::string error;
    if (!prepareTextMotion(payload, description, error))
    {
      RCLCPP_ERROR(nh_->get_logger(), "[TextMotion] rejected: %s", error.c_str());
      publishTextMotionStatus("rejected: " + error);
      continue;
    }

    RCLCPP_INFO(nh_->get_logger(), "[TextMotion] ready: %s", description.c_str());
    publishTextMotionStatus("ready: " + description);
    std_msgs::msg::String ready;
    ready.data = text_motion_policy_;
    pubTextMotionReady_->publish(ready);
  }
}

bool RL_imitation::prepareTextMotion(const std::string& payload, std::string& description, std::string& error) {
  try
  {
    const YAML::Node root = YAML::Load(payload);  // JSON is a strict YAML subset.
    if (!root.IsMap()) throw std::runtime_error("root must be a JSON object");
    if (requiredScalar<std::string>(root, "format") != "gmr_upper_body_joint_trajectory_v1")
      throw std::runtime_error("unsupported format");
    if (requiredScalar<std::string>(root, "robot") != "l0_v3")
      throw std::runtime_error("robot must be l0_v3");
    if (requiredScalar<std::string>(root, "angle_unit") != "radian")
      throw std::runtime_error("angle_unit must be radian");

    const double source_fps = requiredScalar<double>(root, "fps");
    const int declared_count = requiredScalar<int>(root, "frame_count");
    if (!std::isfinite(source_fps) || source_fps <= 0.0 || source_fps > 500.0)
      throw std::runtime_error("fps must be finite and in (0, 500]");
    if (declared_count < 2 || declared_count > text_motion_max_frames_)
      throw std::runtime_error("frame_count is outside the configured range");

    const YAML::Node names_node = root["joint_names"];
    const YAML::Node frames_node = root["frames"];
    if (!names_node || !names_node.IsSequence() || names_node.size() == 0)
      throw std::runtime_error("joint_names must be a non-empty array");
    if (!frames_node || !frames_node.IsSequence() || static_cast<int>(frames_node.size()) != declared_count)
      throw std::runtime_error("frames length does not equal frame_count");

    std::vector<std::string> source_names;
    std::unordered_map<std::string, size_t> unique_names;
    for (const auto& item : names_node)
    {
      const std::string name = item.as<std::string>();
      if (name.empty() || !unique_names.emplace(name, source_names.size()).second)
        throw std::runtime_error("joint_names contains an empty or duplicate name");
      source_names.push_back(name);
    }

    std::vector<std::vector<double>> source_frames;
    source_frames.reserve(frames_node.size());
    for (size_t frame_index = 0; frame_index < frames_node.size(); ++frame_index)
    {
      const YAML::Node frame = frames_node[frame_index];
      if (!frame.IsMap() || requiredScalar<int>(frame, "frame_index") != static_cast<int>(frame_index))
        throw std::runtime_error("frame_index values must be contiguous from zero");
      const YAML::Node positions = frame["joint_positions"];
      if (!positions || !positions.IsSequence() || positions.size() != source_names.size())
        throw std::runtime_error("joint_positions width does not equal joint_names width");
      std::vector<double> values;
      values.reserve(positions.size());
      for (const auto& value_node : positions)
      {
        const double value = value_node.as<double>();
        if (!std::isfinite(value) || std::abs(value) > 6.4)
          throw std::runtime_error("joint_positions contains a non-finite or implausible angle");
        values.push_back(value);
      }
      for (size_t column = 0; column < values.size(); ++column)
      {
        const auto limit = kL0V3TextJointLimits.find(source_names[column]);
        if (limit == kL0V3TextJointLimits.end())
          throw std::runtime_error("text trajectory may only command L0 v3 upper-body joints");
        constexpr double kLimitTolerance = 1e-3;
        if (values[column] < limit->second.first - kLimitTolerance ||
            values[column] > limit->second.second + kLimitTolerance)
          throw std::runtime_error("joint limit exceeded: " + source_names[column]);
      }
      source_frames.push_back(std::move(values));
    }

    std::lock_guard<std::mutex> model_lock(model_mutex_);
    if (imitation_active_)
      throw std::runtime_error("another imitation action is active; submit text motion again after it finishes");
    std::string robot_cfg_path;
    std::string load_message;
    if (!loadPolicyLocked(text_motion_policy_, robot_cfg_path, load_message))
      throw std::runtime_error("tracking policy load failed: " + load_message);

    if (params.native_dynamic_reference)
    {
      if (params.num_actions != 21 || params.command_dim != 240)
        throw std::runtime_error("native dynamic tracking policy must use 21 actions and command[240]");

      // The generator's camera convention is mirrored relative to the robot.
      // Mirror before any kinematics/reference construction: pitch swaps,
      // while roll axes also change sign because left/right positive axes differ.
      const auto sourceColumn = [&unique_names](const std::string& name) -> size_t {
        const auto it = unique_names.find(name);
        if (it == unique_names.end()) throw std::runtime_error("missing required arm joint for mirroring: " + name);
        return it->second;
      };
      const size_t lsp = sourceColumn("left_shoulder_pitch_joint");
      const size_t lsr = sourceColumn("left_shoulder_roll_joint");
      const size_t ler = sourceColumn("left_elbow_roll_joint");
      const size_t rsp = sourceColumn("right_shoulder_pitch_joint");
      const size_t rsr = sourceColumn("right_shoulder_roll_joint");
      const size_t rer = sourceColumn("right_elbow_roll_joint");
      for (auto& frame : source_frames)
      {
        const double old_lsp = frame[lsp], old_lsr = frame[lsr], old_ler = frame[ler];
        const double old_rsp = frame[rsp], old_rsr = frame[rsr], old_rer = frame[rer];
        frame[lsp] = old_rsp;
        frame[rsp] = old_lsp;
        frame[lsr] = -old_rsr;
        frame[rsr] = -old_lsr;
        frame[ler] = -old_rer;
        frame[rer] = -old_ler;
      }

      std::vector<size_t> target_columns;
      target_columns.reserve(source_names.size());
      for (const std::string& name : source_names)
      {
        const auto it = std::find(params.joint_order_isaac.begin(), params.joint_order_isaac.end(), name);
        if (it == params.joint_order_isaac.end()) throw std::runtime_error("unknown joint name: " + name);
        target_columns.push_back(static_cast<size_t>(std::distance(params.joint_order_isaac.begin(), it)));
      }

      const YAML::Node stand_profile = YAML::LoadFile(text_motion_amp_stand_profile_);
      if (requiredScalar<std::string>(stand_profile, "robot_type") != params.robot_type)
        throw std::runtime_error("AMP stand profile robot_type does not match the tracking policy");
      const YAML::Node stand_names = stand_profile["joint_names"];
      const YAML::Node stand_positions = stand_profile["joint_position_median_rad"];
      const YAML::Node base_position = stand_profile["base_position_median_m"];
      const YAML::Node base_quat = stand_profile["base_quaternion_median_xyzw"];
      if (!stand_names || !stand_positions || stand_names.size() != 21 || stand_positions.size() != 21 ||
          !base_position || base_position.size() != 3 || !base_quat || base_quat.size() != 4)
        throw std::runtime_error("AMP stand profile is incomplete");

      Eigen::VectorXd stand_joint_pos(21);
      for (int joint = 0; joint < 21; ++joint)
      {
        if (stand_names[static_cast<size_t>(joint)].as<std::string>() !=
            params.joint_order_isaac[static_cast<size_t>(joint)])
          throw std::runtime_error("AMP stand profile joint order differs from the tracking policy");
        stand_joint_pos(joint) = stand_positions[static_cast<size_t>(joint)].as<double>();
      }

      Eigen::Vector3d reference_root(0.0, 0.0, base_position[2].as<double>());
      Eigen::Quaterniond reference_quat(base_quat[3].as<double>(), base_quat[0].as<double>(),
                                        base_quat[1].as<double>(), base_quat[2].as<double>());
      reference_quat.normalize();
      const double yaw = std::atan2(2.0 * (reference_quat.w() * reference_quat.z() +
                                           reference_quat.x() * reference_quat.y()),
                                    1.0 - 2.0 * (reference_quat.y() * reference_quat.y() +
                                                 reference_quat.z() * reference_quat.z()));
      reference_quat = Eigen::Quaterniond(std::cos(-0.5 * yaw), 0.0, 0.0, std::sin(-0.5 * yaw)) * reference_quat;
      reference_quat.normalize();

      const int target_fps = std::max(1, params.policy_frequency);
      const double duration = static_cast<double>(declared_count - 1) / source_fps;
      const int output_count = std::max(2, static_cast<int>(std::llround(duration * target_fps)) + 1);
      if (output_count > text_motion_max_frames_)
        throw std::runtime_error("resampled motion exceeds text_motion_max_frames");

      std::vector<MotionData> packaged(static_cast<size_t>(output_count));
      for (int output_index = 0; output_index < output_count; ++output_index)
      {
        const double source_position = std::min(duration, static_cast<double>(output_index) / target_fps) * source_fps;
        const size_t left = std::min(static_cast<size_t>(std::floor(source_position)), source_frames.size() - 1);
        const size_t right = std::min(left + 1, source_frames.size() - 1);
        const double alpha = source_position - static_cast<double>(left);
        MotionData& frame = packaged[static_cast<size_t>(output_index)];
        frame.joint_pos = stand_joint_pos;
        frame.joint_vel = Eigen::VectorXd::Zero(21);
        frame.root_pos = reference_root;
        frame.root_quat.resize(4);
        frame.root_quat << reference_quat.w(), reference_quat.x(), reference_quat.y(), reference_quat.z();
        frame.body_quat = reference_quat;
        for (size_t column = 0; column < target_columns.size(); ++column)
          frame.joint_pos(static_cast<Eigen::Index>(target_columns[column])) =
              source_frames[left][column] * (1.0 - alpha) + source_frames[right][column] * alpha;
      }

      // Gaussian low-pass only on joints supplied by infer. It changes neither
      // frame count nor duration, and leaves AMP-filled lower-body joints exact.
      if (text_motion_lowpass_cutoff_hz_ > 0.0)
      {
        const double sigma = target_fps / (2.0 * M_PI * text_motion_lowpass_cutoff_hz_);
        const int radius = std::max(1, static_cast<int>(std::ceil(3.0 * sigma)));
        std::vector<double> kernel(static_cast<size_t>(2 * radius + 1));
        double sum = 0.0;
        for (int k = -radius; k <= radius; ++k)
        {
          const double weight = std::exp(-0.5 * k * k / (sigma * sigma));
          kernel[static_cast<size_t>(k + radius)] = weight;
          sum += weight;
        }
        for (double& weight : kernel) weight /= sum;
        const std::vector<MotionData> unfiltered = packaged;
        for (int i = 1; i + 1 < output_count; ++i)
        {
          for (size_t target : target_columns)
          {
            double value = 0.0;
            for (int k = -radius; k <= radius; ++k)
            {
              const int sample = std::clamp(i + k, 0, output_count - 1);
              value += kernel[static_cast<size_t>(k + radius)] *
                       unfiltered[static_cast<size_t>(sample)].joint_pos(static_cast<Eigen::Index>(target));
            }
            packaged[static_cast<size_t>(i)].joint_pos(static_cast<Eigen::Index>(target)) = value;
          }
        }
      }

      for (int i = 0; i < output_count; ++i)
      {
        const int before = std::max(0, i - 1);
        const int after = std::min(output_count - 1, i + 1);
        const double scale = (after == before) ? 0.0 : target_fps / static_cast<double>(after - before);
        packaged[static_cast<size_t>(i)].joint_vel =
            (packaged[static_cast<size_t>(after)].joint_pos - packaged[static_cast<size_t>(before)].joint_pos) * scale;
      }

      precomputed_motion_data_.swap(packaged);
      motion_data_size_ = output_count;
      imitation_active_ = false;
      resetImitationState();
      refreshInferenceEnabledLocked();
      const std::string text = root["text"] && root["text"].IsScalar() ? root["text"].as<std::string>() : "";
      description = std::to_string(output_count) + " dynamic-reference frames at " +
                    std::to_string(target_fps) + " Hz, fixed_policy=" + text_motion_policy_ +
                    (text.empty() ? "" : ", text=" + text);
      return true;
    }

    if (precomputed_motion_data_.empty())
      throw std::runtime_error("tracking policy has no template motion");

    bool accepts_joint_pos = false;
    bool accepts_joint_vel = false;
    for (const auto& item : params.command_items)
    {
      accepts_joint_pos = accepts_joint_pos || (item.type == "joint_pos" && item.dim == params.num_actions);
      accepts_joint_vel = accepts_joint_vel || (item.type == "joint_vel" && item.dim == params.num_actions);
    }
    if (!accepts_joint_pos || !accepts_joint_vel)
      throw std::runtime_error("configured tracking policy must command full joint_pos and joint_vel");

    std::vector<size_t> target_columns;
    target_columns.reserve(source_names.size());
    for (const std::string& name : source_names)
    {
      const auto it = std::find(params.joint_order_isaac.begin(), params.joint_order_isaac.end(), name);
      if (it == params.joint_order_isaac.end()) throw std::runtime_error("unknown joint name: " + name);
      target_columns.push_back(static_cast<size_t>(std::distance(params.joint_order_isaac.begin(), it)));
    }

    const MotionData template_frame = precomputed_motion_data_.front();
    if (template_frame.joint_pos.size() != params.num_actions)
      throw std::runtime_error("tracking policy template has invalid joint_pos size");

    // Match build_wbc_motion_from_amp_stand.py: begin with the measured AMP
    // standing profile, keep joints absent from the text trajectory (the 12
    // lower-body joints) fixed there, and smoothly move only supplied upper
    // joints into and out of the generated motion.
    const YAML::Node stand_profile = YAML::LoadFile(text_motion_amp_stand_profile_);
    if (requiredScalar<std::string>(stand_profile, "robot_type") != params.robot_type)
      throw std::runtime_error("AMP stand profile robot_type does not match the tracking policy");
    const YAML::Node stand_names = stand_profile["joint_names"];
    const YAML::Node stand_positions = stand_profile["joint_position_median_rad"];
    if (!stand_names || !stand_names.IsSequence() || !stand_positions || !stand_positions.IsSequence() ||
        static_cast<int>(stand_names.size()) != params.num_actions ||
        static_cast<int>(stand_positions.size()) != params.num_actions)
      throw std::runtime_error("AMP stand profile must contain all policy joints");

    Eigen::VectorXd stand_joint_pos(params.num_actions);
    for (int joint = 0; joint < params.num_actions; ++joint)
    {
      if (stand_names[static_cast<size_t>(joint)].as<std::string>() !=
          params.joint_order_isaac[static_cast<size_t>(joint)])
        throw std::runtime_error("AMP stand profile joint order differs from the tracking policy");
      const double value = stand_positions[static_cast<size_t>(joint)].as<double>();
      if (!std::isfinite(value) || std::abs(value) > 6.4)
        throw std::runtime_error("AMP stand profile contains an invalid joint angle");
      stand_joint_pos(joint) = value;
    }

    const int target_fps = std::max(1, params.policy_frequency);
    const double duration = static_cast<double>(declared_count - 1) / source_fps;
    const int source_output_count = std::max(2, static_cast<int>(std::llround(duration * target_fps)) + 1);
    const int prefix_frames = text_motion_hold_frames_ + text_motion_transition_frames_;
    const int source_end_frame = prefix_frames + source_output_count;
    const int return_end_frame = source_end_frame + text_motion_return_frames_;
    const int output_count = return_end_frame + text_motion_return_settle_frames_;
    if (output_count > text_motion_max_frames_)
      throw std::runtime_error("AMP-padded motion exceeds text_motion_max_frames");

    std::vector<MotionData> packaged(static_cast<size_t>(output_count), template_frame);
    for (MotionData& frame : packaged)
    {
      frame.joint_pos = stand_joint_pos;
      frame.joint_vel = Eigen::VectorXd::Zero(params.num_actions);
    }

    for (int transition_index = 0; transition_index < text_motion_transition_frames_; ++transition_index)
    {
      double alpha = static_cast<double>(transition_index) / (text_motion_transition_frames_ - 1);
      alpha = alpha * alpha * (3.0 - 2.0 * alpha);
      MotionData& frame = packaged[static_cast<size_t>(text_motion_hold_frames_ + transition_index)];
      for (size_t column = 0; column < target_columns.size(); ++column)
      {
        const Eigen::Index target = static_cast<Eigen::Index>(target_columns[column]);
        frame.joint_pos(target) = stand_joint_pos(target) + alpha * (source_frames.front()[column] - stand_joint_pos(target));
      }
    }

    for (int output_index = 0; output_index < source_output_count; ++output_index)
    {
      const double source_position = std::min(duration, static_cast<double>(output_index) / target_fps) * source_fps;
      const size_t left = std::min(static_cast<size_t>(std::floor(source_position)), source_frames.size() - 1);
      const size_t right = std::min(left + 1, source_frames.size() - 1);
      const double alpha = source_position - static_cast<double>(left);
      MotionData& frame = packaged[static_cast<size_t>(prefix_frames + output_index)];
      for (size_t column = 0; column < target_columns.size(); ++column)
      {
        frame.joint_pos(static_cast<Eigen::Index>(target_columns[column])) =
            source_frames[left][column] * (1.0 - alpha) + source_frames[right][column] * alpha;
      }
    }

    for (int return_index = 0; return_index < text_motion_return_frames_; ++return_index)
    {
      double alpha = static_cast<double>(return_index + 1) / text_motion_return_frames_;
      alpha = alpha * alpha * (3.0 - 2.0 * alpha);
      MotionData& frame = packaged[static_cast<size_t>(source_end_frame + return_index)];
      for (size_t column = 0; column < target_columns.size(); ++column)
      {
        const Eigen::Index target = static_cast<Eigen::Index>(target_columns[column]);
        frame.joint_pos(target) = source_frames.back()[column] + alpha * (stand_joint_pos(target) - source_frames.back()[column]);
      }
    }

    const double velocity_scale = 0.5 * target_fps;
    for (int i = 0; i < output_count; ++i)
    {
      const int before = std::max(0, i - 1);
      const int after = std::min(output_count - 1, i + 1);
      const double scale = (after == before) ? 0.0 : (i == 0 || i == output_count - 1 ? target_fps : velocity_scale);
      packaged[static_cast<size_t>(i)].joint_vel =
          (packaged[static_cast<size_t>(after)].joint_pos - packaged[static_cast<size_t>(before)].joint_pos) * scale;
      if (packaged[static_cast<size_t>(i)].joint_vel.cwiseAbs().maxCoeff() > 10.0)
        throw std::runtime_error("resampled trajectory exceeds 10 rad/s joint velocity limit");
    }

    precomputed_motion_data_.swap(packaged);
    motion_data_size_ = output_count;
    imitation_active_ = false;
    resetImitationState();
    refreshInferenceEnabledLocked();

    const std::string text = root["text"] && root["text"].IsScalar() ? root["text"].as<std::string>() : "";
    description = std::to_string(output_count) + " AMP-padded frames at " + std::to_string(target_fps) +
                  " Hz, policy=" + text_motion_policy_ + (text.empty() ? "" : ", text=" + text);
    return true;
  } catch (const std::exception& exc)
  {
    error = exc.what();
    return false;
  }
}
