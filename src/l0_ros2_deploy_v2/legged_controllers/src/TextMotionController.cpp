#include "legged_controllers/TextMotionController.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace legged {
namespace {

const std::vector<std::string> kExpectedJoints = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_roll_joint",
    "neck_yaw_joint", "neck_pitch_joint", "neck_roll_joint"};

const std::map<std::string, std::pair<double, double>> kL0V5Limits = {
    {"left_shoulder_pitch_joint", {-2.618, 2.618}}, {"left_shoulder_roll_joint", {-1.571, 1.571}},
    {"left_elbow_roll_joint", {-1.571, 1.571}},     {"right_shoulder_pitch_joint", {-2.618, 2.618}},
    {"right_shoulder_roll_joint", {-1.571, 1.571}}, {"right_elbow_roll_joint", {-1.571, 1.571}},
    {"neck_yaw_joint", {-1.571, 1.571}},           {"neck_pitch_joint", {-0.34906585, 0.17453293}},
    {"neck_roll_joint", {-0.297, 0.297}}};

}  // namespace

TextMotionController::~TextMotionController() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    worker_stop_ = true;
  }
  queue_cv_.notify_one();
  if (worker_thread_.joinable()) worker_thread_.join();
}

bool TextMotionController::configure(const Params& params, const std::vector<std::string>& robot_joint_names,
                                     rclcpp::Logger logger, std::string& error) {
  params_ = params;
  logger_ = logger;
  robot_joint_names_ = robot_joint_names;
  if (!std::isfinite(params_.entry_blend_sec) || params_.entry_blend_sec < 0.0 ||
      !std::isfinite(params_.exit_blend_sec) || params_.exit_blend_sec < 0.0 ||
      params_.max_payload_bytes < 1024U || params_.max_frames < 2U)
  {
    error = "invalid text-motion limits/blend configuration";
    return false;
  }
  for (const auto& name : kExpectedJoints)
  {
    if (std::find(robot_joint_names_.begin(), robot_joint_names_.end(), name) == robot_joint_names_.end())
    {
      error = "robot joint list is missing " + name;
      return false;
    }
  }
  controlled_joints_.assign(robot_joint_names_.size(), false);
  q_cmd_.assign(robot_joint_names_.size(), 0.0);
  blend_start_.assign(robot_joint_names_.size(), 0.0);
  if (params_.enabled && !worker_thread_.joinable()) worker_thread_ = std::thread(&TextMotionController::workerLoop, this);
  return true;
}

bool TextMotionController::finiteInRange(const std::string& name, double value) {
  const auto limit = kL0V5Limits.find(name);
  return limit != kL0V5Limits.end() && std::isfinite(value) && value >= limit->second.first - 1e-6 &&
         value <= limit->second.second + 1e-6;
}

bool TextMotionController::parseJson(const std::string& payload, Trajectory& parsed, std::string& error) const {
  try
  {
    const YAML::Node root = YAML::Load(payload);
    if (!root.IsMap() || root["format"].as<std::string>("") != "gmr_upper_body_joint_trajectory_v1" ||
        root["robot"].as<std::string>("") != "l0_v5" || root["angle_unit"].as<std::string>("") != "rad")
      throw std::runtime_error("format/robot/angle_unit mismatch");
    parsed.fps = root["fps"].as<double>(0.0);
    if (!std::isfinite(parsed.fps) || parsed.fps <= 0.0 || parsed.fps > 200.0)
      throw std::runtime_error("fps must be finite and in (0, 200]");

    const YAML::Node names = root["joint_names"];
    if (!names.IsSequence() || names.size() != kExpectedJoints.size())
      throw std::runtime_error("joint_names must contain exactly 9 joints");
    std::set<std::string> unique_names;
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      const std::string name = names[i].as<std::string>();
      if (name != kExpectedJoints[i] || !unique_names.insert(name).second)
        throw std::runtime_error("joint_names order does not match the L0 v5 text model");
      const auto found = std::find(robot_joint_names_.begin(), robot_joint_names_.end(), name);
      if (found == robot_joint_names_.end()) throw std::runtime_error("unknown robot joint " + name);
      parsed.joint_indices.push_back(static_cast<std::size_t>(found - robot_joint_names_.begin()));
    }

    const YAML::Node frames = root["frames"];
    const std::size_t declared = root["frame_count"].as<std::size_t>(0U);
    if (!frames.IsSequence() || frames.size() != declared || declared < 2U || declared > params_.max_frames)
      throw std::runtime_error("frame_count is invalid or does not match frames");
    parsed.frames.reserve(declared);
    for (std::size_t frame_index = 0; frame_index < declared; ++frame_index)
    {
      const YAML::Node frame = frames[frame_index];
      if (!frame.IsMap() || frame["frame_index"].as<std::size_t>(declared) != frame_index)
        throw std::runtime_error("frame_index sequence is invalid");
      const YAML::Node positions = frame["joint_positions"];
      if (!positions.IsSequence() || positions.size() != names.size())
        throw std::runtime_error("each frame must contain exactly 9 joint positions");
      std::vector<double> row;
      row.reserve(positions.size());
      for (std::size_t j = 0; j < positions.size(); ++j)
      {
        const double value = positions[j].as<double>();
        if (!finiteInRange(kExpectedJoints[j], value))
          throw std::runtime_error("non-finite or out-of-limit value for " + kExpectedJoints[j]);
        row.push_back(value);
      }
      parsed.frames.push_back(std::move(row));
    }
    const YAML::Node end_pose = root["end_pose"];
    if (end_pose && end_pose.IsMap() && end_pose["source"].as<std::string>("") == "provided")
    {
      const YAML::Node positions = end_pose["joint_positions"];
      if (!positions.IsSequence() || positions.size() != kExpectedJoints.size())
        throw std::runtime_error("provided end_pose must contain nine joint positions");
      for (std::size_t j = 0; j < positions.size(); ++j)
      {
        const double target = positions[j].as<double>();
        if (!finiteInRange(kExpectedJoints[j], target) ||
            std::fabs(parsed.frames.back()[j] - target) > 0.005)
          throw std::runtime_error("trajectory does not finish at provided end_pose");
      }
      parsed.has_explicit_end_pose = true;
    }
    parsed.active_joints.assign(names.size(), true);
    // ONNX fills all nine channels even when a prompt only moves one limb.
    // Constant channels must stay under AMP control; otherwise an arm-only
    // prompt also raises the head or moves the opposite arm to the ONNX pose.
    constexpr double kMinJointMotionRad = 0.01;
    for (std::size_t j = 0U; j < names.size(); ++j)
    {
      const double initial = parsed.frames.front()[j];
      parsed.active_joints[j] = std::any_of(parsed.frames.begin(), parsed.frames.end(),
                                           [j, initial, kMinJointMotionRad](const std::vector<double>& frame) {
                                             return std::fabs(frame[j] - initial) > kMinJointMotionRad;
                                           });
    }
  } catch (const std::exception& exc)
  {
    error = exc.what();
    return false;
  }

  return true;
}

bool TextMotionController::submitJson(const std::string& payload, std::string& error) {
  if (!params_.enabled)
  {
    error = "text motion is disabled";
    return false;
  }
  if (payload.empty() || payload.size() > params_.max_payload_bytes)
  {
    error = "payload is empty or exceeds text_motion.max_payload_bytes";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queued_payload_ = payload;
    queued_request_id_ = ++latest_request_id_;
  }
  // A callback is invoked only after DDS has delivered/deserialized the complete String message.
  // The callback only copies and signals; all parsing is performed off the executor/control paths.
  queue_cv_.notify_one();
  return true;
}

void TextMotionController::pushPreparationStatus(std::string status) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  preparation_status_.push_back(std::move(status));
}

std::string TextMotionController::takePreparationStatus() {
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (preparation_status_.empty()) return {};
  std::string status = std::move(preparation_status_.front());
  preparation_status_.pop_front();
  return status;
}

void TextMotionController::workerLoop() {
  while (true)
  {
    std::string payload;
    std::uint64_t request_id = 0;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() { return worker_stop_ || !queued_payload_.empty(); });
      if (worker_stop_) return;
      payload.swap(queued_payload_);
      request_id = queued_request_id_;
    }

    Trajectory parsed;
    std::string error;
    if (!parseJson(payload, parsed, error))
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (request_id == latest_request_id_) pushPreparationStatus("rejected: " + error);
      continue;
    }

    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      if (request_id != latest_request_id_) continue;
      std::lock_guard<std::mutex> pending_lock(pending_mutex_);
      pending_ = std::move(parsed);
      pending_available_ = true;  // Visible only after every declared frame has passed validation.
    }
    pushPreparationStatus("ready: complete trajectory validated");
  }
}

void TextMotionController::sampleTrajectory(double frame_position) {
  const std::size_t last = current_.frames.size() - 1U;
  const double bounded = std::max(0.0, std::min(frame_position, static_cast<double>(last)));
  const std::size_t first = static_cast<std::size_t>(std::floor(bounded));
  const std::size_t second = std::min(first + 1U, last);
  const double alpha = bounded - static_cast<double>(first);
  for (std::size_t j = 0; j < current_.joint_indices.size(); ++j)
  {
    const std::size_t global = current_.joint_indices[j];
    q_cmd_[global] = current_.frames[first][j] + alpha * (current_.frames[second][j] - current_.frames[first][j]);
  }
}

std::string TextMotionController::step(double wall_sec, const std::vector<double>& base_positions,
                                       bool playback_allowed) {
  Trajectory incoming;
  bool have_incoming = false;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_available_)
    {
      incoming = std::move(pending_);
      pending_available_ = false;
      have_incoming = true;
    }
  }
  if (have_incoming)
  {
    if (!playback_allowed || base_positions.size() != robot_joint_names_.size())
    {
      stop();
      return "rejected: controller must be in unobstructed Walking mode";
    }
    const bool replacing_active_motion = active_.load();
    const std::vector<bool> previous_controlled = controlled_joints_;
    const std::vector<double> previous_command = q_cmd_;
    current_ = std::move(incoming);
    std::fill(controlled_joints_.begin(), controlled_joints_.end(), false);
    for (std::size_t j = 0; j < current_.joint_indices.size(); ++j)
      if (current_.active_joints[j]) controlled_joints_[current_.joint_indices[j]] = true;
    blend_start_ = base_positions;
    if (replacing_active_motion)
    {
      for (const std::size_t index : current_.joint_indices)
        if (index < previous_controlled.size() && previous_controlled[index]) blend_start_[index] = previous_command[index];
    }
    started_at_ = wall_sec;
    active_.store(true);
  }
  if (!active_.load()) return have_incoming ? "started" : "";
  if (!playback_allowed || base_positions.size() != robot_joint_names_.size())
  {
    stop();
    return "stopped: playback was preempted by controller state";
  }

  const double elapsed = std::max(0.0, wall_sec - started_at_);
  const double trajectory_sec = static_cast<double>(current_.frames.size() - 1U) / current_.fps;
  if (elapsed <= trajectory_sec)
  {
    sampleTrajectory(elapsed * current_.fps);
    if (params_.entry_blend_sec > 1e-9 && elapsed < params_.entry_blend_sec)
    {
      const double alpha = elapsed / params_.entry_blend_sec;
      for (const std::size_t index : current_.joint_indices)
        q_cmd_[index] = blend_start_[index] + alpha * (q_cmd_[index] - blend_start_[index]);
    }
    return have_incoming ? "started" : "";
  }

  if (params_.exit_blend_sec > 1e-9 && elapsed < trajectory_sec + params_.exit_blend_sec)
  {
    const double alpha = (elapsed - trajectory_sec) / params_.exit_blend_sec;
    for (std::size_t j = 0; j < current_.joint_indices.size(); ++j)
    {
      const std::size_t index = current_.joint_indices[j];
      // A supplied end_pose is the explicit return target. Blending it back
      // to AMP here can cause an extra high-gain nod after the model's return.
      q_cmd_[index] = current_.has_explicit_end_pose ? current_.frames.back()[j] :
          current_.frames.back()[j] + alpha * (base_positions[index] - current_.frames.back()[j]);
    }
    return have_incoming ? "started" : "";
  }
  stop();
  return "completed";
}

bool TextMotionController::getJointPosition(std::size_t joint_index, double& q) const {
  if (!active_.load() || joint_index >= controlled_joints_.size() || !controlled_joints_[joint_index]) return false;
  q = q_cmd_[joint_index];
  return true;
}

void TextMotionController::stop() {
  active_.store(false);
  current_ = Trajectory{};
  std::fill(controlled_joints_.begin(), controlled_joints_.end(), false);
}

}  // namespace legged
