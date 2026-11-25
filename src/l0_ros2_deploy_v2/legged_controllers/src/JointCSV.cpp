#include "legged_controllers/JointCSV.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "legged_controllers/JointCsvFormat.h"

namespace legged {

namespace {
static constexpr double kDefaultCsvKp = 3;
static constexpr double kDefaultCsvKd = 0.3;

int global_index_of(const std::vector<std::string>& names, const std::string& j) {
  for (size_t i = 0; i < names.size(); ++i)
  {
    if (names[i] == j) return static_cast<int>(i);
  }
  return -1;
}

}  // namespace

void JointCsvController::setEnabled(bool e) {
  enabled_ = e;
  if (!e)
  {
    phase_ = Phase::Idle;
    blend_elapsed_ = 0.0;
    finished_event_ = false;
  }
}

void JointCsvController::setBlendDuration(double sec) {
  // sec <= 0: disable RL→CSV blend (jump to CSV frame 0 immediately on trigger).
  blend_duration_sec_ = (sec > 1e-9) ? sec : 0.0;
}

void JointCsvController::initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger) {
  logger_ = logger;
  robot_joint_names_ = joint_names;
  is_csv_joint_.assign(joint_names.size(), false);
  q_cmd_.assign(joint_names.size(), 0.0);
  kp_cmd_.assign(joint_names.size(), kDefaultCsvKp);
  kd_cmd_.assign(joint_names.size(), kDefaultCsvKd);
  q_blend_start_.assign(joint_names.size(), 0.0);
  phase_ = Phase::Idle;
  blend_elapsed_ = 0.0;
  finished_event_ = false;
}

void JointCsvController::stop() {
  phase_ = Phase::Idle;
  blend_elapsed_ = 0.0;
  play_anchor_wall_sec_ = 0.0;
  finished_event_ = false;
}

bool JointCsvController::load(const std::string& path) {
  frames_.clear();
  col_to_global_.clear();
  frame_time_ = 0.0;
  has_csv_kp_ = false;
  has_csv_kd_ = false;
  phase_ = Phase::Idle;
  blend_elapsed_ = 0.0;
  finished_event_ = false;
  std::fill(kp_cmd_.begin(), kp_cmd_.end(), kDefaultCsvKp);
  std::fill(kd_cmd_.begin(), kd_cmd_.end(), kDefaultCsvKd);

  std::ifstream in(path);
  if (!in)
  {
    RCLCPP_ERROR(logger_, "JointCsv: cannot open %s", path.c_str());
    return false;
  }

  std::string line;
  int declared_frames = -1;

  if (!std::getline(in, line))
  {
    RCLCPP_ERROR(logger_, "JointCsv: empty file %s", path.c_str());
    return false;
  }
  {
    std::istringstream iss(trimJointCsvWhitespace(line));
    std::string tag;
    iss >> tag;
    if (tag != "Frames:")
    {
      RCLCPP_ERROR(logger_, "JointCsv: expected 'Frames:' first line");
      return false;
    }
    iss >> declared_frames;
    if (declared_frames < 0)
    {
      RCLCPP_ERROR(logger_, "JointCsv: bad Frames count");
      return false;
    }
  }

  if (!std::getline(in, line))
  {
    RCLCPP_ERROR(logger_, "JointCsv: missing Frame Time line");
    return false;
  }
  {
    std::istringstream iss(trimJointCsvWhitespace(line));
    std::string a, b;
    iss >> a >> b;
    if (a != "Frame" || b != "Time:")
    {
      RCLCPP_ERROR(logger_, "JointCsv: expected 'Frame Time:' second line");
      return false;
    }
    iss >> frame_time_;
    if (frame_time_ <= 0.0)
    {
      RCLCPP_ERROR(logger_, "JointCsv: bad frame time");
      return false;
    }
  }

  if (!std::getline(in, line))
  {
    RCLCPP_ERROR(logger_, "JointCsv: missing header line");
    return false;
  }
  std::vector<std::string> header = splitJointCsvLine(line);
  if (header.empty())
  {
    RCLCPP_ERROR(logger_, "JointCsv: empty header");
    return false;
  }

  std::fill(is_csv_joint_.begin(), is_csv_joint_.end(), false);
  col_to_global_.clear();
  col_to_global_.reserve(header.size());
  for (const std::string& name : header)
  {
    int gi = global_index_of(robot_joint_names_, name);
    if (gi < 0)
    {
      RCLCPP_ERROR(logger_, "JointCsv: joint '%s' not in robot joint list", name.c_str());
      return false;
    }
    col_to_global_.push_back(static_cast<size_t>(gi));
    is_csv_joint_[static_cast<size_t>(gi)] = true;
  }

  auto parse_gain_line = [this, &header](const std::string& raw, const char* tag, std::vector<double>& target) -> bool {
    const std::string prefix = std::string(tag) + ":";
    if (raw.rfind(prefix, 0) != 0) return false;
    const std::string payload = trimJointCsvWhitespace(raw.substr(prefix.size()));
    std::vector<std::string> cells = splitJointCsvLine(payload);
    if (cells.size() != header.size())
    {
      RCLCPP_ERROR(logger_, "JointCsv: %s expects %zu values, got %zu", tag, header.size(), cells.size());
      return false;
    }
    for (size_t c = 0; c < cells.size(); ++c)
    {
      double v = 0.0;
      try
      { v = std::stod(cells[c]); } catch (...)
      {
        RCLCPP_ERROR(logger_, "JointCsv: bad number in %s row", tag);
        return false;
      }
      target[col_to_global_[c]] = v;
    }
    return true;
  };

  while (std::getline(in, line))
  {
    line = trimJointCsvWhitespace(line);
    if (line.empty()) continue;
    if (line.rfind("KP:", 0) == 0)
    {
      if (!parse_gain_line(line, "KP", kp_cmd_)) return false;
      has_csv_kp_ = true;
      continue;
    }
    if (line.rfind("KD:", 0) == 0)
    {
      if (!parse_gain_line(line, "KD", kd_cmd_)) return false;
      has_csv_kd_ = true;
      continue;
    }
    std::vector<std::string> cells = splitJointCsvLine(line);
    if (cells.size() != header.size())
    {
      RCLCPP_WARN(logger_, "JointCsv: skip row with %zu cols (expected %zu)", cells.size(), header.size());
      continue;
    }
    std::vector<double> row;
    row.reserve(cells.size());
    for (const std::string& c : cells)
    {
      try
      { row.push_back(std::stod(c)); } catch (...)
      {
        RCLCPP_ERROR(logger_, "JointCsv: bad number in row");
        return false;
      }
    }
    frames_.push_back(std::move(row));
  }

  if (frames_.empty())
  {
    RCLCPP_ERROR(logger_, "JointCsv: no data rows");
    return false;
  }

  if (declared_frames >= 0 && static_cast<int>(frames_.size()) != declared_frames)
  {
    RCLCPP_WARN(logger_, "JointCsv: Frames: %d but got %zu data rows — using rows", declared_frames, frames_.size());
  }

  return true;
}

void JointCsvController::applyFirstFrameToQCmd() {
  if (frames_.empty() || col_to_global_.empty()) return;
  const std::vector<double>& r0 = frames_[0];
  const size_t cols = std::min(col_to_global_.size(), r0.size());
  for (size_t c = 0; c < cols; ++c)
  {
    q_cmd_[col_to_global_[c]] = r0[c];
  }
}

void JointCsvController::applyLastFrameToQCmd() {
  if (frames_.empty() || col_to_global_.empty()) return;
  const std::vector<double>& rN = frames_.back();
  const size_t cols = std::min(col_to_global_.size(), rN.size());
  for (size_t c = 0; c < cols; ++c)
  {
    q_cmd_[col_to_global_[c]] = rN[c];
  }
}

void JointCsvController::setQCmdFromPlaybackTimeNoLoop(double t_play) {
  const size_t n = frames_.size();
  if (n == 0 || frame_time_ <= 0.0 || col_to_global_.empty()) return;

  double t = t_play;
  if (t < 0.0) t = 0.0;
  const double dur = static_cast<double>(n) * frame_time_;
  if (t > dur) t = dur;

  double frame_f = t / frame_time_;
  size_t i0 = static_cast<size_t>(std::floor(frame_f));
  if (i0 >= n) i0 = n - 1;
  const size_t i1 = std::min(i0 + 1, n - 1);
  const double alpha = frame_f - std::floor(frame_f);

  const std::vector<double>& r0 = frames_[i0];
  const std::vector<double>& r1 = frames_[i1];
  const size_t cols = std::min({r0.size(), r1.size(), col_to_global_.size()});

  for (size_t c = 0; c < cols; ++c)
  {
    const double q = (1.0 - alpha) * r0[c] + alpha * r1[c];
    q_cmd_[col_to_global_[c]] = q;
  }
}

void JointCsvController::setQCmdFromTimelineTime(double t_loop_sec) {
  const size_t n = frames_.size();
  if (n == 0 || frame_time_ <= 0.0 || col_to_global_.empty()) return;

  double dur = static_cast<double>(n) * frame_time_;
  double t = std::fmod(t_loop_sec, dur);
  if (t < 0.0) t += dur;

  double frame_f = t / frame_time_;
  size_t i0 = static_cast<size_t>(std::floor(frame_f));
  if (i0 >= n) i0 = n - 1;
  size_t i1 = (i0 + 1 < n) ? (i0 + 1) : 0;
  double alpha = frame_f - std::floor(frame_f);

  const std::vector<double>& r0 = frames_[i0];
  const std::vector<double>& r1 = frames_[i1];
  const size_t cols = std::min({r0.size(), r1.size(), col_to_global_.size()});

  for (size_t c = 0; c < cols; ++c)
  {
    double q = (1.0 - alpha) * r0[c] + alpha * r1[c];
    q_cmd_[col_to_global_[c]] = q;
  }
}

void JointCsvController::updateFromTime(double t_loop_sec) { setQCmdFromTimelineTime(t_loop_sec); }

void JointCsvController::onTrigger(const std::vector<double>& q_all_motor, double wall_sec) {
  if (!enabled_ || empty()) return;
  if (phase_ == Phase::Playing) return;

  if (phase_ == Phase::Idle || phase_ == Phase::Done)
  {
    if (blend_duration_sec_ <= 1e-9)
    {
      blend_elapsed_ = 0.0;
      play_anchor_wall_sec_ = wall_sec;
      setQCmdFromPlaybackTimeNoLoop(0.0);
      phase_ = Phase::Playing;
      finished_event_ = false;
      return;
    }
    const size_t n = std::min(q_all_motor.size(), q_blend_start_.size());
    for (size_t j = 0; j < n; ++j)
    {
      if (is_csv_joint_[j]) q_blend_start_[j] = q_all_motor[j];
    }
    blend_elapsed_ = 0.0;
    phase_ = Phase::Blending;
    finished_event_ = false;
    return;
  }

  // Optional: skip remaining blend and start timeline immediately.
  if (phase_ == Phase::Blending)
  {
    phase_ = Phase::Playing;
    play_anchor_wall_sec_ = wall_sec;
    setQCmdFromPlaybackTimeNoLoop(0.0);
    finished_event_ = false;
  }
}

void JointCsvController::step(double dt, double wall_sec) {
  if (!enabled_ || empty() || duration() <= 0.0) return;

  switch (phase_)
  {
    case Phase::Idle:
      break;
    case Phase::Blending: {
      blend_elapsed_ += dt;
      const double denom = blend_duration_sec_;
      double alpha = (denom > 1e-9) ? (blend_elapsed_ / denom) : 1.0;
      if (alpha >= 1.0)
      {
        applyFirstFrameToQCmd();
        play_anchor_wall_sec_ = wall_sec;
        setQCmdFromPlaybackTimeNoLoop(0.0);
        phase_ = Phase::Playing;
        break;
      }
      if (frames_.empty()) break;
      const std::vector<double>& r0 = frames_[0];
      const size_t cols = std::min(col_to_global_.size(), r0.size());
      for (size_t c = 0; c < cols; ++c)
      {
        const size_t gi = col_to_global_[c];
        const double q0 = q_blend_start_[gi];
        const double q1 = r0[c];
        q_cmd_[gi] = (1.0 - alpha) * q0 + alpha * q1;
      }
      break;
    }
    case Phase::Playing: {
      const double t = wall_sec - play_anchor_wall_sec_;
      const double dur = duration();
      if (t >= dur - 1e-9)
      {
        applyLastFrameToQCmd();
        finished_event_ = true;
        phase_ = hold_last_frame_after_finish_ ? Phase::Holding : Phase::Done;
        break;
      }
      setQCmdFromPlaybackTimeNoLoop(t);
      break;
    }
    case Phase::Holding:
      break;
    case Phase::Done:
      break;
  }
}

bool JointCsvController::getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd,
                                     double& tau_ff) const {
  if (!enabled_ || global_j >= is_csv_joint_.size() || !is_csv_joint_[global_j]) return false;
  // Release command ownership after playback is complete so RL can resume automatically.
  if (phase_ == Phase::Idle || phase_ == Phase::Done) return false;
  q = q_cmd_[global_j];
  dq = 0.0;
  kp = kp_cmd_[global_j];
  kd = kd_cmd_[global_j];
  tau_ff = 0.0;
  return true;
}

bool JointCsvController::consumeFinishedEvent() {
  const bool finished = finished_event_;
  finished_event_ = false;
  return finished;
}

void JointCsvController::copyGainCommands(std::vector<double>& kps, std::vector<double>& kds) const {
  if (kps.size() != kp_cmd_.size()) kps.resize(kp_cmd_.size(), 0.0);
  if (kds.size() != kd_cmd_.size()) kds.resize(kd_cmd_.size(), 0.0);

  for (size_t j = 0; j < is_csv_joint_.size(); ++j)
  {
    if (!is_csv_joint_[j]) continue;
    kps[j] = kp_cmd_[j];
    kds[j] = kd_cmd_[j];
  }
}

bool JointCsvController::coversAllJoints() const {
  return !is_csv_joint_.empty() && std::all_of(is_csv_joint_.begin(), is_csv_joint_.end(), [](bool v) { return v; });
}

}  // namespace legged
