//
// Head script control: HeadScriptController implementation.
// Timeline parsing and evaluation for neck joints.
//
#include "legged_controllers/HeadScript.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace legged {

static const double kPi = 3.141592653589793;
static const double kDegToRad = kPi / 180.0;

// ===========================================================================
// Static helpers: head timeline parsing and evaluation
// ===========================================================================

static void loadHeadScriptGainsFromNode(const YAML::Node& hs, double kp[3], double kd[3], double tau_ff[3]) {
  auto load_key = [&hs](const char* key, double dst[3]) {
    if (!hs[key]) return;
    if (hs[key].IsScalar())
    {
      const double v = hs[key].as<double>();
      dst[0] = dst[1] = dst[2] = v;
    }
    else if (hs[key].IsSequence() && hs[key].size() >= 3)
    {
      for (int i = 0; i < 3; ++i) dst[i] = hs[key][i].as<double>(dst[i]);
    }
  };
  load_key("kp", kp);
  load_key("kd", kd);
  load_key("tau_ff", tau_ff);
}

static bool parseHeadConfig(const std::string& path, std::vector<HeadScriptSegment>& segments_out, double& duration_out,
                            double kp_script[3], double kd_script[3], double tau_script[3], rclcpp::Logger logger) {
  std::ifstream f(path);
  if (!f.good())
  {
    RCLCPP_WARN(logger, "Head script config file not found: %s", path.c_str());
    return false;
  }
  YAML::Node root;
  try
  { root = YAML::LoadFile(path); } catch (const std::exception& e)
  {
    RCLCPP_WARN(logger, "Head script config YAML error: %s", e.what());
    return false;
  }
  YAML::Node hs = root["head_script"];
  if (!hs)
  {
    RCLCPP_WARN(logger, "Head script config missing 'head_script' key");
    return false;
  }
  loadHeadScriptGainsFromNode(hs, kp_script, kd_script, tau_script);
  YAML::Node segs = hs["segments"];
  if (!segs || !segs.IsSequence())
  {
    RCLCPP_WARN(logger, "Head script config missing or invalid 'segments'");
    return false;
  }

  segments_out.clear();
  for (size_t i = 0; i < segs.size(); ++i)
  {
    const YAML::Node& seg = segs[i];
    HeadScriptSegment s;
    s.t_start = seg["t_start"].as<double>(0.0);
    s.t_end = seg["t_end"].as<double>(0.0);

    auto parse_axis = [&seg](const char* from_key, const char* to_key, const char* sin_amp_key, const char* sin_hz_key,
                             HeadScriptSegment::AxisMode& out_mode, double& from, double& to, double& sin_amp,
                             double& sin_hz) {
      if (seg[from_key] && seg[to_key])
      {
        out_mode = HeadScriptSegment::AxisMode::Lerp;
        from = seg[from_key].as<double>();
        to = seg[to_key].as<double>();
      }
      else if (seg[sin_amp_key] && seg[sin_hz_key])
      {
        out_mode = HeadScriptSegment::AxisMode::Sin;
        sin_amp = seg[sin_amp_key].as<double>();
        sin_hz = seg[sin_hz_key].as<double>();
      }
      else
      {
        out_mode = HeadScriptSegment::AxisMode::Constant;
      }
    };

    parse_axis("yaw_from", "yaw_to", "yaw_sin_amplitude_deg", "yaw_sin_frequency_hz", s.yaw_mode, s.yaw_from, s.yaw_to,
               s.yaw_sin_amp_deg, s.yaw_sin_hz);
    parse_axis("pitch_from", "pitch_to", "pitch_sin_amplitude_deg", "pitch_sin_frequency_hz", s.pitch_mode,
               s.pitch_from, s.pitch_to, s.pitch_sin_amp_deg, s.pitch_sin_hz);
    parse_axis("roll_from", "roll_to", "roll_sin_amplitude_deg", "roll_sin_frequency_hz", s.roll_mode, s.roll_from,
               s.roll_to, s.roll_sin_amp_deg, s.roll_sin_hz);

    if (seg["yaw"]) s.yaw = seg["yaw"].as<double>();
    if (seg["pitch"]) s.pitch = seg["pitch"].as<double>();
    if (seg["roll"]) s.roll = seg["roll"].as<double>();

    segments_out.push_back(s);
  }

  if (segments_out.empty()) return false;

  duration_out = 0.0;
  for (const auto& s : segments_out)
    if (s.t_end > duration_out) duration_out = s.t_end;
  return true;
}

static void evalHeadAtTime(double t_sec, const std::vector<HeadScriptSegment>& segments, std::vector<double>& cmd_out,
                           std::vector<double>& cmd_dq_out) {
  if (segments.empty()) return;

  double yaw = 0.0, pitch = 0.0, roll = 0.0;
  double dyaw = 0.0, dpitch = 0.0, droll = 0.0;

  const HeadScriptSegment* seg = nullptr;
  for (const auto& s : segments)
  {
    if (t_sec >= s.t_start && t_sec < s.t_end)
    {
      seg = &s;
      break;
    }
  }

  if (seg)
  {
    double dt = seg->t_end - seg->t_start;
    double u = (dt > 1e-6) ? std::max(0.0, std::min(1.0, (t_sec - seg->t_start) / dt)) : 0.0;
    auto lerp = [](double a, double b, double t) { return a + (b - a) * t; };

    auto eval_axis = [&](HeadScriptSegment::AxisMode mode, double cst, double from_deg, double to_deg,
                         double sin_amp_deg, double sin_hz, double t0, double& out, double& dout) {
      if (mode == HeadScriptSegment::AxisMode::Constant)
      {
        out = cst * kDegToRad;
        dout = 0.0;
      }
      else if (mode == HeadScriptSegment::AxisMode::Lerp)
      {
        out = lerp(from_deg * kDegToRad, to_deg * kDegToRad, u);
        dout = (dt > 1e-6) ? ((to_deg - from_deg) * kDegToRad / dt) : 0.0;
      }
      else
      {
        double phase = 2.0 * kPi * sin_hz * (t_sec - t0);
        double amp_rad = sin_amp_deg * kDegToRad;
        out = amp_rad * std::sin(phase);
        dout = amp_rad * (2.0 * kPi * sin_hz) * std::cos(phase);
      }
    };

    eval_axis(seg->yaw_mode, seg->yaw, seg->yaw_from, seg->yaw_to, seg->yaw_sin_amp_deg, seg->yaw_sin_hz, seg->t_start,
              yaw, dyaw);
    eval_axis(seg->pitch_mode, seg->pitch, seg->pitch_from, seg->pitch_to, seg->pitch_sin_amp_deg, seg->pitch_sin_hz,
              seg->t_start, pitch, dpitch);
    eval_axis(seg->roll_mode, seg->roll, seg->roll_from, seg->roll_to, seg->roll_sin_amp_deg, seg->roll_sin_hz,
              seg->t_start, roll, droll);
  }

  if (cmd_out.size() < 3) cmd_out.resize(3, 0.0);
  if (cmd_dq_out.size() < 3) cmd_dq_out.resize(3, 0.0);
  cmd_out[0] = yaw;
  cmd_out[1] = pitch;
  cmd_out[2] = roll;
  cmd_dq_out[0] = dyaw;
  cmd_dq_out[1] = dpitch;
  cmd_dq_out[2] = droll;
}

// ===========================================================================
// HeadScriptController
// ===========================================================================

void HeadScriptController::initJoints(const std::vector<std::string>& joint_names, rclcpp::Logger logger) {
  logger_ = logger;
  jointIndices_.clear();

  const auto find_axis = [&joint_names](const char* axis) -> size_t {
    const std::string neck_axis = std::string("neck_") + axis;
    const std::string head_axis = std::string("head_") + axis;
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      if (joint_names[i].find(neck_axis) != std::string::npos || joint_names[i].find(head_axis) != std::string::npos)
      {
        return i;
      }
    }
    return joint_names.size();
  };

  for (const char* axis : {"yaw", "pitch", "roll"})
  {
    const size_t index = find_axis(axis);
    if (index < joint_names.size()) jointIndices_.push_back(index);
  }
  cmd_.assign(jointIndices_.size(), 0.0);
  cmd_dq_.assign(jointIndices_.size(), 0.0);

  if (!jointIndices_.empty())
  {
    RCLCPP_INFO(logger_, "Head controller: %zu head/neck joint(s) found (yaw/pitch/roll axis order).",
                jointIndices_.size());
  }
}

bool HeadScriptController::loadConfig(const std::string& path) {
  const bool ok = parseHeadConfig(path, segments_, duration_, kp_script_, kd_script_, tau_ff_script_, logger_);
  if (ok)
  {
    RCLCPP_INFO(logger_,
                "Head script-mode PD (yaw/pitch/roll): kp=[%.1f %.1f %.1f]  kd=[%.2f %.2f %.2f]  tau=[%.2f %.2f %.2f]",
                kp_script_[0], kp_script_[1], kp_script_[2], kd_script_[0], kd_script_[1], kd_script_[2],
                tau_ff_script_[0], tau_ff_script_[1], tau_ff_script_[2]);
  }
  return ok;
}

void HeadScriptController::setTopicBlendSec(double sec) { topic_blend_sec_ = std::max(0.002, std::min(0.5, sec)); }

void HeadScriptController::setFallbackBlendSec(double sec) { fallback_blend_sec_ = std::max(0.02, std::min(1.0, sec)); }

void HeadScriptController::updateFromTime(double t_sec) {
  if (segments_.empty()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  evalHeadAtTime(t_sec, segments_, cmd_, cmd_dq_);
}

void HeadScriptController::seedCurrentPose(double t_now, double yaw, double pitch, double roll) {
  std::lock_guard<std::mutex> lock(mutex_);
  p_cmd_rpy_[0] = yaw;
  p_cmd_rpy_[1] = pitch;
  p_cmd_rpy_[2] = roll;
  // Mark as ready so the next setTarget takes the normal branch and uses these
  // as wp_start_, not the new target.
  wp_start_ = {t_now, yaw, pitch, roll};
  wp_end_ = {t_now, yaw, pitch, roll};
  wp_ready_ = true;
}

void HeadScriptController::setTarget(double t_recv, double yaw_target, double pitch_target) {
  std::lock_guard<std::mutex> lock(mutex_);
  topic_roll_enabled_ = false;

  const double span = std::max(0.002, std::min(0.5, topic_blend_sec_));

  if (!wp_ready_)
  {
    // First message: initialise both endpoints at the target; no motion on first cycle.
    wp_start_ = {t_recv, yaw_target, pitch_target, p_cmd_rpy_[2]};
    wp_end_ = {t_recv + span, yaw_target, pitch_target, p_cmd_rpy_[2]};
    p_cmd_rpy_[0] = yaw_target;
    p_cmd_rpy_[1] = pitch_target;
    wp_ready_ = true;
  }
  else
  {
    // Always blend from the current commanded pose toward the latest target over fixed wall time.
    wp_start_ = {t_recv, p_cmd_rpy_[0], p_cmd_rpy_[1], p_cmd_rpy_[2]};
    wp_end_ = {t_recv + span, yaw_target, pitch_target, p_cmd_rpy_[2]};
  }
}

void HeadScriptController::setTarget(double t_recv, double yaw_target, double pitch_target, double roll_target) {
  std::lock_guard<std::mutex> lock(mutex_);
  topic_roll_enabled_ = true;

  const double span = std::max(0.002, std::min(0.5, topic_blend_sec_));

  if (!wp_ready_)
  {
    // First message: initialise both endpoints at the target; no motion on first cycle.
    wp_start_ = {t_recv, yaw_target, pitch_target, roll_target};
    wp_end_ = {t_recv + span, yaw_target, pitch_target, roll_target};
    p_cmd_rpy_[0] = yaw_target;
    p_cmd_rpy_[1] = pitch_target;
    p_cmd_rpy_[2] = roll_target;
    wp_ready_ = true;
  }
  else
  {
    // Always blend from the current commanded pose toward the latest target over fixed wall time.
    wp_start_ = {t_recv, p_cmd_rpy_[0], p_cmd_rpy_[1], p_cmd_rpy_[2]};
    wp_end_ = {t_recv + span, yaw_target, pitch_target, roll_target};
  }
}

void HeadScriptController::stepHeadOutput(double dt, double t_wall_sec, double t_loop_sec) {
  (void)dt;
  std::lock_guard<std::mutex> lock(mutex_);

  if (!topicEnabled_)
  {
    if (scriptEnabled_ && !segments_.empty()) evalHeadAtTime(t_loop_sec, segments_, cmd_, cmd_dq_);
    if (cmd_.size() >= 1) p_cmd_rpy_[0] = cmd_[0];
    if (cmd_.size() >= 2) p_cmd_rpy_[1] = cmd_[1];
    if (cmd_.size() >= 3) p_cmd_rpy_[2] = cmd_[2];
    last_topic_signal_valid_ = topic_signal_valid_;
    return;
  }

  std::vector<double> script_q(3, 0.0);
  std::vector<double> script_dq(3, 0.0);
  if (!segments_.empty()) evalHeadAtTime(t_loop_sec, segments_, script_q, script_dq);

  const bool hybrid = scriptEnabled_ && !segments_.empty() && duration_ > 0.0;

  if (topic_signal_valid_)
  {
    fallback_blend_active_ = false;
    // Linear blend at control rate: u advances with wall time over fixed topic_blend_sec_, independent of detector
    // rate. u is clamped to [0,1], holding the final pose until setTarget refreshes wp_*.
    if (wp_ready_)
    {
      const double span = wp_end_.t - wp_start_.t;
      if (span > 1e-9)
      {
        const double u = std::max(0.0, std::min(1.0, (t_wall_sec - wp_start_.t) / span));
        const double dyaw = wp_end_.yaw - wp_start_.yaw;
        const double dpitch = wp_end_.pitch - wp_start_.pitch;
        p_cmd_rpy_[0] = wp_start_.yaw + u * dyaw;
        p_cmd_rpy_[1] = wp_start_.pitch + u * dpitch;
        if (cmd_dq_.size() >= 1) cmd_dq_[0] = dyaw / span;
        if (cmd_dq_.size() >= 2) cmd_dq_[1] = dpitch / span;
        if (topic_roll_enabled_)
        {
          const double droll = wp_end_.roll - wp_start_.roll;
          p_cmd_rpy_[2] = wp_start_.roll + u * droll;
          if (cmd_dq_.size() >= 3) cmd_dq_[2] = droll / span;
        }
      }
    }
  }
  else if (hybrid)
  {
    if (last_topic_signal_valid_)
    {
      fallback_blend_active_ = true;
      fallback_blend_start_t_ = t_wall_sec;
      fallback_blend_start_yaw_ = p_cmd_rpy_[0];
      fallback_blend_start_pitch_ = p_cmd_rpy_[1];
      fallback_blend_start_roll_ = p_cmd_rpy_[2];
    }

    if (fallback_blend_active_)
    {
      const double span = std::max(0.02, std::min(1.0, fallback_blend_sec_));
      const double u = std::max(0.0, std::min(1.0, (t_wall_sec - fallback_blend_start_t_) / span));
      p_cmd_rpy_[0] = fallback_blend_start_yaw_ + u * (script_q[0] - fallback_blend_start_yaw_);
      p_cmd_rpy_[1] = fallback_blend_start_pitch_ + u * (script_q[1] - fallback_blend_start_pitch_);
      p_cmd_rpy_[2] = fallback_blend_start_roll_ + u * (script_q[2] - fallback_blend_start_roll_);
      if (u >= 1.0) fallback_blend_active_ = false;
    }
    else
    {
      p_cmd_rpy_[0] = script_q[0];
      p_cmd_rpy_[1] = script_q[1];
      p_cmd_rpy_[2] = script_q[2];
    }
  }
  else
  {
    fallback_blend_active_ = false;
    p_cmd_rpy_[0] = 0.0;
    p_cmd_rpy_[1] = 0.0;
    p_cmd_rpy_[2] = 0.0;
  }

  last_topic_signal_valid_ = topic_signal_valid_;

  if (cmd_.size() >= 1) cmd_[0] = p_cmd_rpy_[0];
  if (cmd_.size() >= 2) cmd_[1] = p_cmd_rpy_[1];
  if (!topic_signal_valid_)
  {
    if (cmd_dq_.size() >= 1) cmd_dq_[0] = hybrid ? script_dq[0] : 0.0;
    if (cmd_dq_.size() >= 2) cmd_dq_[1] = hybrid ? script_dq[1] : 0.0;
    if (cmd_.size() >= 3) cmd_[2] = p_cmd_rpy_[2];
    if (cmd_dq_.size() >= 3) cmd_dq_[2] = hybrid ? script_dq[2] : 0.0;
  }
  else
  {
    const double span = (wp_ready_) ? (wp_end_.t - wp_start_.t) : 0.0;
    if (!wp_ready_ || span <= 1e-9)
    {
      if (cmd_dq_.size() >= 1) cmd_dq_[0] = 0.0;
      if (cmd_dq_.size() >= 2) cmd_dq_[1] = 0.0;
    }
    if (cmd_.size() >= 3)
    {
      cmd_[2] = topic_roll_enabled_ ? p_cmd_rpy_[2] : (hybrid ? script_q[2] : 0.0);
    }
    if (cmd_dq_.size() >= 3 && !topic_roll_enabled_) cmd_dq_[2] = hybrid ? script_dq[2] : 0.0;
  }
}

bool HeadScriptController::getJointCmd(size_t global_j, double& q, double& dq, double& kp, double& kd,
                                       double& tau_ff) const {
  // Script and topic both off: do not claim neck joints — LeggedController applies walking
  // posCmd (RL or imitation per PolicyMux) with the same gains as other policy joints.
  if (jointIndices_.empty() || (!scriptEnabled_ && !topicEnabled_)) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < jointIndices_.size(); ++i)
  {
    if (jointIndices_[i] == global_j && i < cmd_.size() && i < cmd_dq_.size())
    {
      q = cmd_[i];
      dq = cmd_dq_[i];
      const size_t gi = std::min(i, static_cast<size_t>(2));
      kp = kp_script_[gi];
      kd = kd_script_[gi];
      tau_ff = tau_ff_script_[gi];
      return true;
    }
  }
  return false;
}

}  // namespace legged
