#include "legged_controllers/TextMotionController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace legged {
namespace {

const std::array<std::string, 9> kJoints{{
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_roll_joint",
    "neck_yaw_joint", "neck_pitch_joint", "neck_roll_joint"}};
constexpr double kEntrySec = 0.2;
constexpr double kExitSec = 0.2;
constexpr std::size_t kMaxFrames = 5000;
constexpr std::size_t kMaxBytes = 2U * 1024U * 1024U;

std::int64_t unixTimeMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

bool validId(const std::string& id) {
  return !id.empty() && id.size() <= 64 && std::all_of(id.begin(), id.end(), [](unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
}

double smooth(double x) {
  const double u = std::max(0.0, std::min(1.0, x));
  return u * u * (3.0 - 2.0 * u);
}

std::string jsonString(const std::string& value) {
  static const char hex[] = "0123456789abcdef";
  std::string out = "\"";
  for (const unsigned char c : value)
  {
    if (c == '\\' || c == '"')
    {
      out += '\\';
      out += static_cast<char>(c);
    }
    else if (c < 0x20)
    {
      out += "\\u00";
      out += hex[c >> 4];
      out += hex[c & 0xf];
    }
    else out += static_cast<char>(c);
  }
  return out + '"';
}

}  // namespace

bool TextMotionController::configure(const std::vector<std::string>& joints, const std::string& mapping_file,
                                     std::string& error) {
  try
  {
    const auto motors = YAML::LoadFile(mapping_file)["motors"];
    if (!motors.IsSequence()) throw std::runtime_error("mapping_file has no motors sequence");
    for (std::size_t j = 0; j < kJoints.size(); ++j)
    {
      const auto found = std::find(joints.begin(), joints.end(), kJoints[j]);
      if (found == joints.end() || std::count(joints.begin(), joints.end(), kJoints[j]) != 1)
        throw std::runtime_error("missing/duplicate text-motion joint: " + kJoints[j]);
      indices_[j] = static_cast<std::size_t>(found - joints.begin());
      unsigned matches = 0;
      for (const auto& motor : motors)
      {
        if (motor["joint_name"].as<std::string>("") != kJoints[j]) continue;
        ++matches;
        lower_[j] = motor["joint_lower"].as<double>();
        upper_[j] = motor["joint_upper"].as<double>();
        if (!motor["enabled"].as<bool>(false) || !std::isfinite(lower_[j]) || !std::isfinite(upper_[j]) ||
            lower_[j] >= upper_[j]) throw std::runtime_error("invalid/disabled joint in text-motion mapping");
      }
      if (matches != 1) throw std::runtime_error("missing/duplicate joint limit in text-motion mapping");
    }
  }
  catch (const std::exception& exc)
  {
    error = exc.what();
    return false;
  }
  return true;
}

bool TextMotionController::seen(const std::string& id) const {
  return std::find(recent_ids_.begin(), recent_ids_.end(), id) != recent_ids_.end();
}

void TextMotionController::remember(const std::string& id) {
  if (!seen(id)) recent_ids_.push_back(id);
  if (recent_ids_.size() > 256) recent_ids_.pop_front();
}

TextMotionController::Event TextMotionController::submitJson(const std::string& payload, bool allowed) {
  const auto gate_generation = gate_generation_.load();
  Event event{"", "rejected", ""};
  try
  {
    if (payload.empty() || payload.size() > kMaxBytes) throw std::runtime_error("trajectory exceeds payload limit");
    const YAML::Node root = YAML::Load(payload);
    event.request_id = root["request_id"].as<std::string>("");
    if (!validId(event.request_id))
    {
      event.request_id.clear();
      throw std::runtime_error("invalid request_id");
    }
    if (!allowed) throw std::runtime_error("controller must be in unobstructed Walking with a fresh AMP command");
    if (root["format"].as<std::string>("") != "l0_text_motion_v2" ||
        root["robot"].as<std::string>("") != "l0_v5" || root["angle_unit"].as<std::string>("") != "rad" ||
        root["fps"].as<double>(0.0) != 50.0 ||
        root["coordinate_contract"].as<std::string>("") != "robot_anatomical_v1" ||
        root["source_zip_sha256"].as<std::string>("") !=
            "48485ea7f14fd7e52920f7f59e4da20515b9f043e0aa388ee2a3e43130a84119")
      throw std::runtime_error("trajectory contract mismatch");
    auto incoming = std::make_shared<Trajectory>();
    incoming->id = event.request_id;
    incoming->expires_at_ms = root["expires_at_ms"].as<std::int64_t>();
    const auto names = root["joint_names"];
    const auto mask = root["active_joints"];
    if (!names.IsSequence() || names.size() != 9 || !mask.IsSequence() || mask.size() != 9)
      throw std::runtime_error("expected nine joint names and explicit mask");
    for (std::size_t j = 0; j < 9; ++j)
    {
      if (names[j].as<std::string>() != kJoints[j]) throw std::runtime_error("incorrect joint order");
      const auto flag = mask[j].as<std::string>();
      if (flag != "true" && flag != "false") throw std::runtime_error("active_joints must be booleans");
      incoming->mask[j] = flag == "true";
    }
    for (std::size_t j = 0; j < 9; j += 3)
      if (incoming->mask[j] != incoming->mask[j + 1] || incoming->mask[j] != incoming->mask[j + 2])
        throw std::runtime_error("ownership must cover complete body parts");
    if (std::none_of(incoming->mask.begin(), incoming->mask.end(), [](bool value) { return value; }))
      throw std::runtime_error("no active body part");
    const auto frames = root["positions"];
    const auto count = root["frame_count"].as<std::size_t>(0);
    if (!frames.IsSequence() || count != frames.size() || count < 2 || count > kMaxFrames)
      throw std::runtime_error("invalid frame count");
    incoming->frames.reserve(count);
    for (const auto& frame : frames)
    {
      if (!frame.IsSequence() || frame.size() != 9) throw std::runtime_error("invalid frame dimensions");
      Pose row;
      for (std::size_t j = 0; j < 9; ++j)
      {
        row[j] = frame[j].as<double>();
        if (!std::isfinite(row[j]) || row[j] < lower_[j] - 1e-6 || row[j] > upper_[j] + 1e-6)
          throw std::runtime_error("nonfinite/out-of-limit joint: " + kJoints[j]);
        row[j] = std::max(lower_[j], std::min(upper_[j], row[j]));
      }
      incoming->frames.push_back(row);
    }
    const auto age = incoming->expires_at_ms - unixTimeMs();
    if (age <= 0 || age > 5000) throw std::runtime_error("expired or excessive trajectory TTL");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!allowed_.load() || gate_generation != gate_generation_.load())
      throw std::runtime_error("controller gate changed during trajectory validation");
    if (seen(incoming->id))
      return {incoming->id, "duplicate", "request already received or cancelled"};
    if (busy_.load()) throw std::runtime_error("another text motion is preparing/playing/releasing");
    remember(incoming->id);
    slot_id_ = incoming->id;
    pending_ = std::move(incoming);
    cancelled_ = false;
    busy_.store(true);
    event.state = "ready";
    event.detail = "complete trajectory validated";
  }
  catch (const std::exception& exc)
  {
    event.detail = exc.what();
  }
  return event;
}

TextMotionController::Event TextMotionController::cancel(const std::string& request_id) {
  if (!validId(request_id)) return {"", "rejected", "invalid cancellation request_id"};
  std::lock_guard<std::mutex> lock(mutex_);
  // Remember cancellation even if inference/validation has not delivered a motion yet.
  remember(request_id);
  if (slot_id_ == request_id)
  {
    cancelled_ = true;
    return {request_id, "cancelling", "cancellation queued for control loop"};
  }
  return {request_id, "cancelled", "cancelled before trajectory admission"};
}

void TextMotionController::setAllowed(bool allowed) {
  if (allowed_.exchange(allowed) && !allowed) ++gate_generation_;
}

TextMotionController::Event TextMotionController::finish(const char* state, const char* detail) {
  Event event{slot_id_, state, detail};
  current_.reset();
  pending_.reset();
  slot_id_.clear();
  cancelled_ = false;
  releasing_ = false;
  active_.store(false);
  busy_.store(false);
  return event;
}

TextMotionController::Event TextMotionController::step(const std::vector<double>& base,
    const std::vector<JointMotorCommand>& previous, bool allowed) {
  if (!busy_.load()) return {};
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return {};  // Never wait for a ROS callback on the control loop.
  const auto now = std::chrono::steady_clock::now();
  for (const auto index : indices_)
    if (index >= base.size() || !std::isfinite(base[index])) allowed = false;
  if (!allowed) return finish("stopped", "Walking/AMP command/safety gate no longer permits playback");
  Event event;
  if (pending_)
  {
    if (cancelled_) return finish("cancelled", "cancelled before playback");
    if (pending_->expires_at_ms <= unixTimeMs()) return finish("rejected", "trajectory expired before playback");
    current_ = std::move(pending_);
    for (std::size_t j = 0; j < 9; ++j)
    {
      const auto index = indices_[j];
      // Blend from the last composed command, including head/arm overlays.
      entry_[j] = index < previous.size() ? previous[index].q_des : base[index];
      if (!std::isfinite(entry_[j]) || entry_[j] < lower_[j] - 1e-6 || entry_[j] > upper_[j] + 1e-6)
        return finish("rejected", "previous joint command is invalid");
      q_[j] = entry_[j];
    }
    started_ = now;
    releasing_ = false;
    release_alpha_ = 0.0;
    active_.store(true);
    event = {slot_id_, "started", "upper-body playback; AMP continues controlling legs"};
  }
  if (!current_) return {};
  const double elapsed = std::chrono::duration<double>(now - started_).count();
  const double duration = static_cast<double>(current_->frames.size() - 1) / 50.0;
  if (!releasing_ && (cancelled_ || elapsed >= kEntrySec + duration))
  {
    releasing_ = true;
    cancelled_release_ = cancelled_;
    releasing_at_ = now;
    release_ = cancelled_ ? q_ : current_->frames.back();
  }
  if (releasing_)
  {
    cancelled_release_ = cancelled_release_ || cancelled_;
    const double u = std::chrono::duration<double>(now - releasing_at_).count() / kExitSec;
    release_alpha_ = smooth(u);
    if (u >= 1.0) return finish(cancelled_release_ ? "cancelled" : "completed", "released to current background command");
    return event;
  }
  if (elapsed < kEntrySec)
  {
    const double alpha = smooth(elapsed / kEntrySec);
    for (std::size_t j = 0; j < 9; ++j)
      q_[j] = entry_[j] + alpha * (current_->frames.front()[j] - entry_[j]);
    return event;
  }
  const double frame = std::min((elapsed - kEntrySec) * 50.0, static_cast<double>(current_->frames.size() - 1));
  const auto first = static_cast<std::size_t>(std::floor(frame));
  const auto second = std::min(first + 1, current_->frames.size() - 1);
  const double alpha = frame - first;
  for (std::size_t j = 0; j < 9; ++j)
    q_[j] = current_->frames[first][j] + alpha * (current_->frames[second][j] - current_->frames[first][j]);
  return event;
}

bool TextMotionController::getJointPosition(std::size_t index, double& q) const {
  // Called only from the same control thread as step().
  if (!active_.load() || !current_) return false;
  for (std::size_t j = 0; j < 9; ++j)
    if (indices_[j] == index && current_->mask[j])
    {
      // Composer supplies the current background command (AMP plus any allowed
      // existing overlay). This also makes the last release target continuous.
      q = releasing_ ? release_[j] + release_alpha_ * (q - release_[j]) : q_[j];
      return true;
    }
  return false;
}

std::string TextMotionController::encodeEvent(const Event& event) {
  return "{\"request_id\":" + jsonString(event.request_id) + ",\"state\":" + jsonString(event.state) +
         ",\"detail\":" + jsonString(event.detail) + "}";
}

}  // namespace legged
