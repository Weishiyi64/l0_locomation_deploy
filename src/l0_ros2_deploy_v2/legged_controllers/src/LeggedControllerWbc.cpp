#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "legged_controllers/LeggedController.h"

namespace legged {

namespace {

constexpr int64_t kWbcActionMaxAgeMs = 5000;
constexpr int64_t kWbcActionMaxFutureMs = 1000;
constexpr std::size_t kMaxRecentWbcActionRequests = 256;

int64_t unixTimeMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

bool validActionId(const std::string& action_id) {
  if (action_id.empty() || action_id.size() > 32) return false;
  return std::all_of(action_id.begin(), action_id.end(), [](char value) {
    const unsigned char c = static_cast<unsigned char>(value);
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_';
  });
}

}  // namespace

bool LeggedController::startPolicyHandoff(const PolicyInputs& start, double duration_sec, const char* label) {
  if (duration_sec <= 1e-9 || start.q_snapshot.empty()) return false;
  const std::size_t count = jointNames_.size();
  std::vector<bool> mask(count, false);
  std::vector<JointCommandSample> commands(count);
  const std::size_t available = std::min(count, start.q_snapshot.size());
  for (std::size_t joint = 0; joint < available; ++joint)
  {
    mask[joint] = true;
    commands[joint].q = start.q_snapshot[joint];
    commands[joint].dq = 0.0;
    commands[joint].kp = joint < start.gains.kps.size() ? start.gains.kps[joint] : 0.0;
    commands[joint].kd = joint < start.gains.kds.size() ? start.gains.kds[joint] : 0.0;
    commands[joint].tau_ff = 0.0;
  }
  double wall_sec = last_controller_wall_sec_;
  if (wall_sec <= 0.0) wall_sec = nh_->now().seconds();
  JointCommandTransition& handoff = sequenceRuntime_.releaseHandoff();
  handoff.clear();
  if (!handoff.start(mask, commands, wall_sec, duration_sec)) return false;
  RCLCPP_INFO(nh_->get_logger(), "[WBC] %s handoff blend %.3fs", label, duration_sec);
  return true;
}

bool LeggedController::startWbcMotion(
    const std::string& motion_id, int64_t timestamp_ms,
    std::shared_ptr<luvbot_control_interfaces::srv::ControlWbcMotion::Response> response) {
  if (!wbc_enabled_)
  {
    response->message = "WBC is disabled; launch with wbc_enabled:=true";
    return false;
  }
  // Keep the preparation slot until cancellation is acknowledged, even after a timeout.
  const auto cancel_preparation = [this, &motion_id]() {
    const auto cancelled = policyMux_.prepareWbcMotion(
        luvbot_control_interfaces::srv::ControlWbcMotion::Request::CANCEL, motion_id, wbc_command_timeout_sec_);
    wbc_cleanup_pending_ = !cancelled || !cancelled->success;
    if (wbc_cleanup_pending_)
      RCLCPP_WARN(nh_->get_logger(), "[WBC] Worker cancellation unconfirmed; refusing new preparation until cleanup");
  };
  if (wbc_cleanup_pending_)
  {
    cancel_preparation();
    if (wbc_cleanup_pending_)
    {
      response->message = "WBC worker cleanup has not been acknowledged";
      return false;
    }
  }
  uint64_t generation;
  geometry_msgs::msg::Quaternion q;
  geometry_msgs::msg::Twist velocity;
  {
    std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
    if (!wbc_start_allowed_ || wbc_return_pending_.load() || wbc_handoff_in_progress_.load())
    {
      response->message = "controller state does not allow a WBC motion";
      return false;
    }
    generation = wbc_generation_;
    q = wbc_orientation_;
    velocity = wbc_velocity_;
  }
  if (policyMux_.wbcActiveCommanded() || policyMux_.source() == PolicyCmdSource::kWbc)
  {
    response->message = "a WBC motion is already active";
    return false;
  }
  const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
  if (safetyChecker_ && !safetyChecker_->checkOrientation(roll, pitch).ok)
  {
    response->message = "current roll/pitch violates the safety posture limits";
    return false;
  }
  if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) || !std::isfinite(velocity.angular.z) ||
      std::abs(velocity.linear.x) > 0.05 || std::abs(velocity.linear.y) > 0.05 || std::abs(velocity.angular.z) > 0.10)
  {
    response->message = "WBC requires |vx|<=0.05, |vy|<=0.05 and |wz|<=0.10";
    return false;
  }

  const PolicyInputs amp_start = policyMux_.selectInputs(true);
  if (policyMux_.source() != PolicyCmdSource::kRl || !amp_start.valid)
  {
    response->message = "a fresh AMP command is required before WBC start";
    return false;
  }
  const uint64_t previous_wbc_revision = policyMux_.wbcCommandRevision();
  auto prepared = policyMux_.prepareWbcMotion(luvbot_control_interfaces::srv::ControlWbcMotion::Request::START,
                                              motion_id, wbc_command_timeout_sec_);
  if (!prepared)
  {
    cancel_preparation();
    response->message = "WBC worker did not respond";
    return false;
  }
  response->motion_id = prepared->motion_id;
  response->robot_cfg_path = prepared->robot_cfg_path;
  response->duration_sec = prepared->duration_sec;
  if (!prepared->success)
  {
    cancel_preparation();
    response->message = prepared->message;
    return false;
  }
  if (!policyMux_.loadWbcWalkingGains(prepared->robot_cfg_path, prepared->motion_id))
  {
    cancel_preparation();
    response->message = "failed to load WBC gains";
    return false;
  }
  if (!policyMux_.waitForWbcCommandAfter(previous_wbc_revision, wbc_command_timeout_sec_))
  {
    cancel_preparation();
    response->message = "WBC pre-inference command was not received";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
    const PolicyInputs current_amp = policyMux_.selectInputs(true);
    if (generation == wbc_generation_ && wbc_start_allowed_ && policyMux_.source() == PolicyCmdSource::kRl &&
        current_amp.valid && policyMux_.waitForWbcCommandAfter(previous_wbc_revision, 0.0))
    {
      wbc_handoff_start_ = current_amp;
      wbc_handoff_pending_ = true;
      wbc_handoff_in_progress_.store(true);
      policyMux_.publishWbcActive(true, "[WBC] Activate worker");
      wbc_handoff_target_ = PolicyCmdSource::kWbc;
      std::lock_guard<std::mutex> state_lock(wbc_action_state_mutex_);
      wbc_action_active_ = true;
      wbc_action_timestamp_ms_ = timestamp_ms;
      wbc_action_id_ = motion_id;
      publishWbcActionFeedback(timestamp_ms, motion_id, "started");
      response->success = true;
      response->message = "WBC motion started";
      return true;
    }
  }
  cancel_preparation();
  response->message = "controller state changed or policy command expired during WBC preparation";
  return false;
}

bool LeggedController::finishWbcMotion(const char* reason, std::string& message) {
  uint64_t generation;
  PolicyInputs wbc_start;
  uint64_t previous_rl_revision;
  {
    std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
    if (!policyMux_.wbcActiveCommanded() && policyMux_.source() != PolicyCmdSource::kWbc)
    {
      message = "no WBC motion is active";
      return false;
    }
    generation = wbc_generation_;
    wbc_start = policyMux_.selectInputs(true);
    previous_rl_revision = policyMux_.rlCommandRevision();
    wbc_return_pending_.store(true);
    policyMux_.publishWbcActive(false, reason);
  }
  if (!wbc_start.valid)
  {
    message = "WBC command expired; entering emergency damping";
    triggerEmergencyStop();
    return false;
  }
  const auto cancelled = policyMux_.prepareWbcMotion(
      luvbot_control_interfaces::srv::ControlWbcMotion::Request::CANCEL, "", wbc_command_timeout_sec_);
  wbc_cleanup_pending_ = !cancelled || !cancelled->success;
  publishRlObsReset();
  if (!policyMux_.waitForRlCommandAfter(previous_rl_revision, wbc_command_timeout_sec_))
  {
    message = "fresh AMP command timed out; entering emergency damping";
    RCLCPP_ERROR(nh_->get_logger(), "[WBC] %s", message.c_str());
    wbc_return_pending_.store(false);
    triggerEmergencyStop();
    return false;
  }
  std::unique_lock<std::mutex> lock(wbc_transition_mutex_);
  if (generation != wbc_generation_ || !wbc_controller_walking_ ||
      !policyMux_.waitForRlCommandAfter(previous_rl_revision, 0.0))
  {
    wbc_return_pending_.store(false);
    message = "controller left Walking or AMP command expired while returning";
    lock.unlock();
    triggerEmergencyStop();
    return false;
  }
  wbc_handoff_start_ = wbc_start;
  wbc_handoff_pending_ = true;
  wbc_handoff_in_progress_.store(true);
  wbc_handoff_target_ = PolicyCmdSource::kRl;
  message = "WBC motion stopped; blending to AMP";
  return true;
}

void LeggedController::abortWbcImmediately(const char* reason) {
  std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
  ++wbc_generation_;
  wbc_start_allowed_ = false;
  wbc_controller_walking_ = false;
  wbc_handoff_pending_ = false;
  wbc_handoff_clear_pending_ = true;
  wbc_handoff_in_progress_.store(false);
  if (!policyMux_.wbcActiveCommanded() && policyMux_.source() != PolicyCmdSource::kWbc && !wbc_return_pending_.load())
    return;
  policyMux_.publishWbcActive(false, reason);
  wbc_return_pending_.store(false);
  setPolicySource(PolicyCmdSource::kRl, "[PolicyMux] WBC aborted; AMP source restored");
  publishRlObsReset();
  publishActiveWbcActionTerminal("failed");
}

void LeggedController::publishWbcActionFeedback(int64_t timestamp_ms, const std::string& action_id,
                                                const char* status) {
  if (!wbcActionFeedbackPub_) return;
  std_msgs::msg::String feedback;
  feedback.data = "{\"timestamp_ms\":" + std::to_string(timestamp_ms) + ",\"action_id\":\"" + action_id +
                  "\",\"status\":\"" + status + "\"}";
  wbcActionFeedbackPub_->publish(feedback);
}

void LeggedController::publishActiveWbcActionTerminal(const char* status) {
  std::lock_guard<std::mutex> lock(wbc_action_state_mutex_);
  if (!wbc_action_active_) return;
  publishWbcActionFeedback(wbc_action_timestamp_ms_, wbc_action_id_, status);
  wbc_action_active_ = false;
  wbc_action_timestamp_ms_ = 0;
  wbc_action_id_.clear();
}

void LeggedController::wbcActionCommandCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  int64_t timestamp_ms = 0;
  std::string action_id;
  try
  {
    const YAML::Node command = YAML::Load(msg->data);
    if (!command.IsMap() || !command["timestamp_ms"] || !command["action_id"])
      throw std::runtime_error("expected timestamp_ms and action_id");
    timestamp_ms = command["timestamp_ms"].as<int64_t>();
    action_id = command["action_id"].as<std::string>();
  } catch (const std::exception& exception)
  {
    RCLCPP_WARN(nh_->get_logger(), "[WBC Action] Rejected malformed command: %s", exception.what());
    publishWbcActionFeedback(0, "", "rejected");
    return;
  }

  if (!validActionId(action_id))
  {
    RCLCPP_WARN(nh_->get_logger(), "[WBC Action] Rejected invalid action_id");
    publishWbcActionFeedback(timestamp_ms, "", "rejected");
    return;
  }

  const int64_t now_ms = unixTimeMs();
  if (timestamp_ms <= 0 || timestamp_ms < now_ms - kWbcActionMaxAgeMs || timestamp_ms > now_ms + kWbcActionMaxFutureMs)
  {
    RCLCPP_WARN(nh_->get_logger(), "[WBC Action] Rejected timestamp %lld at %lld", static_cast<long long>(timestamp_ms),
                static_cast<long long>(now_ms));
    publishWbcActionFeedback(timestamp_ms, action_id, "rejected");
    return;
  }

  const auto received_at = std::chrono::steady_clock::now();
  const std::string request_key = std::to_string(timestamp_ms) + ":" + action_id;
  {
    std::lock_guard<std::mutex> lock(wbc_action_state_mutex_);
    while (!recent_wbc_action_requests_.empty() &&
           received_at - recent_wbc_action_requests_.front().second > std::chrono::seconds(6))
      recent_wbc_action_requests_.pop_front();
    const auto duplicate = std::find_if(recent_wbc_action_requests_.begin(), recent_wbc_action_requests_.end(),
                                        [&request_key](const auto& request) { return request.first == request_key; });
    if (duplicate != recent_wbc_action_requests_.end())
    {
      RCLCPP_WARN(nh_->get_logger(), "[WBC Action] Rejected duplicate request %s", request_key.c_str());
      publishWbcActionFeedback(timestamp_ms, action_id, "rejected");
      return;
    }
    if (recent_wbc_action_requests_.size() >= kMaxRecentWbcActionRequests) recent_wbc_action_requests_.pop_front();
    recent_wbc_action_requests_.emplace_back(request_key, received_at);
  }

  std::unique_lock<std::mutex> lock(auxiliary_action_trigger_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    publishWbcActionFeedback(timestamp_ms, action_id, "rejected");
    RCLCPP_WARN(nh_->get_logger(), "[WBC Action] Rejected %s: another action request is being handled", action_id.c_str());
    return;
  }
  auto response = std::make_shared<luvbot_control_interfaces::srv::ControlWbcMotion::Response>();
  if (startWbcMotion(action_id, timestamp_ms, response)) return;
  RCLCPP_WARN(nh_->get_logger(), "[WBC Action] Rejected %s: %s", action_id.c_str(), response->message.c_str());
  publishWbcActionFeedback(timestamp_ms, action_id, "rejected");
}

void LeggedController::wbcMotionDoneCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(auxiliary_action_trigger_mutex_);
  if (!policyMux_.wbcActiveCommanded()) return;
  if (!msg->data)
  {
    RCLCPP_ERROR(nh_->get_logger(), "[WBC Action] Worker failed; entering emergency damping");
    triggerEmergencyStop();
    return;
  }
  std::string message;
  const bool returned_to_amp = finishWbcMotion("[WBC] Motion completed", message);
  if (!returned_to_amp) RCLCPP_WARN(nh_->get_logger(), "[WBC Action] %s", message.c_str());
  publishActiveWbcActionTerminal(returned_to_amp ? "completed" : "failed");
}

void LeggedController::textWbcCancelCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  if (msg->data.compare(0, 2, "T_") != 0 || msg->data.size() > 32) return;
  std::lock_guard<std::mutex> lock(auxiliary_action_trigger_mutex_);
  {
    std::lock_guard<std::mutex> state_lock(wbc_action_state_mutex_);
    if (!wbc_action_active_ || wbc_action_id_ != msg->data) return;
  }
  std::string message;
  const bool returning = finishWbcMotion("[Text WBC] Cancelled", message);
  publishActiveWbcActionTerminal(returning ? "cancelled" : "failed");
}

void LeggedController::updateWbcControlState() {
  if (!wbc_enabled_) return;
  std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
  if (!wbc_handoff_pending_ && !sequenceRuntime_.releaseHandoffActive(last_controller_wall_sec_))
    wbc_handoff_in_progress_.store(false);
  wbc_controller_walking_ = fsm_ && fsm_->isWalking();
  wbc_start_allowed_ = wbc_controller_walking_ && !sequenceRuntime_.isActive() &&
                       !policyMux_.imitationActiveCommanded() && !wbc_handoff_pending_ &&
                       !sequenceRuntime_.releaseHandoffActive(last_controller_wall_sec_);
  const auto& q = policy_observation_.imu.orientation;
  const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
  const auto& velocity = policy_observation_.velocity_command;
  wbc_start_allowed_ = wbc_start_allowed_ && std::isfinite(roll) && std::isfinite(pitch) &&
                       (!safetyChecker_ || safetyChecker_->checkOrientation(roll, pitch).ok) &&
                       std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) &&
                       std::isfinite(velocity.angular.z) && std::abs(velocity.linear.x) <= 0.05 &&
                       std::abs(velocity.linear.y) <= 0.05 && std::abs(velocity.angular.z) <= 0.10;
  wbc_orientation_ = policy_observation_.imu.orientation;
  wbc_velocity_ = policy_observation_.velocity_command;
  if (wbc_handoff_clear_pending_)
  {
    sequenceRuntime_.releaseHandoff().clear();
    wbc_handoff_clear_pending_ = false;
  }
  if (wbc_handoff_pending_)
  {
    if (wbc_controller_walking_)
    {
      startPolicyHandoff(wbc_handoff_start_, wbc_transition_sec_, "policy switch");
      setPolicySource(wbc_handoff_target_, "[PolicyMux] WBC handoff applied");
    }
    wbc_return_pending_.store(false);
    wbc_handoff_pending_ = false;
  }
}

}  // namespace legged
