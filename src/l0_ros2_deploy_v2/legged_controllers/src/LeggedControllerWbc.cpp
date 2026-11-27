#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <nlohmann/json.hpp>

#include "LeggedController.h"

namespace legged {

bool LeggedController::startPolicyHandoff(double duration_sec, const char* label, bool include_legs) {
  if (duration_sec <= 1e-9 || !robot_hw_) return false;
  const std::size_t count = jointNames_.size();
  // Only complete validated frames enter this cache, including overlays and the previous blend.
  const auto& previous = jointCommandComposer_.lastCommands();
  if (count != 21 || previous.size() != count) return false;
  std::vector<bool> mask(count, false);
  std::vector<JointCommandSample> commands(count);
  for (std::size_t joint = 0; joint < count; ++joint)
  {
    if (include_legs || joint >= 12) mask[joint] = true;
    const auto& from = previous[joint];
    commands[joint].q = from.q_des;
    commands[joint].dq = from.dq_des;
    commands[joint].kp = from.kp;
    commands[joint].kd = from.kd;
    commands[joint].tau_ff = from.tau_ff;
  }
  double wall_sec = last_controller_wall_sec_;
  if (wall_sec <= 0.0) wall_sec = nh_->now().seconds();
  JointCommandTransition& handoff = policy_handoff_;
  handoff.clear();
  if (!handoff.start(mask, commands, wall_sec, duration_sec)) return false;
  RCLCPP_DEBUG(nh_->get_logger(), "[WBC] %s blend %.3fs (%s)",
              label, duration_sec, include_legs ? "full body" : "arms/head");
  return true;
}

bool LeggedController::startWbcMotion(
    const std::string& motion_id, int64_t timestamp_ms,
    std::shared_ptr<robot_interfaces::srv::ControlWbcMotion::Response> response) {
  if (!wbc_enabled_)
  {
    response->message = "WBC is disabled; launch with wbc_enabled:=true";
    return false;
  }
  // Keep the preparation slot until cancellation is acknowledged, even after a timeout.
  const auto cancel_preparation = [this, &motion_id]() {
    {
      std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
      if (wbc_preposition_pending_ || wbc_preposition_active_.load()) wbc_preposition_cancel_pending_ = true;
    }
    const auto cancelled = policyMux_.prepareWbcMotion(
        robot_interfaces::srv::ControlWbcMotion::Request::CANCEL, motion_id, wbc_command_timeout_sec_);
    wbc_cleanup_pending_ = !cancelled || !cancelled->success;
    if (wbc_cleanup_pending_)
      RCLCPP_WARN(nh_->get_logger(), "[WBC] Worker cancellation unconfirmed; refusing new preparation until cleanup");
  };
  if (wbc_cleanup_pending_.load())
  {
    cancel_preparation();
    if (wbc_cleanup_pending_.load())
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
  if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) || !std::isfinite(velocity.angular.z))
  {
    response->message = "WBC requires finite velocity commands";
    return false;
  }

  const PolicyInputs amp_start = policyMux_.selectInputs(true);
  if (policyMux_.source() != PolicyCmdSource::kRl || !amp_start.valid)
  {
    response->message = "a fresh AMP command is required before WBC start";
    return false;
  }
  const uint64_t previous_wbc_revision = policyMux_.wbcCommandRevision();
  std::string request_id;
  bool fixed_motion;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    request_id = motion_request_id_;
    fixed_motion = motion_source_ == "wbc";
  }
  if (fixed_motion)
  {
    const auto reference = policyMux_.prepareWbcMotion(
        robot_interfaces::srv::ControlWbcMotion::Request::REFERENCE,
        motion_id, wbc_command_timeout_sec_, request_id);
    if (!reference || !reference->success || reference->joint_names != jointNames_ ||
        jointNames_.size() != 21 || reference->reference_positions.size() != 21 ||
        !std::all_of(reference->reference_positions.begin(), reference->reference_positions.end(),
                     [](double value) { return std::isfinite(value); }))
    {
      cancel_preparation();
      response->message = reference && !reference->success ? reference->message : "invalid/unavailable fixed WBC start reference";
      return false;
    }
    std::unique_lock<std::mutex> lock(wbc_transition_mutex_);
    if (generation != wbc_generation_ || !wbc_start_allowed_ || motion_cancelled_.load() ||
        !policyMux_.selectInputs(true).valid)
    {
      lock.unlock();
      cancel_preparation();
      response->message = "controller state changed before upper-body preposition";
      return false;
    }
    wbc_preposition_positions_ = reference->reference_positions;
    wbc_preposition_pending_ = true;
    wbc_preposition_ready_ = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(wbc_transition_sec_ + wbc_command_timeout_sec_));
    const bool ready = wbc_transition_cv_.wait_until(lock, deadline, [this, generation]() {
      return wbc_preposition_ready_ || generation != wbc_generation_ || motion_cancelled_.load();
    });
    if (!ready || generation != wbc_generation_ || motion_cancelled_.load() || !wbc_preposition_ready_ ||
        !wbc_start_allowed_ || !policyMux_.selectInputs(true).valid)
    {
      lock.unlock();
      cancel_preparation();
      response->message = "upper-body preposition cancelled, unsafe or timed out";
      return false;
    }
    RCLCPP_DEBUG(nh_->get_logger(), "[WBC] Arms/head start-reference preposition complete; preparing fresh WBC output");
  }
  auto prepared = policyMux_.prepareWbcMotion(robot_interfaces::srv::ControlWbcMotion::Request::START,
                                              motion_id, wbc_command_timeout_sec_, request_id);
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
        !motion_cancelled_.load() && current_amp.valid && policyMux_.waitForWbcCommandAfter(previous_wbc_revision, 0.0))
    {
      wbc_handoff_pending_ = true;
      wbc_handoff_in_progress_.store(true);
      policyMux_.publishWbcActive(true);
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

void LeggedController::softStopWbc(
    const std::shared_ptr<robot_interfaces::srv::ControlWbcMotion::Request> request,
    std::shared_ptr<robot_interfaces::srv::ControlWbcMotion::Response> response) {
  using Service = robot_interfaces::srv::ControlWbcMotion;
  if (request->command != Service::Request::STOP) {
    response->message = "expected STOP (command=4)";
    return;
  }
  std::string id;
  std::string action;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!motion_reserved_.load() || policyMux_.source() != PolicyCmdSource::kWbc) {
      response->message = "no active WBC action";
      return;
    }
    id = motion_request_id_;
    action = motion_action_id_;
  }
  if ((!request->request_id.empty() && request->request_id != id) ||
      (!request->motion_id.empty() && request->motion_id != action)) {
    response->message = "STOP execution does not match the active action";
    return;
  }
  const auto result = policyMux_.prepareWbcMotion(Service::Request::STOP,"",wbc_command_timeout_sec_,id);
  if (result) *response = *result;
  else response->message = "WBC STOP request was not acknowledged";
}

bool LeggedController::requestWbcReturn(const char* reason, std::string& message, bool failed, bool live_preview) {
  std::unique_lock<std::mutex> lock(wbc_transition_mutex_);
  if (!policyMux_.wbcActiveCommanded() && policyMux_.source() != PolicyCmdSource::kWbc)
  {
    message = "no WBC motion is active";
    return false;
  }
  const auto outcome = failed ? WbcReturnOutcome::Failed :
      (reason ? WbcReturnOutcome::Cancelled : WbcReturnOutcome::Completed);
  if (wbc_return_pending_.load())
  {
    if (failed || (reason && wbc_return_outcome_ == WbcReturnOutcome::Completed)) wbc_return_outcome_ = outcome;
    message = "WBC return is already pending";
    return true;
  }
  wbc_cleanup_pending_.store(true);
  if (policyMux_.source() == PolicyCmdSource::kRl && wbc_handoff_pending_ &&
      wbc_handoff_target_ == PolicyCmdSource::kWbc)
  {
    // Keep AMP if completion/failure/cancellation arrives before the first control-thread switch.
    wbc_handoff_pending_ = false;
    wbc_handoff_in_progress_.store(false);
    if (wbc_preposition_active_.load()) wbc_preposition_cancel_pending_ = true;
    policyMux_.publishWbcActive(false, "[WBC] Stopped before handoff; AMP retained");
    publishActiveWbcActionTerminal(reason && !failed ? "cancelled" : "failed", "AMP control retained");
    message = "WBC stopped before handoff; AMP retained";
    return true;
  }
  wbc_live_return_ = live_preview;
  wbc_return_id_ = policyMux_.beginRlReturn(live_preview);
  wbc_return_outcome_ = outcome;
  wbc_return_pending_.store(true);
  if (!live_preview) policyMux_.publishWbcActive(false, reason);
  message = "WBC return requested; holding last validated command until fresh matching AMP output";
  RCLCPP_DEBUG(nh_->get_logger(), "[WBC Return] %s; outcome=%s", message.c_str(),
              failed ? "failed" : (reason ? "cancelled" : "completed"));
  // The control thread completes the handoff. Worker CANCEL/retirement belongs to reservation cleanup after AMP takes over.
  return true;
}

void LeggedController::abortWbcImmediately(const char* reason) {
  std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
  ++wbc_generation_;
  motion_cancelled_.store(true);
  wbc_start_allowed_ = false;
  wbc_controller_walking_ = false;
  wbc_handoff_pending_ = false;
  wbc_handoff_clear_pending_ = true;
  wbc_handoff_in_progress_.store(false);
  {
    std::lock_guard<std::mutex> reservation(motion_mutex_);
    motion_head_pending_ = false;
    text_capture_stamp_ns_ = 0;
  }
  wbc_preposition_pending_ = wbc_preposition_ready_ = wbc_preposition_cancel_pending_ = false;
  wbc_preposition_active_.store(false);
  wbc_transition_cv_.notify_all();
  policyMux_.clearRlReturn();
  wbc_live_return_ = false;
  if (!policyMux_.wbcActiveCommanded() && policyMux_.source() != PolicyCmdSource::kWbc && !wbc_return_pending_.load())
    return;
  policyMux_.publishWbcActive(false, reason);
  wbc_return_pending_.store(false);
  wbc_return_id_.clear();
  policyMux_.setSource(PolicyCmdSource::kRl, "[PolicyMux] WBC aborted; AMP source restored");
  policyMux_.publishRlObsReset();
  publishActiveWbcActionTerminal("failed");
}

void LeggedController::publishWbcActionFeedback(int64_t timestamp_ms, const std::string& action_id,
                                                const char* status, const char* detail) {
  if (!wbcActionFeedbackPub_) return;
  std_msgs::msg::String feedback;
  nlohmann::json event = {{"timestamp_ms", timestamp_ms}, {"action_id", action_id}, {"status", status}};
  if (detail) event["detail"] = detail;
  feedback.data = event.dump();
  wbcActionFeedbackPub_->publish(feedback);
}

void LeggedController::publishActiveWbcActionTerminal(const char* status, const char* detail) {
  std::lock_guard<std::mutex> lock(wbc_action_state_mutex_);
  if (!wbc_action_active_) return;
  publishWbcActionFeedback(wbc_action_timestamp_ms_, wbc_action_id_, status, detail);
  wbc_action_active_ = false;
  wbc_action_timestamp_ms_ = 0;
  wbc_action_id_.clear();
}

void LeggedController::wbcMotionDoneCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  std::string request_id;
  bool success;
  bool soft_stop = false;
  try {
    if (msg->data.size() > 1024) return;
    const auto event = YAML::Load(msg->data);
    request_id = event["request_id"].as<std::string>();
    success = event["success"].as<bool>();
    soft_stop = event["soft_stop"].as<bool>(false);
  } catch (const std::exception&) { return; }
  std::lock_guard<std::mutex> lock(auxiliary_action_trigger_mutex_);
  {
    std::lock_guard<std::mutex> reservation(motion_mutex_);
    if (!motion_reserved_.load() || request_id != motion_request_id_) return;
  }
  if (!policyMux_.wbcActiveCommanded() && !wbc_return_pending_.load()) return;
  if (!success)
  {
    RCLCPP_WARN(nh_->get_logger(), "[WBC Action] Worker failed; requesting AMP recovery");
  }
  std::string message;
  const bool return_requested = requestWbcReturn(nullptr, message, !success, success && soft_stop);
  if (!return_requested) RCLCPP_WARN(nh_->get_logger(), "[WBC Action] %s", message.c_str());
}

void LeggedController::updateWbcControlState() {
  if (!wbc_enabled_) return;
  std::unique_lock<std::mutex> lock(wbc_transition_mutex_);
  if (wbc_handoff_clear_pending_)
  {
    policy_handoff_.clear();
    wbc_handoff_clear_pending_ = false;
  }
  wbc_controller_walking_ = fsm_ && fsm_->isWalking();
  const auto& q = policy_observation_.imu.orientation;
  const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
  const auto& velocity = policy_observation_.velocity_command;
  const bool safe = wbc_controller_walking_ && !policyMux_.imitationActiveCommanded() &&
                       std::isfinite(roll) && std::isfinite(pitch) &&
                       (!safetyChecker_ || safetyChecker_->checkOrientation(roll, pitch).ok) &&
                       std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) &&
                       std::isfinite(velocity.angular.z);
  wbc_orientation_ = policy_observation_.imu.orientation;
  wbc_velocity_ = policy_observation_.velocity_command;
  if (wbc_controller_walking_ && policyMux_.source() == PolicyCmdSource::kWbc)
  {
    if (!wbc_return_pending_.load() && !policyMux_.selectInputs(true).valid)
    {
      lock.unlock();
      std::string message;
      requestWbcReturn("[WBC Recovery] WBC output unavailable; holding last command and requesting AMP", message, true);
      lock.lock();
      if (!fsm_->isWalking()) return;
    }
    if (wbc_return_pending_.load() && (!wbc_live_return_ || safe) &&
        policyMux_.rlReturnCommandReady(wbc_return_id_))
    {
      wbc_handoff_pending_ = true;
      wbc_handoff_in_progress_.store(true);
      wbc_handoff_target_ = PolicyCmdSource::kRl;
    }
  }
  if ((wbc_preposition_pending_ || wbc_preposition_active_.load()) && !wbc_preposition_cancel_pending_ &&
      (!safe || motion_cancelled_.load() || policyMux_.source() != PolicyCmdSource::kRl ||
       !policyMux_.selectInputs(true).valid))
  {
    ++wbc_generation_;
    wbc_preposition_cancel_pending_ = true;
    wbc_transition_cv_.notify_all();
  }
  if (wbc_preposition_cancel_pending_)
  {
    if (wbc_handoff_pending_ && wbc_handoff_target_ == PolicyCmdSource::kWbc)
    {
      // A cancellation between activation publication and the control tick must not switch the legs to WBC.
      wbc_handoff_pending_ = false;
      policyMux_.publishWbcActive(false, "[WBC] Cancelled before policy switch; keeping AMP");
      wbc_cleanup_pending_.store(true);
      publishActiveWbcActionTerminal("failed");
    }
    const bool moved = wbc_preposition_active_.load();
    if (moved && wbc_controller_walking_ && wbc_transition_sec_ > 1e-9 &&
        !startPolicyHandoff(wbc_transition_sec_, "cancelled preposition -> AMP"))
    {
      policy_handoff_.clear();
      RCLCPP_WARN(nh_->get_logger(), "[WBC] No valid upper-body blend origin; returning directly to AMP");
    }
    wbc_preposition_active_.store(false);
    wbc_preposition_pending_ = wbc_preposition_ready_ = wbc_preposition_cancel_pending_ = false;
    if (moved) wbc_handoff_in_progress_.store(true);
    wbc_transition_cv_.notify_all();
  }
  if (wbc_preposition_pending_)
  {
    if (wbc_transition_sec_ > 1e-9 && !startPolicyHandoff(wbc_transition_sec_, "AMP upper-body preposition"))
    {
      ++wbc_generation_;
      wbc_preposition_pending_ = false;
      wbc_preposition_cancel_pending_ = true;
      wbc_transition_cv_.notify_all();
      RCLCPP_WARN(nh_->get_logger(), "[WBC] No valid preposition origin; rejecting preparation and keeping AMP");
      return;
    }
    wbc_preposition_pending_ = false;
    wbc_preposition_active_.store(true);
    RCLCPP_DEBUG(nh_->get_logger(), "[WBC] Preposition arms/head to first reference; legs remain under AMP");
  }
  if (wbc_preposition_active_.load() && !wbc_preposition_ready_ &&
      policy_handoff_.expired(last_controller_wall_sec_))
  {
    wbc_preposition_ready_ = true;
    wbc_transition_cv_.notify_all();
  }
  const bool handoff_active = policy_handoff_.active() && !policy_handoff_.expired(last_controller_wall_sec_);
  if (!wbc_handoff_pending_ && !handoff_active) {
    wbc_handoff_in_progress_.store(false);
    if (wbc_live_return_ && policyMux_.source() == PolicyCmdSource::kRl) {
      policyMux_.finishRlReturnPreview();
      wbc_live_return_ = false;
    }
  }
  wbc_start_allowed_ = safe && !wbc_return_pending_.load() && !wbc_handoff_pending_ && !handoff_active &&
                       !wbc_preposition_pending_ && !wbc_preposition_cancel_pending_ &&
                       (!wbc_preposition_active_.load() || wbc_preposition_ready_);
  if (wbc_handoff_pending_)
  {
    if (wbc_controller_walking_)
    {
      if (wbc_handoff_target_ == PolicyCmdSource::kWbc && !policyMux_.waitForWbcCommandAfter(0, 0.0))
      {
        lock.unlock();
        std::string message;
        requestWbcReturn("[WBC] Prepared output expired before handoff; keeping AMP", message, true);
        return;
      }
      if (wbc_handoff_target_ == PolicyCmdSource::kRl &&
          (!wbc_return_pending_.load() || (wbc_live_return_ && !safe) ||
           !policyMux_.rlReturnCommandReady(wbc_return_id_)))
      {
        wbc_handoff_pending_ = false;
        // A result can expire at this boundary; keep the held command while AMP retries.
        return;
      }
      const bool soft_return = wbc_handoff_target_ == PolicyCmdSource::kRl && wbc_live_return_;
      const double blend = soft_return ? .35 : wbc_transition_sec_;
      if (blend > 1e-9 && !startPolicyHandoff(blend, "policy switch", soft_return))
      {
        policy_handoff_.clear();
        if (wbc_handoff_target_ == PolicyCmdSource::kWbc)
        {
          lock.unlock();
          std::string message;
          requestWbcReturn("[WBC] No valid handoff origin; keeping AMP", message, true);
          return;
        }
        RCLCPP_WARN(nh_->get_logger(), "[WBC] No valid upper-body blend origin; using fresh AMP directly");
      }
      policyMux_.setSource(wbc_handoff_target_);
      wbc_preposition_active_.store(false);
      wbc_preposition_pending_ = wbc_preposition_ready_ = false;
      if (wbc_handoff_target_ == PolicyCmdSource::kRl)
      {
        if (soft_return) policyMux_.publishWbcActive(false,"[WBC] Soft STOP transferred to AMP");
        const bool failed = wbc_return_outcome_ == WbcReturnOutcome::Failed;
        const char* status = failed ? "failed" :
            (wbc_return_outcome_ == WbcReturnOutcome::Cancelled ? "cancelled" : "completed");
        publishActiveWbcActionTerminal(status, failed ? "WBC recovery completed; AMP control restored" : "AMP control restored");
        RCLCPP_DEBUG(nh_->get_logger(), "[WBC Return] AMP control restored; action=%s; worker cleanup remains required", status);
        wbc_return_id_.clear();
      }
    }
    wbc_return_pending_.store(false);
    wbc_handoff_pending_ = false;
  }
}

}  // namespace legged
