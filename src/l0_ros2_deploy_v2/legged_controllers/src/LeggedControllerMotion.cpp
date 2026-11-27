#include "LeggedController.h"

#include <chrono>
#include <cmath>

namespace legged {
namespace {
int64_t motionNowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
int64_t motionSteadyMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool motionId(const std::string& id) {
  return !id.empty() && id.size() <= 32 && std::all_of(id.begin(), id.end(), [](unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
}
}  // namespace

bool LeggedController::motionBaseReady() {
  if (!wbc_enabled_ || wbc_cleanup_pending_.load() || policyMux_.wbcActiveCommanded() ||
      policyMux_.source() != PolicyCmdSource::kRl || policyMux_.imitationActiveCommanded()) return false;
  {
    std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
    if (!wbc_start_allowed_ || !wbc_controller_walking_ || wbc_return_pending_.load() ||
        wbc_handoff_in_progress_.load() || wbc_preposition_pending_ || wbc_preposition_active_.load() ||
        wbc_preposition_cancel_pending_) return false;
  }
  return policyMux_.selectInputs(true).valid;
}

void LeggedController::updateMotionPreparation() {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  if (!motion_reserved_.load()) motion_head_pause_applied_ = false;
  else if (policyMux_.source() != PolicyCmdSource::kRl || wbc_preposition_active_.load())
    motion_head_pause_applied_ = true;
  if (!motion_reserved_.load() || !fsm_->isWalking() || policyMux_.source() != PolicyCmdSource::kRl ||
      wbc_preposition_active_.load())
  {
    motion_head_pending_ = false;
    motion_head_hold_active_ = false;
    motion_upper_snapshot_valid_ = false;
    text_capture_stamp_ns_ = 0;
    return;
  }
  if (motion_cancelled_.load())
  {
    text_capture_stamp_ns_ = 0;
  }
  if (!motion_head_pending_) return;
  const auto& positions = policy_observation_.joints.position;
  if (jointNames_.size() != 21 || positions.size() != 21 ||
      !std::all_of(positions.begin() + 18, positions.end(), [](double q) { return std::isfinite(q); })) return;
  std::copy(positions.begin() + 18, positions.end(), motion_head_positions_.begin());
  motion_head_pending_ = false;
  motion_head_hold_active_ = true;
  motion_head_pause_applied_ = true;
  motion_upper_snapshot_valid_ = false;
  RCLCPP_DEBUG(nh_->get_logger(), "[Motion] Holding measured head pose during preparation for %s",
              motion_request_id_.c_str());
}

void LeggedController::initializeMotionAdmission() {
  if (!wbc_enabled_) return;
  motion_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  motion_service_ = nh_->create_service<robot_interfaces::srv::MotionTransaction>(
      "/control/motion/transaction",
      std::bind(&LeggedController::motionTransaction, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, motion_callback_group_);
  motion_watchdog_ = nh_->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    if (!motion_reserved_.load() || motionSteadyMs() - motion_gateway_seen_ms_.load() < 1000) return;
    auto request = std::make_shared<robot_interfaces::srv::MotionTransaction::Request>();
    request->command = robot_interfaces::srv::MotionTransaction::Request::CANCEL;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      if (!motion_reserved_.load()) return;
      request->request_id = motion_request_id_;
      request->source = motion_source_;
    }
    auto response = std::make_shared<robot_interfaces::srv::MotionTransaction::Response>();
    // Losing the gateway cancels execution, but cannot release an unconfirmed text cleanup.
    motionTransaction(request, response);
  }, motion_callback_group_);
}

void LeggedController::motionTransaction(
    const std::shared_ptr<robot_interfaces::srv::MotionTransaction::Request> request,
    std::shared_ptr<robot_interfaces::srv::MotionTransaction::Response> response) {
  using Request = robot_interfaces::srv::MotionTransaction::Request;
  std::unique_lock<std::mutex> admission(auxiliary_action_trigger_mutex_, std::defer_lock);
  if (request->command == Request::RESERVE && !admission.try_lock()) {
    response->message = "another action admission/preparation is running";
    return;
  }
  const auto now = motionNowMs();
  const bool base_ready = motionBaseReady();
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    const bool ready = base_ready && !motion_reserved_.load();
    if (ready && !motion_ready_seen_) motion_ready_since_ms_ = now;
    motion_ready_seen_ = ready;
    response->ready = ready;
    response->ready_since_ms = motion_ready_since_ms_;
    if (request->command == Request::STATUS) {
      motion_gateway_seen_ms_.store(motionSteadyMs());
      response->success = true;
      response->payload_json = "{\"request_id\":\"" + motion_request_id_ + "\",\"source\":\"" + motion_source_ +
          "\",\"cancelled\":" + (motion_cancelled_.load() ? "true" : "false") +
          ",\"text_capture_stamp_ns\":" + std::to_string(text_capture_stamp_ns_) + "}";
      response->message = ready ? "AMP ready" : "busy, transitioning, stale AMP or unsafe controller state";
      return;
    }
    if (!motionId(request->request_id)) { response->message = "invalid request_id"; return; }
    if (request->command == Request::RESERVE) {
      if ((request->source != "wbc" && request->source != "txt2motion") || !ready ||
          request->timestamp_ms <= 0 || request->timestamp_ms < now - 5000 ||
          request->timestamp_ms > now + 1000 || request->timestamp_ms < motion_ready_since_ms_ ||
          std::find(motion_cancelled_ids_.begin(), motion_cancelled_ids_.end(), request->request_id) != motion_cancelled_ids_.end()) {
        response->message = "request rejected: not current AMP-ready epoch, busy or cancelled";
        return;
      }
      const auto action_id = request->source == "txt2motion" ? request->request_id : request->action_id;
      if (!motionId(action_id)) { response->message = "invalid action_id"; return; }
      motion_request_id_ = request->request_id;
      motion_source_ = request->source;
      motion_action_id_ = action_id;
      motion_timestamp_ms_ = request->timestamp_ms;
      motion_cancelled_.store(false);
      motion_head_pending_ = true;
      text_capture_stamp_ns_ = 0;
      motion_reserved_.store(true);
      motion_gateway_seen_ms_.store(motionSteadyMs());
      motion_ready_seen_ = false;
      response->ready = false;
      response->success = true;
      response->message = "reserved; AMP remains in control during generation/preparation";
      return;
    }
    if (motion_reserved_.load() && motion_request_id_ != request->request_id) {
      response->message = "request does not own the motion reservation";
      return;
    }
    if (request->command == Request::CANCEL) {
      if (std::find(motion_cancelled_ids_.begin(), motion_cancelled_ids_.end(), request->request_id) == motion_cancelled_ids_.end())
        motion_cancelled_ids_.push_back(request->request_id);
      if (motion_cancelled_ids_.size() > 256) motion_cancelled_ids_.pop_front();
      if (!motion_reserved_.load()) { response->success = true; return; }
      motion_cancelled_.store(true);
      text_capture_stamp_ns_ = 0;
    } else if (request->command == Request::EXECUTE) {
      if (!motion_reserved_.load() || motion_cancelled_.load() || request->source != motion_source_ ||
          request->action_id != motion_action_id_ || request->timestamp_ms != motion_timestamp_ms_) {
        response->message = "missing, cancelled or mismatched motion reservation";
        return;
      }
    } else if (request->command != Request::RELEASE) {
      response->message = "unsupported transaction command";
      return;
    } else if (!motion_reserved_.load()) {
      response->success = base_ready;
      response->message = base_ready ? "already released; AMP ready" : "waiting for stable AMP";
      return;
    }
  }
  if (request->command == Request::CANCEL) {
    std::lock_guard<std::mutex> lock(wbc_transition_mutex_);
    std::lock_guard<std::mutex> reservation(motion_mutex_);
    if (!motion_reserved_.load() || motion_request_id_ != request->request_id) {
      response->message = "cancellation belongs to an already retired reservation";
      return;
    }
    if (!wbc_return_pending_.load()) ++wbc_generation_;  // Invalidate preparation, not an in-flight return.
    if (wbc_preposition_pending_ || wbc_preposition_active_.load()) wbc_preposition_cancel_pending_ = true;
    wbc_transition_cv_.notify_all();
  }
  std::unique_lock<std::mutex> trigger(auxiliary_action_trigger_mutex_, std::try_to_lock);
  if (!trigger.owns_lock()) { response->message = "controller preparation/cleanup still retiring"; return; }
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!motion_reserved_.load() || motion_request_id_ != request->request_id) {
      response->message = "reservation changed before transaction; retry cleanup status";
      return;
    }
  }
  if (request->command == Request::EXECUTE) {
    if (motion_cancelled_.load()) { response->message = "cancelled before preparation"; return; }
    auto prepared = std::make_shared<robot_interfaces::srv::ControlWbcMotion::Response>();
    response->success = startWbcMotion(request->action_id, motion_timestamp_ms_, prepared);
    response->message = prepared->message;
    return;
  }
  if (policyMux_.wbcActiveCommanded() || policyMux_.source() == PolicyCmdSource::kWbc) {
    requestWbcReturn("[Motion] Cancel/release", response->message);
    // A staged return is not a completed return; RELEASE must check again.
    response->success = false;
    return;
  }
  const auto cancelled = policyMux_.prepareWbcMotion(
      robot_interfaces::srv::ControlWbcMotion::Request::CANCEL,
      request->source == "txt2motion" ? request->request_id : "", wbc_command_timeout_sec_, request->request_id);
  wbc_cleanup_pending_.store(!cancelled || !cancelled->success);
  if (wbc_cleanup_pending_.load()) { response->message = "worker cleanup not acknowledged"; return; }
  if (request->command == Request::CANCEL) { response->success = true; return; }
  if (!motionBaseReady()) { response->message = "waiting for fresh AMP and completed return blend"; return; }
  std::lock_guard<std::mutex> lock(motion_mutex_);
  motion_reserved_.store(false);
  motion_head_pending_ = false;
  text_capture_stamp_ns_ = 0;
  motion_request_id_.clear();
  motion_source_.clear();
  motion_ready_since_ms_ = motionNowMs();
  motion_ready_seen_ = true;
  response->ready = true;
  response->ready_since_ms = motion_ready_since_ms_;
  response->success = true;
  response->message = "released after confirmed AMP recovery";
}
}  // namespace legged
