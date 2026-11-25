#include "legged_controllers/LeggedController.h"

namespace legged {

void LeggedController::textMotionCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  std::unique_lock<std::mutex> lock(auxiliary_action_trigger_mutex_, std::try_to_lock);
  const bool allowed = lock.owns_lock() && text_motion_start_allowed_.load() &&
                       !sequenceRuntime_.isActive() && !policyMux_.imitationActiveCommanded() &&
                       !policyMux_.wbcActiveCommanded() && !wbc_return_pending_.load() &&
                       !wbc_handoff_in_progress_.load();
  // Complete JSON parsing/validation stays on this dedicated executor group.
  publishTextMotionEvent(textMotionCtrl_.submitJson(msg->data, allowed));
}

void LeggedController::textMotionCancelCallback(const std_msgs::msg::String::ConstSharedPtr msg) {
  publishTextMotionEvent(textMotionCtrl_.cancel(msg->data));
}

void LeggedController::publishTextMotionEvent(const TextMotionController::Event& event) {
  if (event.state.empty()) return;
  std::lock_guard<std::mutex> lock(text_motion_events_mutex_);
  if (text_motion_events_.size() >= 64) text_motion_events_.pop_front();
  text_motion_events_.push_back(event);
}

void LeggedController::flushTextMotionEvents() {
  std::deque<TextMotionController::Event> events;
  {
    std::lock_guard<std::mutex> lock(text_motion_events_mutex_);
    events.swap(text_motion_events_);
  }
  for (const auto& event : events)
  {
    std_msgs::msg::String message;
    message.data = TextMotionController::encodeEvent(event);
    textMotionStatusPub_->publish(message);
  }
}

}  // namespace legged
