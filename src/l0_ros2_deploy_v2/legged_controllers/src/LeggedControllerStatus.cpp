#include "LeggedController.h"
#include <nlohmann/json.hpp>

namespace legged {

void LeggedController::captureControllerStatus(bool walking, const PolicyInputs& inputs) {
  // The control loop never waits for the telemetry timer.
  std::unique_lock<std::mutex> lock(status_snapshot_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  auto& out = status_snapshot_;
  out.sampled = std::chrono::steady_clock::now();
  out.mode = fsm_->state();
  out.source = inputs.source;
  out.required = walking;
  out.valid = inputs.valid;
  out.policy_age_ms = inputs.last_update_age_ms;
  out.policy_valid_for_ms = inputs.valid_for_ms;
  out.emergency = fsm_->isEmergency();
  out.reasons = stop_reasons_.load();
  out.holding_last_command = walking && !out.emergency && holding_policy_command_;
}

void LeggedController::publishControllerStatus() {
  StatusSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(status_snapshot_mutex_);
    snapshot = status_snapshot_;
  }
  if (snapshot.sampled == std::chrono::steady_clock::time_point{}) return;
  using Json = nlohmann::json;
  const double age = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - snapshot.sampled).count();
  const char* mode = "standby";
  switch (snapshot.mode)
  {
    case StateMachine::State::Walking: mode = "motion"; break;
    case StateMachine::State::Calibration: mode = "calibration"; break;
    case StateMachine::State::EmergencyStop: mode = "emergency"; break;
    default: break;
  }
  Json reasons = Json::array();
  for (const auto& reason : std::vector<std::pair<uint32_t, const char*>>{
           {ExternalStop, "external_or_hardware_stop"}, {ManualStop, "manual_stop"},
           {OrientationStop, "orientation_limit"}, {NoValidCommand, "no_valid_joint_command"}})
    if (snapshot.reasons & reason.first) reasons.push_back(reason.second);
  const Json status = {
      {"schema_version", 1}, {"sample_age_ms", age},
      {"sample_monotonic_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
          snapshot.sampled.time_since_epoch()).count()}, {"mode", mode},
      {"emergency", {{"active", snapshot.emergency}, {"reasons", reasons}}},
      {"policy", {{"source", snapshot.source == PolicyCmdSource::kWbc ? "wbc" :
                                  (snapshot.source == PolicyCmdSource::kImitation ? "imitation" : "amp")},
                  {"required", snapshot.required},
                  {"holding_last_command", snapshot.holding_last_command},
                  {"valid", snapshot.required ? Json(snapshot.valid && snapshot.policy_valid_for_ms > age) : Json(nullptr)},
                  {"valid_for_ms", snapshot.required ? Json(std::max(0.0, snapshot.policy_valid_for_ms - age)) : Json(nullptr)},
                  {"last_update_age_ms", snapshot.policy_age_ms < 0 ? Json(nullptr) :
                                             Json(snapshot.policy_age_ms + age)}}}};
  std_msgs::msg::String message;
  message.data = status.dump();
  controller_status_pub_->publish(message);
}

}  // namespace legged
