#include "LeggedReal.h"
#include <nlohmann/json.hpp>

namespace legged {

void LeggedReal::captureHardwareStatus(bool protection, bool command_invalid) {
  // Called only by write(), with the existing lifecycle/feedback locks held.
  std::unique_lock<std::mutex> lock(hardware_status_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  auto& out = hardware_status_;
  out.sampled = std::chrono::steady_clock::now();
  out.protection = protection;
  out.command_invalid = command_invalid;
  out.motor_fault = motor_fault_.load();
  out.imu_fault = imu_fault_.load();
  out.lifecycle_fault = lifecycle_fault_.load();
  out.motors_enabled = motors_enabled_.load();
  out.calibration = calibration_mode_.load();
  const auto accepted = imu_accepted_ns_.load();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(out.sampled.time_since_epoch()).count();
  out.imu_age_ms = accepted == 0 ? -1.0 : double(now_ns - accepted) / 1e6;
  for (std::size_t i = 0; i < motor_mapping_->size(); ++i)
  {
    const auto& wrap = feedback_wrap_[i];
    const auto& feedback = control_feedback_[i];
    auto& motor = out.motors[i];
    motor.fault = feedback_fault_[i] || wrap.fault;
    motor.position_ready = wrap.ready();
    motor.valid = feedback.valid;
    motor.initializing = feedback.initializing;
    motor.error_pending = feedback.error_pending;
    motor.error_code = wrap.error_code != 0 ? wrap.error_code : feedback.error;
    motor.missing_cycles = consecutive_missing_feedback_[i];
  }
}

void LeggedReal::publishHardwareStatus() {
  HardwareStatus snapshot;
  {
    std::lock_guard<std::mutex> lock(hardware_status_mutex_);
    snapshot = hardware_status_;
  }
  if (snapshot.sampled == std::chrono::steady_clock::time_point{}) return;
  using Json = nlohmann::json;
  const double age = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - snapshot.sampled).count();
  Json reasons = Json::array(), motors = Json::array();
  if (snapshot.motor_fault) reasons.push_back("motor_feedback_fault");
  if (snapshot.imu_fault) reasons.push_back("imu_timeout");
  if (snapshot.lifecycle_fault) reasons.push_back("motor_enable_channel_timeout");
  if (snapshot.command_invalid) reasons.push_back("motor_command_rejected");
  bool position_unconfirmed = false;
  for (std::size_t i = 0; i < motor_mapping_->size(); ++i)
  {
    const auto& mapping = motor_mapping_->entries()[i];
    if (!mapping.enabled) continue;
    const auto& motor = snapshot.motors[i];
    position_unconfirmed = position_unconfirmed || !motor.position_ready;
    const char* reason = "ok";
    if (motor.missing_cycles >= max_consecutive_missing_) reason = "feedback_timeout";
    else if (motor.fault) reason = "feedback_fault";
    else if (motor.error_pending || motor.error_code != 0) reason = "motor_error";
    else if (!motor.position_ready || motor.initializing) reason = "position_unconfirmed";
    else if (!motor.valid) reason = "invalid_feedback";
    motors.push_back({{"index", i}, {"joint", mapping.joint_name}, {"fault", motor.fault},
                      {"position_ready", motor.position_ready}, {"error_code", motor.error_code}, {"reason", reason}});
  }
  if (snapshot.protection && position_unconfirmed) reasons.push_back("position_unconfirmed");
  const double imu_age = snapshot.imu_age_ms < 0 ? -1 : snapshot.imu_age_ms + age;
  const Json status = {
      {"schema_version", 1}, {"sample_age_ms", age},
      {"sample_monotonic_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
          snapshot.sampled.time_since_epoch()).count()},
      {"protection", {{"active", snapshot.protection}, {"reasons", reasons}}},
      {"motors_enabled", snapshot.motors_enabled}, {"calibration", snapshot.calibration},
      {"enabled_motors", motors}, {"motor_feedback_timeout_ms", feedback_timeout_ms_},
      {"imu", {{"enabled", imu_enabled_}, {"timeout_ms", imu_connection_timeout_ * 1000.0},
               {"valid", !imu_enabled_ ? Json(nullptr) : Json(imu_age >= 0 &&
                         imu_age <= imu_connection_timeout_ * 1000.0 && !snapshot.imu_fault)},
               {"last_update_age_ms", imu_age < 0 ? Json(nullptr) : Json(imu_age)}}}};
  std_msgs::msg::String message;
  message.data = status.dump();
  fault_detail_pub_->publish(message);
}

}  // namespace legged
