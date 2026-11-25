#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace legged_controllers {

// Input: [goal_x_odom, goal_y_odom, linear_speed, target_heading_odom, angular_speed, visible].
// Zero speed fields select the deployment defaults; nonzero fields are bounded to deployment ranges.
class FollowGoalTracker {
 public:
  static constexpr std::size_t kGoalSize = 6;
  static constexpr std::size_t kPlannerInputSize = 9;

  enum class AcceptResult { Accepted, Stopped, Invalid };

  struct Output {
    std::array<float, kPlannerInputSize> data{};
    const char* mode{"Hold"};
  };

  struct Pose2D {
    float x{0.0f};
    float y{0.0f};
    float yaw{0.0f};
  };

  AcceptResult accept(const std::vector<float>& values, double now_seconds) {
    if (values.size() != kGoalSize || !std::isfinite(now_seconds) ||
        !std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); }))
    {
      reset();
      return AcceptResult::Invalid;
    }
    if (values[5] == 0.0f)
    {
      reset();
      return AcceptResult::Stopped;
    }
    if (values[5] != 1.0f)
    {
      reset();
      return AcceptResult::Invalid;
    }
    if (active_ && (now_seconds < last_goal_seconds_ || now_seconds - last_goal_seconds_ > kGoalTimeoutSeconds))
      reset();

    const float linear_speed =
        values[2] == 0.0f ? kDefaultLinearSpeed : clamp(values[2], kMinLinearSpeed, kMaxLinearSpeed);
    const float requested_angular_speed = std::abs(values[4]);
    const float angular_speed =
        requested_angular_speed == 0.0f ? kDefaultAngularSpeed : clamp(requested_angular_speed, 0.0f, kMaxAngularSpeed);
    goal_ = {values[0], values[1], linear_speed, wrap(values[3]), angular_speed};
    last_goal_seconds_ = now_seconds;
    active_ = true;
    return AcceptResult::Accepted;
  }

  Output update(double now_seconds, const Pose2D* pose) {
    if (!active_ || !std::isfinite(now_seconds) || now_seconds < last_goal_seconds_ ||
        now_seconds - last_goal_seconds_ > kGoalTimeoutSeconds)
    {
      reset();
      return {};
    }
    if (pose == nullptr) return {};

    const float delta_x_odom = goal_[0] - pose->x;
    const float delta_y_odom = goal_[1] - pose->y;
    const float distance = std::hypot(delta_x_odom, delta_y_odom);
    if (!std::isfinite(distance)) return {};
    if (position_locked_ && distance > kPositionExitRadius)
    {
      position_locked_ = false;
      heading_locked_ = false;
    }
    if (!position_locked_)
    {
      if (distance > kPositionEnterRadius)
      {
        const float cosine = std::cos(pose->yaw);
        const float sine = std::sin(pose->yaw);
        const float delta_x_body = cosine * delta_x_odom + sine * delta_y_odom;
        const float delta_y_body = -sine * delta_x_odom + cosine * delta_y_odom;
        return navOutput(delta_x_body, delta_y_body);
      }
      position_locked_ = true;
      heading_locked_ = false;
    }

    const float relative_heading = wrap(goal_[3] - pose->yaw);
    const float heading_error = std::abs(relative_heading);
    heading_locked_ = heading_locked_ ? heading_error <= kHeadingExitError : heading_error <= kHeadingEnterError;
    return heading_locked_ ? Output{} : faceOutput(relative_heading);
  }

  void reset() {
    active_ = false;
    position_locked_ = false;
    heading_locked_ = false;
    goal_.fill(0.0f);
    last_goal_seconds_ = 0.0;
  }

  bool active() const { return active_; }

 private:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kDefaultLinearSpeed = 0.35f;
  static constexpr float kMinLinearSpeed = 0.3f;
  static constexpr float kMaxLinearSpeed = 0.4f;
  static constexpr float kDefaultAngularSpeed = 1.57f;
  static constexpr float kMaxAngularSpeed = 1.57f;
  static constexpr float kPositionEnterRadius = 0.25f;
  static constexpr float kPositionExitRadius = 0.5f;
  static constexpr float kHeadingEnterError = kPi / 12.0f;
  static constexpr float kHeadingExitError = kPi / 4.0f;
  static constexpr float kWaypointSpacing = 1.0f;
  static constexpr double kGoalTimeoutSeconds = 0.5;

  static float clamp(float value, float lower, float upper) { return std::max(lower, std::min(value, upper)); }

  static float wrap(float angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

  Output navOutput(float delta_x_body, float delta_y_body) const {
    const float distance = std::hypot(delta_x_body, delta_y_body);
    const float first_scale = std::min(1.0f, kWaypointSpacing / distance);
    const float second_scale = std::min(1.0f, 2.0f * kWaypointSpacing / distance);
    const float third_scale = std::min(1.0f, 3.0f * kWaypointSpacing / distance);
    Output output;
    output.data = {delta_x_body * first_scale,
                   delta_y_body * first_scale,
                   goal_[2],
                   0.0f,
                   0.0f,
                   delta_x_body * second_scale,
                   delta_y_body * second_scale,
                   delta_x_body * third_scale,
                   delta_y_body * third_scale};
    output.mode = "Nav";
    return output;
  }

  Output faceOutput(float heading_error) const {
    Output output;
    output.data = {0.0f, 0.0f, 0.0f, heading_error, goal_[4], 0.0f, 0.0f, 0.0f, 0.0f};
    output.mode = "Face";
    return output;
  }

  std::array<float, 5> goal_{};
  double last_goal_seconds_{0.0};
  bool active_{false};
  bool position_locked_{false};
  bool heading_locked_{false};
};

}  // namespace legged_controllers
