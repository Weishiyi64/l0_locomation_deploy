#pragma once

#include <yaml-cpp/yaml.h>

#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <string>
#include <utility>

#include "legged_controllers/HeadScript.h"

namespace legged {

/// Loads and holds head gaze parameters from head_topic_controller.yaml, and
/// applies /eye_target pixel deltas to accumulated yaw/pitch targets.
///
/// Does not subscribe to topics, touch hardware, or publish motor commands —
/// LeggedController owns ROS I/O and motor command publishing.
class HeadTopicController {
 public:
  /// Parameters loaded from head_topic_controller.yaml.
  struct Params {
    double deadZone{0.05};
    double scale{0.232};
    double yawClampMin{-0.8};
    double yawClampMax{0.8};
    double pitchClampMin{-0.4};
    double pitchClampMax{0.4};
    int invalidFrameThreshold{10};
    /// /eye_target 模式下颈部 PD（顺序：偏航、俯仰、滚动），来自 head_topic_controller.yaml。
    double kp[3]{};
    double kd[3]{};
    double tauFf[3]{};
    double topicBlendSec{0.1};
    double fallbackBlendSec{};
  };

  HeadTopicController() = default;

  /// Load tunable parameters from the YAML file at path.
  /// Returns false and logs a warning if the file is missing or malformed;
  /// default parameter values remain in effect in that case.
  bool loadConfig(const std::string& path, rclcpp::Logger logger);

  /// Process one /eye_target message: updates topic validity on \p headCtrl,
  /// integrates pixel offsets into internal yaw/pitch, and calls setTarget.
  ///
  /// When transitioning from invalid→valid and waypoint state is not ready,
  /// \p read_neck_yaw_pitch_rad is invoked to read actual neck joints from
  /// hardware (implemented by LeggedController using robot_hw_).
  void processEyeTarget(const geometry_msgs::msg::Point& msg, double t_wall_sec, HeadScriptController& headCtrl,
                        const std::function<std::pair<double, double>()>& read_neck_yaw_pitch_rad,
                        bool allow_integration);

  /// Pause/resume topic integration to match command ownership. On resume the
  /// integrator is re-seeded from the actual neck pose before accepting targets.
  void setIntegrationAllowed(bool allowed, double t_wall_sec, HeadScriptController& headCtrl,
                             const std::function<std::pair<double, double>()>& read_neck_yaw_pitch_rad);

  const Params& params() const { return params_; }

 private:
  void setIntegrationAllowedLocked(bool allowed, double t_wall_sec, HeadScriptController& headCtrl,
                                   const std::function<std::pair<double, double>()>& read_neck_yaw_pitch_rad);

  Params params_;
  rclcpp::Logger logger_{rclcpp::get_logger("head_topic_ctrl")};
  mutable std::mutex mutex_;

  /// Accumulated topic-driven target angles (radians).
  double headTopicYaw_{0.0};
  double headTopicPitch_{0.0};

  /// Invalid-z debounce.
  bool lastEyeTargetZValid_{false};
  int eyeTargetInvalidCount_{0};
  bool integrationPaused_{false};
  bool reseedOnNextValid_{false};
};

}  // namespace legged
