#pragma once

#include <array>
#include <functional>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <string>

#include "legged_controllers/HeadScript.h"

namespace legged {

class FaceFollowController {
 public:
  struct Target {
    double x{0.5};
    double y{0.5};
    bool visible{false};
  };

  bool parsePayload(const std::string& payload, Target& target, std::string& error) const;
  void processTarget(const Target& target, double wall_sec, HeadScriptController& head,
                     const std::function<std::array<double, 3>()>& read_head, bool allowed);
  void setAllowed(bool allowed, double wall_sec, HeadScriptController& head,
                  const std::function<std::array<double, 3>()>& read_head);

 private:
  void setAllowedLocked(bool allowed, double wall_sec, HeadScriptController& head,
                        const std::function<std::array<double, 3>()>& read_head);

  mutable std::mutex mutex_;
  double yaw_{0.0};
  double pitch_{0.0};
  bool last_valid_{false};
  bool paused_{true};
  bool reseed_{true};
  int invalid_count_{10};
};

}  // namespace legged
