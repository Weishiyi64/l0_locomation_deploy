#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <legged_controllers/IHardwareLayer.h>

namespace legged {

// Parsing runs in a separate ROS callback group. Only step() writes playback
// state and joint commands; callbacks hand over an immutable, complete motion.
class TextMotionController {
 public:
  struct Event {
    std::string request_id;
    std::string state;
    std::string detail;
  };

  bool configure(const std::vector<std::string>& joints, const std::string& mapping_file, std::string& error);
  Event submitJson(const std::string& payload, bool allowed);
  Event cancel(const std::string& request_id);
  void setAllowed(bool allowed);
  Event step(const std::vector<double>& base, const std::vector<JointMotorCommand>& previous, bool allowed);
  bool getJointPosition(std::size_t index, double& q) const;
  bool busy() const { return busy_.load(); }
  bool active() const { return active_.load(); }
  static std::string encodeEvent(const Event& event);

 private:
  using Pose = std::array<double, 9>;
  struct Trajectory {
    std::string id;
    std::int64_t expires_at_ms{0};
    std::array<bool, 9> mask;
    std::vector<Pose> frames;
  };
  Event finish(const char* state, const char* detail);
  bool seen(const std::string& id) const;
  void remember(const std::string& id);

  std::array<std::size_t, 9> indices_;
  Pose lower_, upper_, q_, entry_, release_;
  std::mutex mutex_;
  std::shared_ptr<const Trajectory> pending_;
  std::shared_ptr<const Trajectory> current_;
  std::deque<std::string> recent_ids_;
  std::string slot_id_;
  bool cancelled_{false};
  std::atomic<bool> busy_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> allowed_{false};
  std::atomic<std::uint64_t> gate_generation_{0};
  std::chrono::steady_clock::time_point started_, releasing_at_;
  bool releasing_{false};
  bool cancelled_release_{false};
  double release_alpha_{0.0};
};

}  // namespace legged
