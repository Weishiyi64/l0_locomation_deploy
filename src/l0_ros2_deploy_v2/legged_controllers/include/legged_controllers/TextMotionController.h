#pragma once

#include <atomic>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <string>
#include <thread>
#include <vector>

namespace legged {

// Runtime overlay for the validated 9-joint trajectory produced by texttomotion/run_offline.sh.
// The ROS callback queues a complete message; a worker validates every frame before playback.
class TextMotionController {
 public:
  struct Params {
    bool enabled{true};
    double entry_blend_sec{0.2};
    double exit_blend_sec{0.2};
    std::size_t max_payload_bytes{4U * 1024U * 1024U};
    std::size_t max_frames{5000U};
  };

  TextMotionController() : logger_(rclcpp::get_logger("text_motion")) {}
  ~TextMotionController();

  bool configure(const Params& params, const std::vector<std::string>& robot_joint_names, rclcpp::Logger logger,
                 std::string& error);
  // Queues one already-complete ROS message. Parsing and frame validation happen on the worker thread.
  bool submitJson(const std::string& payload, std::string& error);
  std::string takePreparationStatus();
  std::string step(double wall_sec, const std::vector<double>& base_positions, bool playback_allowed);
  bool getJointPosition(std::size_t joint_index, double& q) const;
  bool active() const { return active_.load(); }
  bool enabled() const { return params_.enabled; }
  void stop();

 private:
  struct Trajectory {
    double fps{0.0};
    std::vector<std::size_t> joint_indices;
    std::vector<std::vector<double>> frames;
    std::vector<bool> active_joints;
    bool has_explicit_end_pose{false};
  };

  static bool finiteInRange(const std::string& name, double value);
  bool parseJson(const std::string& payload, Trajectory& parsed, std::string& error) const;
  void workerLoop();
  void pushPreparationStatus(std::string status);
  void sampleTrajectory(double frame_position);

  rclcpp::Logger logger_;
  Params params_;
  std::vector<std::string> robot_joint_names_;
  std::vector<bool> controlled_joints_;
  std::vector<double> q_cmd_;
  std::vector<double> blend_start_;

  mutable std::mutex pending_mutex_;
  Trajectory pending_;
  bool pending_available_{false};

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::string queued_payload_;
  std::uint64_t queued_request_id_{0};
  std::uint64_t latest_request_id_{0};
  bool worker_stop_{false};
  std::thread worker_thread_;

  std::mutex status_mutex_;
  std::deque<std::string> preparation_status_;

  Trajectory current_;
  std::atomic<bool> active_{false};
  double started_at_{0.0};
};

}  // namespace legged
