// =============================================================================
// diag_subscriber_node — C++ subscriber-side frame-rate & latency diagnostic (real-time)
// =============================================================================
// Subscribes to all driver data topics and counts messages at the *subscriber*
// level (after full ROS serialize → DDS → deserialize path).  Much more
// accurate than `ros2 topic hz` (Python) for large messages like PointCloud2.
//
// No separate service — listens to the driver's `diag_status` topic
// (published by `set_diag_monitor` service) to sync enable/disable.
// Defaults to enabled so it starts printing immediately on launch.
//
// Usage:
//   ros2 run odin_ros_driver_rev1 diag_subscriber_node
//     --ros-args -p topic_prefix:=/manifold/device0/
// =============================================================================

#include "odin_app_version.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if defined(ODIN_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using ImuMsg = sensor_msgs::msg::Imu;
using OdometryMsg = nav_msgs::msg::Odometry;
using BoolMsg = std_msgs::msg::Bool;
#else
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

using PointCloud2Msg = sensor_msgs::PointCloud2;
using CompressedImageMsg = sensor_msgs::CompressedImage;
using ImuMsg = sensor_msgs::Imu;
using OdometryMsg = nav_msgs::Odometry;
using BoolMsg = std_msgs::Bool;
#endif

// ---------------------------------------------------------------------------
// Per-channel statistics
// ---------------------------------------------------------------------------
struct ChannelStats {
  std::atomic<uint64_t> count{0};
  std::atomic<int64_t> last_latency_ns{0};
  uint64_t prev_count = 0;  // accessed under diag_mutex only
  // Sliding window for latency stats (ring buffer, O(1) avg/min/max update)
  std::vector<int64_t> latency_window;
  size_t latency_window_index = 0;
  int64_t latency_window_sum = 0;
  int64_t latency_window_min = 0;
  int64_t latency_window_max = 0;
};

class DiagSubscriberNode
#if defined(ODIN_ROS2)
    : public rclcpp::Node
#endif
{
 public:
#if defined(ODIN_ROS2)
  DiagSubscriberNode() : Node("diag_subscriber") {
    topic_prefix_ = declare_parameter<std::string>("topic_prefix", "/manifold/device0/");
    InitSubscribers();
    InitDiagStatusListener();
  }
#else
  DiagSubscriberNode() {
    ros::NodeHandle pnh("~");
    topic_prefix_ = pnh.param<std::string>("topic_prefix", "/manifold/device0/");
    InitSubscribers();
    InitDiagStatusListener();
  }
#endif

  ~DiagSubscriberNode() { diag_enabled_.store(false); }

 private:
  std::string topic_prefix_;
  std::atomic<bool> diag_enabled_{true};   // default enabled
  std::mutex diag_mutex_;
  std::chrono::steady_clock::time_point prev_print_time_;

  ChannelStats raw_point_;
  ChannelStats slam_point_;
  ChannelStats image0_;
  ChannelStats imu_;
  ChannelStats odom_lf_;
  ChannelStats odom_hf_;

#if defined(ODIN_ROS2)
  rclcpp::Subscription<BoolMsg>::SharedPtr diag_status_sub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
#else
  ros::Subscriber diag_status_sub_;
  ros::Timer diag_timer_;
  std::vector<ros::Subscriber> subs_;
#endif

  // -- Sync enable/disable from driver's diag_status topic --
  void ResetChannelStats(ChannelStats& s) {
    s.prev_count = s.count.load();
    s.latency_window.clear();
    s.latency_window_index = 0;
    s.latency_window_sum = 0;
    s.latency_window_min = 0;
    s.latency_window_max = 0;
  }

  void OnDiagStatus(bool enable) {
    diag_enabled_.store(enable);
    if (enable) {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      ResetChannelStats(raw_point_);
      ResetChannelStats(slam_point_);
      ResetChannelStats(image0_);
      ResetChannelStats(imu_);
      ResetChannelStats(odom_lf_);
      ResetChannelStats(odom_hf_);
      prev_print_time_ = std::chrono::steady_clock::time_point();
    }
    printf("[diag_subscriber] %s\n", enable ? "enabled (synced from driver)" : "disabled (synced from driver)");
  }

  void InitDiagStatusListener() {
    // TRANSIENT_LOCAL so we get the last value if we start late
#if defined(ODIN_ROS2)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    diag_status_sub_ = create_subscription<BoolMsg>(
        topic_prefix_ + "diag_status", qos,
        [this](const BoolMsg::SharedPtr msg) { OnDiagStatus(msg->data); });
#else
    ros::NodeHandle nh;
    // ROS1 latch=true gives TRANSIENT_LOCAL equivalent
    diag_status_sub_ = nh.subscribe<BoolMsg>(
        topic_prefix_ + "diag_status", 1,
        [this](const BoolMsg::ConstPtr& msg) { OnDiagStatus(msg->data); });
#endif
    printf("[diag_subscriber] Listening to %sdiag_status\n", topic_prefix_.c_str());
  }

  // -- Helper: record a message arrival --
  void RecordMsg(ChannelStats& stats, uint64_t stamp_ns) {
    if (!diag_enabled_.load(std::memory_order_relaxed)) return;
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
    int64_t latency = static_cast<int64_t>(now_ns) - static_cast<int64_t>(stamp_ns);
    stats.count.fetch_add(1, std::memory_order_relaxed);
    stats.last_latency_ns.store(latency, std::memory_order_relaxed);

    // Update sliding-window latency stats (last N frames)
    constexpr size_t kLatencyWindowSize = 50;
    std::lock_guard<std::mutex> lock(diag_mutex_);
    if (stats.latency_window.size() < kLatencyWindowSize) {
      if (stats.latency_window.empty()) {
        stats.latency_window_min = stats.latency_window_max = latency;
      } else {
        stats.latency_window_min = std::min(stats.latency_window_min, latency);
        stats.latency_window_max = std::max(stats.latency_window_max, latency);
      }
      stats.latency_window.push_back(latency);
      stats.latency_window_sum += latency;
    } else {
      int64_t old_latency = stats.latency_window[stats.latency_window_index];
      stats.latency_window[stats.latency_window_index] = latency;
      stats.latency_window_sum += latency - old_latency;
      stats.latency_window_index = (stats.latency_window_index + 1) % kLatencyWindowSize;

      if (old_latency == stats.latency_window_min || old_latency == stats.latency_window_max) {
        auto minmax =
            std::minmax_element(stats.latency_window.begin(), stats.latency_window.end());
        stats.latency_window_min = *minmax.first;
        stats.latency_window_max = *minmax.second;
      } else {
        stats.latency_window_min = std::min(stats.latency_window_min, latency);
        stats.latency_window_max = std::max(stats.latency_window_max, latency);
      }
    }
  }

  // -- Helper: extract nanosecond timestamp from header.stamp --
#if defined(ODIN_ROS2)
  static uint64_t StampToNs(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
  }
#else
  static uint64_t StampToNs(const ros::Time& stamp) {
    return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + stamp.nsec;
  }
#endif

  void InitSubscribers() {
#if defined(ODIN_ROS2)
    // RELIABLE QoS to match driver publishers
    auto qos_low = rclcpp::QoS(rclcpp::KeepLast(5))
                       .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                       .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    auto qos_high = rclcpp::QoS(rclcpp::KeepLast(100))
                        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    subs_.push_back(create_subscription<PointCloud2Msg>(
        topic_prefix_ + "cloud/raw", qos_low,
        [this](const PointCloud2Msg::SharedPtr msg) {
          RecordMsg(raw_point_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(create_subscription<PointCloud2Msg>(
        topic_prefix_ + "cloud/slam", qos_low,
        [this](const PointCloud2Msg::SharedPtr msg) {
          RecordMsg(slam_point_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(create_subscription<CompressedImageMsg>(
        topic_prefix_ + "camera0/compressed", qos_low,
        [this](const CompressedImageMsg::SharedPtr msg) {
          RecordMsg(image0_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(create_subscription<ImuMsg>(
        topic_prefix_ + "imu", qos_high,
        [this](const ImuMsg::SharedPtr msg) {
          RecordMsg(imu_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(create_subscription<OdometryMsg>(
        topic_prefix_ + "odometry_lf", qos_high,
        [this](const OdometryMsg::SharedPtr msg) {
          RecordMsg(odom_lf_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(create_subscription<OdometryMsg>(
        topic_prefix_ + "odometry_hf", qos_high,
        [this](const OdometryMsg::SharedPtr msg) {
          RecordMsg(odom_hf_, StampToNs(msg->header.stamp));
        }));

    diag_timer_ = create_wall_timer(std::chrono::seconds(1),
                                   [this]() { TimerCallback(); });
    printf("[diag_subscriber] Subscribed to topics under: %s\n", topic_prefix_.c_str());
#else
    ros::NodeHandle nh;
    int queue = 1000;
    subs_.push_back(nh.subscribe<PointCloud2Msg>(topic_prefix_ + "cloud/raw", queue,
        [this](const PointCloud2Msg::ConstPtr& msg) {
          RecordMsg(raw_point_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(nh.subscribe<PointCloud2Msg>(topic_prefix_ + "cloud/slam", queue,
        [this](const PointCloud2Msg::ConstPtr& msg) {
          RecordMsg(slam_point_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(nh.subscribe<CompressedImageMsg>(topic_prefix_ + "camera0/compressed", queue,
        [this](const CompressedImageMsg::ConstPtr& msg) {
          RecordMsg(image0_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(nh.subscribe<ImuMsg>(topic_prefix_ + "imu", queue,
        [this](const ImuMsg::ConstPtr& msg) {
          RecordMsg(imu_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(nh.subscribe<OdometryMsg>(topic_prefix_ + "odometry_lf", queue,
        [this](const OdometryMsg::ConstPtr& msg) {
          RecordMsg(odom_lf_, StampToNs(msg->header.stamp));
        }));
    subs_.push_back(nh.subscribe<OdometryMsg>(topic_prefix_ + "odometry_hf", queue,
        [this](const OdometryMsg::ConstPtr& msg) {
          RecordMsg(odom_hf_, StampToNs(msg->header.stamp));
        }));

    diag_timer_ = nh.createTimer(ros::Duration(1.0),
        [this](const ros::TimerEvent&) { TimerCallback(); });
    printf("[diag_subscriber] Subscribed to topics under: %s\n", topic_prefix_.c_str());
#endif
  }

  void TimerCallback() {
    if (!diag_enabled_.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> lock(diag_mutex_);

    // Compute elapsed time since last print (supports any timer interval)
    double elapsed_sec = 0.0;
    auto now = std::chrono::steady_clock::now();
    if (prev_print_time_.time_since_epoch().count() == 0) {
      prev_print_time_ = now;
    } else {
      elapsed_sec = std::chrono::duration<double>(now - prev_print_time_).count();
      prev_print_time_ = now;
    }

    auto fps = [this, elapsed_sec](ChannelStats& s) -> double {
      uint64_t cur = s.count.load();
      double f = (elapsed_sec > 0.0)
                     ? static_cast<double>(cur - s.prev_count) / elapsed_sec
                     : 0.0;
      s.prev_count = cur;
      return f;
    };
    auto lat_avg = [](ChannelStats& s) -> int64_t {
      if (s.latency_window.empty()) return 0;
      return s.latency_window_sum / static_cast<int64_t>(s.latency_window.size());
    };
    auto lat_min = [](ChannelStats& s) -> int64_t {
      if (s.latency_window.empty()) return 0;
      return s.latency_window_min;
    };
    auto lat_max = [](ChannelStats& s) -> int64_t {
      if (s.latency_window.empty()) return 0;
      return s.latency_window_max;
    };
    printf("\n");
    printf("[DiagSubscriber] FPS / Latency(ns) — subscriber-side (after ROS transport)\n");
    printf("  Channel      FPS       Avg(ns)       Min(ns)       Max(ns)\n");
    printf("  ---------  -------  ------------  ------------  ------------\n");
    printf("  RawPoint    %5.1f   %13ld   %13ld   %13ld\n", fps(raw_point_),
           (long)lat_avg(raw_point_), (long)lat_min(raw_point_), (long)lat_max(raw_point_));
    printf("  SlamPoint   %5.1f   %13ld   %13ld   %13ld\n", fps(slam_point_),
           (long)lat_avg(slam_point_), (long)lat_min(slam_point_), (long)lat_max(slam_point_));
    printf("  Image0      %5.1f   %13ld   %13ld   %13ld\n", fps(image0_),
           (long)lat_avg(image0_), (long)lat_min(image0_), (long)lat_max(image0_));
    printf("  IMU         %5.1f   %13ld   %13ld   %13ld\n", fps(imu_),
           (long)lat_avg(imu_), (long)lat_min(imu_), (long)lat_max(imu_));
    printf("  OdomLF      %5.1f   %13ld   %13ld   %13ld\n", fps(odom_lf_),
           (long)lat_avg(odom_lf_), (long)lat_min(odom_lf_), (long)lat_max(odom_lf_));
    printf("  OdomHF      %5.1f   %13ld   %13ld   %13ld\n", fps(odom_hf_),
           (long)lat_avg(odom_hf_), (long)lat_min(odom_hf_), (long)lat_max(odom_hf_));
  }
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
#if defined(ODIN_ROS2)
int main(int argc, char** argv) {
  odin::print_app_version("diag_subscriber_node");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiagSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
#else
int main(int argc, char** argv) {
  odin::print_app_version("diag_subscriber_node");
  ros::init(argc, argv, "diag_subscriber");
  DiagSubscriberNode node;
  ros::spin();
  return 0;
}
#endif
