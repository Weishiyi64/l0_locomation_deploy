#include "odin_app_version.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <map>
#include <set>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <functional>
#include <mutex>

#include <yaml-cpp/yaml.h>

// ROS message headers must be included before the SDK logger.h, because
// logger.h defines WARN/ERROR/INFO/DEBUG as string macros that conflict
// with diagnostic_msgs enum values of the same names.
#if defined(ODIN_ROS2)
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#else
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <point_cloud_transport/point_cloud_transport.h>
#endif

#include "odin_lidar_api_ext.h"
#include "logger.h"

#include "service_types.h"
#if defined(ODIN_ROS2)
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace odin_ros_driver {

namespace sdk = odin::sdk;

// Helper to get package share directory (for loading installed config files)
static std::string getPackageSourceDirectory() {
#if defined(ODIN_ROS2)
  try {
    return ament_index_cpp::get_package_share_directory(ODIN_ROS_PACKAGE_NAME);
  } catch (const std::exception& e) {
    return "";
  }
#else
  // ROS1: use ros::package::getPath
  return ros::package::getPath(ODIN_ROS_PACKAGE_NAME);
#endif
}

/**
 * @brief Thread-safe async queue for non-blocking callback processing
 */
template <typename T>
class AsyncQueue {
 public:
  using ProcessFunc = std::function<void(const T&)>;

  AsyncQueue(const std::string& name, size_t max_size = 100)
      : name_(name), max_size_(max_size), running_(false), drop_count_(0) {}

  ~AsyncQueue() { Stop(); }

  void Start(ProcessFunc process_func) {
    if (running_) return;
    process_func_ = process_func;
    running_ = true;
    worker_ = std::thread(&AsyncQueue::WorkerLoop, this);
  }

  void Stop() {
    if (!running_) return;
    running_ = false;
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  bool Push(T&& item) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= max_size_) {
      ++drop_count_;
      if (drop_count_ % 100 == 1) {
        LOG_WARN("AsyncQueue[%s]: queue full, dropped %lu items\n", name_.c_str(), drop_count_.load());
      }
      return false;
    }
    queue_.push(std::move(item));
    cv_.notify_one();
    return true;
  }

  size_t Size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  uint64_t DropCount() const { return drop_count_; }

 private:
  void WorkerLoop() {
    while (running_) {
      T item;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || !running_; });
        if (!running_ && queue_.empty()) break;
        if (queue_.empty()) continue;
        item = std::move(queue_.front());
        queue_.pop();
      }
      if (process_func_) {
        process_func_(item);
      }
    }
  }

  std::string name_;
  size_t max_size_;
  std::atomic<bool> running_;
  std::atomic<uint64_t> drop_count_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::queue<T> queue_;
  std::thread worker_;
  ProcessFunc process_func_;
};

// Locate the package root directory via __FILE__
static std::string GetPackageRoot() {
  std::string path(__FILE__);  // .../src/odin_ros_driver_node.cpp
  for (int i = 0; i < 10; ++i) {
    size_t pos = path.rfind('/');
    if (pos == std::string::npos) break;
    path = path.substr(0, pos);
    struct stat st;
    std::string pkg_xml = path + "/package.xml";
    if (stat(pkg_xml.c_str(), &st) == 0) {
      return path;
    }
  }
  return "";
}

#if defined(ODIN_ROS2)
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PointFieldMsg = sensor_msgs::msg::PointField;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using ImuMsg = sensor_msgs::msg::Imu;
using OdometryMsg = nav_msgs::msg::Odometry;
using PathMsg = nav_msgs::msg::Path;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using QuaternionMsg = geometry_msgs::msg::Quaternion;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
using BoolMsg = std_msgs::msg::Bool;
using MarkerMsg = visualization_msgs::msg::Marker;
using RosTime = rclcpp::Time;
using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
using KeyValueMsg = diagnostic_msgs::msg::KeyValue;

template <typename MsgT>
using PublisherType = typename rclcpp::Publisher<MsgT>::SharedPtr;
#else
using PointCloud2Msg = sensor_msgs::PointCloud2;
using PointFieldMsg = sensor_msgs::PointField;
using CompressedImageMsg = sensor_msgs::CompressedImage;
using ImuMsg = sensor_msgs::Imu;
using OdometryMsg = nav_msgs::Odometry;
using PathMsg = nav_msgs::Path;
using PoseStampedMsg = geometry_msgs::PoseStamped;
using QuaternionMsg = geometry_msgs::Quaternion;
using TransformStampedMsg = geometry_msgs::TransformStamped;
using BoolMsg = std_msgs::Bool;
using MarkerMsg = visualization_msgs::Marker;
using RosTime = ros::Time;
using DiagnosticArrayMsg = diagnostic_msgs::DiagnosticArray;
using DiagnosticStatusMsg = diagnostic_msgs::DiagnosticStatus;
using KeyValueMsg = diagnostic_msgs::KeyValue;

template <typename MsgT>
using PublisherType = ros::Publisher;
#endif

#define PCLPROCESSTYPE uint16_t

// Sanitize a single topic name component so it is a valid ROS topic token.
// ROS topic names may only contain alphanumeric characters and underscores;
// any other character (e.g. spaces in a device model like "Manifold Tech
// Limited Odin1") is replaced with '_' to prevent runtime crashes.
inline std::string SanitizeTopicComponent(const std::string &input) {
  std::string output = input;
  for (char &c : output) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
      c = '_';
    }
  }
  return output;
}

class odinRosDriver {
 public:
#if defined(ODIN_ROS2)
  // Constructor with pre-discovered device and device index
  odinRosDriver(const rclcpp::Node::SharedPtr &node, const sdk::DiscoveredDevice &device,
                int device_index = -1)
      : node_(node),
        discovered_device_(device),
        device_sn_(device.sn),
        device_model_(device.model),
        device_index_(device_index) {}
#else
  // Constructor with pre-discovered device and device index
  // main_pnh: main node's private NodeHandle for reading global parameters (image_width, etc.)
  // device_pnh: device-specific NodeHandle for dynamic reconfigure
  odinRosDriver(const ros::NodeHandle &nh, const ros::NodeHandle &main_pnh,
                const ros::NodeHandle &device_pnh,
                const sdk::DiscoveredDevice &device, int device_index = -1)
      : nh_(nh),
        pnh_(device_pnh),
        main_pnh_(main_pnh),
        discovered_device_(device),
        device_sn_(device.sn),
        device_model_(device.model),
        device_index_(device_index) {}
#endif

  ~odinRosDriver() {
    Shutdown();
    // Explicitly destroy ROS members in a safe order to prevent
    // use-after-free during implicit member destruction. The key issue:
    // param_callback_handle_ and service objects hold lambdas capturing
    // `this`; if they outlive node_ during implicit destruction (reverse
    // declaration order), the node may be gone when they try to unregister.
    // By resetting them explicitly here while node_ is still alive, we
    // guarantee clean unregistration.
#if defined(ODIN_ROS2)
    param_callback_handle_.reset();
    diag_service_.reset();
    diag_timer_.reset();
    diag_status_pub_.reset();
    set_channel_config_service_.reset();
    get_channel_config_service_.reset();
    calib_service_.reset();
    sensor_mode_service_.reset();
    reset_pose_service_.reset();
    get_adaptive_param_service_.reset();
    set_adaptive_param_service_.reset();
    get_sync_mode_service_.reset();
    set_sync_mode_service_.reset();
    get_ptp_sync_status_service_.reset();
    restore_default_config_service_.reset();
    tf_republish_timer_.reset();
    static_tf_broadcaster_.reset();
    tf_broadcaster_.reset();
    device_online_pub_.reset();
    mapping_active_pub_.reset();
    resolution_change_pub_.reset();
    node_.reset();
#else
    diag_service_.shutdown();
    diag_timer_.stop();
    diag_status_pub_.shutdown();
    set_channel_config_service_.shutdown();
    get_channel_config_service_.shutdown();
    calib_service_.shutdown();
    sensor_mode_service_.shutdown();
    reset_pose_service_.shutdown();
    get_adaptive_param_service_.shutdown();
    set_adaptive_param_service_.shutdown();
    get_sync_mode_service_.shutdown();
    set_sync_mode_service_.shutdown();
    get_ptp_sync_status_service_.shutdown();
    restore_default_config_service_.shutdown();
    tf_republish_timer_.stop();
    static_tf_broadcaster_.reset();
    tf_broadcaster_.reset();
    device_online_pub_.shutdown();
    mapping_active_pub_.shutdown();
    resolution_change_pub_.shutdown();
#endif
  }

  // Called by HotplugManager when the device is physically removed (unplugged).
  // This allows Shutdown() to skip sending standby mode commands to a device
  // that is already gone, avoiding unnecessary timeouts.
  void MarkDeviceRemoved() { device_removed_.store(true); }

  bool Init() {
    if (!LoadParameters()) {
      return false;
    }
    BuildCalibFilePath();
    BuildTopicPrefix();
    if (!ConnectDevice()) {
      return false;
    }
    // Advertise topics and start queues AFTER successful connection so that
    // a failed connection does not leave ROS objects (timers, publishers,
    // services) on the node — which can crash ~CallbackGroup() during node
    // destruction when the device is unreachable (e.g. wrong subnet).
    AdvertiseTopics();
    InitTfBroadcaster();
    InitAsyncQueues();
    // Set up dynamic reconfigure AFTER connecting so UI sliders (e.g.
    // sensor_mode) initialize from the device's actual state that
    // ConnectDevice() queried, instead of the launch default value.
    SetupDynamicReconfigure();
    SyncImageParamsToROS();
    RegisterCallbacks();
    InitDiagMonitor();
    // Publish device_online AFTER calibration is ready, so post_process_node can fetch it
    PublishTopicPrefix();
    return true;
  }

  void InitTfBroadcaster() {
#if defined(ODIN_ROS2)
    if (node_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
      static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    }
#else
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
#endif
    LogInfo("TF broadcaster initialized");

    // Create TF republish timer if tf_publish_rate > 0
    if (tf_publish_rate_ > 0.0) {
      const double period_sec = 1.0 / tf_publish_rate_;
#if defined(ODIN_ROS2)
      if (node_) {
        tf_republish_timer_ = node_->create_wall_timer(
            std::chrono::duration<double>(period_sec),
            [this]() { TfRepublishCallback(); });
      }
#else
      tf_republish_timer_ = nh_.createTimer(
          ros::Duration(period_sec),
          [this](const ros::TimerEvent&) { TfRepublishCallback(); });
#endif
      LogInfo("TF republish timer started at %.1f Hz", tf_publish_rate_);
    }
  }

  void TfRepublishCallback() {
    if (shutting_down_.load()) return;
    std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
    if (shutting_down_.load()) return;
    auto now_steady = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(tf_cache_mutex_);
    if (cached_transforms_.empty()) return;

    std::vector<TransformStampedMsg> transforms;
    transforms.reserve(cached_transforms_.size());
    for (auto &cached : cached_transforms_) {
      double elapsed = std::chrono::duration<double>(now_steady - cached.cache_time).count();
      auto tf_copy = cached.msg;
#if defined(ODIN_ROS2)
      double orig_sec = rclcpp::Time(tf_copy.header.stamp).seconds();
#else
      double orig_sec = tf_copy.header.stamp.toSec();
#endif
      double new_sec = orig_sec + elapsed;

      // Guarantee strictly monotonic timestamps to avoid TF_OLD_DATA
      if (new_sec <= cached.last_published_sec) {
        new_sec = cached.last_published_sec + 0.001;
      }
      cached.last_published_sec = new_sec;

#if defined(ODIN_ROS2)
      int64_t new_ns = static_cast<int64_t>(new_sec * 1e9);
      tf_copy.header.stamp = rclcpp::Time(new_ns);
#else
      tf_copy.header.stamp = ros::Time(new_sec);
#endif
      transforms.push_back(tf_copy);
    }
    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(transforms);
    }
  }

  // Build TF message from an OdometryMsg's pose and publish it.
  void BroadcastTfFromOdom(const OdometryMsg &msg) {
    if (!tf_broadcaster_) return;
    TransformStampedMsg tf_msg;
    tf_msg.header.stamp = msg.header.stamp;
    tf_msg.header.frame_id = msg.header.frame_id;
    tf_msg.child_frame_id = msg.child_frame_id;
    tf_msg.transform.translation.x = msg.pose.pose.position.x;
    tf_msg.transform.translation.y = msg.pose.pose.position.y;
    tf_msg.transform.translation.z = msg.pose.pose.position.z;
    tf_msg.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void InitAsyncQueues() {
    // Create async queues with appropriate sizes
    raw_point_queue_ = std::make_unique<AsyncQueue<sdk::OdinPointCloudPacket>>("RawPoint", 300);
    slam_point_queue_ = std::make_unique<AsyncQueue<sdk::OdinSlamPacket>>("SlamPoint", 300);
    image0_queue_ = std::make_unique<AsyncQueue<sdk::OdinCameraPacket>>("Image0", 300);
    imu_queue_ = std::make_unique<AsyncQueue<sdk::OdinImuPacket>>("IMU", 4000);
    odom_lf_queue_ = std::make_unique<AsyncQueue<OdomQueueItem>>("OdomLF", 400);
    odom_hf_queue_ = std::make_unique<AsyncQueue<OdomQueueItem>>("OdomHF", 4000);
    tf_queue_ = std::make_unique<AsyncQueue<sdk::OdinTfPacket>>("Tf", 4000);
    lc_optimization_status_queue_ =
        std::make_unique<AsyncQueue<sdk::OdinLcOptimizationStatusPacket>>("LcOptStatus", 20);
    lc_frame_queue_ = std::make_unique<AsyncQueue<sdk::OdinLcFramePacket>>("LcFrame", 300);
    lc_optimization_result_queue_ =
        std::make_unique<AsyncQueue<sdk::OdinLcOptimizationResultPacket>>("LcOptResult", 5);
    lc_map_cloud_queue_ = std::make_unique<AsyncQueue<sdk::OdinLcMapCloudPacket>>("LcMapCloud", 5);

    // Start worker threads with processing functions
    raw_point_queue_->Start([this](const sdk::OdinPointCloudPacket& pkt) {
      ProcessRawPointCloud(pkt);
    });
    slam_point_queue_->Start([this](const sdk::OdinSlamPacket& pkt) {
      ProcessSlamPointCloud(pkt);
    });
    image0_queue_->Start([this](const sdk::OdinCameraPacket& pkt) {
      ProcessImage(pkt);
    });
    imu_queue_->Start([this](const sdk::OdinImuPacket& pkt) {
      ProcessImu(pkt);
    });
    odom_lf_queue_->Start([this](const OdomQueueItem& item) {
      ProcessOdom(item.packet, item.odom_type);
    });
    odom_hf_queue_->Start([this](const OdomQueueItem& item) {
      ProcessOdom(item.packet, item.odom_type);
    });
    tf_queue_->Start([this](const sdk::OdinTfPacket& pkt) {
      ProcessTf(pkt);
    });
    lc_optimization_status_queue_->Start([this](const sdk::OdinLcOptimizationStatusPacket& pkt) {
      ProcessLcOptimizationStatus(pkt);
    });
    lc_frame_queue_->Start([this](const sdk::OdinLcFramePacket& pkt) {
      ProcessLcFrame(pkt);
    });
    lc_optimization_result_queue_->Start([this](const sdk::OdinLcOptimizationResultPacket& pkt) {
      ProcessLcOptimizationResult(pkt);
    });
    lc_map_cloud_queue_->Start([this](const sdk::OdinLcMapCloudPacket& pkt) {
      ProcessLcMapCloud(pkt);
    });
    LogInfo("Async queues initialized");
  }

  // Public setters for HotplugManager to forward parameter changes
  void SetConfidenceThreshold(int value) {
    m_confidence_threshold = value;
    // Clamp to current mode's minimum (Adaptive disables client-side filter).
    ApplyModeConfidence();
    LogInfo("confidence_threshold set to: %d (mode=%s)", m_confidence_threshold,
            SensorModeName(sensor_mode_));
  }

  static const char *SensorModeName(sdk::OdinSensorMode mode) {
    switch (mode) {
      case sdk::OdinSensorMode::kHDR:       return "HDR";
      case sdk::OdinSensorMode::kHighPeak:  return "High Peak";
      case sdk::OdinSensorMode::kLowPeak:   return "Low Peak";
      case sdk::OdinSensorMode::kAdaptive:  return "Adaptive";
      default:                               return "Unknown";
    }
  }

  static sdk::OdinSensorMode IntToSensorMode(int val) {
    if (val >= static_cast<int>(sdk::OdinSensorMode::kHDR) &&
        val <= static_cast<int>(sdk::OdinSensorMode::kAdaptive)) {
      return static_cast<sdk::OdinSensorMode>(val);
    }
    return sdk::OdinSensorMode::kHighPeak;  // fallback default
  }

  // Per-mode minimum client-side confidence threshold.
  // HighPeak: 7 (no lower), LowPeak: 2 (no lower), Adaptive: 0 (disabled —
  // device-side filtering handles it), HDR: 0 (no floor).
  static int GetModeConfidenceMin(sdk::OdinSensorMode mode) {
    switch (mode) {
      case sdk::OdinSensorMode::kHighPeak:  return 7;
      case sdk::OdinSensorMode::kLowPeak:   return 2;
      case sdk::OdinSensorMode::kAdaptive:  return 0;  // client-side disabled
      case sdk::OdinSensorMode::kHDR:       return 0;
      default:                               return 0;
    }
  }

  // Per-mode default client-side confidence threshold.
  static int GetModeConfidenceDefault(sdk::OdinSensorMode mode) {
    switch (mode) {
      case sdk::OdinSensorMode::kHighPeak:  return 7;
      case sdk::OdinSensorMode::kLowPeak:   return 2;
      case sdk::OdinSensorMode::kAdaptive:  return 0;  // client-side disabled
      case sdk::OdinSensorMode::kHDR:       return 7;
      default:                               return 7;
    }
  }

  // Clamp m_confidence_threshold to the current sensor mode's minimum.
  // For Adaptive mode the client-side filter is disabled (threshold forced to 0).
  void ApplyModeConfidence() {
    int mode_min = GetModeConfidenceMin(sensor_mode_);
    if (sensor_mode_ == sdk::OdinSensorMode::kAdaptive) {
      // Adaptive mode: device handles filtering; force client-side off.
      if (m_confidence_threshold != 0) {
        LogInfo("Adaptive mode: client-side confidence filter disabled (was %d)",
                m_confidence_threshold);
        m_confidence_threshold = 0;
      }
    } else if (m_confidence_threshold < mode_min) {
      LogInfo("confidence_threshold %d below %s mode minimum %d, clamping",
              m_confidence_threshold, SensorModeName(sensor_mode_), mode_min);
      m_confidence_threshold = mode_min;
    }
  }

  void SetSensorMode(int mode) {
    sensor_mode_ = IntToSensorMode(mode);
    LogInfo("sensor_mode set to: %s (%d)", SensorModeName(sensor_mode_),
            static_cast<int>(sensor_mode_));
    ApplyModeConfidence();
    if (connected_ && device_handle_ != sdk::kInvalidDeviceHandle) {
      sdk::SetSensorMode(device_handle_, sensor_mode_);
    }
  }

 private:
  static odinRosDriver *FromClientData(void *data) { return static_cast<odinRosDriver *>(data); }

  // SDK callbacks - push to async queues immediately (non-blocking)
  static void PointCloudCallback(const sdk::OdinPointCloudPacket &packet, void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleRawPointCloud(packet);
    }
  }
  static void SlamCallback(const sdk::OdinSlamPacket &packet, void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleSlamPointCloud(packet);
    }
  }

  static void CameraCallback(const sdk::OdinCameraPacket &packet, void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleImage(packet);
    }
  }

  static void ImuCallback(const sdk::OdinImuPacket &packet, void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleImu(packet);
    }
  }

  static void OdomCallback(const sdk::OdinOdomPacket &packet, sdk::OdomSourceType odom_type,
                           void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleOdom(packet, odom_type);
    }
  }

  static void TfCallback(const sdk::OdinTfPacket &packet, void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleTf(packet);
    }
  }

  static void LcOptimizationStatusCallback(const sdk::OdinLcOptimizationStatusPacket &packet,
                                           void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleLcOptimizationStatus(packet);
    }
  }

  static void LcFrameCallback(const sdk::OdinLcFramePacket &packet, void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleLcFrame(packet);
    }
  }

  static void LcOptimizationResultCallback(const sdk::OdinLcOptimizationResultPacket &packet,
                                           void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleLcOptimizationResult(packet);
    }
  }

  static void LcMapCloudCallback(const sdk::OdinLcMapCloudPacket &packet, void *client) {
    if (auto *driver = FromClientData(client)) {
      driver->HandleLcMapCloud(packet);
    }
  }

  // Diagnostic: record (host_time - device_timestamp) per channel
  void RecordDiag(const char* channel, uint64_t device_ts_ns) {
    if (!diag_enabled_.load()) return;
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    int64_t latency_ns = static_cast<int64_t>(now_ns) - static_cast<int64_t>(device_ts_ns);
    std::lock_guard<std::mutex> lock(diag_mutex_);
    auto& ch = diag_stats_[channel];
    ch.frame_count++;
    ch.last_latency_ns = latency_ns;

    // Update sliding-window latency stats (last N frames)
    constexpr size_t kLatencyWindowSize = 50;
    if (ch.latency_window.size() < kLatencyWindowSize) {
      if (ch.latency_window.empty()) {
        ch.latency_window_min = ch.latency_window_max = latency_ns;
      } else {
        ch.latency_window_min = std::min(ch.latency_window_min, latency_ns);
        ch.latency_window_max = std::max(ch.latency_window_max, latency_ns);
      }
      ch.latency_window.push_back(latency_ns);
      ch.latency_window_sum += latency_ns;
    } else {
      int64_t old_latency = ch.latency_window[ch.latency_window_index];
      ch.latency_window[ch.latency_window_index] = latency_ns;
      ch.latency_window_sum += latency_ns - old_latency;
      ch.latency_window_index = (ch.latency_window_index + 1) % kLatencyWindowSize;

      if (old_latency == ch.latency_window_min || old_latency == ch.latency_window_max) {
        auto minmax =
            std::minmax_element(ch.latency_window.begin(), ch.latency_window.end());
        ch.latency_window_min = *minmax.first;
        ch.latency_window_max = *minmax.second;
      } else {
        ch.latency_window_min = std::min(ch.latency_window_min, latency_ns);
        ch.latency_window_max = std::max(ch.latency_window_max, latency_ns);
      }
    }
  }

  // Handle functions - push to async queue (fast, non-blocking)
  void HandleRawPointCloud(const sdk::OdinPointCloudPacket &packet) {
    RecordDiag("RawPoint", packet.timestamp);
    if (raw_point_queue_) {
      raw_point_queue_->Push(sdk::OdinPointCloudPacket(packet));
    }
  }

  void HandleSlamPointCloud(const sdk::OdinSlamPacket &packet) {
    RecordDiag("SlamPoint", packet.timestamp);
    if (slam_point_queue_) {
      slam_point_queue_->Push(sdk::OdinSlamPacket(packet));
    }
  }

  void HandleImage(const sdk::OdinCameraPacket &packet) {
    RecordDiag("Image0", packet.timestamp);
    if (image0_queue_) {
      image0_queue_->Push(sdk::OdinCameraPacket(packet));
    }
  }

  void HandleImu(const sdk::OdinImuPacket &packet) {
    RecordDiag("IMU", packet.timestamp);
    if (imu_queue_) {
      imu_queue_->Push(sdk::OdinImuPacket(packet));
    }
  }

  void HandleOdom(const sdk::OdinOdomPacket &packet, sdk::OdomSourceType odom_type) {
    const char* ch = (odom_type == sdk::OdomSourceType::kOdom2ImuHigh) ? "OdomHF" : "OdomLF";
    RecordDiag(ch, packet.timestamp);
    if (odom_type == sdk::OdomSourceType::kOdom2ImuHigh) {
      if (odom_hf_queue_) odom_hf_queue_->Push(OdomQueueItem{packet, odom_type});
    } else {
      if (odom_lf_queue_) odom_lf_queue_->Push(OdomQueueItem{packet, odom_type});
    }
  }

  void HandleTf(const sdk::OdinTfPacket &packet) {
    RecordDiag("Tf", packet.timestamp);
    if (tf_queue_) {
      tf_queue_->Push(sdk::OdinTfPacket(packet));
    }
  }

  void HandleLcOptimizationStatus(const sdk::OdinLcOptimizationStatusPacket &packet) {
    if (lc_optimization_status_queue_) {
      lc_optimization_status_queue_->Push(sdk::OdinLcOptimizationStatusPacket(packet));
    }
  }

  void HandleLcFrame(const sdk::OdinLcFramePacket &packet) {
    if (lc_frame_queue_) {
      lc_frame_queue_->Push(sdk::OdinLcFramePacket(packet));
    }
  }

  void HandleLcOptimizationResult(const sdk::OdinLcOptimizationResultPacket &packet) {
    if (lc_optimization_result_queue_) {
      lc_optimization_result_queue_->Push(sdk::OdinLcOptimizationResultPacket(packet));
    }
  }

  void HandleLcMapCloud(const sdk::OdinLcMapCloudPacket &packet) {
    if (lc_map_cloud_queue_) {
      lc_map_cloud_queue_->Push(sdk::OdinLcMapCloudPacket(packet));
    }
  }

  // Helper to build driver topic name: "/{prefix}/driver/{suffix}" or "/driver/{suffix}" if prefix is empty
  // Unified with device topic format: /{prefix}/device{N}/...
  std::string BuildDriverTopicName(const std::string &suffix) const {
    if (topic_prefix_base_.empty()) {
      return "/driver/" + suffix;
    }
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  template <typename T>
  T GetParam(const std::string &name, const T &default_value) {
#if defined(ODIN_ROS2)
    if (!node_) {
      return default_value;
    }
    if (!node_->has_parameter(name)) {
      // Launch-time config params (frame ids, image dimensions, device mode,
      // etc.) are not runtime-tunable. Declare them read_only so rqt_reconfigure
      // shows them greyed-out, visually separating them from the tunable
      // sliders declared in SetupDynamicReconfigure().
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.read_only = true;
      node_->declare_parameter<T>(name, default_value, desc);
    }
    T value = default_value;
    node_->get_parameter(name, value);
    return value;
#else
    // Use main_pnh_ to read global parameters set in launch file
    return main_pnh_.param(name, default_value);
#endif
  }

  bool LoadChannelEnableConfig() {
#if defined(ODIN_ROS2)
    // Read from parameter overrides (launch/CLI) without declaring, since
    // SetupDynamicReconfigure() will declare these as dynamic parameters.
    const auto &overrides = node_->get_node_parameters_interface()->get_parameter_overrides();
    auto get_bool_override = [&overrides](const char *name, bool default_val) -> bool {
      auto it = overrides.find(name);
      if (it != overrides.end() &&
          it->second.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        return it->second.get<bool>();
      }
      return default_val;
    };
    enable_raw_point_  = get_bool_override("enable_raw_point",  true);
    enable_slam_point_ = get_bool_override("enable_slam_point", true);
    enable_image0_     = get_bool_override("enable_image0",     true);
    const bool enable_image1 = get_bool_override("enable_image1", false);
    enable_imu_        = get_bool_override("enable_imu",        true);
    enable_odom_lf_    = get_bool_override("enable_odom_lf",    true);
    enable_odom_hf_    = get_bool_override("enable_odom_hf",    true);
    enable_tf_         = get_bool_override("enable_tf",         true);
#else
    enable_raw_point_  = GetParam<bool>("enable_raw_point",  true);
    enable_slam_point_ = GetParam<bool>("enable_slam_point", true);
    enable_image0_     = GetParam<bool>("enable_image0",     true);
    const bool enable_image1 = GetParam<bool>("enable_image1", false);
    enable_imu_        = GetParam<bool>("enable_imu",        true);
    enable_odom_lf_    = GetParam<bool>("enable_odom_lf",    true);
    enable_odom_hf_    = GetParam<bool>("enable_odom_hf",    true);
    enable_tf_         = GetParam<bool>("enable_tf",         true);
#endif
    if (enable_image1) {
      LogError("This hardware has only camera0; enable_image1 must be false.");
      return false;
    }
    LogInfo("Channel config: raw=%d slam=%d img0=%d imu=%d odom_lf=%d odom_hf=%d tf=%d",
            enable_raw_point_, enable_slam_point_, enable_image0_,
            enable_imu_, enable_odom_lf_, enable_odom_hf_, enable_tf_);
    return true;
  }

  void AdvertiseTopics() {
#if defined(ODIN_ROS2)
    if (!node_) {
      return;
    }
    // QoS for low-frequency data (point clouds, images): small depth for real-time display
    auto qos_low_freq = rclcpp::QoS(5)
                            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    // QoS for high-frequency data (IMU, odom): larger depth for rosbag recording
    auto qos_high_freq = rclcpp::QoS(100)
                             .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                             .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    // Point cloud publishers use point_cloud_transport for plugin-based
    // compression support (zlib, zstd, draco, etc.).
    // NOTE: the simple advertise(topic, queue_size) overload defaults to
    // rmw_qos_profile_sensor_data (BEST_EFFORT), which differs from the rest
    // of the driver. Reuse the same qos_low_freq profile as the other
    // low-frequency publishers so behavior stays consistent.
    point_cloud_transport::PointCloudTransport pct(node_);
    rmw_qos_profile_t cloud_qos = qos_low_freq.get_rmw_qos_profile();
    raw_cloud_pub_ = pct.advertise(topic_prefix_ + "cloud/raw", cloud_qos);
    slam_cloud_pub_ = pct.advertise(topic_prefix_ + "cloud/slam", cloud_qos);
    image_pub_ = node_->create_publisher<CompressedImageMsg>(topic_prefix_ + "camera0/compressed",
                                                             qos_low_freq);
    imu_pub_ = node_->create_publisher<ImuMsg>(topic_prefix_ + "imu", qos_high_freq);
    odom_pub_ = node_->create_publisher<OdometryMsg>(topic_prefix_ + "odometry_lf", qos_high_freq);
    odom_hf_pub_ = node_->create_publisher<OdometryMsg>(topic_prefix_ + "odometry_hf", qos_high_freq);
    const auto qos_loop_result = rclcpp::QoS(1).reliable().transient_local();
    lc_optimized_cloud_pub_ = node_->create_publisher<PointCloud2Msg>(
        topic_prefix_ + "loop_closure/optimized_cloud", qos_loop_result);
    lc_optimized_path_pub_ = node_->create_publisher<PathMsg>(
        topic_prefix_ + "loop_closure/optimized_path", qos_loop_result);
    lc_map_cloud_pub_ = node_->create_publisher<PointCloud2Msg>(
        topic_prefix_ + "loop_closure/map_cloud", qos_loop_result);
    lc_corrected_pose_pub_ = node_->create_publisher<PoseStampedMsg>(
        topic_prefix_ + "loop_closure/corrected_pose", qos_high_freq);
    lc_corrected_cloud_pub_ = node_->create_publisher<PointCloud2Msg>(
        topic_prefix_ + "loop_closure/corrected_cloud", qos_high_freq);
    lc_optimization_busy_pub_ = node_->create_publisher<BoolMsg>(
        topic_prefix_ + "loop_closure/optimization_busy", qos_loop_result);
    lc_optimization_status_marker_pub_ = node_->create_publisher<MarkerMsg>(
        topic_prefix_ + "loop_closure/optimization_status_marker", qos_loop_result);

    // Setup calibration service
    calib_service_ = node_->create_service<odin_driver_interfaces::srv::GetCalibration>(
        topic_prefix_ + "get_calibration",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::GetCalibration::Request> request,
               std::shared_ptr<odin_driver_interfaces::srv::GetCalibration::Response> response) {
          (void)request;
          if (calib_ready_ && !calib_yaml_content_.empty()) {
            response->success = true;
            response->yaml_content = calib_yaml_content_;
            response->message = "Calibration data retrieved successfully";
            LogInfo("Calibration service: sent %zu bytes", calib_yaml_content_.size());
          } else {
            response->success = false;
            response->yaml_content = "";
            response->message = "Calibration data not ready yet";
            LogWarn("Calibration service: data not ready");
          }
        });
    LogInfo("Calibration service ready: %sget_calibration", topic_prefix_.c_str());

    // Setup sensor mode service
    sensor_mode_service_ = node_->create_service<odin_driver_interfaces::srv::SetSensorMode>(
        topic_prefix_ + "set_sensor_mode",
        [this](const std::shared_ptr<odin_ros_driver_rev1::srv::SetSensorMode::Request> request,
               std::shared_ptr<odin_ros_driver_rev1::srv::SetSensorMode::Response> response) {
          if (request->mode > static_cast<int>(sdk::OdinSensorMode::kAdaptive)) {
            response->success = false;
            response->error_code = static_cast<int32_t>(sdk::OdinResult::kInvalidArgument);
            response->message = "Invalid mode (0=HDR 1=High Peak 2=Low Peak 3=Adaptive)";
            LogWarn("SetSensorMode service: invalid mode %d", request->mode);
            return;
          }
          auto mode = IntToSensorMode(request->mode);
          const char *mode_name = SensorModeName(mode);
          if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
            response->success = false;
            response->error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
            response->message = "Device not connected";
            LogWarn("SetSensorMode service: device not connected");
            return;
          }
          sdk::OdinResult result = sdk::SetSensorMode(device_handle_, mode);
          response->error_code = static_cast<int32_t>(result);
          if (result == sdk::OdinResult::kOk) {
            sensor_mode_ = mode;
            response->success = true;
            response->message = mode_name;
            LogInfo("SetSensorMode service: set to %s (%d)", mode_name, static_cast<int>(sensor_mode_));
          } else {
            response->success = false;
            response->message = "SDK error: " + std::to_string(static_cast<int>(result));
            LogWarn("SetSensorMode service: SDK returned error %d", static_cast<int>(result));
          }
        });
    LogInfo("SensorMode service ready: %sset_sensor_mode", topic_prefix_.c_str());

    // Setup reset initial pose service
    reset_pose_service_ = node_->create_service<odin_driver_interfaces::srv::ResetInitialPose>(
        topic_prefix_ + "reset_initial_pose",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::ResetInitialPose::Request> request,
               std::shared_ptr<odin_driver_interfaces::srv::ResetInitialPose::Response> response) {
          (void)request;
          if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
            response->success = false;
            response->error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
            response->message = "Device not connected";
            LogWarn("ResetInitialPose service: device not connected");
            return;
          }
          sdk::OdinResult result = sdk::ResetInitialPose(device_handle_);
          response->error_code = static_cast<int32_t>(result);
          if (result == sdk::OdinResult::kOk) {
            response->success = true;
            response->message = "Initial pose reset successfully";
            LogInfo("ResetInitialPose service: pose reset OK");
          } else {
            response->success = false;
            response->message = "SDK error: " + std::to_string(static_cast<int>(result));
            LogWarn("ResetInitialPose service: SDK returned error %d", static_cast<int>(result));
          }
        });
    LogInfo("ResetInitialPose service ready: %sreset_initial_pose", topic_prefix_.c_str());

    // --- Extended debugging services (ROS2) ---

    // GetAdaptiveParam
    get_adaptive_param_service_ = node_->create_service<odin_driver_interfaces::srv::GetAdaptiveParam>(
        topic_prefix_ + "get_adaptive_param",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::GetAdaptiveParam::Request>,
               std::shared_ptr<odin_driver_interfaces::srv::GetAdaptiveParam::Response> r) {
          if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
            r->success = false; r->error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
            r->message = "Device not connected"; return; }
          sdk::OdinAdaptiveFilterParam p;
          auto res = sdk::GetAdaptiveParam(device_handle_, p);
          r->error_code = static_cast<int32_t>(res);
          if (res == sdk::OdinResult::kOk) {
            r->success = true; r->cdf_percentile = p.cdf_percentile; r->outdoor_threshold = p.outdoor_threshold; r->message = "OK";
          } else { r->success = false; r->message = "SDK error: " + std::to_string(static_cast<int>(res)); }
        });
    LogInfo("GetAdaptiveParam service ready: %sget_adaptive_param", topic_prefix_.c_str());

    // SetAdaptiveParam
    set_adaptive_param_service_ = node_->create_service<odin_driver_interfaces::srv::SetAdaptiveParam>(
        topic_prefix_ + "set_adaptive_param",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::SetAdaptiveParam::Request> req,
               std::shared_ptr<odin_driver_interfaces::srv::SetAdaptiveParam::Response> r) {
          if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
            r->success = false; r->error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
            r->message = "Device not connected"; return; }
          sdk::OdinAdaptiveFilterParam p(req->cdf_percentile, req->outdoor_threshold);
          auto res = sdk::SetAdaptiveParam(device_handle_, p);
          r->error_code = static_cast<int32_t>(res);
          if (res == sdk::OdinResult::kOk) { r->success = true; r->message = "OK"; }
          else { r->success = false; r->message = "SDK error: " + std::to_string(static_cast<int>(res)); }
        });
    LogInfo("SetAdaptiveParam service ready: %sset_adaptive_param", topic_prefix_.c_str());

    // GetSyncMode
    get_sync_mode_service_ = node_->create_service<odin_driver_interfaces::srv::GetSyncMode>(
        topic_prefix_ + "get_sync_mode",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::GetSyncMode::Request>,
               std::shared_ptr<odin_driver_interfaces::srv::GetSyncMode::Response> r) {
          if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
            r->success = false; r->error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
            r->message = "Device not connected"; return; }
          sdk::OdinTimeSyncMode mode;
          auto res = sdk::GetDefaultSyncMode(device_handle_, mode);
          r->error_code = static_cast<int32_t>(res);
          if (res == sdk::OdinResult::kOk) {
            r->success = true; r->sync_mode = static_cast<uint8_t>(mode);
            r->message = mode == sdk::OdinTimeSyncMode::kPtp ? "PTP" : "SysTick";
          } else { r->success = false; r->message = "SDK error: " + std::to_string(static_cast<int>(res)); }
        });
    LogInfo("GetSyncMode service ready: %sget_sync_mode", topic_prefix_.c_str());

    // SetSyncMode
    set_sync_mode_service_ = node_->create_service<odin_driver_interfaces::srv::SetSyncMode>(
        topic_prefix_ + "set_sync_mode",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::SetSyncMode::Request> req,
               std::shared_ptr<odin_driver_interfaces::srv::SetSyncMode::Response> r) {
          if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
            r->success = false; r->error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
            r->message = "Device not connected"; return; }
          auto mode = static_cast<sdk::OdinTimeSyncMode>(req->sync_mode);
          auto res = sdk::SetDefaultSyncMode(device_handle_, mode);
          r->error_code = static_cast<int32_t>(res);
          if (res == sdk::OdinResult::kOk) { r->success = true; r->message = "Sync mode set (reboot required)"; }
          else { r->success = false; r->message = "SDK error: " + std::to_string(static_cast<int>(res)); }
        });
    LogInfo("SetSyncMode service ready: %sset_sync_mode", topic_prefix_.c_str());

    // GetPtpSyncStatus
    get_ptp_sync_status_service_ = node_->create_service<odin_driver_interfaces::srv::GetPtpSyncStatus>(
        topic_prefix_ + "get_ptp_sync_status",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::GetPtpSyncStatus::Request>,
               std::shared_ptr<odin_driver_interfaces::srv::GetPtpSyncStatus::Response> r) {
          if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
            r->success = false; r->error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
            r->message = "Device not connected"; return; }
          sdk::OdinPtpSyncStatus s;
          auto res = sdk::GetPtpSyncStatus(device_handle_, s);
          r->error_code = static_cast<int32_t>(res);
          if (res == sdk::OdinResult::kOk) {
            r->success = true; r->sync_state = s.sync_state; r->servo_state = s.servo_state;
            r->offset_ns = s.offset_ns; r->path_delay_ns = s.path_delay_ns; r->freq_ppb = s.freq_ppb; r->message = "OK";
          } else { r->success = false; r->message = "SDK error: " + std::to_string(static_cast<int>(res)); }
        });
    LogInfo("GetPtpSyncStatus service ready: %sget_ptp_sync_status", topic_prefix_.c_str());

    // RestoreDefaultConfig
    restore_default_config_service_ = node_->create_service<odin_driver_interfaces::srv::RestoreDefaultConfig>(
        topic_prefix_ + "restore_default_config",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::RestoreDefaultConfig::Request>,
               std::shared_ptr<odin_driver_interfaces::srv::RestoreDefaultConfig::Response> r) {
          if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
            r->success = false; r->error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
            r->message = "Device not connected"; return; }
          auto res = sdk::RestoreDefaultConfig(device_handle_);
          r->error_code = static_cast<int32_t>(res);
          if (res == sdk::OdinResult::kOk) { r->success = true; r->message = "Default config restored (reboot required)"; }
          else { r->success = false; r->message = "SDK error: " + std::to_string(static_cast<int>(res)); }
        });
    LogInfo("RestoreDefaultConfig service ready: %srestore_default_config", topic_prefix_.c_str());

    // Setup loop-closure map services
    save_map_service_ = node_->create_service<odin_driver_interfaces::srv::SaveMap>(
        topic_prefix_ + "save_map",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::SaveMap::Request> request,
               std::shared_ptr<odin_driver_interfaces::srv::SaveMap::Response> response) {
          RunSaveMap(request->path, response->success, response->error_code, response->message);
        });
    LogInfo("SaveMap service ready: %ssave_map", topic_prefix_.c_str());

    load_map_service_ = node_->create_service<odin_driver_interfaces::srv::LoadMap>(
        topic_prefix_ + "load_map",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::LoadMap::Request> request,
               std::shared_ptr<odin_driver_interfaces::srv::LoadMap::Response> response) {
          RunLoadMap(request->path, response->success, response->error_code, response->message);
        });
    LogInfo("LoadMap service ready: %sload_map", topic_prefix_.c_str());

    exploy_map_service_ = node_->create_service<odin_driver_interfaces::srv::ExployMap>(
        topic_prefix_ + "exploy_map",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::ExployMap::Request> request,
               std::shared_ptr<odin_driver_interfaces::srv::ExployMap::Response> response) {
          RunExployMap(request->path, response->success, response->error_code, response->message);
        });
    LogInfo("ExployMap service ready: %sexploy_map", topic_prefix_.c_str());

    // Setup device-mode service
    device_mode_service_ = node_->create_service<odin_driver_interfaces::srv::DeviceMode>(
        topic_prefix_ + "device_mode",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::DeviceMode::Request> request,
               std::shared_ptr<odin_driver_interfaces::srv::DeviceMode::Response> response) {
          RunDeviceMode(request->mode, response->success, response->error_code,
                        response->message);
        });
    LogInfo("DeviceMode service ready: %sdevice_mode", topic_prefix_.c_str());

#else
    // Point cloud publishers use point_cloud_transport for plugin-based
    // compression support (zlib, zstd, draco, etc.)
    point_cloud_transport::PointCloudTransport pct(nh_);
    raw_cloud_pub_ = pct.advertise(topic_prefix_ + "cloud/raw", 100);
    slam_cloud_pub_ = pct.advertise(topic_prefix_ + "cloud/slam", 100);
    image_pub_ = nh_.advertise<CompressedImageMsg>(topic_prefix_ + "camera0/compressed", 100);
    imu_pub_ = nh_.advertise<ImuMsg>(topic_prefix_ + "imu", 1000);
    odom_pub_ = nh_.advertise<OdometryMsg>(topic_prefix_ + "odometry_lf", 1000);
    odom_hf_pub_ = nh_.advertise<OdometryMsg>(topic_prefix_ + "odometry_hf", 1000);
    lc_optimized_cloud_pub_ = nh_.advertise<PointCloud2Msg>(
        topic_prefix_ + "loop_closure/optimized_cloud", 1, true);
    lc_optimized_path_pub_ = nh_.advertise<PathMsg>(
        topic_prefix_ + "loop_closure/optimized_path", 1, true);
    lc_map_cloud_pub_ = nh_.advertise<PointCloud2Msg>(
        topic_prefix_ + "loop_closure/map_cloud", 1, true);
    lc_corrected_pose_pub_ = nh_.advertise<PoseStampedMsg>(
        topic_prefix_ + "loop_closure/corrected_pose", 10);
    lc_corrected_cloud_pub_ = nh_.advertise<PointCloud2Msg>(
        topic_prefix_ + "loop_closure/corrected_cloud", 10);
    lc_optimization_busy_pub_ = nh_.advertise<BoolMsg>(
        topic_prefix_ + "loop_closure/optimization_busy", 1, true);
    lc_optimization_status_marker_pub_ = nh_.advertise<MarkerMsg>(
        topic_prefix_ + "loop_closure/optimization_status_marker", 1, true);

    // Setup calibration service
    calib_service_ = nh_.advertiseService(topic_prefix_ + "get_calibration",
                                          &odinRosDriver::HandleCalibrationService, this);
    LogInfo("Calibration service ready: %sget_calibration", topic_prefix_.c_str());

    // Setup sensor mode service
    sensor_mode_service_ = nh_.advertiseService(topic_prefix_ + "set_sensor_mode",
                                                &odinRosDriver::HandleSensorModeService, this);
    LogInfo("SensorMode service ready: %sset_sensor_mode", topic_prefix_.c_str());

    // Setup reset initial pose service
    reset_pose_service_ = nh_.advertiseService(topic_prefix_ + "reset_initial_pose",
                                               &odinRosDriver::HandleResetInitialPoseService, this);
    LogInfo("ResetInitialPose service ready: %sreset_initial_pose", topic_prefix_.c_str());

    // --- Extended debugging services (ROS1) ---
    get_adaptive_param_service_ = nh_.advertiseService(topic_prefix_ + "get_adaptive_param",
                                                        &odinRosDriver::HandleGetAdaptiveParamService, this);
    LogInfo("GetAdaptiveParam service ready: %sget_adaptive_param", topic_prefix_.c_str());
    set_adaptive_param_service_ = nh_.advertiseService(topic_prefix_ + "set_adaptive_param",
                                                        &odinRosDriver::HandleSetAdaptiveParamService, this);
    LogInfo("SetAdaptiveParam service ready: %sset_adaptive_param", topic_prefix_.c_str());
    get_sync_mode_service_ = nh_.advertiseService(topic_prefix_ + "get_sync_mode",
                                                  &odinRosDriver::HandleGetSyncModeService, this);
    LogInfo("GetSyncMode service ready: %sget_sync_mode", topic_prefix_.c_str());
    set_sync_mode_service_ = nh_.advertiseService(topic_prefix_ + "set_sync_mode",
                                                  &odinRosDriver::HandleSetSyncModeService, this);
    LogInfo("SetSyncMode service ready: %sset_sync_mode", topic_prefix_.c_str());
    get_ptp_sync_status_service_ = nh_.advertiseService(topic_prefix_ + "get_ptp_sync_status",
                                                         &odinRosDriver::HandleGetPtpSyncStatusService, this);
    LogInfo("GetPtpSyncStatus service ready: %sget_ptp_sync_status", topic_prefix_.c_str());
    restore_default_config_service_ = nh_.advertiseService(topic_prefix_ + "restore_default_config",
                                                            &odinRosDriver::HandleRestoreDefaultConfigService, this);
    LogInfo("RestoreDefaultConfig service ready: %srestore_default_config", topic_prefix_.c_str());

    // Setup loop-closure map services
    save_map_service_ = nh_.advertiseService(topic_prefix_ + "save_map",
                                             &odinRosDriver::HandleSaveMapService, this);
    LogInfo("SaveMap service ready: %ssave_map", topic_prefix_.c_str());
    load_map_service_ = nh_.advertiseService(topic_prefix_ + "load_map",
                                             &odinRosDriver::HandleLoadMapService, this);
    LogInfo("LoadMap service ready: %sload_map", topic_prefix_.c_str());
    exploy_map_service_ = nh_.advertiseService(topic_prefix_ + "exploy_map",
                                               &odinRosDriver::HandleExployMapService, this);
    LogInfo("ExployMap service ready: %sexploy_map", topic_prefix_.c_str());

    // Setup device-mode service
    device_mode_service_ = nh_.advertiseService(topic_prefix_ + "device_mode",
                                                &odinRosDriver::HandleDeviceModeService, this);
    LogInfo("DeviceMode service ready: %sdevice_mode", topic_prefix_.c_str());
#endif
  }

  template <typename MsgT>
  void PublishMessage(PublisherType<MsgT> &pub, const MsgT &msg) {
#if defined(ODIN_ROS2)
    if (pub) {
      pub->publish(msg);
    }
#else
    pub.publish(msg);
#endif
  }

  // Overload for point_cloud_transport::Publisher (both ROS1 and ROS2)
  void PublishMessage(point_cloud_transport::Publisher &pub, const PointCloud2Msg &msg) {
    pub.publish(msg);
  }

  bool LoadParameters() {
    // Ports are now auto-generated by SDK, no need to configure them
    discovery_timeout_ms_ = GetParam<int>("discovery_timeout_ms", 2000);

    // Topic prefix base (default: "manifold")
    // Final topic format: /{topic_prefix_base}/device{index}/
    topic_prefix_base_ = GetParam<std::string>("topic_prefix", "manifold");

    raw_frame_id_ = GetParam<std::string>("raw_frame_id", "lidar");
    slam_frame_id_ = GetParam<std::string>("slam_frame_id", "odom");
    image_frame_id_ = GetParam<std::string>("image_frame_id", "camera0");
    imu_frame_id_ = GetParam<std::string>("imu_frame_id", "imu");
    odom_frame_id_ = GetParam<std::string>("odom_frame_id", "odom");
    tf_publish_rate_ = GetParam<double>("tf_publish_rate", 50.0);

    std::string mode = GetParam<std::string>("device_mode", "normal");
    desired_mode_ = sdk::OdinDeviceMode::kNormal;
    if (mode == "standby") {
      desired_mode_ = sdk::OdinDeviceMode::kStandby;
    } else if (mode != "normal") {
      LogWarn("Unknown device_mode '%s', defaulting to 'normal'.", mode.c_str());
    }

    // Launch "sensor_mode" for the initial device mode pushed on connect. Read
    // the raw launch override here (the rqt slider parameter is declared with
    // ignore_override to keep its [0,3] range, so it does not carry this value).
    // -1 (default) or any out-of-range value means "keep the device's current
    // mode" and ConnectDevice() will not call SetSensorMode().
    startup_sensor_mode_ = -1;
#if defined(ODIN_ROS2)
    // node_->get_node_options().parameter_overrides() only holds overrides set
    // programmatically in C++, NOT launch-file / CLI parameters. The node
    // parameters interface override map DOES include launch/CLI values, so read
    // from there (this runs before SetupDynamicReconfigure() declares the rqt
    // slider, so the ranged declaration there is unaffected).
    const auto &overrides = node_->get_node_parameters_interface()->get_parameter_overrides();
    auto it = overrides.find("sensor_mode");
    if (it != overrides.end() &&
        it->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      startup_sensor_mode_ = static_cast<int>(it->second.get<int64_t>());
    }
#else
    main_pnh_.getParam("sensor_mode", startup_sensor_mode_);
#endif

    m_confidence_threshold = 7;

#if defined(ODIN_ROS2)
    // Helper lambda for reading int params from overrides (used by multiple sections)
    auto get_int_override = [&overrides](const char *name, int default_val) -> int {
      auto oit = overrides.find(name);
      if (oit != overrides.end() &&
          oit->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<int>(oit->second.get<int64_t>());
      }
      return default_val;
    };
#endif

    // Row filter parameters (client-side, zero out points below min_row)
    // enable_row_filter_ is hot-switchable, read from overrides in ROS2 to
    // avoid conflict with dynamic reconfigure declaration.
#if defined(ODIN_ROS2)
    {
      auto it = overrides.find("enable_row_filter");
      enable_row_filter_ = (it != overrides.end() &&
                            it->second.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                            ? it->second.get<bool>() : false;
      auto it2 = overrides.find("min_row");
      min_row_ = (it2 != overrides.end() &&
                  it2->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                  ? static_cast<int>(it2->second.get<int64_t>()) : 0;
    }
#else
    enable_row_filter_ = GetParam<bool>("enable_row_filter", false);
    min_row_ = GetParam<int>("min_row", 0);
#endif

    // Image resolution parameters (0 = use first capability from device)
    // In ROS2, read from overrides to avoid conflict with dynamic reconfigure declaration.
#if defined(ODIN_ROS2)
    image_width_  = get_int_override("image_width",  0);
    image_height_ = get_int_override("image_height", 0);
    image_fps_    = get_int_override("image_fps",    0);
    {
      auto oit = overrides.find("image_format");
      image_format_ = (oit != overrides.end() &&
                       oit->second.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
                      ? oit->second.get<std::string>() : "mjpeg";
    }
#else
    image_width_  = GetParam<int>("image_width", 0);
    image_height_ = GetParam<int>("image_height", 0);
    image_fps_    = GetParam<int>("image_fps", 0);
    image_format_ = GetParam<std::string>("image_format", "mjpeg");
#endif

    // Device-side filter parameters (-1 = device default, skip override)
    // In ROS2, read from parameter overrides only — GetParam() would declare
    // the parameter here, conflicting with the ranged declaration in
    // SetupDynamicReconfigure() (ParameterAlreadyDeclaredException).
#if defined(ODIN_ROS2)
    filter_confidence_threshold_    = get_int_override("filter_confidence_threshold",    -1);
    filter_adaptive_cdf_percentile_ = get_int_override("filter_adaptive_cdf_percentile", -1);
    filter_adaptive_outdoor_thr_    = get_int_override("filter_adaptive_outdoor_thr",    -1);
    filter_range_min_depth_         = get_int_override("filter_range_min_depth",         -1);
#else
    filter_confidence_threshold_    = GetParam<int>("filter_confidence_threshold",    -1);
    filter_adaptive_cdf_percentile_ = GetParam<int>("filter_adaptive_cdf_percentile", -1);
    filter_adaptive_outdoor_thr_    = GetParam<int>("filter_adaptive_outdoor_thr",    -1);
    filter_range_min_depth_         = GetParam<int>("filter_range_min_depth",         -1);
#endif

    // Channel enable parameters - load from launch params
    return LoadChannelEnableConfig();
  }

  void SyncImageParamsToROS() {
#if defined(ODIN_ROS2)
    if (node_ && (image_width_ > 0 || image_height_ > 0 || image_fps_ > 0)) {
      sync_image_params_ = true;
      node_->set_parameter(rclcpp::Parameter("image_width",  image_width_));
      node_->set_parameter(rclcpp::Parameter("image_height", image_height_));
      node_->set_parameter(rclcpp::Parameter("image_fps",    image_fps_));
      sync_image_params_ = false;
      LogInfo("Synced image params to ROS: %dx%d@%dFPS",
              image_width_, image_height_, image_fps_);
    }
#endif
  }

  void SetupDynamicReconfigure() {
#if defined(ODIN_ROS2)
    // ROS2: confidence_threshold slider
    // The lower bound tracks the current sensor mode's minimum (e.g. HighPeak≥7,
    // LowPeak≥2, Adaptive=0). If the mode changes later, the callback enforces
    // the new minimum at runtime.
    int mode_conf_min = GetModeConfidenceMin(sensor_mode_);
    rcl_interfaces::msg::ParameterDescriptor slider_desc;
    slider_desc.description = "Confidence threshold (min depends on sensor mode: "
        "HDR≥0, HighPeak≥7, LowPeak≥2, Adaptive=off)";
    slider_desc.integer_range.resize(1);
    slider_desc.integer_range[0].from_value = mode_conf_min;
    slider_desc.integer_range[0].to_value = 0xff;
    slider_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("confidence_threshold", m_confidence_threshold, slider_desc);
    node_->get_parameter("confidence_threshold", m_confidence_threshold);
    // Clamp to mode-specific minimum now that launch override is applied.
    ApplyModeConfidence();

    rcl_interfaces::msg::ParameterDescriptor mode_desc;
    mode_desc.description = "Sensor mode: 0=HDR, 1=HighPeak, 2=LowPeak, 3=Adaptive (OdinSensorMode)";
    mode_desc.integer_range.resize(1);
    mode_desc.integer_range[0].from_value = 0;
    mode_desc.integer_range[0].to_value = static_cast<int>(sdk::OdinSensorMode::kAdaptive);
    mode_desc.integer_range[0].step = 1;
    // ignore_override=true: the launch "sensor_mode" may be the -1 sentinel
    // (meaning "do not set on start"), which is outside this slider's [0,3]
    // range. That launch value is consumed separately in LoadParameters()
    // (startup_sensor_mode_), so here we ignore the override and seed the
    // slider from sensor_mode_ (the device's actual mode queried in
    // ConnectDevice()). set_parameter() below then forces the displayed value.
    node_->declare_parameter<int>("sensor_mode", static_cast<int>(sensor_mode_), mode_desc, true);
    node_->set_parameter(rclcpp::Parameter("sensor_mode", static_cast<int>(sensor_mode_)));

    // ROS2: log_level slider (0=ERROR, 1=INFO, 2=WARN, 3=DEBUG)
    rcl_interfaces::msg::ParameterDescriptor log_desc;
    log_desc.description = "Log level: 0=ERROR, 1=WARN,2=INFO, 3=DEBUG";
    log_desc.integer_range.resize(1);
    log_desc.integer_range[0].from_value = 0;
    log_desc.integer_range[0].to_value = 3;
    log_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("log_level", log_level_, log_desc);

    // ROS2: device-side filter parameters
    rcl_interfaces::msg::ParameterDescriptor filter_conf_desc;
    filter_conf_desc.description = "Device confidence filter threshold [0..255], -1=device default (no override)";
    filter_conf_desc.integer_range.resize(1);
    filter_conf_desc.integer_range[0].from_value = -1;
    filter_conf_desc.integer_range[0].to_value = 255;
    filter_conf_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("filter_confidence_threshold", filter_confidence_threshold_, filter_conf_desc, true);

    rcl_interfaces::msg::ParameterDescriptor filter_cdf_desc;
    filter_cdf_desc.description = "Adaptive filter CDF percentile [1..10000] (unit: 0.01%), -1=device default";
    filter_cdf_desc.integer_range.resize(1);
    filter_cdf_desc.integer_range[0].from_value = -1;
    filter_cdf_desc.integer_range[0].to_value = 10000;
    filter_cdf_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("filter_adaptive_cdf_percentile", filter_adaptive_cdf_percentile_, filter_cdf_desc, true);

    rcl_interfaces::msg::ParameterDescriptor filter_outdoor_desc;
    filter_outdoor_desc.description = "Adaptive filter outdoor threshold (mm), -1=device default";
    filter_outdoor_desc.integer_range.resize(1);
    filter_outdoor_desc.integer_range[0].from_value = -1;
    filter_outdoor_desc.integer_range[0].to_value = 65535;
    filter_outdoor_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("filter_adaptive_outdoor_thr", filter_adaptive_outdoor_thr_, filter_outdoor_desc, true);

    rcl_interfaces::msg::ParameterDescriptor filter_range_desc;
    filter_range_desc.description = "Range filter min depth (mm), -1=device default";
    filter_range_desc.integer_range.resize(1);
    filter_range_desc.integer_range[0].from_value = -1;
    filter_range_desc.integer_range[0].to_value = 65535;
    filter_range_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("filter_range_min_depth", filter_range_min_depth_, filter_range_desc, true);

    // Row filter parameter (hot-switch via rqt)
    rcl_interfaces::msg::ParameterDescriptor min_row_desc;
    min_row_desc.description = "Minimum row index (points below this are zeroed)";
    min_row_desc.integer_range.resize(1);
    min_row_desc.integer_range[0].from_value = 0;
    min_row_desc.integer_range[0].to_value = 255;
    min_row_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("min_row", min_row_, min_row_desc, true);

    // Image resolution parameters (adjustable when image channels are off)
    rcl_interfaces::msg::ParameterDescriptor img_width_desc;
    img_width_desc.description = "Image width (0=device default, disable image channels first)";
    img_width_desc.integer_range.resize(1);
    img_width_desc.integer_range[0].from_value = 0;
    img_width_desc.integer_range[0].to_value = 4096;
    img_width_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("image_width", image_width_, img_width_desc, true);

    rcl_interfaces::msg::ParameterDescriptor img_height_desc;
    img_height_desc.description = "Image height (0=device default, disable image channels first)";
    img_height_desc.integer_range.resize(1);
    img_height_desc.integer_range[0].from_value = 0;
    img_height_desc.integer_range[0].to_value = 4096;
    img_height_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("image_height", image_height_, img_height_desc, true);

    rcl_interfaces::msg::ParameterDescriptor img_fps_desc;
    img_fps_desc.description = "Image FPS (0=device default, disable image channels first)";
    img_fps_desc.integer_range.resize(1);
    img_fps_desc.integer_range[0].from_value = 0;
    img_fps_desc.integer_range[0].to_value = 120;
    img_fps_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("image_fps", image_fps_, img_fps_desc, true);

    rcl_interfaces::msg::ParameterDescriptor img_fmt_desc;
    img_fmt_desc.description = "Image format: mjpeg | h264 | nv12 (disable image channels first)";
    node_->declare_parameter<std::string>("image_format", image_format_, img_fmt_desc, true);

    // Channel enable parameters (hot-switch via rqt)
    node_->declare_parameter<bool>("enable_row_filter", enable_row_filter_);
    node_->declare_parameter<bool>("enable_raw_point", enable_raw_point_);
    node_->declare_parameter<bool>("enable_slam_point", enable_slam_point_);
    node_->declare_parameter<bool>("enable_image0", enable_image0_);
    rcl_interfaces::msg::ParameterDescriptor camera1_desc;
    camera1_desc.read_only = true;
    camera1_desc.description = "Unsupported: this hardware has only camera0";
    node_->declare_parameter<bool>("enable_image1", false, camera1_desc);
    node_->declare_parameter<bool>("enable_imu", enable_imu_);
    node_->declare_parameter<bool>("enable_odom_lf", enable_odom_lf_);
    node_->declare_parameter<bool>("enable_odom_hf", enable_odom_hf_);
    node_->declare_parameter<bool>("enable_tf", enable_tf_);

    // Parameter callback
    param_callback_handle_ =
        node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &param : params) {
            if (param.get_name() == "confidence_threshold") {
              int requested = param.as_int();
              int min_val = GetModeConfidenceMin(sensor_mode_);
              if (sensor_mode_ != sdk::OdinSensorMode::kAdaptive && requested < min_val) {
                result.successful = false;
                result.reason = "confidence_threshold " + std::to_string(requested) +
                    " below " + SensorModeName(sensor_mode_) + std::string(" mode minimum ") +
                    std::to_string(min_val);
              } else {
                m_confidence_threshold = requested;
                ApplyModeConfidence();
                LogInfo("confidence_threshold changed to: %d (mode=%s)",
                        m_confidence_threshold, SensorModeName(sensor_mode_));
              }
            } else if (param.get_name() == "sensor_mode") {
              sensor_mode_ = IntToSensorMode(param.as_int());
              LogInfo("sensor_mode changed to: %s (%d)", SensorModeName(sensor_mode_),
                      static_cast<int>(sensor_mode_));
              ApplyModeConfidence();
              if (connected_ && device_handle_ != sdk::kInvalidDeviceHandle) {
                sdk::SetSensorMode(device_handle_, sensor_mode_);
              }
            } else if (param.get_name() == "log_level") {
              log_level_ = param.as_int();
              if (log_level_ < 0) log_level_ = 0;
              if (log_level_ > 3) log_level_ = 3;
              log_config(static_cast<unsigned int>(log_level_), printf);
              const char *level_names[] = {"ERROR", "INFO", "WARN", "DEBUG"};
              LogInfo("log_level changed to: %s (%d)", level_names[log_level_], log_level_);
            } else if (param.get_name() == "filter_confidence_threshold" ||
                       param.get_name() == "filter_adaptive_cdf_percentile" ||
                       param.get_name() == "filter_adaptive_outdoor_thr"   ||
                       param.get_name() == "filter_range_min_depth") {
              if (param.get_name() == "filter_confidence_threshold")    filter_confidence_threshold_    = param.as_int();
              if (param.get_name() == "filter_adaptive_cdf_percentile") filter_adaptive_cdf_percentile_ = param.as_int();
              if (param.get_name() == "filter_adaptive_outdoor_thr")    filter_adaptive_outdoor_thr_    = param.as_int();
              if (param.get_name() == "filter_range_min_depth")         filter_range_min_depth_         = param.as_int();
              LogInfo("device filter param changed: confidence=%d adaptive_cdf=%d outdoor=%d range_min=%d",
                      filter_confidence_threshold_, filter_adaptive_cdf_percentile_,
                      filter_adaptive_outdoor_thr_, filter_range_min_depth_);
              ApplyDeviceFilters();
            } else if (param.get_name() == "enable_row_filter") {
              enable_row_filter_ = param.as_bool();
              LogInfo("enable_row_filter changed to: %d", enable_row_filter_);
            } else if (param.get_name() == "min_row") {
              min_row_ = param.as_int();
              LogInfo("min_row changed to: %d", min_row_);
            } else if (param.get_name() == "image_width"  ||
                       param.get_name() == "image_height" ||
                       param.get_name() == "image_fps"    ||
                       param.get_name() == "image_format") {
              if ((enable_image0_) && !sync_image_params_) {
                result.successful = false;
                result.reason = "Disable image channels (enable_image0) before changing image params";
              } else {
                if (param.get_name() == "image_width")   image_width_   = param.as_int();
                if (param.get_name() == "image_height")  image_height_  = param.as_int();
                if (param.get_name() == "image_fps")     image_fps_     = param.as_int();
                if (param.get_name() == "image_format")  image_format_  = param.as_string();
                LogInfo("Image param changed: %dx%d@%dFPS format=%s",
                        image_width_, image_height_, image_fps_, image_format_.c_str());
              }
            } else if (param.get_name() == "enable_raw_point"  ||
                       param.get_name() == "enable_slam_point" ||
                       param.get_name() == "enable_image0"     ||
                       param.get_name() == "enable_imu"        ||
                       param.get_name() == "enable_odom_lf"    ||
                       param.get_name() == "enable_odom_hf"    ||
                       param.get_name() == "enable_tf") {
              if (param.get_name() == "enable_raw_point")  enable_raw_point_  = param.as_bool();
              if (param.get_name() == "enable_slam_point") enable_slam_point_ = param.as_bool();
              if (param.get_name() == "enable_image0")     enable_image0_     = param.as_bool();
              if (param.get_name() == "enable_imu")        enable_imu_        = param.as_bool();
              if (param.get_name() == "enable_odom_lf")    enable_odom_lf_    = param.as_bool();
              if (param.get_name() == "enable_odom_hf")    enable_odom_hf_    = param.as_bool();
              if (param.get_name() == "enable_tf")         enable_tf_         = param.as_bool();
              LogInfo("Channel enable changed: raw=%d slam=%d img0=%d imu=%d odom_lf=%d odom_hf=%d tf=%d",
                      enable_raw_point_, enable_slam_point_, enable_image0_,
                      enable_imu_, enable_odom_lf_, enable_odom_hf_, enable_tf_);
              ReconfigureChannels();
            }
          }
          return result;
        });
    LogInfo("Dynamic reconfigure initialized");
#else
    // ROS1: Setup ddynamic_reconfigure
    LogInfo("Setting up ddynamic_reconfigure with namespace: %s", pnh_.getNamespace().c_str());
    ddr_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(pnh_);

    // confidence_threshold slider
    // Lower bound reflects current sensor mode minimum (HighPeak≥7, LowPeak≥2, etc.).
    ddr_->registerVariable<int>("confidence_threshold", m_confidence_threshold,
                                boost::bind(&odinRosDriver::onConfidenceThresholdChanged, this, _1),
                                "Confidence threshold (min depends on sensor mode)",
                                GetModeConfidenceMin(sensor_mode_), 255);

    // sensor_mode slider (0=HDR 1=HighPeak, 2=LowPeak, 3=Adaptive)
    ddr_->registerVariable<int>("sensor_mode", static_cast<int>(sensor_mode_),
                                boost::bind(&odinRosDriver::onSensorModeChanged, this, _1),
                                "Sensor mode: 0=HDR, 1=HighPeak, 2=LowPeak, 3=Adaptive",
                                0, static_cast<int>(sdk::OdinSensorMode::kAdaptive));

    // log_level slider (0=ERROR, 1=INFO, 2=WARN, 3=DEBUG)
    ddr_->registerVariable<int>("log_level", log_level_,
                                boost::bind(&odinRosDriver::onLogLevelChanged, this, _1),
                                "Log level: 0=ERROR, 1=INFO, 2=WARN, 3=DEBUG", 0, 3);

    // device-side filter parameters
    ddr_->registerVariable<int>("filter_confidence_threshold", filter_confidence_threshold_,
                                boost::bind(&odinRosDriver::onFilterParamChanged, this, _1,
                                            std::string("filter_confidence_threshold")),
                                "Device confidence filter threshold [0..255], -1=device default", -1, 255);
    ddr_->registerVariable<int>("filter_adaptive_cdf_percentile", filter_adaptive_cdf_percentile_,
                                boost::bind(&odinRosDriver::onFilterParamChanged, this, _1,
                                            std::string("filter_adaptive_cdf_percentile")),
                                "Adaptive filter CDF percentile [1..10000] (0.01%), -1=device default", -1, 10000);
    ddr_->registerVariable<int>("filter_adaptive_outdoor_thr", filter_adaptive_outdoor_thr_,
                                boost::bind(&odinRosDriver::onFilterParamChanged, this, _1,
                                            std::string("filter_adaptive_outdoor_thr")),
                                "Adaptive filter outdoor threshold (mm), -1=device default", -1, 65535);
    ddr_->registerVariable<int>("filter_range_min_depth", filter_range_min_depth_,
                                boost::bind(&odinRosDriver::onFilterParamChanged, this, _1,
                                            std::string("filter_range_min_depth")),
                                "Range filter min depth (mm), -1=device default", -1, 65535);

    // Row filter parameters (hot-switch)
    ddr_->registerVariable<bool>("enable_row_filter", enable_row_filter_,
                                boost::bind(&odinRosDriver::onRowFilterEnableChanged, this, _1),
                                "Enable client-side row filter", false, true);
    ddr_->registerVariable<int>("min_row", min_row_,
                                boost::bind(&odinRosDriver::onMinRowChanged, this, _1),
                                "Minimum row index (points below this are zeroed)", 0, 255);

    // Image resolution parameters (adjustable when image channels are off)
    ddr_->registerVariable<int>("image_width", image_width_,
                                boost::bind(&odinRosDriver::onImageParamChanged, this, _1,
                                            std::string("image_width")),
                                "Image width (0=device default, disable image channels first)", 0, 4096);
    ddr_->registerVariable<int>("image_height", image_height_,
                                boost::bind(&odinRosDriver::onImageParamChanged, this, _1,
                                            std::string("image_height")),
                                "Image height (0=device default, disable image channels first)", 0, 4096);
    ddr_->registerVariable<int>("image_fps", image_fps_,
                                boost::bind(&odinRosDriver::onImageParamChanged, this, _1,
                                            std::string("image_fps")),
                                "Image FPS (0=device default, disable image channels first)", 0, 120);
    ddr_->registerVariable<std::string>("image_format", image_format_,
                                boost::bind(&odinRosDriver::onImageFormatChanged, this, _1),
                                "Image format: mjpeg | h264 | nv12 (disable image channels first)");

    ddr_->registerVariable<bool>("enable_raw_point", enable_raw_point_,
                                boost::bind(&odinRosDriver::onChannelEnableChanged, this, _1,
                                            std::string("enable_raw_point")),
                                "Enable raw point cloud channel", false, true);
    ddr_->registerVariable<bool>("enable_slam_point", enable_slam_point_,
                                boost::bind(&odinRosDriver::onChannelEnableChanged, this, _1,
                                            std::string("enable_slam_point")),
                                "Enable SLAM point cloud channel", false, true);
    ddr_->registerVariable<bool>("enable_image0", enable_image0_,
                                boost::bind(&odinRosDriver::onChannelEnableChanged, this, _1,
                                            std::string("enable_image0")),
                                "Enable camera0 image channel", false, true);
    ddr_->registerVariable<bool>("enable_imu", enable_imu_,
                                boost::bind(&odinRosDriver::onChannelEnableChanged, this, _1,
                                            std::string("enable_imu")),
                                "Enable IMU data channel", false, true);
    ddr_->registerVariable<bool>("enable_odom_lf", enable_odom_lf_,
                                boost::bind(&odinRosDriver::onChannelEnableChanged, this, _1,
                                            std::string("enable_odom_lf")),
                                "Enable low-frequency odometry channel", false, true);
    ddr_->registerVariable<bool>("enable_odom_hf", enable_odom_hf_,
                                boost::bind(&odinRosDriver::onChannelEnableChanged, this, _1,
                                            std::string("enable_odom_hf")),
                                "Enable high-frequency odometry channel", false, true);
    ddr_->registerVariable<bool>("enable_tf", enable_tf_,
                                boost::bind(&odinRosDriver::onChannelEnableChanged, this, _1,
                                            std::string("enable_tf")),
                                "Enable TF transform channel", false, true);

    ddr_->publishServicesTopics();
    LogInfo("Dynamic reconfigure initialized");
#endif
  }

#if !defined(ODIN_ROS2)
  void onConfidenceThresholdChanged(int new_val) {
    int min_val = GetModeConfidenceMin(sensor_mode_);
    if (sensor_mode_ != sdk::OdinSensorMode::kAdaptive && new_val < min_val) {
      LogWarn("confidence_threshold %d below %s mode minimum %d, clamping to %d",
              new_val, SensorModeName(sensor_mode_), min_val, min_val);
    }
    m_confidence_threshold = new_val;
    ApplyModeConfidence();
    LogInfo("confidence_threshold changed to: %d (mode=%s)",
            m_confidence_threshold, SensorModeName(sensor_mode_));
  }

  void onSensorModeChanged(int new_mode) {
    sensor_mode_ = IntToSensorMode(new_mode);
    LogInfo("sensor_mode changed to: %s (%d)", SensorModeName(sensor_mode_),
            static_cast<int>(sensor_mode_));
    ApplyModeConfidence();
    if (connected_ && device_handle_ != sdk::kInvalidDeviceHandle) {
      sdk::SetSensorMode(device_handle_, sensor_mode_);
    }
  }

  void onLogLevelChanged(int new_level) {
    log_level_ = new_level;
    if (log_level_ < 0) log_level_ = 0;
    if (log_level_ > 3) log_level_ = 3;
    log_config(static_cast<unsigned int>(log_level_), printf);
    const char *level_names[] = {"ERROR", "INFO", "WARN", "DEBUG"};
    LogInfo("log_level changed to: %s (%d)", level_names[log_level_], log_level_);
  }

  void onFilterParamChanged(int new_val, const std::string& param_name) {
    if (param_name == "filter_confidence_threshold")    filter_confidence_threshold_    = new_val;
    if (param_name == "filter_adaptive_cdf_percentile") filter_adaptive_cdf_percentile_ = new_val;
    if (param_name == "filter_adaptive_outdoor_thr")    filter_adaptive_outdoor_thr_    = new_val;
    if (param_name == "filter_range_min_depth")         filter_range_min_depth_         = new_val;
    LogInfo("device filter param changed: confidence=%d adaptive_cdf=%d outdoor=%d range_min=%d",
            filter_confidence_threshold_, filter_adaptive_cdf_percentile_,
            filter_adaptive_outdoor_thr_, filter_range_min_depth_);
    ApplyDeviceFilters();
  }

  void onRowFilterEnableChanged(bool new_val) {
    enable_row_filter_ = new_val;
    LogInfo("enable_row_filter changed to: %d", enable_row_filter_);
  }

  void onMinRowChanged(int new_val) {
    min_row_ = new_val;
    LogInfo("min_row changed to: %d", min_row_);
  }

  void onImageParamChanged(int new_val, const std::string& param_name) {
    if (enable_image0_) {
      LogWarn("Cannot change %s while image channels are active, disable enable_image0 first",
              param_name.c_str());
      return;
    }
    if (param_name == "image_width")   image_width_   = new_val;
    if (param_name == "image_height")  image_height_  = new_val;
    if (param_name == "image_fps")     image_fps_     = new_val;
    LogInfo("Image param changed: %dx%d@%dFPS format=%s",
            image_width_, image_height_, image_fps_, image_format_.c_str());
  }

  void onImageFormatChanged(const std::string& new_val) {
    if (enable_image0_) {
      LogWarn("Cannot change image_format while image channels are active, disable enable_image0 first");
      return;
    }
    image_format_ = new_val;
    LogInfo("Image param changed: %dx%d@%dFPS format=%s",
            image_width_, image_height_, image_fps_, image_format_.c_str());
  }

  void onChannelEnableChanged(bool new_val, const std::string& param_name) {
    if (param_name == "enable_raw_point")  enable_raw_point_  = new_val;
    if (param_name == "enable_slam_point") enable_slam_point_ = new_val;
    if (param_name == "enable_image0")     enable_image0_     = new_val;
    if (param_name == "enable_imu")        enable_imu_        = new_val;
    if (param_name == "enable_odom_lf")    enable_odom_lf_    = new_val;
    if (param_name == "enable_odom_hf")    enable_odom_hf_    = new_val;
    if (param_name == "enable_tf")         enable_tf_         = new_val;
    LogInfo("Channel enable changed: raw=%d slam=%d img0=%d imu=%d odom_lf=%d odom_hf=%d tf=%d",
            enable_raw_point_, enable_slam_point_, enable_image0_,
            enable_imu_, enable_odom_lf_, enable_odom_hf_, enable_tf_);
    ReconfigureChannels();
  }
#endif

  void ReconfigureChannels() {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      LogInfo("ReconfigureChannels: not connected, will apply on connect");
      return;
    }

    auto desired = BuildDesiredChannels();
    auto transport = sdk::OdinTransportMode::kUdp;

    // Close channels that are active but no longer desired
    for (auto ch : active_channels_) {
      if (desired.find(ch) == desired.end()) {
        LogInfo("Closing channel: %d", static_cast<int>(ch));
        sdk::CloseStream(device_handle_, ch);
      }
    }

    // Open channels that are desired but not yet active
    for (auto ch : desired) {
      if (active_channels_.find(ch) == active_channels_.end()) {
        LogInfo("Opening channel: %d", static_cast<int>(ch));
        StartSingleChannel(ch, transport);
      }
    }

    active_channels_ = desired;
    LogInfo("Channel reconfigure complete");
  }

  void RegisterCallbacks() {
    if (callbacks_registered_) {
      return;
    }
    sdk::RegisterPointCloudCallback(device_handle_, &odinRosDriver::PointCloudCallback, this);
    sdk::RegisterSlamCallback(device_handle_, &odinRosDriver::SlamCallback, this);
    sdk::RegisterCameraCallback(device_handle_, &odinRosDriver::CameraCallback, this);
    sdk::RegisterImuCallback(device_handle_, &odinRosDriver::ImuCallback, this);
    sdk::RegisterOdomCallback(device_handle_, &odinRosDriver::OdomCallback, this);
    sdk::RegisterTfCallback(device_handle_, &odinRosDriver::TfCallback, this);
    sdk::RegisterLcOptimizationStatusCallback(
        device_handle_, &odinRosDriver::LcOptimizationStatusCallback, this);
    sdk::RegisterLcFrameCallback(device_handle_, &odinRosDriver::LcFrameCallback, this);
    sdk::RegisterLcOptimizationResultCallback(
        device_handle_, &odinRosDriver::LcOptimizationResultCallback, this);
    sdk::RegisterLcMapCloudCallback(
        device_handle_, &odinRosDriver::LcMapCloudCallback, this);
    callbacks_registered_ = true;
  }

  // ---------------------------------------------------------------------------
  // Diagnostic Monitor: real-time FPS + latency print & service
  // ---------------------------------------------------------------------------
  void InitDiagMonitor() {
#if defined(ODIN_ROS2)
    if (node_) {
      // Publisher for diag status (TRANSIENT_LOCAL so late subscribers get last value)
      auto qos_status = rclcpp::QoS(rclcpp::KeepLast(1))
                            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      diag_status_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
          topic_prefix_ + "diag_status", qos_status);

      // Standard ROS diagnostics publisher (/diagnostics)
      ros_diagnostics_pub_ = node_->create_publisher<DiagnosticArrayMsg>(
          "/diagnostics", rclcpp::QoS(10));

      diag_service_ = node_->create_service<odin_driver_interfaces::srv::SetDiagMonitor>(
          topic_prefix_ + "set_diag_monitor",
          [this](const std::shared_ptr<odin_driver_interfaces::srv::SetDiagMonitor::Request> req,
                 std::shared_ptr<odin_driver_interfaces::srv::SetDiagMonitor::Response> r) {
            diag_enabled_.store(req->enable);
            if (req->enable) {
              // Reset stats on enable
              std::lock_guard<std::mutex> lock(diag_mutex_);
              diag_stats_.clear();
              diag_prev_counts_.clear();
              diag_prev_time_ = std::chrono::steady_clock::time_point();
            }
            // Publish status so subscriber nodes sync automatically
            auto msg = std_msgs::msg::Bool();
            msg.data = req->enable;
            diag_status_pub_->publish(msg);
            r->success = true;
            r->message = req->enable ? "Diag monitor enabled" : "Diag monitor disabled";
            r->monitor_enabled = req->enable;
            // Fill current stats into response
            FillDiagResponse(r);
          });
      diag_timer_ = node_->create_wall_timer(
          std::chrono::seconds(1),
          [this]() { DiagTimerCallback(); });
    }
    LogInfo("DiagMonitor service ready: %sset_diag_monitor", topic_prefix_.c_str());

    // Channel config service (hot-switch)
    set_channel_config_service_ = node_->create_service<odin_driver_interfaces::srv::SetChannelConfig>(
        topic_prefix_ + "set_channel_config",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::SetChannelConfig::Request> req,
               std::shared_ptr<odin_driver_interfaces::srv::SetChannelConfig::Response> res) {
          HandleSetChannelConfigService(req, res);
        });
    LogInfo("ChannelConfig service ready: %sset_channel_config", topic_prefix_.c_str());

    // Channel config query service
    get_channel_config_service_ = node_->create_service<odin_driver_interfaces::srv::GetChannelConfig>(
        topic_prefix_ + "get_channel_config",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::GetChannelConfig::Request> req,
               std::shared_ptr<odin_driver_interfaces::srv::GetChannelConfig::Response> res) {
          HandleGetChannelConfigService(req, res);
        });
    LogInfo("ChannelConfig query service ready: %sget_channel_config", topic_prefix_.c_str());
#else
    diag_status_pub_ = nh_.advertise<std_msgs::Bool>(
        topic_prefix_ + "diag_status", 1, true);  // latch = true
    // Standard ROS diagnostics publisher (/diagnostics)
    ros_diagnostics_pub_ = nh_.advertise<DiagnosticArrayMsg>("/diagnostics", 10);
    diag_service_ = nh_.advertiseService(
        topic_prefix_ + "set_diag_monitor",
        &odinRosDriver::HandleSetDiagMonitorService, this);
    diag_timer_ = nh_.createTimer(ros::Duration(1.0),
        [this](const ros::TimerEvent&) { DiagTimerCallback(); });
    LogInfo("DiagMonitor service ready: %sset_diag_monitor", topic_prefix_.c_str());

    // Channel config service (hot-switch)
    set_channel_config_service_ = nh_.advertiseService(
        topic_prefix_ + "set_channel_config",
        &odinRosDriver::HandleSetChannelConfigService, this);
    LogInfo("ChannelConfig service ready: %sset_channel_config", topic_prefix_.c_str());

    // Channel config query service
    get_channel_config_service_ = nh_.advertiseService(
        topic_prefix_ + "get_channel_config",
        &odinRosDriver::HandleGetChannelConfigService, this);
    LogInfo("ChannelConfig query service ready: %sget_channel_config", topic_prefix_.c_str());
#endif
  }

  void FillDiagResponse(
      std::shared_ptr<odin_driver_interfaces::srv::SetDiagMonitor::Response> r) {
    std::lock_guard<std::mutex> lock(diag_mutex_);

    // Compute elapsed time since last print (supports any timer interval)
    double elapsed_sec = 0.0;
    auto now = std::chrono::steady_clock::now();
    if (diag_prev_time_.time_since_epoch().count() == 0) {
      diag_prev_time_ = now;
    } else {
      elapsed_sec = std::chrono::duration<double>(now - diag_prev_time_).count();
      diag_prev_time_ = now;
    }

    auto getFps = [&](const char* ch) -> double {
      auto it = diag_stats_.find(ch);
      if (it == diag_stats_.end()) return 0.0;
      auto prev = diag_prev_counts_.find(ch);
      uint64_t cur_count = it->second.frame_count;
      uint64_t prev_count = (prev != diag_prev_counts_.end()) ? prev->second : 0;
      double fps = (elapsed_sec > 0.0)
                       ? static_cast<double>(cur_count - prev_count) / elapsed_sec
                       : 0.0;
      diag_prev_counts_[ch] = cur_count;
      return fps;
    };
    auto getLatency = [&](const char* ch) -> int64_t {
      auto it = diag_stats_.find(ch);
      if (it == diag_stats_.end() || it->second.latency_window.empty()) return 0;
      return it->second.latency_window_sum /
             static_cast<int64_t>(it->second.latency_window.size());
    };
    r->raw_point_fps     = getFps("RawPoint");
    r->slam_point_fps    = getFps("SlamPoint");
    r->image0_fps        = getFps("Image0");
    r->image1_fps        = 0.0;
    r->imu_fps           = getFps("IMU");
    r->odom_lf_fps       = getFps("OdomLF");
    r->odom_hf_fps       = getFps("OdomHF");
    r->tf_fps            = getFps("Tf");
    r->raw_point_latency_ns  = getLatency("RawPoint");
    r->slam_point_latency_ns = getLatency("SlamPoint");
    r->image0_latency_ns     = getLatency("Image0");
    r->image1_latency_ns     = 0;
    r->imu_latency_ns        = getLatency("IMU");
    r->odom_lf_latency_ns    = getLatency("OdomLF");
    r->odom_hf_latency_ns    = getLatency("OdomHF");
    r->tf_latency_ns         = getLatency("Tf");
  }

  void DiagTimerCallback() {
    if (shutting_down_.load()) return;
    std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
    if (shutting_down_.load()) return;
    if (!diag_enabled_.load()) return;
    std::shared_ptr<odin_driver_interfaces::srv::SetDiagMonitor::Response> r =
        std::make_shared<odin_driver_interfaces::srv::SetDiagMonitor::Response>();
    FillDiagResponse(r);

    // Min/max latency are not in the service response, fetch them for terminal print
    std::lock_guard<std::mutex> lock(diag_mutex_);
    auto min_lat = [this](const char* ch) -> int64_t {
      auto it = diag_stats_.find(ch);
      if (it == diag_stats_.end() || it->second.latency_window.empty()) return 0;
      return it->second.latency_window_min;
    };
    auto max_lat = [this](const char* ch) -> int64_t {
      auto it = diag_stats_.find(ch);
      if (it == diag_stats_.end() || it->second.latency_window.empty()) return 0;
      return it->second.latency_window_max;
    };

    printf("\n");
    printf("[DiagMonitor] FPS / Latency(ns) — host_time - device_timestamp\n");
    printf("  Channel      FPS       Avg(ns)       Min(ns)       Max(ns)\n");
    printf("  ---------  -------  ------------  ------------  ------------\n");
    printf("  RawPoint    %5.1f   %13ld   %13ld   %13ld\n", r->raw_point_fps,
           (long)r->raw_point_latency_ns, (long)min_lat("RawPoint"), (long)max_lat("RawPoint"));
    printf("  SlamPoint   %5.1f   %13ld   %13ld   %13ld\n", r->slam_point_fps,
           (long)r->slam_point_latency_ns, (long)min_lat("SlamPoint"), (long)max_lat("SlamPoint"));
    printf("  Image0      %5.1f   %13ld   %13ld   %13ld\n", r->image0_fps,
           (long)r->image0_latency_ns, (long)min_lat("Image0"), (long)max_lat("Image0"));
    printf("  IMU         %5.1f   %13ld   %13ld   %13ld\n", r->imu_fps,
           (long)r->imu_latency_ns, (long)min_lat("IMU"), (long)max_lat("IMU"));
    printf("  OdomLF      %5.1f   %13ld   %13ld   %13ld\n", r->odom_lf_fps,
           (long)r->odom_lf_latency_ns, (long)min_lat("OdomLF"), (long)max_lat("OdomLF"));
    printf("  OdomHF      %5.1f   %13ld   %13ld   %13ld\n", r->odom_hf_fps,
           (long)r->odom_hf_latency_ns, (long)min_lat("OdomHF"), (long)max_lat("OdomHF"));
    printf("  Tf          %5.1f   %13ld   %13ld   %13ld\n", r->tf_fps,
           (long)r->tf_latency_ns, (long)min_lat("Tf"), (long)max_lat("Tf"));

    // Publish standard ROS diagnostics to /diagnostics
    PublishDiagnostics(r, min_lat, max_lat);
  }

  // Publish diagnostic_msgs/DiagnosticArray to /diagnostics so standard ROS
  // tools (rqt_robot_monitor, diagnostic_aggregator, etc.) can visualize
  // driver health. One DiagnosticStatus per data channel.
  void PublishDiagnostics(
      std::shared_ptr<odin_driver_interfaces::srv::SetDiagMonitor::Response> r,
      const std::function<int64_t(const char*)>& min_lat,
      const std::function<int64_t(const char*)>& max_lat) {
    if (!ros_diagnostics_pub_) return;

    DiagnosticArrayMsg diag_array;
#if defined(ODIN_ROS2)
    if (node_) {
      diag_array.header.stamp = node_->now();
    }
#else
    diag_array.header.stamp = ros::Time::now();
#endif

    const std::string hw_id = device_sn_.empty() ? "odin" : device_sn_;

    auto addStatus = [&](const std::string& name, double fps, int64_t avg_ns,
                         int64_t min_ns, int64_t max_ns) {
      DiagnosticStatusMsg status;
      status.name = "odin_driver: " + name;
      status.hardware_id = hw_id;

      // Determine status level: OK(0) if data flowing, WARN(1) if zero FPS, ERROR(2) if no data
      // Use numeric values to avoid macro conflict with SDK logger.h (WARN/ERROR macros)
      if (fps > 0.0) {
        status.level = 0;  // OK
        status.message = name + " stream active";
      } else {
        auto it = diag_stats_.find(name);
        if (it == diag_stats_.end() || it->second.frame_count == 0) {
          status.level = 2;  // ERROR
          status.message = name + " stream: no data received";
        } else {
          status.level = 1;  // WARN
          status.message = name + " stream: FPS dropped to 0";
        }
      }

      KeyValueMsg kv_fps;
      kv_fps.key = "fps";
      kv_fps.value = std::to_string(fps);
      status.values.push_back(kv_fps);

      KeyValueMsg kv_avg;
      kv_avg.key = "latency_avg_ns";
      kv_avg.value = std::to_string(avg_ns);
      status.values.push_back(kv_avg);

      KeyValueMsg kv_min;
      kv_min.key = "latency_min_ns";
      kv_min.value = std::to_string(min_ns);
      status.values.push_back(kv_min);

      KeyValueMsg kv_max;
      kv_max.key = "latency_max_ns";
      kv_max.value = std::to_string(max_ns);
      status.values.push_back(kv_max);

      diag_array.status.push_back(status);
    };

    addStatus("RawPoint", r->raw_point_fps, r->raw_point_latency_ns,
              min_lat("RawPoint"), max_lat("RawPoint"));
    addStatus("SlamPoint", r->slam_point_fps, r->slam_point_latency_ns,
              min_lat("SlamPoint"), max_lat("SlamPoint"));
    addStatus("Image0", r->image0_fps, r->image0_latency_ns,
              min_lat("Image0"), max_lat("Image0"));
    addStatus("IMU", r->imu_fps, r->imu_latency_ns,
              min_lat("IMU"), max_lat("IMU"));
    addStatus("OdomLF", r->odom_lf_fps, r->odom_lf_latency_ns,
              min_lat("OdomLF"), max_lat("OdomLF"));
    addStatus("OdomHF", r->odom_hf_fps, r->odom_hf_latency_ns,
              min_lat("OdomHF"), max_lat("OdomHF"));
    addStatus("Tf", r->tf_fps, r->tf_latency_ns,
              min_lat("Tf"), max_lat("Tf"));

    PublishMessage(ros_diagnostics_pub_, diag_array);
  }

#if !defined(ODIN_ROS2)
  bool HandleSetDiagMonitorService(
      odin_driver_interfaces::srv::SetDiagMonitor::Request &req,
      odin_driver_interfaces::srv::SetDiagMonitor::Response &res) {
    diag_enabled_.store(req.enable);
    if (req.enable) {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      diag_stats_.clear();
      diag_prev_counts_.clear();
      diag_prev_time_ = std::chrono::steady_clock::time_point();
    }
    // Publish status so subscriber nodes sync automatically
    std_msgs::Bool status_msg;
    status_msg.data = req.enable;
    diag_status_pub_.publish(status_msg);
    res.success = true;
    res.message = req.enable ? "Diag monitor enabled" : "Diag monitor disabled";
    res.monitor_enabled = req.enable;
    auto r = std::make_shared<odin_driver_interfaces::srv::SetDiagMonitor::Response>();
    FillDiagResponse(r);
    res.raw_point_fps     = r->raw_point_fps;
    res.slam_point_fps    = r->slam_point_fps;
    res.image0_fps        = r->image0_fps;
    res.image1_fps        = r->image1_fps;
    res.imu_fps           = r->imu_fps;
    res.odom_lf_fps       = r->odom_lf_fps;
    res.odom_hf_fps       = r->odom_hf_fps;
    res.tf_fps            = r->tf_fps;
    res.raw_point_latency_ns  = r->raw_point_latency_ns;
    res.slam_point_latency_ns = r->slam_point_latency_ns;
    res.image0_latency_ns     = r->image0_latency_ns;
    res.image1_latency_ns     = r->image1_latency_ns;
    res.imu_latency_ns        = r->imu_latency_ns;
    res.odom_lf_latency_ns    = r->odom_lf_latency_ns;
    res.odom_hf_latency_ns    = r->odom_hf_latency_ns;
    res.tf_latency_ns         = r->tf_latency_ns;
    return true;
  }
#endif

  template <typename Response>
  void FillChannelState(Response &res) const {
    res.raw_point_enabled = enable_raw_point_;
    res.slam_point_enabled = enable_slam_point_;
    res.image0_enabled = enable_image0_;
    res.image1_enabled = false;
    res.imu_enabled = enable_imu_;
    res.odom_lf_enabled = enable_odom_lf_;
    res.odom_hf_enabled = enable_odom_hf_;
    res.tf_enabled = enable_tf_;
  }

  void HandleSetChannelConfigService(
      const std::shared_ptr<odin_driver_interfaces::srv::SetChannelConfig::Request> req,
      std::shared_ptr<odin_driver_interfaces::srv::SetChannelConfig::Response> res) {
    if (req->enable_image1 > 0) {
      res->success = false;
      res->message = "This hardware has only camera0; camera1 is unsupported";
      FillChannelState(*res);
      return;
    }
    if (req->enable_raw_point >= 0)  enable_raw_point_  = (req->enable_raw_point != 0);
    if (req->enable_slam_point >= 0) enable_slam_point_ = (req->enable_slam_point != 0);
    if (req->enable_image0 >= 0)     enable_image0_     = (req->enable_image0 != 0);
    if (req->enable_imu >= 0)        enable_imu_        = (req->enable_imu != 0);
    if (req->enable_odom_lf >= 0)    enable_odom_lf_    = (req->enable_odom_lf != 0);
    if (req->enable_odom_hf >= 0)    enable_odom_hf_    = (req->enable_odom_hf != 0);
    if (req->enable_tf >= 0)         enable_tf_         = (req->enable_tf != 0);

    ReconfigureChannels();

    res->success = true;
    res->message = "Channel config updated";
    FillChannelState(*res);
  }

  void HandleGetChannelConfigService(
      const std::shared_ptr<odin_driver_interfaces::srv::GetChannelConfig::Request> /*req*/,
      std::shared_ptr<odin_driver_interfaces::srv::GetChannelConfig::Response> res) {
    res->success = true;
    res->message = "Current channel config";
    FillChannelState(*res);
  }

#if !defined(ODIN_ROS2)
  bool HandleSetChannelConfigService(
      odin_driver_interfaces::srv::SetChannelConfig::Request &req,
      odin_driver_interfaces::srv::SetChannelConfig::Response &res) {
    if (req.enable_image1 > 0) {
      res.success = false;
      res.message = "This hardware has only camera0; camera1 is unsupported";
      FillChannelState(res);
      return true;
    }
    if (req.enable_raw_point >= 0)  enable_raw_point_  = (req.enable_raw_point != 0);
    if (req.enable_slam_point >= 0) enable_slam_point_ = (req.enable_slam_point != 0);
    if (req.enable_image0 >= 0)     enable_image0_     = (req.enable_image0 != 0);
    if (req.enable_imu >= 0)        enable_imu_        = (req.enable_imu != 0);
    if (req.enable_odom_lf >= 0)    enable_odom_lf_    = (req.enable_odom_lf != 0);
    if (req.enable_odom_hf >= 0)    enable_odom_hf_    = (req.enable_odom_hf != 0);
    if (req.enable_tf >= 0)         enable_tf_         = (req.enable_tf != 0);

    ReconfigureChannels();

    res.success = true;
    res.message = "Channel config updated";
    FillChannelState(res);
    return true;
  }

  bool HandleGetChannelConfigService(
      odin_driver_interfaces::srv::GetChannelConfig::Request &/*req*/,
      odin_driver_interfaces::srv::GetChannelConfig::Response &res) {
    res.success = true;
    res.message = "Current channel config";
    FillChannelState(res);
    return true;
  }
#endif

  bool ConnectDevice() {
    device_handle_ = sdk::ConnectDevice(discovered_device_);
    if (device_handle_ == sdk::kInvalidDeviceHandle) {
      LogError("Failed to connect to device %s.", discovered_device_.network.ip_address.c_str());
      return false;
    }
    connected_ = true;
    LogInfo("Connected to %s handle=%u (ports auto-generated)", discovered_device_.network.ip_address.c_str(),
            static_cast<unsigned>(device_handle_));
    // Step 1: Set standby mode first to prepare for file transfer
    LogInfo("Setting device to standby mode for calibration fetch...");
    if (sdk::SetDeviceMode(device_handle_, sdk::OdinDeviceMode::kStandby) != sdk::OdinResult::kOk) {
      LogWarn("Failed to set standby mode, continuing anyway.");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Step 2: Fetch calibration file from device
    FetchCalibrationFile();

    // Step 3: Set desired device mode
    const auto mode_result = sdk::SetDeviceMode(device_handle_, desired_mode_);
    if (mode_result != sdk::OdinResult::kOk) {
      LogWarn("Failed to send mode configuration command.");
    }

    // Push a sensor mode to the device only when the launch "sensor_mode" is a
    // concrete value in [0,3]. The default -1 sentinel means "keep the module's
    // current mode" (it persists across reboots), so we skip setting it.
    if (startup_sensor_mode_ >= static_cast<int>(sdk::OdinSensorMode::kHDR) &&
        startup_sensor_mode_ <= static_cast<int>(sdk::OdinSensorMode::kAdaptive)) {
      auto launch_sensor_mode = static_cast<sdk::OdinSensorMode>(startup_sensor_mode_);
      if (sdk::SetSensorMode(device_handle_, launch_sensor_mode,10000) == sdk::OdinResult::kOk) {
        sensor_mode_ = launch_sensor_mode;
        LogInfo("Applied launch sensor mode: %s (%d)", SensorModeName(sensor_mode_),
                static_cast<int>(sensor_mode_));
      } else {
        LogWarn("Failed to apply launch sensor mode: %s (%d)",
                SensorModeName(launch_sensor_mode), static_cast<int>(launch_sensor_mode));
      }
    }

    sdk::OdinSensorMode actual_mode = sensor_mode_;
    if (sdk::GetSensorMode(device_handle_, actual_mode) == sdk::OdinResult::kOk) {
      sensor_mode_ = actual_mode;
      startup_sensor_mode_ = static_cast<int>(sensor_mode_);
      LogInfo("Queried device sensor mode: %s (%d)", SensorModeName(sensor_mode_),
              static_cast<int>(sensor_mode_));
    } else {
      LogWarn("GetSensorMode failed; keeping configured mode: %s (%d)",
              SensorModeName(sensor_mode_), static_cast<int>(sensor_mode_));
    }

    // Apply mode-specific confidence constraints after sensor mode is known.
    ApplyModeConfidence();

   

    std::string firmware_version;
    if (sdk::GetFirmwareVersion(device_handle_, firmware_version) == sdk::OdinResult::kOk) {
      printf("\n");
      printf("========================================\n");
      printf("  FIRMWARE VERSION: %s\n", firmware_version.c_str());
      printf("========================================\n");
      printf("\n");
    }

    // Step 4: Query and print all sensor capabilities
    QueryAndPrintSensorCapabilities();

    // Step 5: Apply device-side filter parameters
    ApplyDeviceFilters();

    // Step 7: Read back actual device values so that rqt sliders reflect
    // the real device state, not the launch defaults (e.g. -1 = "no override").
    QueryDeviceParams();

    // Step 8: Configure all data streams (query capability per channel)
    ConfigureStreams(sdk::OdinTransportMode::kUdp);
    PublishMappingActive(mode_result == sdk::OdinResult::kOk ? desired_mode_ : sdk::OdinDeviceMode::kUnknown);

    return true;
  }

  // Read back actual device-side parameter values and update member variables.
  // Called after Apply* functions so that rqt/dynamic_reconfigure sliders are
  // seeded from the device's real state rather than launch defaults.
  void QueryDeviceParams() {
    // Adaptive filter params
    sdk::OdinAdaptiveFilterParam adaptive;
    if (sdk::GetAdaptiveParam(device_handle_, adaptive) == sdk::OdinResult::kOk) {
      filter_adaptive_cdf_percentile_ = static_cast<int>(adaptive.cdf_percentile);
      filter_adaptive_outdoor_thr_    = static_cast<int>(adaptive.outdoor_threshold);
      LogInfo("Device adaptive filter readback: cdf=%d outdoor=%d",
              filter_adaptive_cdf_percentile_, filter_adaptive_outdoor_thr_);
    } else {
      LogWarn("GetAdaptiveParam readback failed, keeping launch values");
    }

    // Confidence/range filter params
    sdk::OdinLidarFilterParams filter;
    if (sdk::GetFilterParam(device_handle_, filter) == sdk::OdinResult::kOk) {
      if (filter.has_confidence) {
        filter_confidence_threshold_ = static_cast<int>(filter.confidence.confidence_threshold);
      }
      if (filter.has_range) {
        filter_range_min_depth_ = static_cast<int>(filter.range.min_depth);
      }
      LogInfo("Device filter readback: confidence=%d range_min=%d",
              filter_confidence_threshold_, filter_range_min_depth_);
    } else {
      LogWarn("GetFilterParam readback failed, keeping launch values");
    }
  }

  void ConfigureStreams(sdk::OdinTransportMode transport) {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      return;
    }
    const char* transport_name = (transport == sdk::OdinTransportMode::kTcp) ? "TCP" : "UDP";
    LogInfo("Configuring data streams (%s)...", transport_name);

    LogInfo("Channel config: raw=%d slam=%d img0=%d imu=%d odom_lf=%d odom_hf=%d tf=%d",
            enable_raw_point_, enable_slam_point_, enable_image0_,
            enable_imu_, enable_odom_lf_, enable_odom_hf_, enable_tf_);

    auto desired = BuildDesiredChannels();
    for (auto channel : desired) {
      StartSingleChannel(channel, transport);
    }

    active_channels_ = desired;
    LogInfo("Stream configuration complete");
  }

  // Apply device-side filter parameters read from launch args.
  // Adaptive filter uses dedicated SetAdaptiveParam API; confidence/range use SetFilterParam.
  void ApplyDeviceFilters() {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      return;
    }

    bool applied_any = false;

    // Adaptive filter: use SetAdaptiveParam (same API as set_adaptive_param service)
    if (filter_adaptive_cdf_percentile_ >= 0 || filter_adaptive_outdoor_thr_ >= 0) {
      sdk::OdinAdaptiveFilterParam param;
      if (filter_adaptive_cdf_percentile_ >= 0) {
        param.cdf_percentile = static_cast<uint16_t>(filter_adaptive_cdf_percentile_);
      }
      if (filter_adaptive_outdoor_thr_ >= 0) {
        param.outdoor_threshold = static_cast<uint16_t>(filter_adaptive_outdoor_thr_);
      }
      LogInfo("Applying adaptive filter params: cdf=%d outdoor=%d",
              filter_adaptive_cdf_percentile_, filter_adaptive_outdoor_thr_);
      sdk::OdinResult result = sdk::SetAdaptiveParam(device_handle_, param);
      if (result != sdk::OdinResult::kOk) {
        LogWarn("SetAdaptiveParam failed: %d", static_cast<int>(result));
      } else {
        LogInfo("SetAdaptiveParam OK");
        applied_any = true;
      }
    }

    // Confidence/range filters: use SetFilterParam
    sdk::OdinLidarFilterParams params;
    if (filter_confidence_threshold_ >= 0) {
      params.has_confidence = true;
      params.confidence.confidence_threshold = static_cast<uint8_t>(filter_confidence_threshold_);
    }
    if (filter_range_min_depth_ >= 0) {
      params.has_range = true;
      params.range.min_depth = static_cast<uint16_t>(filter_range_min_depth_);
    }

    if (params.has_confidence || params.has_range) {
      LogInfo("Applying device filter params: confidence=%s(%d) range=%s(min=%d)",
              params.has_confidence ? "yes" : "no", filter_confidence_threshold_,
              params.has_range      ? "yes" : "no", filter_range_min_depth_);
      sdk::OdinResult result = sdk::SetFilterParam(device_handle_, params);
      if (result != sdk::OdinResult::kOk) {
        LogWarn("SetFilterParam failed: %d", static_cast<int>(result));
      } else {
        LogInfo("SetFilterParam OK");
        applied_any = true;
      }
    }

    if (!applied_any) {
      LogInfo("Device filter params: all defaults, skipping");
    }
  }

  std::vector<sdk::OdinSensorCapability> QueryAndPrintSensorCapabilities() {
    std::vector<sdk::OdinSensorCapability> capabilities;
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      return capabilities;
    }
    auto result = sdk::GetSensorCapability(device_handle_, capabilities, {}, 2000);
    if (result != sdk::OdinResult::kOk) {
      LogWarn("Failed to query sensor capabilities, result=%d", static_cast<int>(result));
      return capabilities;
    }

    // Helper lambda to convert channel to string
    auto channelToString = [](sdk::OdinDataChannel ch) -> std::string {
      auto lookup = [](sdk::OdinDataChannel c) -> const char* {
        switch (c) {
          case sdk::OdinDataChannel::kRawPoint:      return "RawPoint";
          case sdk::OdinDataChannel::kSlamPoint:     return "SlamPoint";
          case sdk::OdinDataChannel::kImage0:        return "Image0 (Left)";
          case sdk::OdinDataChannel::kImage1:        return "Image1 (Right)";
          case sdk::OdinDataChannel::kImu:           return "IMU";
          case sdk::OdinDataChannel::kOdomLF:        return "OdomLF";
          case sdk::OdinDataChannel::kOdomHF:        return "OdomHF";
          case sdk::OdinDataChannel::kAlgoDownlink:  return "AlgoDownlink";
          case sdk::OdinDataChannel::kCalibFile:     return "CalibFile";
          case sdk::OdinDataChannel::kTelemetry:     return "Telemetry";
          case sdk::OdinDataChannel::kTf:        return "Tf";
          default: return nullptr;
        }
      }(ch);
      if (lookup != nullptr) {
        return lookup;
      }
      char buf[32];
      snprintf(buf, sizeof(buf), "Unknown(0x%02X)", static_cast<uint8_t>(ch));
      return buf;
    };

    // Helper lambda to convert format to string
    // Image formats use bitmask — decode individual bits.
    auto formatToString = [](sdk::OdinDataFormat fmt) -> std::string {
      uint8_t v = static_cast<uint8_t>(fmt);
      // Image formats: bitmask (0x01|0x02|0x04|0x08)
      if (v > 0 && v < 0x10) {
        std::string result;
        if (v & 0x01) result += (result.empty() ? "" : "|") + std::string("NV12");
        if (v & 0x02) result += (result.empty() ? "" : "|") + std::string("MJPEG");
        if (v & 0x04) result += (result.empty() ? "" : "|") + std::string("H264");
        if (v & 0x08) result += (result.empty() ? "" : "|") + std::string("H265");
        if (!result.empty() && (v & ~uint8_t(0x0F)) == 0) return result;
      }
      switch (fmt) {
        case sdk::OdinDataFormat::kImageNv12:        return "NV12";
        case sdk::OdinDataFormat::kImageMjpeg:      return "MJPEG";
        case sdk::OdinDataFormat::kImageH264:       return "H264";
        case sdk::OdinDataFormat::kImageH265:       return "H265";
        case sdk::OdinDataFormat::kRawPointXyzic:   return "XYZIC";
        case sdk::OdinDataFormat::kSlamPointXyzrgba: return "XYZRGBA";
        case sdk::OdinDataFormat::kImu6Axis:        return "6-Axis";
        case sdk::OdinDataFormat::kOdomStandard:    return "Standard";
        case sdk::OdinDataFormat::kTfStandard:      return "TF-Standard";
        default: {
          char buf[32];
          snprintf(buf, sizeof(buf), "Unknown(0x%02X)", v);
          return buf;
        }
      }
    };

    // Print formatted capabilities
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║                    SENSOR CAPABILITIES                           ║\n");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");

    for (const auto& cap : capabilities) {
      printf("║  Channel: %-20s                                 ║\n", channelToString(cap.channel).c_str());
      printf("║  ┌────────────────────────────────────────────────────────────┐ ║\n");
      printf("║  │  Mode   Width   Height    FPS    Format                   │ ║\n");
      printf("║  ├────────────────────────────────────────────────────────────┤ ║\n");

      int mode_idx = 0;
      for (const auto& cfg : cap.modes) {
        char width_str[16], height_str[16], fps_str[16];
        if (cfg.width > 0) {
          snprintf(width_str, sizeof(width_str), "%5u", cfg.width);
        } else {
          snprintf(width_str, sizeof(width_str), "  -  ");
        }
        if (cfg.height > 0) {
          snprintf(height_str, sizeof(height_str), "%5u", cfg.height);
        } else {
          snprintf(height_str, sizeof(height_str), "  -  ");
        }
        if (cfg.fps > 0) {
          snprintf(fps_str, sizeof(fps_str), "%3u", cfg.fps);
        } else {
          snprintf(fps_str, sizeof(fps_str), " - ");
        }

        printf("║  │  [%2d]  %s   %s    %s    %-20s │ ║\n",
               mode_idx++, width_str, height_str, fps_str, formatToString(cfg.format).c_str());
      }

      if (cap.modes.empty()) {
        printf("║  │  (No configurations available)                            │ ║\n");
      }

      printf("║  └────────────────────────────────────────────────────────────┘ ║\n");
      printf("║                                                                  ║\n");
    }

    if (capabilities.empty()) {
      printf("║  (No sensor capabilities reported by device)                    ║\n");
    }

    printf("╚══════════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    LogInfo("Sensor capabilities query complete: %zu channels", capabilities.size());
    return capabilities;
  }

  void RequestStandbyMode() {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      return;
    }
    if (sdk::SetDeviceMode(device_handle_, sdk::OdinDeviceMode::kStandby) != sdk::OdinResult::kOk) {
      LogWarn("Failed to send standby mode command during shutdown.");
    } else {
      LogInfo("Requested standby mode before disconnect.");
    }
  }

  // Optional progress logger for HTTP fallback path (called with progress 0..99).
  // Not called on the 0x41 path since that channel has no incremental progress.
  static void CalibProgressCallback(sdk::OdinDeviceHandle device, float progress,
                                     void* /*client_data*/) {
    (void)device;
    if (progress >= 0.0f && progress < 100.0f) {
      printf("[odin_driver] Calibration download progress: %.1f%%\n", progress);
    }
  }

  void BuildCalibFilePath() {
    std::string pkg_root = GetPackageRoot();
    std::string filename = device_sn_.empty()
        ? "camera_calib.yaml"
        : ("camera_calib_" + device_sn_ + ".yaml");

    // Primary save location: /tmp (always writable, no dependency on source tree)
    calib_file_path_ = "/tmp/" + filename;
    LogInfo("Calibration file path: %s", calib_file_path_.c_str());

    // Backup location: {pkg_root}/config/ if the directory exists
    if (!pkg_root.empty()) {
      std::string config_dir = pkg_root + "/config";
      struct stat st;
      if (stat(config_dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
        calib_backup_path_ = config_dir + "/" + filename;
        LogInfo("Calibration backup path: %s", calib_backup_path_.c_str());
      } else {
        calib_backup_path_.clear();
        LogInfo("Config dir %s not found, backup disabled", config_dir.c_str());
      }
    } else {
      calib_backup_path_.clear();
      LogWarn("Could not locate package root, backup disabled.");
    }
  }

  void BuildTopicPrefix() {
    // Generate topic_prefix from the device index only — the device MODEL must
    // not appear in topic/service paths, so that changing or mixing device
    // models never changes the ROS API. Model/SN/IP are announced via the
    // /{prefix}/driver/device_online topic instead.
    // Format: /{topic_prefix_base}/device{index}/ or /device{index}/ if prefix is empty
    // topic_prefix_base is configurable via launch parameter (default: "manifold")

    // Build prefix part: "/prefix" or "" if empty
    std::string prefix_part =
        topic_prefix_base_.empty() ? "" : ("/" + SanitizeTopicComponent(topic_prefix_base_));

    if (device_index_ < 0) {
      // Use default name when there is a single device or no index specified
      topic_prefix_ = prefix_part + "/device/";
    } else {
      topic_prefix_ = prefix_part + "/device" + std::to_string(device_index_) + "/";
    }
    LogInfo("Topic prefix: %s (SN: %s, Model: %s, IP: %s, MAC: %s, HostIP: %s, DHCP: %d, PTP: %d)",
            topic_prefix_.c_str(), device_sn_.c_str(), device_model_.c_str(),
            discovered_device_.network.ip_address.c_str(), discovered_device_.network.mac.c_str(),
            discovered_device_.host_ip.c_str(),
            discovered_device_.network.dhcp ? 1 : 0, discovered_device_.network.ptp ? 1 : 0);

    // Add device prefix to all frame_ids for multi-device support
    // Format: device{index}/odom, device{index}/base_link, device{index}/map
    std::string frame_prefix = (device_index_ < 0) ? "device/" : "device" + std::to_string(device_index_) + "/";
    frame_prefix_ = frame_prefix;
    raw_frame_id_ = frame_prefix + raw_frame_id_;
    slam_frame_id_ = frame_prefix + slam_frame_id_;
    image_frame_id_ = frame_prefix + image_frame_id_;
    imu_frame_id_ = frame_prefix + imu_frame_id_;
    odom_frame_id_ = frame_prefix + odom_frame_id_;
  }

  void PublishTopicPrefix() {
    // Note: Device topic prefix is included in device_online JSON ("prefix" field),
    // so a separate /driver/topic_prefix topic is not needed.
    // The topic is named "device_online" to be semantically paired with "device_offline".
#if defined(ODIN_ROS2)
    // Publish device online event as JSON for multi-device support
    if (!device_online_pub_) {
      device_online_pub_ = node_->create_publisher<std_msgs::msg::String>(
          BuildDriverTopicName("device_online"), rclcpp::QoS(1).transient_local());
    }
    std_msgs::msg::String info_msg;
    info_msg.data = BuildDeviceInfoJson();
    device_online_pub_->publish(info_msg);
    LogInfo("Published device_online: %s", info_msg.data.c_str());
#else
    // Publish device online event as JSON for multi-device support
    // Use queue size 10 for multi-device support (latched messages from multiple publishers)
    if (!device_online_pub_) {
      device_online_pub_ = nh_.advertise<std_msgs::String>(BuildDriverTopicName("device_online"), 100, true);
    }
    std_msgs::String info_msg;
    info_msg.data = BuildDeviceInfoJson();
    device_online_pub_.publish(info_msg);
    LogInfo("Published device_online: %s", info_msg.data.c_str());
#endif
  }

  std::string BuildDeviceInfoJson() {
    // Format: {"sn":"xxx","model":"Odin2Special","prefix":"/manifold/device0/","ip":"192.168.1.251"}
    std::ostringstream oss;
    oss << "{\"sn\":\"" << device_sn_ << "\","
        << "\"model\":\"" << device_model_ << "\","
        << "\"prefix\":\"" << topic_prefix_ << "\","
        << "\"ip\":\"" << discovered_device_.network.ip_address << "\"}";
    return oss.str();
  }

  void FetchCalibrationFile() {
    if (calib_file_path_.empty()) {
      LogWarn("Calibration file path not configured, skipping fetch.");
      return;
    }
    LogInfo("Fetching calibration file from device...");
    LogInfo("  Save path: %s", calib_file_path_.c_str());

    // GetDeviceFile is synchronous/blocking: returns only after the file has
    // been saved to calib_file_path_ (or the transfer has failed/timed out).
    sdk::OdinResult result =
        sdk::GetDeviceFile(device_handle_, CalibProgressCallback,
                           sdk::OdinFileType::kCalibrationFile, calib_file_path_, 5000);

    if (result != sdk::OdinResult::kOk) {
      LogError(
          "Failed to fetch calibration file (error %d). Calibration service will not "
          "be available.",
          static_cast<int>(result));
      calib_ready_ = false;
      return;
    }

    LogInfo("Calibration file fetched successfully: %s", calib_file_path_.c_str());
    std::ifstream file(calib_file_path_);
    if (file.is_open()) {
      std::stringstream buffer;
      buffer << file.rdbuf();
      calib_yaml_content_ = buffer.str();
      calib_ready_ = true;
      LogInfo("Calibration content loaded (%zu bytes)", calib_yaml_content_.size());
      // Backup to config dir if available
      if (!calib_backup_path_.empty()) {
        std::ofstream backup(calib_backup_path_, std::ios::binary);
        if (backup.is_open()) {
          backup << calib_yaml_content_;
          backup.close();
          LogInfo("Calibration backed up to: %s", calib_backup_path_.c_str());
        } else {
          LogWarn("Failed to write calibration backup to: %s", calib_backup_path_.c_str());
        }
      }
    } else {
      LogError("Failed to read calibration file after download: %s", calib_file_path_.c_str());
      calib_ready_ = false;
    }
  }

  // Shared (ROS1/ROS2) loop-closure map operations. Fill the response triplet.
  void RunSaveMap(const std::string &path, bool &success, int32_t &error_code,
                  std::string &message) {
    if (path.empty()) {
      success = false;
      error_code = static_cast<int32_t>(sdk::OdinResult::kInvalidArgument);
      message = "Empty map path";
      LogWarn("SaveMap service: empty path");
      return;
    }
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      success = false;
      error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      message = "Device not connected";
      LogWarn("SaveMap service: device not connected");
      return;
    }
    sdk::OdinResult result = sdk::SaveMap(device_handle_, path);
    error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      success = true;
      message = "Map saved to: " + path;
      LogInfo("SaveMap service: saved to %s", path.c_str());
    } else {
      success = false;
      message = "SDK error: " + std::to_string(static_cast<int>(result));
      LogWarn("SaveMap service: SDK returned error %d", static_cast<int>(result));
    }
  }

  void RunLoadMap(const std::string &path, bool &success, int32_t &error_code,
                  std::string &message) {
    if (path.empty()) {
      success = false;
      error_code = static_cast<int32_t>(sdk::OdinResult::kInvalidArgument);
      message = "Empty map path";
      LogWarn("LoadMap service: empty path");
      return;
    }
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      success = false;
      error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      message = "Device not connected";
      LogWarn("LoadMap service: device not connected");
      return;
    }
    sdk::OdinResult result = sdk::LoadMap(device_handle_, path);
    error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      success = true;
      message = "Map loaded from: " + path;
      LogInfo("LoadMap service: loaded from %s", path.c_str());
    } else {
      success = false;
      message = "SDK error: " + std::to_string(static_cast<int>(result));
      LogWarn("LoadMap service: SDK returned error %d", static_cast<int>(result));
    }
  }

  void RunExployMap(const std::string &path, bool &success, int32_t &error_code,
                    std::string &message) {
    if (path.empty()) {
      success = false;
      error_code = static_cast<int32_t>(sdk::OdinResult::kInvalidArgument);
      message = "Empty map path";
      LogWarn("ExployMap service: empty path");
      return;
    }
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      success = false;
      error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      message = "Device not connected";
      LogWarn("ExployMap service: device not connected");
      return;
    }
    sdk::OdinResult result = sdk::ExployMap(device_handle_, path);
    error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      success = true;
      message = "Map exported to: " + path;
      LogInfo("ExployMap service: exported to %s", path.c_str());
    } else {
      success = false;
      message = "SDK error: " + std::to_string(static_cast<int>(result));
      LogWarn("ExployMap service: SDK returned error %d", static_cast<int>(result));
    }
  }

  void PublishMappingActive(sdk::OdinDeviceMode mode) {
    BoolMsg status;
    status.data = mode == sdk::OdinDeviceMode::kSlam || mode == sdk::OdinDeviceMode::kReMap;
#if defined(ODIN_ROS2)
    if (!mapping_active_pub_) {
      mapping_active_pub_ = node_->create_publisher<BoolMsg>(
          topic_prefix_ + "mapping_active", rclcpp::QoS(1).reliable().transient_local());
    }
    mapping_active_pub_->publish(status);
#else
    if (!mapping_active_pub_) {
      mapping_active_pub_ = nh_.advertise<BoolMsg>(topic_prefix_ + "mapping_active", 1, true);
    }
    mapping_active_pub_.publish(status);
#endif
  }

  // Shared (ROS1/ROS2) device operating-mode switch. Fill the response triplet.
  void RunDeviceMode(uint8_t mode, bool &success, int32_t &error_code,
                     std::string &message) {
    const char *mode_name;
    switch (mode) {
      case 0: mode_name = "Standby"; break;
      case 1: mode_name = "Normal"; break;
      case 2: mode_name = "Upgrade"; break;
      case 3: mode_name = "SLAM"; break;
      case 4: mode_name = "Relocation"; break;
      case 5: mode_name = "ReMap"; break;
      default:
        success = false;
        error_code = static_cast<int32_t>(sdk::OdinResult::kInvalidArgument);
        message = "Invalid mode (0=Standby 1=Normal 2=Upgrade 3=SLAM 4=Relocation 5=ReMap)";
        LogWarn("DeviceMode service: invalid mode %d", mode);
        return;
    }
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      success = false;
      error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      message = "Device not connected";
      LogWarn("DeviceMode service: device not connected");
      return;
    }
    sdk::OdinResult result = sdk::SetDeviceMode(
        device_handle_, static_cast<sdk::OdinDeviceMode>(mode));
    error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      desired_mode_ = static_cast<sdk::OdinDeviceMode>(mode);
      PublishMappingActive(desired_mode_);
      success = true;
      message = mode_name;
      LogInfo("DeviceMode service: set to %s (%d)", mode_name, mode);
    } else {
      success = false;
      message = "SDK error: " + std::to_string(static_cast<int>(result));
      LogWarn("DeviceMode service: SDK returned error %d", static_cast<int>(result));
    }
  }

#if !defined(ODIN_ROS2)
  bool HandleCalibrationService(odin_driver_interfaces::srv::GetCalibration::Request &req,
                                odin_driver_interfaces::srv::GetCalibration::Response &res) {
    (void)req;
    if (calib_ready_ && !calib_yaml_content_.empty()) {
      res.success = true;
      res.yaml_content = calib_yaml_content_;
      res.message = "Calibration data retrieved successfully";
      LogInfo("Calibration service: sent %zu bytes", calib_yaml_content_.size());
    } else {
      res.success = false;
      res.yaml_content = "";
      res.message = "Calibration data not ready yet";
      LogWarn("Calibration service: data not ready");
    }
    return true;
  }

  bool HandleSensorModeService(odin_ros_driver_rev1::SetSensorMode::Request &req,
                               odin_ros_driver_rev1::SetSensorMode::Response &res) {
    if (req.mode > static_cast<int>(sdk::OdinSensorMode::kAdaptive)) {
      res.success = false;
      res.error_code = static_cast<int32_t>(sdk::OdinResult::kInvalidArgument);
      res.message = "Invalid mode (0=HDR 1=HighPeak 2=LowPeak 3=Adaptive)";
      LogWarn("SetSensorMode service: invalid mode %d", req.mode);
      return true;
    }
    auto mode = IntToSensorMode(req.mode);
    const char *mode_name = SensorModeName(mode);
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      res.success = false;
      res.error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      res.message = "Device not connected";
      LogWarn("SetSensorMode service: device not connected");
      return true;
    }
    sdk::OdinResult result = sdk::SetSensorMode(device_handle_, mode);
    res.error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      sensor_mode_ = mode;
      res.success = true;
      res.message = mode_name;
      LogInfo("SetSensorMode service: set to %s (%d)", mode_name, static_cast<int>(sensor_mode_));
    } else {
      res.success = false;
      res.message = "SDK error: " + std::to_string(static_cast<int>(result));
      LogWarn("SetSensorMode service: SDK returned error %d", static_cast<int>(result));
    }
    return true;
  }

  bool HandleResetInitialPoseService(odin_driver_interfaces::srv::ResetInitialPose::Request &req,
                                     odin_driver_interfaces::srv::ResetInitialPose::Response &res) {
    (void)req;
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      res.success = false;
      res.error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      res.message = "Device not connected";
      LogWarn("ResetInitialPose service: device not connected");
      return true;
    }
    sdk::OdinResult result = sdk::ResetInitialPose(device_handle_);
    res.error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      res.success = true;
      res.message = "Initial pose reset successfully";
      LogInfo("ResetInitialPose service: pose reset OK");
    } else {
      res.success = false;
      res.message = "SDK error: " + std::to_string(static_cast<int>(result));
      LogWarn("ResetInitialPose service: SDK returned error %d", static_cast<int>(result));
    }
    return true;
  }

  bool HandleGetAdaptiveParamService(odin_driver_interfaces::srv::GetAdaptiveParam::Request &,
                                     odin_driver_interfaces::srv::GetAdaptiveParam::Response &res) {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      res.success = false; res.error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      res.message = "Device not connected"; return true; }
    sdk::OdinAdaptiveFilterParam p;
    auto result = sdk::GetAdaptiveParam(device_handle_, p);
    res.error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      res.success = true; res.cdf_percentile = p.cdf_percentile; res.outdoor_threshold = p.outdoor_threshold; res.message = "OK";
    } else { res.success = false; res.message = "SDK error: " + std::to_string(static_cast<int>(result)); }
    return true;
  }

  bool HandleSetAdaptiveParamService(odin_driver_interfaces::srv::SetAdaptiveParam::Request &req,
                                     odin_driver_interfaces::srv::SetAdaptiveParam::Response &res) {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      res.success = false; res.error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      res.message = "Device not connected"; return true; }
    sdk::OdinAdaptiveFilterParam p(req.cdf_percentile, req.outdoor_threshold);
    auto result = sdk::SetAdaptiveParam(device_handle_, p);
    res.error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) { res.success = true; res.message = "OK"; }
    else { res.success = false; res.message = "SDK error: " + std::to_string(static_cast<int>(result)); }
    return true;
  }

  bool HandleGetSyncModeService(odin_driver_interfaces::srv::GetSyncMode::Request &,
                                odin_driver_interfaces::srv::GetSyncMode::Response &res) {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      res.success = false; res.error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      res.message = "Device not connected"; return true; }
    sdk::OdinTimeSyncMode mode;
    auto result = sdk::GetDefaultSyncMode(device_handle_, mode);
    res.error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      res.success = true; res.sync_mode = static_cast<uint8_t>(mode);
      res.message = mode == sdk::OdinTimeSyncMode::kPtp ? "PTP" : "SysTick";
    } else { res.success = false; res.message = "SDK error: " + std::to_string(static_cast<int>(result)); }
    return true;
  }

  bool HandleSetSyncModeService(odin_driver_interfaces::srv::SetSyncMode::Request &req,
                                odin_driver_interfaces::srv::SetSyncMode::Response &res) {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      res.success = false; res.error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      res.message = "Device not connected"; return true; }
    auto mode = static_cast<sdk::OdinTimeSyncMode>(req.sync_mode);
    auto result = sdk::SetDefaultSyncMode(device_handle_, mode);
    res.error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) { res.success = true; res.message = "Sync mode set (reboot required)"; }
    else { res.success = false; res.message = "SDK error: " + std::to_string(static_cast<int>(result)); }
    return true;
  }

  bool HandleGetPtpSyncStatusService(odin_driver_interfaces::srv::GetPtpSyncStatus::Request &,
                                     odin_driver_interfaces::srv::GetPtpSyncStatus::Response &res) {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      res.success = false; res.error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      res.message = "Device not connected"; return true; }
    sdk::OdinPtpSyncStatus s;
    auto result = sdk::GetPtpSyncStatus(device_handle_, s);
    res.error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) {
      res.success = true; res.sync_state = s.sync_state; res.servo_state = s.servo_state;
      res.offset_ns = s.offset_ns; res.path_delay_ns = s.path_delay_ns; res.freq_ppb = s.freq_ppb; res.message = "OK";
    } else { res.success = false; res.message = "SDK error: " + std::to_string(static_cast<int>(result)); }
    return true;
  }

  bool HandleRestoreDefaultConfigService(odin_driver_interfaces::srv::RestoreDefaultConfig::Request &,
                                         odin_driver_interfaces::srv::RestoreDefaultConfig::Response &res) {
    if (!connected_ || device_handle_ == sdk::kInvalidDeviceHandle) {
      res.success = false; res.error_code = static_cast<int32_t>(sdk::OdinResult::kNotInitialized);
      res.message = "Device not connected"; return true; }
    auto result = sdk::RestoreDefaultConfig(device_handle_);
    res.error_code = static_cast<int32_t>(result);
    if (result == sdk::OdinResult::kOk) { res.success = true; res.message = "Default config restored (reboot required)"; }
    else { res.success = false; res.message = "SDK error: " + std::to_string(static_cast<int>(result)); }
    return true;
  }

  bool HandleSaveMapService(odin_driver_interfaces::srv::SaveMap::Request &req,
                            odin_driver_interfaces::srv::SaveMap::Response &res) {
    // ROS1 generates `bool` srv fields as uint8_t, which cannot bind to bool&.
    bool success = false;
    RunSaveMap(req.path, success, res.error_code, res.message);
    res.success = success;
    return true;
  }

  bool HandleLoadMapService(odin_driver_interfaces::srv::LoadMap::Request &req,
                            odin_driver_interfaces::srv::LoadMap::Response &res) {
    // ROS1 generates `bool` srv fields as uint8_t, which cannot bind to bool&.
    bool success = false;
    RunLoadMap(req.path, success, res.error_code, res.message);
    res.success = success;
    return true;
  }

  bool HandleExployMapService(odin_driver_interfaces::srv::ExployMap::Request &req,
                              odin_driver_interfaces::srv::ExployMap::Response &res) {
    bool success = false;
    RunExployMap(req.path, success, res.error_code, res.message);
    res.success = success;
    return true;
  }

  bool HandleDeviceModeService(odin_driver_interfaces::srv::DeviceMode::Request &req,
                               odin_driver_interfaces::srv::DeviceMode::Response &res) {
    // ROS1 generates `bool` srv fields as uint8_t, which cannot bind to bool&.
    bool success = false;
    RunDeviceMode(req.mode, success, res.error_code, res.message);
    res.success = success;
    return true;
  }

#endif

  void Shutdown() {
    // Set shutting_down_ flag first so timer callbacks can bail out early.
    shutting_down_.store(true);
    diag_enabled_.store(false);

#if defined(ODIN_ROS2)
    // Reset timers before anything else to prevent new callback dispatch.
    // The executor may still have an in-flight callback; shutdown_mutex_
    // below waits for it to complete before we touch any member.
    tf_republish_timer_.reset();
    diag_timer_.reset();
#endif

    // Wait for any in-flight timer callbacks to complete.
    // Timer callbacks hold shutdown_mutex_ while executing; acquiring it
    // here guarantees they have returned before we destroy members.
    std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);

    // Stop async queues first to prevent callbacks during disconnect
    if (raw_point_queue_) raw_point_queue_->Stop();
    if (slam_point_queue_) slam_point_queue_->Stop();
    if (image0_queue_) image0_queue_->Stop();
    if (imu_queue_) imu_queue_->Stop();
    if (odom_lf_queue_) odom_lf_queue_->Stop();
    if (odom_hf_queue_) odom_hf_queue_->Stop();
    if (tf_queue_) tf_queue_->Stop();
    if (lc_optimization_status_queue_) lc_optimization_status_queue_->Stop();
    if (lc_frame_queue_) lc_frame_queue_->Stop();
    if (lc_optimization_result_queue_) lc_optimization_result_queue_->Stop();
    if (lc_map_cloud_queue_) lc_map_cloud_queue_->Stop();

#if !defined(ODIN_ROS2)
    // ROS1 timers are reset after queue stop (no executor race in ROS1).
    tf_republish_timer_ = ros::Timer();
    diag_timer_ = ros::Timer();
#endif

    if (connected_ && device_handle_ != sdk::kInvalidDeviceHandle) {
      if (!device_removed_.load()) {
        RequestStandbyMode();
      }
      sdk::DisconnectDevice(device_handle_);
      connected_ = false;
      device_handle_ = sdk::kInvalidDeviceHandle;
    }
  }

  // Wire protocol v2: `timestamp` is in nanoseconds (raw hardware monotonic clock).
  RosTime ToRosTime(uint64_t timestamp) const {
#if defined(ODIN_ROS2)
    if (!node_) {
      return RosTime(0);
    }
    if (timestamp == 0) {
      return node_->now();
    }
    // rclcpp::Time(int64_t) takes nanoseconds directly.
    return RosTime(static_cast<int64_t>(timestamp));
#else
    if (timestamp == 0) {
      return ros::Time::now();
    }
    constexpr uint64_t kNanosPerSec = 1000000000ULL;
    ros::Time t;
    t.sec = static_cast<uint32_t>(timestamp / kNanosPerSec);
    t.nsec = static_cast<uint32_t>(timestamp % kNanosPerSec);
    return t;
#endif
  }

  PointCloud2Msg BuildLcCloudMsg(const std::vector<sdk::OdinLcPointXYZ>& points,
                                 uint64_t timestamp,
                                 const std::string& frame_id) {
    PointCloud2Msg msg;
    msg.header.stamp = ToRosTime(timestamp);
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = PointFieldMsg::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = sizeof(float);
    msg.fields[1].datatype = PointFieldMsg::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = sizeof(float) * 2;
    msg.fields[2].datatype = PointFieldMsg::FLOAT32;
    msg.fields[2].count = 1;
    msg.point_step = sizeof(float) * 3;
    msg.row_step = msg.width * msg.point_step;
    msg.data.resize(msg.row_step);
    for (size_t i = 0; i < points.size(); ++i) {
      uint8_t* dst = msg.data.data() + i * msg.point_step;
      std::memcpy(dst + 0, &points[i].x, sizeof(float));
      std::memcpy(dst + sizeof(float), &points[i].y, sizeof(float));
      std::memcpy(dst + sizeof(float) * 2, &points[i].z, sizeof(float));
    }
    return msg;
  }

  PoseStampedMsg BuildLcPoseMsg(const sdk::OdinLcPoseStamped& pose, uint64_t fallback_ts) {
    PoseStampedMsg msg;
    msg.header.stamp = ToRosTime(pose.timestamp != 0 ? pose.timestamp : fallback_ts);
    msg.header.frame_id = odom_frame_id_;
    msg.pose.position.x = pose.translation[0];
    msg.pose.position.y = pose.translation[1];
    msg.pose.position.z = pose.translation[2];
    msg.pose.orientation.x = pose.rotation[0];
    msg.pose.orientation.y = pose.rotation[1];
    msg.pose.orientation.z = pose.rotation[2];
    msg.pose.orientation.w = pose.rotation[3];
    NormalizeQuaternion(msg.pose.orientation);
    return msg;
  }

  void ProcessLcFrame(const sdk::OdinLcFramePacket& packet) {
    sdk::OdinLcPoseStamped pose;
    pose.timestamp = packet.timestamp;
    std::memcpy(pose.translation, packet.translation, sizeof(pose.translation));
    std::memcpy(pose.rotation, packet.rotation, sizeof(pose.rotation));
    PublishMessage(lc_corrected_pose_pub_, BuildLcPoseMsg(pose, packet.timestamp));
    if (packet.is_keyframe && !packet.cloud.empty()) {
      PublishMessage(lc_corrected_cloud_pub_,
                     BuildLcCloudMsg(packet.cloud, packet.timestamp, odom_frame_id_));
    }
  }

  void ProcessLcOptimizationStatus(const sdk::OdinLcOptimizationStatusPacket& packet) {
    BoolMsg status;
    status.data = packet.busy;
    PublishMessage(lc_optimization_busy_pub_, status);

    MarkerMsg marker;
    marker.header.stamp = ToRosTime(packet.timestamp);
    marker.header.frame_id = odom_frame_id_;
    marker.ns = "loop_closure";
    marker.id = 1;
    marker.type = MarkerMsg::TEXT_VIEW_FACING;
    marker.action = MarkerMsg::ADD;
    marker.pose.position.z = 1.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.4;
    marker.color.a = 1.0;
    marker.color.r = packet.busy ? 1.0 : 0.0;
    marker.color.g = packet.busy ? 0.6 : 1.0;
    marker.color.b = 0.0;
    marker.text = packet.busy ? "OPTIMIZING" : "IDLE";
    PublishMessage(lc_optimization_status_marker_pub_, marker);
  }

  void ProcessLcOptimizationResult(const sdk::OdinLcOptimizationResultPacket& packet) {
    if (!packet.cloud.empty()) {
      PublishMessage(lc_optimized_cloud_pub_,
                     BuildLcCloudMsg(packet.cloud, packet.timestamp, odom_frame_id_));
    }
    if (!packet.poses.empty()) {
      PathMsg path;
      path.header.stamp = ToRosTime(packet.timestamp);
      path.header.frame_id = odom_frame_id_;
      path.poses.reserve(packet.poses.size());
      for (const auto& pose : packet.poses) {
        path.poses.push_back(BuildLcPoseMsg(pose, packet.timestamp));
      }
      PublishMessage(lc_optimized_path_pub_, path);
    }
  }

  void ProcessLcMapCloud(const sdk::OdinLcMapCloudPacket& packet) {
    if (!packet.points.empty()) {
      // 与 EmitRelocAnchorAsTf 发出的 map->odom TF 一致：重定位成功后 TF 树里
      // 存在的是 frame_prefix_ + "map"（如 device0/map），裸 "map" 没有对应
      // TF 帧，RViz 在 Fixed Frame=device0/odom 下会找不到变换而不渲染。
      PublishMessage(lc_map_cloud_pub_, BuildLcCloudMsg(packet.points, packet.timestamp, frame_prefix_ + "map"));
    }
  }

  // Process functions - called by async queue worker threads
  void ProcessRawPointCloud(const sdk::OdinPointCloudPacket &packet) {
    if (packet.points.empty()) {
      if (ShouldThrottle("raw_points_empty", 5.0)) {
        LogDebug("Received raw point cloud packet with empty payload.");
      }
      return;
    }
    const size_t point_count = packet.points.size();

    // Rolling shutter scan parameters:
    // - 256: points per row (h_vld_seg * 16 = 16 * 16)
    // - 6: rows per group (v_pxl_outNum), captured simultaneously
    // - 32 groups total (v_roll_num), subframe_odr = 328.5 Hz
    constexpr int kPointsPerRow = 256;
    constexpr int kRowsPerGroup = 6;
    constexpr int kPointsPerGroup = kPointsPerRow * kRowsPerGroup;  // 1536 points
    constexpr float kSubframeInterval = 1.0f / 328.5f;              // ~3.044 ms between groups

    PointCloud2Msg msg;
    msg.header.stamp = ToRosTime(packet.timestamp);
    msg.header.frame_id = raw_frame_id_;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(point_count);
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.fields.resize(6);

    constexpr uint32_t kOffsetX = 0;
    constexpr uint32_t kOffsetY = sizeof(float);
    constexpr uint32_t kOffsetZ = sizeof(float) * 2;
    constexpr uint32_t kOffsetIntensity = sizeof(float) * 3;
    constexpr uint32_t kOffsetConfidence = sizeof(float) * 3 + 1;
    constexpr uint32_t kOffsetTime = sizeof(float) * 3 + 2;
    constexpr uint32_t kPointStep = sizeof(float) * 4 + 2;

    msg.fields[0].name = "x";
    msg.fields[0].offset = kOffsetX;
    msg.fields[0].datatype = PointFieldMsg::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = kOffsetY;
    msg.fields[1].datatype = PointFieldMsg::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = kOffsetZ;
    msg.fields[2].datatype = PointFieldMsg::FLOAT32;
    msg.fields[2].count = 1;
    msg.fields[3].name = "intensity";
    msg.fields[3].offset = kOffsetIntensity;
    msg.fields[3].datatype = PointFieldMsg::UINT8;
    msg.fields[3].count = 1;
    msg.fields[4].name = "confidence";
    msg.fields[4].offset = kOffsetConfidence;
    msg.fields[4].datatype = PointFieldMsg::UINT8;
    msg.fields[4].count = 1;
    msg.fields[5].name = "offset_time";
    msg.fields[5].offset = kOffsetTime;
    msg.fields[5].datatype = PointFieldMsg::FLOAT32;
    msg.fields[5].count = 1;

    msg.point_step = kPointStep;
    msg.row_step = msg.width * msg.point_step;
    msg.data.resize(msg.row_step);

    uint8_t *dst = msg.data.data();
    for (size_t i = 0; i < point_count; ++i) {
      const int group_idx = static_cast<int>(i) / kPointsPerGroup;
      const float offset_time = group_idx * kSubframeInterval;

      float x = packet.points[i].x * 0.001f;
      float y = packet.points[i].y * 0.001f;
      float z = packet.points[i].z * 0.001f;
      // Client-side confidence filter. For Adaptive mode m_confidence_threshold
      // is forced to 0 (device handles filtering), so the condition is always false
      // and all points pass through.
      if (static_cast<float>(m_confidence_threshold) > packet.points[i].confidence) {
        x = y = z = 0.0f;
      }

      // Client-side row filter: zero out points whose row index (i / 256)
      // is below min_row_. This crops the lower rows of the 256x192 image.
      if (enable_row_filter_ && static_cast<int>(i) / kPointsPerRow < min_row_) {
        x = y = z = 0.0f;
      }

      std::memcpy(dst + kOffsetX, &x, sizeof(float));
      std::memcpy(dst + kOffsetY, &y, sizeof(float));
      std::memcpy(dst + kOffsetZ, &z, sizeof(float));
      std::memcpy(dst + kOffsetIntensity, &packet.points[i].intensity, sizeof(uint8_t));
      std::memcpy(dst + kOffsetConfidence, &packet.points[i].confidence, sizeof(uint8_t));
      std::memcpy(dst + kOffsetTime, &offset_time, sizeof(float));

      dst += kPointStep;
    }

    PublishMessage(raw_cloud_pub_, msg);
  }

  void ProcessSlamPointCloud(const sdk::OdinSlamPacket &packet) {
    // SLAM points are now stored as float xyz after transformation in SDK
    if (packet.points.empty()) {
      if (ShouldThrottle("slam_points_empty", 5.0)) {
        LogDebug("Received slam point cloud packet with empty payload.");
      }
      return;
    }
    const size_t point_count = packet.points.size();

    PointCloud2Msg msg;
    msg.header.stamp = ToRosTime(packet.timestamp);
    msg.header.frame_id = slam_frame_id_;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(point_count);
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.fields.resize(4);

    auto pclType = PointFieldMsg::FLOAT32;
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = pclType;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = sizeof(float);
    msg.fields[1].datatype = pclType;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = sizeof(float) * 2;
    msg.fields[2].datatype = pclType;
    msg.fields[2].count = 1;

    msg.fields[3].name = "rgb";
    msg.fields[3].offset = sizeof(float) * 3;
    msg.fields[3].datatype = pclType;
    msg.fields[3].count = 1;

    msg.point_step = sizeof(sdk::OdinSlamPoint<float>);
    msg.row_step = msg.width * msg.point_step;
    msg.data.resize(msg.row_step);
    uint8_t *dst = msg.data.data();
    for (size_t i = 0; i < point_count; ++i) {
      // xyz from SDK in mm, convert to meters for ROS
      const float x = packet.points[i].x * 0.001f;
      const float y = packet.points[i].y * 0.001f;
      const float z = packet.points[i].z * 0.001f;
      std::memcpy(dst + 0, &x, sizeof(float));
      std::memcpy(dst + sizeof(float), &y, sizeof(float));
      std::memcpy(dst + sizeof(float) * 2, &z, sizeof(float));
      const uint32_t rgba =
          (static_cast<uint32_t>(packet.points[i].a) << 24) | (static_cast<uint32_t>(packet.points[i].r) << 16) |
          (static_cast<uint32_t>(packet.points[i].g) << 8) | static_cast<uint32_t>(packet.points[i].b);
      float rgb_packed = 0.0f;
      std::memcpy(&rgb_packed, &rgba, sizeof(uint32_t));
      std::memcpy(dst + sizeof(float) * 3, &rgb_packed, sizeof(float));
      dst += msg.point_step;
    }

    PublishMessage(slam_cloud_pub_, msg);
  }

  // SOI (0xFFD8) head，EOI (0xFFD9) tail
  bool IsJpegComplete(const std::vector<uint8_t> &data) {
    if (data.size() < 4) return false;
    // Check SOI (Start of Image)
    if (data[0] != 0xFF || data[1] != 0xD8) {
      LogWarn("no header\n");
      return false;
    }
    // EOI (End of Image)
    if (data[data.size() - 2] != 0xFF || data[data.size() - 1] != 0xD9) {
      // LogWarn("no tail,data[data.size() - 2:%02x  data[data.size() - 1]:%02x\n",data[data.size()
      // - 2], data[data.size() - 1]);
      return false;
    }
    return true;
  }

  void ProcessImage(const sdk::OdinCameraPacket &packet) {
    // H264 path: publish Annex-B bitstream on the same camera0/compressed topic.
    // CompressedImage.format = "h264" lets subscribers distinguish from MJPEG
    // (format = "jpeg") without requiring a separate topic.
    // width/height are populated by the SDK from OdinStreamCfg
    // (stream_width/stream_height captured by the StartStream lambda).
    if (packet.format == sdk::OdinDataFormat::kImageH264) {
      CompressedImageMsg msg;
      msg.header.stamp    = ToRosTime(packet.timestamp);
      msg.header.frame_id = image_frame_id_;
      msg.format = "h264";  // H.264 Annex-B bitstream (ITU-T H.264)
      msg.data.assign(packet.payload.begin(), packet.payload.end());
      PublishMessage(image_pub_, msg);
      // Report resolution on the first frame; width/height come from StartStream cfg.
      if (!resolution_sent_ &&
          packet.width > 0 && packet.height > 0) {
        int w = static_cast<int>(packet.width);
        int h = static_cast<int>(packet.height);
        LogInfo("H264 stream resolution (camera0): %dx%d", w, h);
#if defined(ODIN_ROS2)
        if (!resolution_change_pub_) {
          resolution_change_pub_ = node_->create_publisher<std_msgs::msg::String>(
              BuildDriverTopicName("resolution_change"),
              rclcpp::QoS(1).transient_local());
        }
        std_msgs::msg::String res_msg;
#else
        if (!resolution_change_pub_) {
          resolution_change_pub_ =
              nh_.advertise<std_msgs::String>(BuildDriverTopicName("resolution_change"), 1, true);
        }
        std_msgs::String res_msg;
#endif
        res_msg.data = "{\"sn\":\"" + device_sn_ + "\",\"width\":" + std::to_string(w) +
                       ",\"height\":" + std::to_string(h) + "}";
        PublishMessage(resolution_change_pub_, res_msg);
        resolution_sent_ = true;
      }
      return;
    }
    // NV12 path: raw planar frames carry no dimensions in the byte stream, so
    // embed them in CompressedImage.format as "nv12; <width>x<height>" —
    // subscribers need both to reinterpret the flat payload. width/height are
    // populated by the SDK from OdinStreamCfg (same as H264).
    if (packet.format == sdk::OdinDataFormat::kImageNv12) {
      const size_t expected =
          static_cast<size_t>(packet.width) * packet.height * 3 / 2;
      if (packet.width == 0 || packet.height == 0 || packet.payload.size() != expected) {
        if (ShouldThrottle("nv12_invalid_cam0", 5.0)) {
          LogWarn("NV12 frame size mismatch (camera0): %zuB, expected %zuB for %ux%u, skipping",
                  packet.payload.size(), expected, packet.width, packet.height);
        }
        return;
      }
      CompressedImageMsg msg;
      msg.header.stamp    = ToRosTime(packet.timestamp);
      msg.header.frame_id = image_frame_id_;
      msg.format = "nv12; " + std::to_string(packet.width) + "x" +
                   std::to_string(packet.height);
      msg.data.assign(packet.payload.begin(), packet.payload.end());
      PublishMessage(image_pub_, msg);
      // Report resolution on the first frame; every NV12 frame is standalone.
      if (!resolution_sent_) {
        int w = static_cast<int>(packet.width);
        int h = static_cast<int>(packet.height);
        LogInfo("NV12 stream resolution (camera0): %dx%d", w, h);
#if defined(ODIN_ROS2)
        if (!resolution_change_pub_) {
          resolution_change_pub_ = node_->create_publisher<std_msgs::msg::String>(
              BuildDriverTopicName("resolution_change"),
              rclcpp::QoS(1).transient_local());
        }
        std_msgs::msg::String res_msg;
#else
        if (!resolution_change_pub_) {
          resolution_change_pub_ =
              nh_.advertise<std_msgs::String>(BuildDriverTopicName("resolution_change"), 1, true);
        }
        std_msgs::String res_msg;
#endif
        res_msg.data = "{\"sn\":\"" + device_sn_ + "\",\"width\":" + std::to_string(w) +
                       ",\"height\":" + std::to_string(h) + "}";
        PublishMessage(resolution_change_pub_, res_msg);
        resolution_sent_ = true;
      }
      return;
    }
    // MJPEG path
    if (!IsJpegComplete(packet.payload)) {
      if (ShouldThrottle("jpeg_incomplete_cam0", 5.0)) {
        LogWarn("Incomplete JPEG data (camera0), skipping publish");
      }
      return;
    }
    CompressedImageMsg msg;
    msg.header.stamp = ToRosTime(packet.timestamp);
    msg.header.frame_id = image_frame_id_;
    msg.format = "jpeg";
    msg.data.assign(packet.payload.begin(), packet.payload.end());
    PublishMessage(image_pub_, msg);

    // Publish resolution once after stream start (or restart)
    if (!resolution_sent_) {
      int w = 0, h = 0;
      if (ParseJpegResolution(packet.payload, w, h)) {
        LogInfo("Detected image resolution: %dx%d", w, h);
#if defined(ODIN_ROS2)
        if (!resolution_change_pub_) {
          // Use TRANSIENT_LOCAL so late-joining consumers (e.g. pointcloud2depth_node
          // started after the main driver) immediately receive the latest resolution
          // rather than being stuck on YAML defaults.
          resolution_change_pub_ = node_->create_publisher<std_msgs::msg::String>(
              BuildDriverTopicName("resolution_change"),
              rclcpp::QoS(1).transient_local());
        }
        std_msgs::msg::String res_msg;
#else
        if (!resolution_change_pub_) {
          // Latched (third arg = true) so late-joining subscribers get the last value.
          resolution_change_pub_ =
              nh_.advertise<std_msgs::String>(BuildDriverTopicName("resolution_change"), 1, true);
        }
        std_msgs::String res_msg;
#endif
        res_msg.data = "{\"sn\":\"" + device_sn_ + "\",\"width\":" + std::to_string(w) +
                       ",\"height\":" + std::to_string(h) + "}";
        PublishMessage(resolution_change_pub_, res_msg);
        resolution_sent_ = true;
      }
    }
  }

  void ProcessImu(const sdk::OdinImuPacket &packet) {

    if (packet.samples.empty()) {
      if (ShouldThrottle("imu_empty", 5.0)) {
        LogDebug("Received IMU packet without samples.");
      }
      return;
    }
    for (size_t i = 0; i < packet.samples.size(); ++i) {
      ImuMsg msg;
      msg.header.stamp = ToRosTime(packet.timestamp);
      msg.header.frame_id = imu_frame_id_;
      msg.orientation_covariance[0] = -1.0;
      msg.orientation_covariance[4] = 0.0;
      msg.orientation_covariance[8] = 0.0;
      // msg.angular_velocity.x = -values[i].gyro_z;
      // msg.angular_velocity.y = -values[i].gyro_y;
      // msg.angular_velocity.z = -values[i].gyro_x ;
      msg.angular_velocity.x = packet.samples[i].gyro_x;
      msg.angular_velocity.y = packet.samples[i].gyro_y;
      msg.angular_velocity.z = packet.samples[i].gyro_z;
      msg.angular_velocity_covariance[0] = 0.0;
      msg.angular_velocity_covariance[4] = 0.0;
      msg.angular_velocity_covariance[8] = 0.0;
      // msg.linear_acceleration.x = -values[i].acc_z;
      // msg.linear_acceleration.y = -values[i].acc_y;
      // msg.linear_acceleration.z = - values[i].acc_x ;
      msg.linear_acceleration.x = packet.samples[i].acc_x;
      msg.linear_acceleration.y = packet.samples[i].acc_y;
      msg.linear_acceleration.z = packet.samples[i].acc_z;
      msg.linear_acceleration_covariance[0] = 0.0;
      msg.linear_acceleration_covariance[4] = 0.0;
      msg.linear_acceleration_covariance[8] = 0.0;
      PublishMessage(imu_pub_, msg);
    }
  }

  void ProcessOdom(const sdk::OdinOdomPacket &packet, sdk::OdomSourceType odom_type) {
    // Debug: count each odom_type separately
    static std::atomic<uint32_t> count_lf{0}, count_hf{0}, count_other{0};
    switch (odom_type) {
      case sdk::OdomSourceType::kOdom2ImuLow:  ++count_lf;  break;
      case sdk::OdomSourceType::kOdom2ImuHigh: ++count_hf;  break;
      default: ++count_other; break;
    }
    // if (ShouldThrottle("odom_type_debug", 1.0)) {
    //   LogInfo("ProcessOdom stats: LF=%u, HF=%u, other=%u (type=%d, frame=%s)",
    //           count_lf.load(), count_hf.load(), count_other.load(),
    //           static_cast<int>(odom_type), odom_frame_id_.c_str());
    // }

    if (packet.data.empty()) {
      if (ShouldThrottle("odom_empty", 5.0)) {
        LogDebug("Received odom packet without samples.");
      }
      return;
    }

    for (size_t i = 0; i < packet.data.size(); ++i) {
      OdometryMsg msg;
      msg.header.stamp = ToRosTime(packet.timestamp);

      msg.pose.pose.position.x = packet.data[i].pos[0];
      msg.pose.pose.position.y = packet.data[i].pos[1];
      msg.pose.pose.position.z = packet.data[i].pos[2];

      msg.pose.pose.orientation.x = packet.data[i].orient[0];
      msg.pose.pose.orientation.y = packet.data[i].orient[1];
      msg.pose.pose.orientation.z = packet.data[i].orient[2];
      msg.pose.pose.orientation.w = packet.data[i].orient[3];
      NormalizeQuaternion(msg.pose.pose.orientation);

      msg.twist.twist.linear.x = packet.data[i].linear_velocity[0];
      msg.twist.twist.linear.y = packet.data[i].linear_velocity[1];
      msg.twist.twist.linear.z = packet.data[i].linear_velocity[2];

      msg.twist.twist.angular.x = packet.data[i].angular_velocity[0];
      msg.twist.twist.angular.y = packet.data[i].angular_velocity[1];
      msg.twist.twist.angular.z = packet.data[i].angular_velocity[2];

      // Forward the device-reported 6x6 pose covariance (row-major over
      // x, y, z, roll, pitch, yaw) — same layout as nav_msgs/Odometry's
      // pose.covariance, so pass through without reordering. Generation-2
      // firmware fills only the first 18 entries (position rows); the SDK
      // zero-fills the orientation rows, so no masking is needed here.
      // The device does not report a twist covariance; it stays at 0.
      // Debug print: %f defaults to 6 decimals and HIDES precision; %.17g is
      // the round-trip-exact format for double (float -> double is lossless,
      // so this shows every bit the device actually sent).
      for (size_t k = 0; k < 36; ++k) {  
        msg.pose.covariance[k] = static_cast<double>(packet.data[i].cov[k]);
      }
      // Dispatch by OdomSourceType (clear switch-case style):
      //   kOdom2ImuLow  (0) = STANDARD   -> odometry_lf topic + TF(odom->imu)
      //   kOdom2ImuHigh (1) = HIGHFREQ   -> odometry_hf topic only (odom-imu)
      switch (odom_type) {
        case sdk::OdomSourceType::kOdom2ImuLow:
          // STANDARD: odom -> imu
          msg.header.frame_id = odom_frame_id_;
          msg.child_frame_id = imu_frame_id_;
          PublishMessage(odom_pub_, msg);
          break;

        case sdk::OdomSourceType::kOdom2ImuHigh:
          // HIGHFREQ: odom_hf topic only, no TF
          msg.header.frame_id = odom_frame_id_;
          msg.child_frame_id = imu_frame_id_;
          PublishMessage(odom_hf_pub_, msg);
          break;
        default:
          break;
      }
    }
  }

  // Publish a TF transform sourced from the device TF channel (data_type=0x32).
  // The SDK delivers parent/child frame ids as std::string already aligned with
  // ROS2 geometry_msgs/TransformStamped; we just forward to tf2_ros broadcaster.
  void ProcessTf(const sdk::OdinTfPacket &packet) {
    if (!tf_broadcaster_) {
      if (ShouldThrottle("tf_no_broadcaster", 5.0)) {
        LogWarn("ProcessTf: tf_broadcaster_ is NULL, dropping ('%s' -> '%s')",
                packet.frame_id.c_str(), packet.child_frame_id.c_str());
      }
      return;
    }
    if (packet.frame_id.empty() || packet.child_frame_id.empty()) {
      if (ShouldThrottle("tf_empty_frame", 5.0)) {
        LogWarn("ProcessTf: empty frame_id ('%s') or child_frame_id ('%s'), dropping",
                packet.frame_id.c_str(), packet.child_frame_id.c_str());
      }
      return;
    }
    // Prepend the device-scoped frame prefix (e.g. "device0/") to both ends
    // so the device-published TF tree is namespaced per device and won't
    // collide with driver-side or other devices' TFs.
    TransformStampedMsg tf_msg;
    tf2::Transform t;
    tf_msg.header.stamp = ToRosTime(packet.timestamp);
    tf_msg.header.frame_id = frame_prefix_ + packet.child_frame_id;
    tf_msg.child_frame_id = frame_prefix_ + packet.frame_id;
    tf_msg.transform.translation.x = packet.translation[0];
    tf_msg.transform.translation.y = packet.translation[1];
    tf_msg.transform.translation.z = packet.translation[2];
    tf_msg.transform.rotation.x = packet.rotation[0];
    tf_msg.transform.rotation.y = packet.rotation[1];
    tf_msg.transform.rotation.z = packet.rotation[2];
    tf_msg.transform.rotation.w = packet.rotation[3];
    NormalizeQuaternion(tf_msg.transform.rotation);
    tf2::fromMsg(tf_msg.transform, t);
    tf_msg.transform = tf2::toMsg(t.inverse());
    const std::string map_frame = frame_prefix_ + "map";
    if (tf_msg.header.frame_id == map_frame || tf_msg.child_frame_id == map_frame) {
      // Late subscribers need the latest relocalization anchor.
      static_tf_broadcaster_->sendTransform(tf_msg);
      return;
    }
    tf_broadcaster_->sendTransform(tf_msg);

    // Cache the latest TF for republishing by the timer
    {
      std::lock_guard<std::mutex> lock(tf_cache_mutex_);
      // Use parent+child frame pair as key to deduplicate
      bool found = false;
      for (auto &cached : cached_transforms_) {
        if (cached.msg.header.frame_id == tf_msg.header.frame_id &&
            cached.msg.child_frame_id == tf_msg.child_frame_id) {
          cached.msg = tf_msg;
          cached.cache_time = std::chrono::steady_clock::now();
          found = true;
          break;
        }
      }
      if (!found) {
        cached_transforms_.push_back({tf_msg, std::chrono::steady_clock::now(), 0.0});
      }
    }
  }

  // TF Link 3: base_link -> lidar (static, from calibration)
  // TODO: Add when static TF is needed
  // void PublishTfBaseToLidar() { ... }

  // TF Link 4: base_link -> imu (static, from calibration)
  // TODO: Add when static TF is needed
  // void PublishTfBaseToImu() { ... }

  // TF Link 5: base_link -> camera (static, from calibration)
  // TODO: Add when static TF is needed
  // void PublishTfBaseToCamera() { ... }

  void NormalizeQuaternion(QuaternionMsg &q) const {
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm > std::numeric_limits<double>::epsilon()) {
      q.x /= norm;
      q.y /= norm;
      q.z /= norm;
      q.w /= norm;
    } else {
      q.x = q.y = q.z = 0.0;
      q.w = 1.0;
    }
  }

  bool ShouldThrottle(const std::string &key, double period_sec) {
    const auto now = std::chrono::steady_clock::now();
    const auto it = throttle_slots_.find(key);
    if (it == throttle_slots_.end() ||
        std::chrono::duration<double>(now - it->second).count() >= period_sec) {
      throttle_slots_[key] = now;
      return true;
    }
    return false;
  }

  std::string LogPrefix() const {
    return "[" + (device_sn_.empty() ? "unknown" : device_sn_) + "] ";
  }

  template <typename... Args>
  void LogInfo(const char *fmt, Args... args) const {
    std::string prefixed_fmt = LogPrefix() + fmt;
#if defined(ODIN_ROS2)
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), prefixed_fmt.c_str(), args...);
    }
#else
    ROS_INFO(prefixed_fmt.c_str(), args...);
#endif
  }

  template <typename... Args>
  void LogWarn(const char *fmt, Args... args) const {
    std::string prefixed_fmt = LogPrefix() + fmt;
#if defined(ODIN_ROS2)
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), prefixed_fmt.c_str(), args...);
    }
#else
    ROS_WARN(prefixed_fmt.c_str(), args...);
#endif
  }

  template <typename... Args>
  void LogError(const char *fmt, Args... args) const {
    std::string prefixed_fmt = LogPrefix() + fmt;
#if defined(ODIN_ROS2)
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), prefixed_fmt.c_str(), args...);
    }
#else
    ROS_ERROR(prefixed_fmt.c_str(), args...);
#endif
  }

  template <typename... Args>
  void LogDebug(const char *fmt, Args... args) const {
    std::string prefixed_fmt = LogPrefix() + fmt;
#if defined(ODIN_ROS2)
    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(), prefixed_fmt.c_str(), args...);
    }
#else
    ROS_DEBUG(prefixed_fmt.c_str(), args...);
#endif
  }

#if defined(ODIN_ROS2)
  rclcpp::Node::SharedPtr node_;
#else
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;       // Device-specific NodeHandle for dynamic reconfigure
  ros::NodeHandle main_pnh_;  // Main node's private NodeHandle for global parameters
#endif

  point_cloud_transport::Publisher raw_cloud_pub_;
  point_cloud_transport::Publisher slam_cloud_pub_;
  PublisherType<CompressedImageMsg> image_pub_;
  PublisherType<ImuMsg> imu_pub_;
  PublisherType<OdometryMsg> odom_pub_;
  PublisherType<OdometryMsg> odom_hf_pub_;
  PublisherType<PointCloud2Msg> lc_optimized_cloud_pub_;
  PublisherType<PathMsg> lc_optimized_path_pub_;
  PublisherType<PointCloud2Msg> lc_map_cloud_pub_;
  PublisherType<PoseStampedMsg> lc_corrected_pose_pub_;
  PublisherType<PointCloud2Msg> lc_corrected_cloud_pub_;
  PublisherType<BoolMsg> lc_optimization_busy_pub_;
  PublisherType<BoolMsg> mapping_active_pub_;
  PublisherType<MarkerMsg> lc_optimization_status_marker_pub_;
#if defined(ODIN_ROS2)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr device_online_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr resolution_change_pub_;
#else
  ros::Publisher device_online_pub_;
  ros::Publisher resolution_change_pub_;
#endif

  sdk::DiscoveredDevice discovered_device_;  // Discovered device information
  sdk::OdinDeviceHandle device_handle_ = sdk::kInvalidDeviceHandle;
  sdk::OdinDeviceMode desired_mode_ = sdk::OdinDeviceMode::kNormal;
  bool connected_ = false;
  bool callbacks_registered_ = false;

  std::string device_sn_;     // SN of the actually connected device
  std::string device_model_;  // Device model (e.g., "ODIN2")
  std::string topic_prefix_base_ = "manifold";  // Topic prefix base from launch parameter
  std::string topic_prefix_;  // Topic prefix: /{topic_prefix_base}/device0/
  int device_index_ = -1;     // Device index (0, 1, 2, ...)
  int discovery_timeout_ms_ = 2000;

  std::string raw_frame_id_;
  std::string slam_frame_id_;
  std::string image_frame_id_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  // Device-scoped TF prefix (e.g. "device0/") used to namespace frame ids
  // produced by the device TF channel so they coexist with driver-side TFs
  // in multi-device setups.
  std::string frame_prefix_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // TF republish timer members
  double tf_publish_rate_{50.0};
  std::mutex tf_cache_mutex_;
  struct CachedTransform {
    TransformStampedMsg msg;
    std::chrono::steady_clock::time_point cache_time;
    double last_published_sec{0.0};
  };
  std::vector<CachedTransform> cached_transforms_;
#if defined(ODIN_ROS2)
  rclcpp::TimerBase::SharedPtr tf_republish_timer_;
#else
  ros::Timer tf_republish_timer_;
#endif
  std::string calib_file_path_;
  std::string calib_backup_path_;
  std::string calib_yaml_content_;
  bool calib_ready_ = false;
#if defined(ODIN_ROS2)
  rclcpp::Service<odin_driver_interfaces::srv::GetCalibration>::SharedPtr calib_service_;
  rclcpp::Service<odin_driver_interfaces::srv::SetSensorMode>::SharedPtr sensor_mode_service_;
  rclcpp::Service<odin_driver_interfaces::srv::ResetInitialPose>::SharedPtr reset_pose_service_;
  rclcpp::Service<odin_driver_interfaces::srv::GetAdaptiveParam>::SharedPtr get_adaptive_param_service_;
  rclcpp::Service<odin_driver_interfaces::srv::SetAdaptiveParam>::SharedPtr set_adaptive_param_service_;
  rclcpp::Service<odin_driver_interfaces::srv::GetSyncMode>::SharedPtr get_sync_mode_service_;
  rclcpp::Service<odin_driver_interfaces::srv::SetSyncMode>::SharedPtr set_sync_mode_service_;
  rclcpp::Service<odin_driver_interfaces::srv::GetPtpSyncStatus>::SharedPtr get_ptp_sync_status_service_;
  rclcpp::Service<odin_driver_interfaces::srv::RestoreDefaultConfig>::SharedPtr restore_default_config_service_;
  rclcpp::Service<odin_driver_interfaces::srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Service<odin_driver_interfaces::srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<odin_driver_interfaces::srv::ExployMap>::SharedPtr exploy_map_service_;
  rclcpp::Service<odin_driver_interfaces::srv::DeviceMode>::SharedPtr device_mode_service_;
#else
  ros::ServiceServer calib_service_;
  ros::ServiceServer sensor_mode_service_;
  ros::ServiceServer reset_pose_service_;
  ros::ServiceServer get_adaptive_param_service_;
  ros::ServiceServer set_adaptive_param_service_;
  ros::ServiceServer get_sync_mode_service_;
  ros::ServiceServer set_sync_mode_service_;
  ros::ServiceServer get_ptp_sync_status_service_;
  ros::ServiceServer restore_default_config_service_;
  ros::ServiceServer save_map_service_;
  ros::ServiceServer load_map_service_;
  ros::ServiceServer exploy_map_service_;
  ros::ServiceServer device_mode_service_;
#endif

  // Diagnostic monitor members
  struct DiagChannelStats {
    uint64_t frame_count = 0;
    int64_t last_latency_ns = 0;
    // Sliding window for latency stats (ring buffer, O(1) avg/min/max update)
    std::vector<int64_t> latency_window;
    size_t latency_window_index = 0;
    int64_t latency_window_sum = 0;
    int64_t latency_window_min = 0;
    int64_t latency_window_max = 0;
  };
  std::atomic<bool> diag_enabled_{false};
  std::mutex diag_mutex_;
  std::unordered_map<std::string, DiagChannelStats> diag_stats_;
  std::unordered_map<std::string, uint64_t> diag_prev_counts_;  // for FPS delta
  std::chrono::steady_clock::time_point diag_prev_time_;        // for FPS elapsed time
#if defined(ODIN_ROS2)
  rclcpp::Service<odin_driver_interfaces::srv::SetDiagMonitor>::SharedPtr diag_service_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr diag_status_pub_;
  PublisherType<DiagnosticArrayMsg> ros_diagnostics_pub_;
  rclcpp::Service<odin_driver_interfaces::srv::SetChannelConfig>::SharedPtr set_channel_config_service_;
  rclcpp::Service<odin_driver_interfaces::srv::GetChannelConfig>::SharedPtr get_channel_config_service_;
#else
  ros::ServiceServer diag_service_;
  ros::Timer diag_timer_;
  ros::Publisher diag_status_pub_;
  PublisherType<DiagnosticArrayMsg> ros_diagnostics_pub_;
  ros::ServiceServer set_channel_config_service_;
  ros::ServiceServer get_channel_config_service_;
#endif

  int m_confidence_threshold = 7;
  sdk::OdinSensorMode sensor_mode_ = sdk::OdinSensorMode::kHighPeak;  // OdinSensorMode enum
  int startup_sensor_mode_ = -1;  // launch sensor_mode; <0 or out of [0,3] = keep device mode
  int log_level_ = 1;    // 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG

  // Image resolution parameters (0 = use first capability from device)
  int image_width_ = 0;
  int image_height_ = 0;
  int image_fps_ = 0;
  std::string image_format_ = "mjpeg";
  bool sync_image_params_ = false;  // guard flag for SyncImageParamsToROS

  // Device-side filter parameters (-1 = device default, do not override)
  int filter_confidence_threshold_     = -1;  // [0..255]
  int filter_adaptive_cdf_percentile_  = -1;  // [1..10000] unit: 0.01%
  int filter_adaptive_outdoor_thr_     = -1;  // (mm)
  int filter_range_min_depth_          = -1;  // (mm)

  // Row filter parameters (client-side, applied to raw point cloud before publishing)
  // Points with row index < min_row_ are zeroed out (row = point_index / 256).
  bool enable_row_filter_ = false;
  int min_row_ = 0;

  // Channel enable parameters
  bool enable_raw_point_ = true;
  bool enable_slam_point_ = true;
  bool enable_image0_ = true;
  bool enable_imu_ = true;
  bool enable_odom_lf_ = true;
  bool enable_odom_hf_ = true;
  bool enable_tf_ = true;

  // Tracks channels currently active on the device (for diff-based reconfigure)
  std::set<sdk::OdinDataChannel> active_channels_;

  // Start a single channel: query capability, build config, call StartStream
  void StartSingleChannel(sdk::OdinDataChannel channel, sdk::OdinTransportMode transport) {
    auto channelName = [](sdk::OdinDataChannel ch) -> const char* {
      switch (ch) {
        case sdk::OdinDataChannel::kRawPoint:  return "RawPoint";
        case sdk::OdinDataChannel::kSlamPoint: return "SlamPoint";
        case sdk::OdinDataChannel::kImage0:    return "Image0";
        case sdk::OdinDataChannel::kImage1:    return "Image1";
        case sdk::OdinDataChannel::kImu:       return "IMU";
        case sdk::OdinDataChannel::kOdomLF:    return "OdomLF";
        case sdk::OdinDataChannel::kOdomHF:    return "OdomHF";
        case sdk::OdinDataChannel::kTf:        return "Tf";
        default: return "Unknown";
      }
    };

    std::vector<sdk::OdinSensorCapability> caps;
    auto query_result = sdk::GetSensorCapability(device_handle_, caps, {channel}, 1000);
    if (query_result != sdk::OdinResult::kOk || caps.empty()) {
      LogWarn("%s: no capability found, skipping", channelName(channel));
      return;
    }

    const sdk::OdinStreamCfg* cfg_ptr = nullptr;
    sdk::OdinStreamCfg cfg;
    bool is_image = (channel == sdk::OdinDataChannel::kImage0 ||
                     channel == sdk::OdinDataChannel::kImage1);
    if (is_image && !caps[0].modes.empty()) {
      const auto& cap = caps[0].modes[0];
      cfg.width  = (image_width_ > 0)  ? static_cast<uint16_t>(image_width_)  : cap.width;
      cfg.height = (image_height_ > 0) ? static_cast<uint16_t>(image_height_) : cap.height;
      cfg.fps    = (image_fps_ > 0)    ? static_cast<uint8_t>(image_fps_)
                                       : static_cast<uint8_t>(cap.fps);
      cfg.format = (image_format_ == "h264") ? sdk::OdinDataFormat::kImageH264
                 : (image_format_ == "nv12") ? sdk::OdinDataFormat::kImageNv12
                                              : sdk::OdinDataFormat::kImageMjpeg;
      cfg_ptr = &cfg;
      LogInfo("%s cfg: %ux%u@%uFPS format=%s",
              channelName(channel), cfg.width, cfg.height, cfg.fps, image_format_.c_str());
    } else if (!is_image) {
      if (!caps[0].modes.empty()) {
        const auto& cap = caps[0].modes[0];
        LogInfo("%s capability: %ux%u@%uFPS (format=%d)",
                channelName(channel), cap.width, cap.height, cap.fps, static_cast<int>(cap.format));
      }
    }

    auto result = sdk::StartStream(device_handle_, channel, transport, cfg_ptr);
    if (result == sdk::OdinResult::kOk) {
      if (cfg_ptr) {
        LogInfo("%s started: %ux%u@%uFPS", channelName(channel), cfg.width, cfg.height, cfg.fps);
        // Sync actual resolved values back to members so rqt shows the real
        // config instead of 0 (device default placeholder). Parameter sync
        // to ROS is done in SyncImageParamsToROS() after SetupDynamicReconfigure().
        if (channel == sdk::OdinDataChannel::kImage0) {
          image_width_  = cfg.width;
          image_height_ = cfg.height;
          image_fps_    = cfg.fps;
        }
      } else {
        LogInfo("%s started", channelName(channel));
      }
    } else {
      if (cfg_ptr) {
        LogWarn("Failed to start %s stream: requested %ux%u@%uFPS format=%s (result=%d)",
                channelName(channel), cfg.width, cfg.height, cfg.fps,
                image_format_.c_str(), static_cast<int>(result));
      } else {
        LogWarn("Failed to start %s stream (result=%d)",
                channelName(channel), static_cast<int>(result));
      }
    }
  }

  // Build the set of desired channels from current enable flags
  std::set<sdk::OdinDataChannel> BuildDesiredChannels() {
    std::set<sdk::OdinDataChannel> desired;
    if (enable_raw_point_)  desired.insert(sdk::OdinDataChannel::kRawPoint);
    if (enable_slam_point_) desired.insert(sdk::OdinDataChannel::kSlamPoint);
    if (enable_image0_)     desired.insert(sdk::OdinDataChannel::kImage0);
    if (enable_imu_)        desired.insert(sdk::OdinDataChannel::kImu);
    if (enable_odom_lf_)    desired.insert(sdk::OdinDataChannel::kOdomLF);
    if (enable_odom_hf_)    desired.insert(sdk::OdinDataChannel::kOdomHF);
    if (enable_tf_)         desired.insert(sdk::OdinDataChannel::kTf);
    return desired;
  }
#if defined(ODIN_ROS2)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
#else
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
#endif

  uint64_t last_img_stamp_ = 0;
  uint64_t last_pcd_stamp_ = 0;

  // Resolution detection: publish once after stream start, reset on stream stop
  bool resolution_sent_ = false;

  // Async queues for non-blocking SDK callbacks
  struct OdomQueueItem {
    sdk::OdinOdomPacket packet;
    sdk::OdomSourceType odom_type;
  };
  std::unique_ptr<AsyncQueue<sdk::OdinPointCloudPacket>> raw_point_queue_;
  std::unique_ptr<AsyncQueue<sdk::OdinSlamPacket>> slam_point_queue_;
  std::unique_ptr<AsyncQueue<sdk::OdinCameraPacket>> image0_queue_;
  std::unique_ptr<AsyncQueue<sdk::OdinImuPacket>> imu_queue_;
  std::unique_ptr<AsyncQueue<OdomQueueItem>> odom_lf_queue_;
  std::unique_ptr<AsyncQueue<OdomQueueItem>> odom_hf_queue_;
  std::unique_ptr<AsyncQueue<sdk::OdinTfPacket>> tf_queue_;
  std::unique_ptr<AsyncQueue<sdk::OdinLcOptimizationStatusPacket>> lc_optimization_status_queue_;
  std::unique_ptr<AsyncQueue<sdk::OdinLcFramePacket>> lc_frame_queue_;
  std::unique_ptr<AsyncQueue<sdk::OdinLcOptimizationResultPacket>> lc_optimization_result_queue_;
  std::unique_ptr<AsyncQueue<sdk::OdinLcMapCloudPacket>> lc_map_cloud_queue_;

  // Shutdown synchronization: prevents timer callbacks from accessing
  // driver members after Shutdown() has begun destruction.
  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> device_removed_{false};
  std::mutex shutdown_mutex_;

  // Parse JPEG SOF0 marker to extract image width and height
  static bool ParseJpegResolution(const std::vector<uint8_t> &data, int &width, int &height) {
    // Look for SOF0 marker (0xFF 0xC0)
    for (size_t i = 0; i + 8 < data.size(); ++i) {
      if (data[i] == 0xFF && (data[i + 1] == 0xC0 || data[i + 1] == 0xC2)) {
        height = (data[i + 5] << 8) | data[i + 6];
        width = (data[i + 7] << 8) | data[i + 8];
        return (width > 0 && height > 0);
      }
    }
    return false;
  }

  std::unordered_map<std::string, std::chrono::steady_clock::time_point> throttle_slots_;
};

}  // namespace odin_ros_driver

// =============================================================================
// Hotplug Device Manager - Unified for ROS1/ROS2
// =============================================================================

namespace {

// Logging macros for hotplug (work outside ros_compat namespace)
#if defined(ODIN_ROS2)
#define HP_LOG_INFO(...) printf(__VA_ARGS__)
#define HP_LOG_WARN(...) printf(__VA_ARGS__)
#define HP_LOG_ERROR(...) printf(__VA_ARGS__)
#define HP_LOG_FATAL(...) printf(__VA_ARGS__)
#else
#define HP_LOG_INFO(...) ROS_INFO(__VA_ARGS__)
#define HP_LOG_WARN(...) ROS_WARN(__VA_ARGS__)
#define HP_LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define HP_LOG_FATAL(...) ROS_FATAL(__VA_ARGS__)
#endif

// Device instance holder - unified structure
struct DeviceInstance {
#if defined(ODIN_ROS2)
  rclcpp::Node::SharedPtr node;
#endif
  std::unique_ptr<odin_ros_driver::odinRosDriver> driver;
  std::string sn;
  std::string model;      // Device model, included in device_offline announcements
  int device_index = -1;  // Device index, used for recycling
};

// Helper to generate sanitized node name
inline std::string SanitizeNodeName(const std::string &sn, size_t index) {
  std::string name = "odin_driver_" + (sn.empty() ? std::to_string(index) : sn);
  for (char &c : name) {
    if (c == '-' || c == ' ') c = '_';
  }
  return name;
}

// Unified HotplugManager
class HotplugManager {
 public:
  std::mutex mutex;
  std::map<std::string, std::unique_ptr<DeviceInstance>> devices;
  std::set<std::string> pending_devices;  // Devices being initialized asynchronously
  std::vector<std::thread> init_threads;  // Track init threads for cleanup
  std::vector<std::unique_ptr<DeviceInstance>> pending_detaches;  // Deferred destruction queue (processed in main thread)
  std::atomic<bool> running{true};
  size_t next_device_index = 0;     // Next available index
  std::set<int> available_indices;  // Pool of released indices for reuse
  std::string topic_prefix_base_ = "manifold";  // Topic prefix base from launch parameter

#if defined(ODIN_ROS2)
  rclcpp::executors::MultiThreadedExecutor *executor = nullptr;
  rclcpp::Node::SharedPtr manager_node;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr device_offline_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr resolution_change_pub;

  // Helper to build driver topic name for HotplugManager
  // Unified with device topic format: /{prefix}/device{N}/...
  std::string BuildDriverTopicName(const std::string &suffix) const {
    if (topic_prefix_base_.empty()) {
      return "/driver/" + suffix;
    }
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  void Init(const std::string &topic_prefix_base = "manifold") {
    topic_prefix_base_ = topic_prefix_base;
    manager_node = std::make_shared<rclcpp::Node>("odin_hotplug_manager");
    device_offline_pub = manager_node->create_publisher<std_msgs::msg::String>(
        BuildDriverTopicName("device_offline"), rclcpp::QoS(10).transient_local());
    // TRANSIENT_LOCAL so late-joining subscribers (e.g. pointcloud2depth_node
    // started after the driver) receive the last published resolution.
    // Must match the per-device resolution_change_pub_ durability to avoid
    // QoS conflicts on the same topic.
    resolution_change_pub = manager_node->create_publisher<std_msgs::msg::String>(
        BuildDriverTopicName("resolution_change"), rclcpp::QoS(1).transient_local());
    HP_LOG_INFO("[Hotplug] Manager node initialized (parameters on device nodes)\n");
  }

  void PublishDeviceOffline(const std::string &sn, const std::string &model) {
    if (device_offline_pub) {
      std_msgs::msg::String msg;
      msg.data = "{\"sn\":\"" + sn + "\",\"model\":\"" + model + "\"}";
      device_offline_pub->publish(msg);
      HP_LOG_INFO("[Hotplug] Published device_offline for: %s\n", sn.c_str());
    }
  }

  void PublishResolutionChange(const std::string &sn, int width, int height) {
    if (resolution_change_pub) {
      std_msgs::msg::String msg;
      msg.data = "{\"sn\":\"" + sn + "\",\"width\":" + std::to_string(width) +
                 ",\"height\":" + std::to_string(height) + "}";
      resolution_change_pub->publish(msg);
      HP_LOG_INFO("[Hotplug] Published resolution_change for %s: %dx%d\n", sn.c_str(), width,
                  height);
    }
  }
#else
  ros::NodeHandle *nh = nullptr;
  ros::NodeHandle *pnh = nullptr;
  ros::Publisher device_offline_pub;
  ros::Publisher resolution_change_pub;

  // Helper to build driver topic name for HotplugManager
  // Unified with device topic format: /{prefix}/device{N}/...
  std::string BuildDriverTopicName(const std::string &suffix) const {
    if (topic_prefix_base_.empty()) {
      return "/driver/" + suffix;
    }
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  void Init(const std::string &topic_prefix_base = "manifold") {
    topic_prefix_base_ = topic_prefix_base;
    if (nh) {
      device_offline_pub = nh->advertise<std_msgs::String>(BuildDriverTopicName("device_offline"), 10, true);
      resolution_change_pub = nh->advertise<std_msgs::String>(BuildDriverTopicName("resolution_change"), 10);
    }
  }

  void PublishDeviceOffline(const std::string &sn, const std::string &model) {
    if (device_offline_pub) {
      std_msgs::String msg;
      msg.data = "{\"sn\":\"" + sn + "\",\"model\":\"" + model + "\"}";
      device_offline_pub.publish(msg);
      HP_LOG_INFO("[Hotplug] Published device_offline for: %s", sn.c_str());
    }
  }

  void PublishResolutionChange(const std::string &sn, int width, int height) {
    if (resolution_change_pub) {
      std_msgs::String msg;
      msg.data = "{\"sn\":\"" + sn + "\",\"width\":" + std::to_string(width) +
                 ",\"height\":" + std::to_string(height) + "}";
      resolution_change_pub.publish(msg);
      HP_LOG_INFO("[Hotplug] Published resolution_change for %s: %dx%d", sn.c_str(), width, height);
    }
  }
#endif

  void OnAttach(const odin::sdk::DiscoveredDevice &dev) {
    std::string key = dev.sn.empty() ? dev.network.ip_address : dev.sn;

    // Quick check with lock - avoid duplicate initialization
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (devices.find(key) != devices.end()) {
        HP_LOG_INFO("[Hotplug] Device %s already connected, ignoring attach\n", key.c_str());
        return;
      }
      if (pending_devices.find(key) != pending_devices.end()) {
        HP_LOG_INFO("[Hotplug] Device %s already initializing, ignoring attach\n", key.c_str());
        return;
      }
      pending_devices.insert(key);
    }

#if defined(ODIN_ROS2)
    HP_LOG_INFO("[Hotplug] Device attached: SN=%s, IP=%s, Model=%s, Protocol=0x%02X (async init)\n",
                dev.sn.c_str(), dev.network.ip_address.c_str(), dev.model.c_str(),
                dev.protocol_version);
#else
    HP_LOG_INFO("[Hotplug] Device attached: SN=%s, IP=%s, Model=%s, Protocol=0x%02X (async init)",
                dev.sn.c_str(), dev.network.ip_address.c_str(), dev.model.c_str(),
                dev.protocol_version);
#endif

    // Acquire an index from the pool, preferring previously released ones
    int idx;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!available_indices.empty()) {
        idx = *available_indices.begin();
        available_indices.erase(available_indices.begin());
        HP_LOG_INFO("[Hotplug] Reusing device index %d\n", idx);
      } else {
        idx = static_cast<int>(next_device_index++);
        HP_LOG_INFO("[Hotplug] Allocating new device index %d\n", idx);
      }
    }
#if defined(ODIN_ROS2)
    auto *exec_ptr = executor;
#else
    ros::NodeHandle *nh_ptr = nh;
    ros::NodeHandle *pnh_ptr = pnh;
#endif

    // Async initialization in separate thread to avoid blocking existing devices
    init_threads.emplace_back([this, dev, key, idx
#if defined(ODIN_ROS2)
                               ,
                               exec_ptr
#else
                               ,
                               nh_ptr, pnh_ptr
#endif
    ]() {
      auto instance = std::make_unique<DeviceInstance>();
      instance->sn = key;
      instance->model = dev.model;
      instance->device_index = idx;  // Save index for later recycling

#if defined(ODIN_ROS2)
      std::string node_name = SanitizeNodeName(dev.sn, idx);
      instance->node = std::make_shared<rclcpp::Node>(node_name);
      instance->driver = std::make_unique<odin_ros_driver::odinRosDriver>(instance->node, dev, idx);
#else
      // Create device-specific NodeHandle with SN as namespace for independent dynamic reconfigure
      // Pass both main_pnh (for global params like image_width) and device_pnh (for dynamic reconfigure)
      std::string device_ns = dev.sn.empty() ? "device_" + std::to_string(idx) : dev.sn;
      ros::NodeHandle device_pnh(*pnh_ptr, device_ns);
      instance->driver =
          std::make_unique<odin_ros_driver::odinRosDriver>(*nh_ptr, *pnh_ptr, device_pnh, dev, idx);
#endif

      bool init_success = instance->driver->Init();

      // Finalize with lock
      std::lock_guard<std::mutex> lock(mutex);
      pending_devices.erase(key);

      if (!init_success) {
        HP_LOG_ERROR("[Hotplug] Failed to initialize driver for device %s\n", key.c_str());
        return;
      }

#if defined(ODIN_ROS2)
      if (exec_ptr) {
        exec_ptr->add_node(instance->node);
      }
      // Note: Per-device parameters removed due to thread-safety issues
      // Use global default_confidence_threshold and default_sensor_mode instead
#endif

      devices[key] = std::move(instance);
      HP_LOG_INFO("[Hotplug] Device %s connected, total: %zu\n", key.c_str(), devices.size());
    });
  }

  void OnDetach(const odin::sdk::DiscoveredDevice &dev) {
    std::unique_ptr<DeviceInstance> instance;
    int released_index = -1;

    {
      std::lock_guard<std::mutex> lock(mutex);

      std::string key = dev.sn.empty() ? dev.network.ip_address : dev.sn;
      auto it = devices.find(key);
      if (it == devices.end()) {
        HP_LOG_WARN("[Hotplug] Device %s not found for detach\n", key.c_str());
        return;
      }

      HP_LOG_INFO("[Hotplug] Device detaching: SN=%s\n", dev.sn.c_str());
      instance = std::move(it->second);
      released_index = instance->device_index;
      devices.erase(it);

      // Recycle index back to the pool
      if (released_index >= 0) {
        available_indices.insert(released_index);
        HP_LOG_INFO("[Hotplug] Recycled device index %d\n", released_index);
      }
    }

    if (instance) {
      PublishDeviceOffline(instance->sn, instance->model);
#if defined(ODIN_ROS2)
      if (executor && instance->node) {
        executor->remove_node(instance->node);
      }
      instance->node.reset();
#endif
      // Mark device as removed before destroying the driver so Shutdown()
      // skips RequestStandbyMode() (device is already gone, command would
      // just block until timeout).
      if (instance->driver) {
        instance->driver->MarkDeviceRemoved();
      }
      // Defer driver destruction to the main thread. The executor may still
      // have an in-flight service or parameter callback accessing driver
      // members; destroying the driver here (in the hotplug listener thread)
      // would race with those callbacks and cause a use-after-free SIGSEGV.
      // The main loop calls ProcessPendingDetaches() after spin_some()
      // returns, guaranteeing no executor callbacks are in-flight.
      {
        std::lock_guard<std::mutex> lock(mutex);
        pending_detaches.push_back(std::move(instance));
      }
      HP_LOG_INFO("[Hotplug] Device %s scheduled for release, remaining: %zu\n",
                  dev.sn.empty() ? dev.network.ip_address.c_str() : dev.sn.c_str(),
                  devices.size());
    }
  }

  // Destroy deferred drivers in the main thread, safe from executor races.
  void ProcessPendingDetaches() {
    std::vector<std::unique_ptr<DeviceInstance>> to_destroy;
    {
      std::lock_guard<std::mutex> lock(mutex);
      to_destroy.swap(pending_detaches);
    }
    for (auto &inst : to_destroy) {
      if (inst && inst->driver) {
        inst->driver.reset();
      }
      inst.reset();
    }
  }

  void Cleanup() {
    // Process any pending detaches first (drivers deferred from OnDetach)
    ProcessPendingDetaches();

    // Wait for all pending init threads to complete
    for (auto &t : init_threads) {
      if (t.joinable()) {
        t.join();
      }
    }
    init_threads.clear();

    std::lock_guard<std::mutex> lock(mutex);
    for (auto &pair : devices) {
#if defined(ODIN_ROS2)
      if (executor && pair.second->node) {
        executor->remove_node(pair.second->node);
      }
      pair.second->node.reset();
#endif
      pair.second->driver.reset();
    }
    devices.clear();
    pending_devices.clear();
    HP_LOG_INFO("[Hotplug] All devices cleaned up\n");
  }
};

}  // anonymous namespace

// =============================================================================
// Main Entry Point
// =============================================================================

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IOLBF, 0);
  odin::print_app_version("odin_ros_driver_node");
  log_config(LOG_LEVEL_INFO, printf);
  printf("\n");
  printf("========================================\n");
  printf("  SDK VERSION: %s\n", odin::sdk::GetSdkVersion().c_str());
  printf("========================================\n");
  printf("\n");
#if defined(ODIN_ROS2)
  rclcpp::init(argc, argv);
  printf("[odin_ros_driver] Starting with hotplug support (ROS2)...\n");

  // Read topic_prefix from parameter (need a temporary node to read launch params)
  auto temp_node = std::make_shared<rclcpp::Node>("odin_param_reader");
  temp_node->declare_parameter<std::string>("topic_prefix", "manifold");
  std::string topic_prefix_base;
  temp_node->get_parameter("topic_prefix", topic_prefix_base);
  if (temp_node->declare_parameter<bool>("enable_image1", false)) {
    RCLCPP_ERROR(temp_node->get_logger(), "This hardware has only camera0; enable_image1 must be false.");
    rclcpp::shutdown();
    return 1;
  }

  HotplugManager manager;
  manager.Init(topic_prefix_base);
  rclcpp::executors::MultiThreadedExecutor executor;
  manager.executor = &executor;
  executor.add_node(manager.manager_node);
#else
  ros::init(argc, argv, "odin_ros_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("[odin_ros_driver] Starting with hotplug support (ROS1)...");

  // Read topic_prefix from parameter
  std::string topic_prefix_base = pnh.param<std::string>("topic_prefix", "manifold");
  if (pnh.param<bool>("enable_image1", false)) {
    ROS_ERROR("This hardware has only camera0; enable_image1 must be false.");
    return 1;
  }

  HotplugManager manager;
  manager.nh = &nh;
  manager.pnh = &pnh;
  manager.Init(topic_prefix_base);
#endif

  // Setup hotplug listener using SDK API
  odin::sdk::HotplugCallbacks callbacks;
  callbacks.on_attach = [&manager](const odin::sdk::DiscoveredDevice &dev) {
    manager.OnAttach(dev);
  };
  callbacks.on_detach = [&manager](const odin::sdk::DiscoveredDevice &dev) {
    manager.OnDetach(dev);
  };

  if (!odin::sdk::StartHotplugListener(callbacks, true)) {
    HP_LOG_FATAL("[odin_ros_driver] Failed to start hotplug listener!\n");
#if defined(ODIN_ROS2)
    rclcpp::shutdown();
#endif
    return 1;
  }

  HP_LOG_INFO("[odin_ros_driver] Hotplug listener started, running (Ctrl+C to stop)...\n");

#if defined(ODIN_ROS2)
  while (rclcpp::ok() && manager.running.load()) {
    executor.spin_some(std::chrono::milliseconds(100));
    // Destroy deferred drivers in the main thread, safe from executor races.
    manager.ProcessPendingDetaches();
  }
#else
  // ROS1: do NOT use a blocking ros::spin(); it would never process the
  // deferred detach queue, leaving detached drivers (and their heartbeat
  // threads) alive forever after a hotplug offline event. Instead pump
  // callbacks and destroy detached drivers on every iteration.
  ros::Rate loop_rate(10);  // 10 Hz
  while (ros::ok() && manager.running.load()) {
    ros::spinOnce();
    // Destroy deferred drivers in the main thread; this triggers each
    // driver's destructor -> DisconnectDevice() -> StopHeartbeat().
    manager.ProcessPendingDetaches();
    loop_rate.sleep();
  }
#endif

  HP_LOG_INFO("[odin_ros_driver] Shutting down...\n");
  odin::sdk::StopHotplugListener();
  manager.Cleanup();

#if defined(ODIN_ROS2)
  rclcpp::shutdown();
#endif

  HP_LOG_INFO("[odin_ros_driver] Shutdown complete\n");
  return 0;
}
