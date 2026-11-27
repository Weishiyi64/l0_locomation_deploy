// @file depth_to_pointcloud_node.cpp
// @brief Multi-device ROS node that back-projects each Odin device's depth
//        map (from pointcloud_to_depth) onto the synchronized undistorted
//        RGB image and publishes a dense XYZRGB point cloud in the lidar
//        frame.
//
// Architecture mirrors pointcloud_to_depth_node:
//   - DepthToPointCloudNode owns the global config and a map of
//     DeviceWorker objects keyed by serial number.
//   - device_online    -> spawn a new DeviceWorker for that sn
//   - device_offline   -> destroy the DeviceWorker (DDS pubs/subs released)
//   - resolution_change-> dispatched to the matching DeviceWorker
//
// Subscribes per device (under the discovered device prefix):
//   {prefix}pointcloud2depth/depth     sensor_msgs/Image  32FC1 full-res
//   {prefix}camera0/undistort          sensor_msgs/Image  bgr8  full-res
//
// Publishes per device:
//   {prefix}depth2cloud/cloud_color    sensor_msgs/PointCloud2 (x,y,z,rgb)
//
// The plain-types core (depth_to_pointcloud_node.hpp) is intentionally
// header-only and free of PCL/Eigen/OpenCV/ROS so it can be lifted into the
// device-side SDK as a pure C/C++ module without modification.

#include "odin_app_version.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#if defined(ODIN_ROS2)
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "service_types.h"

using StringMsg      = std_msgs::msg::String;
using ImageMsg       = sensor_msgs::msg::Image;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
#else
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/String.h>

#include "service_types.h"

using StringMsg      = std_msgs::String;
using ImageMsg       = sensor_msgs::Image;
using PointCloud2Msg = sensor_msgs::PointCloud2;
#endif

#include "depth_to_pointcloud_node.hpp"
#include "yaml_config_loader.hpp"

namespace odin {

// ---------------------------------------------------------------------------
// Small JSON helpers (same flavor as pointcloud_to_depth_node).
// ---------------------------------------------------------------------------
static bool ParseJsonStringField(const std::string& json, const std::string& key,
                                 std::string& value_out) {
  const std::string search = "\"" + key + "\":\"";
  size_t start = json.find(search);
  if (start == std::string::npos) return false;
  start += search.length();
  const size_t end = json.find('"', start);
  if (end == std::string::npos) return false;
  value_out = json.substr(start, end - start);
  return !value_out.empty();
}

static int ParseJsonIntField(const std::string& json, const std::string& key) {
  const std::string search = "\"" + key + "\":";
  const size_t start = json.find(search);
  if (start == std::string::npos) return 0;
  return std::atoi(json.c_str() + start + search.length());
}

// ---------------------------------------------------------------------------
// Per-device metadata parsed out of the driver's device_online JSON.
// ---------------------------------------------------------------------------
struct DeviceInfo {
  std::string sn;
  std::string model;   // device model name reported by firmware
  std::string prefix;  // already includes leading + trailing '/'
};

static bool ParseDeviceInfoJson(const std::string& json, DeviceInfo& info) {
  ParseJsonStringField(json, "sn", info.sn);
  ParseJsonStringField(json, "model", info.model);
  ParseJsonStringField(json, "prefix", info.prefix);
  return !info.sn.empty() && !info.prefix.empty();
}

// ---------------------------------------------------------------------------
// Shared tunables loaded once from ROS params.
// ---------------------------------------------------------------------------
struct DepthToCloudConfig {
  // Back-projection
  int   step       = 5;       // pixel sampling stride (doc §3.2 default)
  float d_min      = 0.1f;    // meters
  float d_max      = 100.0f;  // meters

  // Synchronization
  int    sync_queue_size = 10;
  double sync_slop_sec   = 0.1;

  // Topic naming (suffixes appended to the per-device prefix)
  std::string depth_topic_suffix  = "pointcloud2depth/depth";
  std::string image_topic_suffix  = "camera0/undistort";
  std::string output_topic_suffix = "depth2cloud/cloud_color";

  // Optional override for the published PointCloud2 frame_id. Empty means
  // "use the depth message's frame_id" (which is the same as cloud/raw in
  // this driver: device{N}/odom).
  std::string output_frame_id;
};

// ---------------------------------------------------------------------------
// DeviceWorker: encapsulates everything needed to process one device.
// ---------------------------------------------------------------------------
class DeviceWorker {
 public:
#if defined(ODIN_ROS2)
  DeviceWorker(rclcpp::Node::SharedPtr node, DeviceInfo info,
               const DepthToCloudConfig& cfg)
      : node_(std::move(node)), info_(std::move(info)), cfg_(cfg) {
    logInfo("Worker created (prefix=" + info_.prefix + ")");
    fetchCalibrationAsync();
  }
#else
  DeviceWorker(ros::NodeHandle& nh, DeviceInfo info,
               const DepthToCloudConfig& cfg)
      : nh_(nh), info_(std::move(info)), cfg_(cfg) {
    logInfo("Worker created (prefix=" + info_.prefix + ")");
    fetchCalibrationAsync();
  }
#endif

  ~DeviceWorker() {
    logInfo("Worker destroying");
#if defined(ODIN_ROS2)
    cloud_pub_.reset();
    calibration_client_.reset();
    if (retry_timer_) retry_timer_->cancel();
    retry_timer_.reset();
    sync_.reset();
    depth_sub_.unsubscribe();
    image_sub_.unsubscribe();
#else
    sync_.reset();
    depth_sub_.unsubscribe();
    image_sub_.unsubscribe();
    cloud_pub_.shutdown();
#endif
  }

  DeviceWorker(const DeviceWorker&) = delete;
  DeviceWorker& operator=(const DeviceWorker&) = delete;

  const std::string& sn()     const { return info_.sn; }
  const std::string& prefix() const { return info_.prefix; }

  // Manager dispatches resolution_change here.
  void onResolutionChange(int width, int height) {
    if (width <= 0 || height <= 0) return;
    if (width == img_width_ && height == img_height_) return;
    logInfo("Resolution change: " + std::to_string(img_width_) + "x" +
            std::to_string(img_height_) + " -> " +
            std::to_string(width) + "x" + std::to_string(height));
    img_width_  = width;
    img_height_ = height;
    if (!yaml_content_.empty()) {
      rebuildIntrinsicsAndExtrinsics();
    }
  }

 private:
  void fetchCalibrationAsync() {
#if defined(ODIN_ROS2)
    calibration_client_ = node_->create_client<odin_driver_interfaces::srv::GetCalibration>(
        info_.prefix + "get_calibration");
    if (!calibration_client_->wait_for_service(std::chrono::seconds(1))) {
      logWarn("get_calibration not available yet, retrying in 1s...");
      retry_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
        retry_timer_->cancel();
        fetchCalibrationAsync();
      });
      return;
    }
    auto request = std::make_shared<odin_driver_interfaces::srv::GetCalibration::Request>();
    calibration_client_->async_send_request(
        request,
        [this](rclcpp::Client<odin_driver_interfaces::srv::GetCalibration>::SharedFuture f) {
          try {
            auto response = f.get();
            if (!response->success) {
              logError("get_calibration returned failure");
              return;
            }
            yaml_content_ = response->yaml_content;
            logInfo("Calibration fetched.");
            rebuildIntrinsicsAndExtrinsics();
            setupPubSub();
          } catch (const std::exception& e) {
            logError(std::string("get_calibration future exception: ") + e.what());
          }
        });
#else
    ros::ServiceClient client = nh_.serviceClient<odin_driver_interfaces::srv::GetCalibration>(
        info_.prefix + "get_calibration");
    if (!client.waitForExistence(ros::Duration(5.0))) {
      logError("get_calibration service not available");
      return;
    }
    odin_driver_interfaces::srv::GetCalibration srv;
    if (!client.call(srv) || !srv.response.success) {
      logError("Failed to call get_calibration");
      return;
    }
    yaml_content_ = srv.response.yaml_content;
    logInfo("Calibration fetched.");
    rebuildIntrinsicsAndExtrinsics();
    setupPubSub();
#endif
  }

  // Parse YAML, resolve cam key by resolution, fill CloudParams + Tlc.
  bool rebuildIntrinsicsAndExtrinsics() {
    YAMLConfigLoader loader;
    if (!loader.loadFromString(yaml_content_)) {
      logError("Failed to parse calibration YAML");
      return false;
    }

    int img_w = img_width_  > 0 ? img_width_  : loader.getValue<int>("image_width", 640);
    int img_h = img_height_ > 0 ? img_height_ : loader.getValue<int>("image_height", 544);
    img_width_  = img_w;
    img_height_ = img_h;

    YAML::Node root = YAML::Load(yaml_content_);
    const std::string cam_key = yaml_utils::ResolveCamKey(root, /*cam_id=*/0, img_w, img_h);
    if (cam_key.empty()) {
      logError("No calibration node found for cam_0 at " + std::to_string(img_w) +
               "x" + std::to_string(img_h));
      return false;
    }
    YAML::Node cam = root[cam_key];

    params_.A11 = static_cast<float>(yaml_utils::safeGet<double>(cam, "A11", 500.0));
    params_.A12 = static_cast<float>(yaml_utils::safeGet<double>(cam, "A12", 0.0));
    params_.A22 = static_cast<float>(yaml_utils::safeGet<double>(cam, "A22", 500.0));
    params_.u0  = static_cast<float>(yaml_utils::safeGet<double>(cam, "u0",  img_w / 2.0));
    params_.v0  = static_cast<float>(yaml_utils::safeGet<double>(cam, "v0",  img_h / 2.0));

    std::vector<double> tcl_v = loader.getVector<double>("Tcl_0");
    if (tcl_v.size() < 16) {
      logWarn("Tcl_0 missing/invalid in calibration. Using identity.");
      for (int i = 0; i < 16; ++i) params_.Tcl[i] = (i % 5 == 0) ? 1.0f : 0.0f;
    } else {
      for (int i = 0; i < 16; ++i) params_.Tcl[i] = static_cast<float>(tcl_v[i]);
    }

    params_.step  = cfg_.step;
    params_.d_min = cfg_.d_min;
    params_.d_max = cfg_.d_max;

    // Precompute Tlc once (doc §3.1 rigid-body shortcut).
    depth_to_cloud_core::InvertRigid(params_.Tcl, Tlc_);

    calibrated_ = true;

    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "Calibration ready. img=%dx%d fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
                  "step=%d d=[%.2f,%.2f]",
                  img_w, img_h, params_.A11, params_.A22, params_.u0, params_.v0,
                  params_.step, params_.d_min, params_.d_max);
    logInfo(buf);
    return true;
  }

  void setupPubSub() {
    if (initialized_.exchange(true)) return;

    const std::string depth_topic  = info_.prefix + cfg_.depth_topic_suffix;
    const std::string image_topic  = info_.prefix + cfg_.image_topic_suffix;
    const std::string output_topic = info_.prefix + cfg_.output_topic_suffix;

#if defined(ODIN_ROS2)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    cloud_pub_ = node_->create_publisher<PointCloud2Msg>(output_topic, qos);

    auto sub_qos = rclcpp::QoS(30)
                       .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                       .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    depth_sub_.subscribe(node_, depth_topic, sub_qos.get_rmw_qos_profile());
    image_sub_.subscribe(node_, image_topic, sub_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<Sync>(SyncPolicy(cfg_.sync_queue_size), depth_sub_, image_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(cfg_.sync_slop_sec));
    sync_->registerCallback(
        std::bind(&DeviceWorker::syncCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
#else
    cloud_pub_ = nh_.advertise<PointCloud2Msg>(output_topic, 10);

    depth_sub_.subscribe(nh_, depth_topic, 30);
    image_sub_.subscribe(nh_, image_topic, 30);
    sync_ = std::make_shared<Sync>(SyncPolicy(cfg_.sync_queue_size), depth_sub_, image_sub_);
    sync_->registerCallback(boost::bind(&DeviceWorker::syncCallback, this, _1, _2));
#endif

    logInfo("Subscribing: " + depth_topic);
    logInfo("Subscribing: " + image_topic);
    logInfo("Publishing : " + output_topic);
  }

  // --------------------------------------------------------------------
  // Hot path
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  void syncCallback(const ImageMsg::ConstSharedPtr& depth_msg,
                    const ImageMsg::ConstSharedPtr& image_msg) {
    processFrame(depth_msg, image_msg);
  }
#else
  void syncCallback(const ImageMsg::ConstPtr& depth_msg,
                    const ImageMsg::ConstPtr& image_msg) {
    processFrame(depth_msg, image_msg);
  }
#endif

  template <typename ImagePtr>
  void processFrame(const ImagePtr& depth_msg, const ImagePtr& image_msg) {
    if (!calibrated_) return;

    const auto frame_t0 = std::chrono::steady_clock::now();

    // ---- Decode depth (32FC1, meters) ----
    cv::Mat depth;
    try {
      depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
    } catch (const cv_bridge::Exception& e) {
      logWarn(std::string("cv_bridge depth exception: ") + e.what());
      return;
    }
    if (depth.empty()) return;

    // ---- Decode RGB (BGR8, already undistorted) ----
    cv::Mat rgb;
    try {
      rgb = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      logWarn(std::string("cv_bridge rgb exception: ") + e.what());
      return;
    }
    if (rgb.empty()) return;

    // ---- Pixel-alignment safety check (doc §1 key precondition) ----
    if (depth.cols != rgb.cols || depth.rows != rgb.rows) {
      logWarnThrottled("depth/rgb size mismatch: depth=" +
                       std::to_string(depth.cols) + "x" + std::to_string(depth.rows) +
                       " rgb=" + std::to_string(rgb.cols) + "x" + std::to_string(rgb.rows));
      return;
    }

    // ---- Late-joining safety net: realign intrinsics if image size changed ----
    if (rgb.cols != img_width_ || rgb.rows != img_height_) {
      logInfo("Image dims differ from internal state (" +
              std::to_string(img_width_) + "x" + std::to_string(img_height_) +
              " -> " + std::to_string(rgb.cols) + "x" + std::to_string(rgb.rows) +
              "), rebuilding intrinsics");
      img_width_  = rgb.cols;
      img_height_ = rgb.rows;
      if (!rebuildIntrinsicsAndExtrinsics()) return;
    }

    // ---- Ensure tightly packed continuous buffers for the core ----
    if (!depth.isContinuous()) depth = depth.clone();
    if (!rgb.isContinuous())   rgb   = rgb.clone();

    // ---- Run library-free back-projection ----
    const int W = depth.cols;
    const int H = depth.rows;
    const int max_pts = depth_to_cloud_core::MaxOutputPoints(W, H, params_.step);
    out_buf_.resize(static_cast<size_t>(max_pts));

    const int n_written = depth_to_cloud_core::DepthToDenseCloud(
        reinterpret_cast<const float*>(depth.data),
        rgb.data,
        W, H, params_, Tlc_, out_buf_.data());

    // ---- Pack into sensor_msgs/PointCloud2 (XYZRGB) ----
    PointCloud2Msg cloud_msg;
    cloud_msg.header = depth_msg->header;
    if (!cfg_.output_frame_id.empty()) {
      cloud_msg.header.frame_id = cfg_.output_frame_id;
    }
    cloud_msg.height = 1;
    cloud_msg.width  = static_cast<uint32_t>(n_written);
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense     = true;

    // Standard PCL XYZRGB layout: 4 floats (x, y, z, rgb) packed as 16 bytes.
    cloud_msg.fields.resize(4);
    auto fillField = [](auto& f, const std::string& name, uint32_t off, uint8_t dtype) {
      f.name = name; f.offset = off; f.datatype = dtype; f.count = 1;
    };
#if defined(ODIN_ROS2)
    fillField(cloud_msg.fields[0], "x",   0,  sensor_msgs::msg::PointField::FLOAT32);
    fillField(cloud_msg.fields[1], "y",   4,  sensor_msgs::msg::PointField::FLOAT32);
    fillField(cloud_msg.fields[2], "z",   8,  sensor_msgs::msg::PointField::FLOAT32);
    fillField(cloud_msg.fields[3], "rgb", 12, sensor_msgs::msg::PointField::FLOAT32);
#else
    fillField(cloud_msg.fields[0], "x",   0,  sensor_msgs::PointField::FLOAT32);
    fillField(cloud_msg.fields[1], "y",   4,  sensor_msgs::PointField::FLOAT32);
    fillField(cloud_msg.fields[2], "z",   8,  sensor_msgs::PointField::FLOAT32);
    fillField(cloud_msg.fields[3], "rgb", 12, sensor_msgs::PointField::FLOAT32);
#endif
    cloud_msg.point_step = 16;
    cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(static_cast<size_t>(cloud_msg.row_step));

    // Pack: rgb encoded as a single float whose bit-pattern is 0x00RRGGBB,
    // matching pcl::PointXYZRGB so RViz "PointCloud2" displays it natively.
    uint8_t* dst = cloud_msg.data.data();
    for (int i = 0; i < n_written; ++i) {
      const auto& p = out_buf_[i];
      std::memcpy(dst + 0,  &p.x, 4);
      std::memcpy(dst + 4,  &p.y, 4);
      std::memcpy(dst + 8,  &p.z, 4);
      const uint32_t rgb_packed =
          (static_cast<uint32_t>(p.r) << 16) |
          (static_cast<uint32_t>(p.g) << 8)  |
          (static_cast<uint32_t>(p.b));
      float rgb_float;
      std::memcpy(&rgb_float, &rgb_packed, 4);
      std::memcpy(dst + 12, &rgb_float, 4);
      dst += 16;
    }

#if defined(ODIN_ROS2)
    if (cloud_pub_) cloud_pub_->publish(cloud_msg);
#else
    cloud_pub_.publish(cloud_msg);
#endif

    if (!timing_logged_.exchange(true)) {
      const auto dt = std::chrono::steady_clock::now() - frame_t0;
      const double ms = std::chrono::duration<double, std::milli>(dt).count();
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "First-frame pipeline latency: %.3f ms (n_pts=%d, %dx%d, step=%d)",
                    ms, n_written, W, H, params_.step);
      logInfo(buf);
    }
  }

  // --------------------------------------------------------------------
  // Logging helpers (ROS1/ROS2 thin wrappers)
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  void logInfo (const std::string& s) { RCLCPP_INFO (node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logWarn (const std::string& s) { RCLCPP_WARN (node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logError(const std::string& s) { RCLCPP_ERROR(node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logWarnThrottled(const std::string& s) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[%s] %s", info_.sn.c_str(), s.c_str());
  }
#else
  void logInfo (const std::string& s) { ROS_INFO ("[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logWarn (const std::string& s) { ROS_WARN ("[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logError(const std::string& s) { ROS_ERROR("[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logWarnThrottled(const std::string& s) {
    ROS_WARN_THROTTLE(2.0, "[%s] %s", info_.sn.c_str(), s.c_str());
  }
#endif

  // --------------------------------------------------------------------
  // Members
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<PointCloud2Msg>::SharedPtr cloud_pub_;
  rclcpp::Client<odin_driver_interfaces::srv::GetCalibration>::SharedPtr calibration_client_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  message_filters::Subscriber<ImageMsg> depth_sub_;
  message_filters::Subscriber<ImageMsg> image_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
  using Sync       = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;
#else
  ros::NodeHandle& nh_;
  ros::Publisher cloud_pub_;

  message_filters::Subscriber<ImageMsg> depth_sub_;
  message_filters::Subscriber<ImageMsg> image_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
  using Sync       = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;
#endif

  DeviceInfo info_;
  const DepthToCloudConfig& cfg_;

  // Per-device state
  std::atomic<bool> initialized_{false};
  std::atomic<bool> timing_logged_{false};
  std::string yaml_content_;
  bool calibrated_ = false;

  int img_width_  = 0;
  int img_height_ = 0;

  depth_to_cloud_core::CloudParams params_{};
  float Tlc_[16] = {0};
  std::vector<depth_to_cloud_core::XyzRgbPoint> out_buf_;
};

// ---------------------------------------------------------------------------
// DepthToPointCloudNode: manager. Owns global parameters and per-device
// workers; reacts to device_online / device_offline / resolution_change.
// ---------------------------------------------------------------------------
class DepthToPointCloudNode {
 public:
#if defined(ODIN_ROS2)
  explicit DepthToPointCloudNode(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    declareParameters();
    init();
  }
#else
  DepthToPointCloudNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    init();
  }
#endif

  ~DepthToPointCloudNode() {
    std::lock_guard<std::mutex> lock(workers_mutex_);
    workers_.clear();
  }

 private:
#if defined(ODIN_ROS2)
  void declareParameters() {
    node_->declare_parameter<std::string>("topic_prefix", "manifold");
    node_->declare_parameter<int>("step", 5);
    node_->declare_parameter<double>("d_min", 0.1);
    node_->declare_parameter<double>("d_max", 100.0);
    node_->declare_parameter<std::string>("depth_topic_suffix",  "pointcloud2depth/depth");
    node_->declare_parameter<std::string>("image_topic_suffix",  "camera0/undistort");
    node_->declare_parameter<std::string>("output_topic_suffix", "depth2cloud/cloud_color");
    node_->declare_parameter<std::string>("output_frame_id",     "");
    node_->declare_parameter<int>("sync_queue_size", 10);
    node_->declare_parameter<double>("sync_slop_sec", 0.1);
  }
#endif

  void loadParameters() {
#if defined(ODIN_ROS2)
    topic_prefix_base_       = node_->get_parameter("topic_prefix").as_string();
    cfg_.step                = node_->get_parameter("step").as_int();
    cfg_.d_min               = static_cast<float>(node_->get_parameter("d_min").as_double());
    cfg_.d_max               = static_cast<float>(node_->get_parameter("d_max").as_double());
    cfg_.depth_topic_suffix  = node_->get_parameter("depth_topic_suffix").as_string();
    cfg_.image_topic_suffix  = node_->get_parameter("image_topic_suffix").as_string();
    cfg_.output_topic_suffix = node_->get_parameter("output_topic_suffix").as_string();
    cfg_.output_frame_id     = node_->get_parameter("output_frame_id").as_string();
    cfg_.sync_queue_size     = node_->get_parameter("sync_queue_size").as_int();
    cfg_.sync_slop_sec       = node_->get_parameter("sync_slop_sec").as_double();
#else
    pnh_.param<std::string>("topic_prefix", topic_prefix_base_, "manifold");
    double dmin_d = 0.1, dmax_d = 100.0;
    pnh_.param<int>("step", cfg_.step, 5);
    pnh_.param<double>("d_min", dmin_d, 0.1);
    pnh_.param<double>("d_max", dmax_d, 100.0);
    cfg_.d_min = static_cast<float>(dmin_d);
    cfg_.d_max = static_cast<float>(dmax_d);
    pnh_.param<std::string>("depth_topic_suffix",  cfg_.depth_topic_suffix,  std::string("pointcloud2depth/depth"));
    pnh_.param<std::string>("image_topic_suffix",  cfg_.image_topic_suffix,  std::string("camera0/undistort"));
    pnh_.param<std::string>("output_topic_suffix", cfg_.output_topic_suffix, std::string("depth2cloud/cloud_color"));
    pnh_.param<std::string>("output_frame_id",     cfg_.output_frame_id,     std::string(""));
    pnh_.param<int>("sync_queue_size", cfg_.sync_queue_size, 10);
    pnh_.param<double>("sync_slop_sec", cfg_.sync_slop_sec, 0.1);
#endif
  }

  void init() {
    loadParameters();
    startDeviceDiscovery();
    logInfo("DepthToPointCloudNode initialized. Waiting for devices...");
  }

  std::string buildDriverTopicName(const std::string& suffix) const {
    if (topic_prefix_base_.empty()) return "/driver/" + suffix;
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  void startDeviceDiscovery() {
#if defined(ODIN_ROS2)
    device_online_sub_ = node_->create_subscription<StringMsg>(
        buildDriverTopicName("device_online"),
        rclcpp::QoS(10).transient_local(),
        [this](const StringMsg::SharedPtr msg) { onDeviceOnline(msg->data); });
    device_offline_sub_ = node_->create_subscription<StringMsg>(
        buildDriverTopicName("device_offline"),
        rclcpp::QoS(10).transient_local(),
        [this](const StringMsg::SharedPtr msg) { onDeviceOffline(msg->data); });
    resolution_change_sub_ = node_->create_subscription<StringMsg>(
        buildDriverTopicName("resolution_change"),
        rclcpp::QoS(1).transient_local(),
        [this](const StringMsg::SharedPtr msg) { onResolutionChange(msg->data); });
#else
    device_online_sub_ = nh_.subscribe<StringMsg>(
        buildDriverTopicName("device_online"), 100,
        [this](const boost::shared_ptr<StringMsg const>& msg) { onDeviceOnline(msg->data); });
    device_offline_sub_ = nh_.subscribe<StringMsg>(
        buildDriverTopicName("device_offline"), 10,
        [this](const boost::shared_ptr<StringMsg const>& msg) { onDeviceOffline(msg->data); });
    resolution_change_sub_ = nh_.subscribe<StringMsg>(
        buildDriverTopicName("resolution_change"), 10,
        [this](const boost::shared_ptr<StringMsg const>& msg) { onResolutionChange(msg->data); });
#endif
  }

  void onDeviceOnline(const std::string& json) {
    DeviceInfo info;
    if (!ParseDeviceInfoJson(json, info)) {
      logWarn("Failed to parse device_online: " + json);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(workers_mutex_);
      auto it = workers_.find(info.sn);
      if (it != workers_.end()) return;
      workers_[info.sn] = nullptr;
    }
    logInfo("New device online: sn=" + info.sn + " model=" + info.model + " prefix=" + info.prefix);

    std::thread([this, info]() {
#if defined(ODIN_ROS2)
      auto worker = std::make_unique<DeviceWorker>(node_, info, cfg_);
#else
      auto worker = std::make_unique<DeviceWorker>(nh_, info, cfg_);
#endif
      std::lock_guard<std::mutex> lock(workers_mutex_);
      workers_[info.sn] = std::move(worker);
      logInfo("DeviceWorker registered for " + info.sn +
              " (total devices: " + std::to_string(workers_.size()) + ")");
    }).detach();
  }

  void onDeviceOffline(const std::string& payload) {
    // device_offline carries JSON {"sn":"...","model":"..."}; fall back to a
    // plain SN string for compatibility with older driver versions.
    std::string sn;
    ParseJsonStringField(payload, "sn", sn);
    if (sn.empty()) sn = payload;
    std::unique_ptr<DeviceWorker> dying;
    {
      std::lock_guard<std::mutex> lock(workers_mutex_);
      auto it = workers_.find(sn);
      if (it == workers_.end()) {
        logWarn("device_offline for unknown sn: " + sn);
        return;
      }
      dying = std::move(it->second);
      workers_.erase(it);
    }
    if (dying) logInfo("DeviceWorker removed for " + sn);
  }

  void onResolutionChange(const std::string& json) {
    std::string sn;
    ParseJsonStringField(json, "sn", sn);
    const int width  = ParseJsonIntField(json, "width");
    const int height = ParseJsonIntField(json, "height");
    if (sn.empty() || width <= 0 || height <= 0) {
      logWarn("Invalid resolution_change JSON: " + json);
      return;
    }
    std::lock_guard<std::mutex> lock(workers_mutex_);
    auto it = workers_.find(sn);
    if (it == workers_.end()) return;
    if (it->second) it->second->onResolutionChange(width, height);
  }

#if defined(ODIN_ROS2)
  void logInfo (const std::string& s) { RCLCPP_INFO (node_->get_logger(), "%s", s.c_str()); }
  void logWarn (const std::string& s) { RCLCPP_WARN (node_->get_logger(), "%s", s.c_str()); }
  void logError(const std::string& s) { RCLCPP_ERROR(node_->get_logger(), "%s", s.c_str()); }
#else
  void logInfo (const std::string& s) { ROS_INFO ("%s", s.c_str()); }
  void logWarn (const std::string& s) { ROS_WARN ("%s", s.c_str()); }
  void logError(const std::string& s) { ROS_ERROR("%s", s.c_str()); }
#endif

#if defined(ODIN_ROS2)
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<StringMsg>::SharedPtr device_online_sub_;
  rclcpp::Subscription<StringMsg>::SharedPtr device_offline_sub_;
  rclcpp::Subscription<StringMsg>::SharedPtr resolution_change_sub_;
#else
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber device_online_sub_;
  ros::Subscriber device_offline_sub_;
  ros::Subscriber resolution_change_sub_;
#endif

  DepthToCloudConfig cfg_;
  std::string topic_prefix_base_ = "manifold";

  std::map<std::string, std::unique_ptr<DeviceWorker>> workers_;
  std::mutex workers_mutex_;
};

}  // namespace odin

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  odin::print_app_version("depth_to_pointcloud_node");
#if defined(ODIN_ROS2)
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("depth_to_pointcloud_node");
  bool enable = node->declare_parameter<bool>("enable_depth_to_pointcloud", false);
  if (!enable) {
    std::cout << "[depth2cloud] disabled by param, exiting" << std::endl;
    rclcpp::shutdown();
    return 0;
  }
  odin::DepthToPointCloudNode manager(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
#else
  ros::init(argc, argv, "depth_to_pointcloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  bool enable = false;
  pnh.param("enable_depth_to_pointcloud", enable, false);
  if (!enable) {
    ROS_INFO("[depth2cloud] disabled by param, exiting");
    return 0;
  }
  odin::DepthToPointCloudNode manager(nh, pnh);
  ros::spin();
#endif
  return 0;
}
