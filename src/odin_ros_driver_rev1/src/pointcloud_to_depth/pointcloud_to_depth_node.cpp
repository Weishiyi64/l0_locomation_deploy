// @file pointcloud2depth_node.cpp
// @brief Multi-device ROS node that projects each Odin device's dToF point
//        cloud onto its undistorted camera image to produce a dense depth
//        map (data + visualizations).
//
// Architecture mirrors post_process_node:
//   - The top-level Pointcloud2DepthNode owns the global parameter set and
//     manages a map of per-device DeviceWorker objects keyed by serial number.
//   - device_online    -> spawn a new DeviceWorker for that sn
//   - device_offline   -> destroy the DeviceWorker (DDS pubs/subs released)
//   - resolution_change-> dispatched to the matching DeviceWorker
//
// Each DeviceWorker holds its own:
//   - get_calibration client and YAML content
//   - low-res projection canvas (Kcl, out_width/out_height)
//   - cloud/raw + camera0/undistort ApproximateTime synchronizer
//   - publishers: depth / depth_preview / depth_color / depth_overlay
//   - first-frame latency one-shot log
//
// Published topics per device (under the discovered device prefix):
//   {prefix}pointcloud2depth/depth          sensor_msgs/Image  32FC1 (full-res)
//   {prefix}pointcloud2depth/depth_preview  sensor_msgs/Image  mono8 (full-res)
//   {prefix}pointcloud2depth/depth_color    sensor_msgs/Image  bgr8  (jet)
//   {prefix}pointcloud2depth/depth_overlay  sensor_msgs/Image  bgr8  (jet on RGB)
//
// The plain-types projection core (pointcloud2depth_node.hpp) is intentionally
// header-only and free of PCL/Eigen/OpenCV/ROS so it can be lifted into the
// device-side SDK as a pure C/C++ module without modification.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
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
#include <std_msgs/String.h>

#include "service_types.h"

using StringMsg      = std_msgs::String;
using ImageMsg       = sensor_msgs::Image;
using PointCloud2Msg = sensor_msgs::PointCloud2;
#endif

#if defined(ODIN_ROS2)
#include <ament_index_cpp/get_package_share_directory.hpp>
#else
#include <ros/package.h>
#endif
#include <yaml-cpp/yaml.h>
#include <cstring>
#include <algorithm>

#include "pointcloud_to_depth_node.hpp"
#include "yaml_config_loader.hpp"

namespace odin {

// ---------------------------------------------------------------------------
// Small JSON helpers (same flavor as pointcloud_reprojector_node).
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
  std::string sn;       // device serial number
  std::string prefix;   // already includes leading + trailing '/'
};

static bool ParseDeviceInfoJson(const std::string& json, DeviceInfo& info) {
  ParseJsonStringField(json, "sn", info.sn);
  ParseJsonStringField(json, "prefix", info.prefix);
  return !info.sn.empty() && !info.prefix.empty();
}

// ---------------------------------------------------------------------------
// Shared tunables. The manager loads these once from ROS params, then passes
// a const reference into every per-device worker so behavior stays uniform.
// ---------------------------------------------------------------------------
struct Pointcloud2DepthConfig {
  // Projection
  double depth_rate_hz = 10.0;
  int output_width = 64;
  int output_height = 40;
  float scale               = 7.0f;   // low-res canvas = img / scale
  float z_min               = 0.1f;
  float z_max               = 50.0f;
  int   dilate_radius       = 1;
  bool  use_z_buffer        = false;
  float edge_grad_threshold = 0.75f;  // Sobel threshold on full-res depth (m)
  bool  raw_depth_preview_enabled = false;
  // Jet colormap normalization (m). vis_depth_min < 0 -> per-frame auto.
  float vis_depth_min = 0.1f;
  float vis_depth_max = 5.0f;
  // Synchronization
  int    sync_queue_size = 5;
  double sync_slop_sec   = 0.1;
  // Topic naming (suffixes appended to the per-device prefix)
  std::string cloud_topic_suffix  = "cloud/raw";
  std::string image_topic_suffix  = "camera0/undistort";
  std::string output_topic_suffix = "pointcloud2depth";
};

// ---------------------------------------------------------------------------
// DeviceWorker: encapsulates everything needed to process one device. One
// instance per discovered serial number; lifetime managed by the manager
// via device_online / device_offline.
// ---------------------------------------------------------------------------
class DeviceWorker {
 public:
#if defined(ODIN_ROS2)
  DeviceWorker(rclcpp::Node::SharedPtr node, DeviceInfo info,
               const Pointcloud2DepthConfig& cfg)
      : node_(std::move(node)), info_(std::move(info)), cfg_(cfg) {
    logInfo("Worker created (prefix=" + info_.prefix + ")");
    fetchCalibrationAsync();
  }
#else
  DeviceWorker(ros::NodeHandle& nh, DeviceInfo info,
               const Pointcloud2DepthConfig& cfg)
      : nh_(nh), info_(std::move(info)), cfg_(cfg) {
    logInfo("Worker created (prefix=" + info_.prefix + ")");
    fetchCalibrationAsync();
  }
#endif

  ~DeviceWorker() {
    logInfo("Worker destroying");
#if defined(ODIN_ROS2)
    alive_->store(false);
    // Reset pubs/subs explicitly so topics disappear immediately on unplug.
    depth_pub_.reset();
    depth_sru_pub_.reset();
    depth_preview_pub_.reset();
    depth_color_pub_.reset();
    depth_overlay_pub_.reset();
    if (calibration_client_ && calibration_pending_)
      calibration_client_->remove_pending_request(calibration_request_id_);
    calibration_client_.reset();
    if (retry_timer_) retry_timer_->cancel();
    retry_timer_.reset();
    if (depth_timer_) depth_timer_->cancel();
    depth_timer_.reset();
    sync_.reset();
    cloud_sub_.unsubscribe();
    image_sub_.unsubscribe();
#else
    sync_.reset();
    cloud_sub_.unsubscribe();
    image_sub_.unsubscribe();
    depth_pub_.shutdown();
    depth_sru_pub_.shutdown();
    depth_preview_pub_.shutdown();
    depth_color_pub_.shutdown();
    depth_overlay_pub_.shutdown();
#endif
  }

  DeviceWorker(const DeviceWorker&) = delete;
  DeviceWorker& operator=(const DeviceWorker&) = delete;

  const std::string& sn()     const { return info_.sn; }
  const std::string& prefix() const { return info_.prefix; }

  // Manager dispatches resolution_change here. Safe to call before the worker
  // finishes calibrating; rebuildKcl picks up the new dimensions later.
  void onResolutionChange(int width, int height) {
    if (width <= 0 || height <= 0) return;
    if (width == img_width_ && height == img_height_) return;
    logInfo("Resolution change: " + std::to_string(img_width_) + "x" +
            std::to_string(img_height_) + " -> " +
            std::to_string(width) + "x" + std::to_string(height));
    img_width_  = width;
    img_height_ = height;
    if (!yaml_content_.empty()) {
      rebuildKcl();
    }
  }

 private:
  void fetchCalibrationAsync() {
#if defined(ODIN_ROS2)
    // Worker creation and all callbacks use the default mutually exclusive
    // callback group. Never block it waiting for a calibration service.
    if (!calibration_client_)
      calibration_client_ = node_->create_client<odin_driver_interfaces::srv::GetCalibration>(
          info_.prefix + "get_calibration");
    if (!retry_timer_)
      retry_timer_ = node_->create_wall_timer(std::chrono::seconds(1),
          [this]() { fetchCalibrationAsync(); });
    if (calibration_pending_) {
      if (std::chrono::steady_clock::now() - calibration_sent_ < std::chrono::seconds(2))
        return;
      calibration_client_->remove_pending_request(calibration_request_id_);
      calibration_pending_ = false;
      logWarn("get_calibration timed out; retrying");
    }
    if (!calibration_client_->service_is_ready()) return;
    auto request = std::make_shared<odin_driver_interfaces::srv::GetCalibration::Request>();
    const std::weak_ptr<std::atomic<bool>> weak_alive = alive_;
    auto pending = calibration_client_->async_send_request(
        request,
        [this, weak_alive](rclcpp::Client<odin_driver_interfaces::srv::GetCalibration>::SharedFuture f) {
          const auto alive = weak_alive.lock();
          if (!alive || !alive->load()) return;
          calibration_pending_ = false;
          try {
            auto response = f.get();
            if (!response->success) {
              logWarn("get_calibration returned failure; retrying");
              return;
            }
            yaml_content_ = response->yaml_content;
            if (!rebuildKcl()) return;
            setupPubSub();
            retry_timer_->cancel();
            logInfo("Calibration fetched.");
          } catch (const std::exception& e) {
            logError(std::string("get_calibration future exception; retrying: ") + e.what());
          }
        });
    calibration_request_id_ = pending.request_id;
    calibration_sent_ = std::chrono::steady_clock::now();
    calibration_pending_ = true;
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
    rebuildKcl();
    setupPubSub();
#endif
  }

  // Parse YAML, resolve cam key by resolution, build DepthParams and the two Kcls.
  bool rebuildKcl() {
    try {
      return rebuildKclImpl();
    } catch (const std::exception& e) {
      out_width_ = out_height_ = 0;
      logError(std::string("Invalid calibration; depth disabled: ") + e.what());
      return false;
    }
  }

  bool rebuildKclImpl() {
    out_width_ = out_height_ = 0;
    YAMLConfigLoader loader;
    if (!loader.loadFromString(yaml_content_)) {
      logError("Failed to parse calibration YAML");
      return false;
    }

    YAML::Node root = YAML::Load(yaml_content_);
    const std::string cam_key = yaml_utils::ResolveCamKey(root, 0, img_width_, img_height_);
    if (cam_key.empty()) { logError("No camera0 calibration"); return false; }
    YAML::Node cam = root[cam_key];
    const int img_w = cam["image_width"].as<int>();
    const int img_h = cam["image_height"].as<int>();
    if (img_w <= 0 || img_h <= 0) { logError("Invalid calibration resolution"); return false; }
    // Keep the original calibrated projection and full-resolution depth product.
    // SRU resizing happens only after the existing depth pipeline has finished.
    if (img_width_ <= 0) img_width_ = img_w;
    if (img_height_ <= 0) img_height_ = img_h;
    depth_core::DepthParams params{};
    params.A11 = cam["A11"].as<float>() * img_width_ / img_w;
    params.A12 = cam["A12"] ? cam["A12"].as<float>() * img_width_ / img_w : 0.f;
    params.A22 = cam["A22"].as<float>() * img_height_ / img_h;
    params.u0 = cam["u0"].as<float>() * img_width_ / img_w;
    params.v0 = cam["v0"].as<float>() * img_height_ / img_h;
    std::vector<double> tcl_v = loader.getVector<double>("Tcl_0");
    if (tcl_v.size() != 16 || !std::all_of(tcl_v.begin(), tcl_v.end(), [](double x){ return std::isfinite(x); }) ||
        !std::isfinite(params.A11) || !std::isfinite(params.A22) || params.A11 <= 0 || params.A22 <= 0) {
      logError("Invalid camera0 intrinsics/Tcl_0; depth disabled (no identity calibration fallback)");
      return false;
    }
    for (int i = 0; i < 16; ++i) params.Tcl[i] = static_cast<float>(tcl_v[i]);
    params.scale = cfg_.scale;
    params.out_width = std::max(1, static_cast<int>(img_width_ / cfg_.scale));
    params.out_height = std::max(1, static_cast<int>(img_height_ / cfg_.scale));
    params.z_min         = cfg_.z_min;
    params.z_max         = cfg_.z_max;
    params.dilate_radius = cfg_.dilate_radius;

    out_width_  = params.out_width;
    out_height_ = params.out_height;

    depth_core::compute_kcl(params, Kcl_depth_);
    // Include the calibrated skew, matching camera0/undistort.
    for (int j = 0; j < 4; ++j) Kcl_depth_[j] += params.A12 * params.Tcl[4+j] / params.scale;

    {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "Calibration ready. img=%dx%d -> depth=%dx%d (scale=%.2f) "
                    "fx=%.2f fy=%.2f cx=%.2f cy=%.2f z=[%.2f,%.2f] dilate=%d",
                    img_w, img_h, params.out_width, params.out_height, params.scale,
                    params.A11, params.A22, params.u0, params.v0,
                    params.z_min, params.z_max, params.dilate_radius);
      logInfo(buf);
    }
    return true;
  }

  void setupPubSub() {
    if (initialized_.exchange(true)) return;  // already set up

    const std::string depth_topic         = info_.prefix + cfg_.output_topic_suffix + "/depth";
    const std::string depth_sru_topic = info_.prefix + cfg_.output_topic_suffix + "/depth_sru";
    const std::string depth_preview_topic = info_.prefix + cfg_.output_topic_suffix + "/depth_preview";
    const std::string depth_color_topic   = info_.prefix + cfg_.output_topic_suffix + "/depth_color";
    const std::string depth_overlay_topic = info_.prefix + cfg_.output_topic_suffix + "/depth_overlay";

#if defined(ODIN_ROS2)
    // Image streams use RELIABLE + KEEP_LAST(5) + VOLATILE: this matches the
    // RViz Image display default (RELIABLE) so RViz can subscribe at all,
    // while the small KEEP_LAST queue prevents stale-frame backpressure that
    // caused the previous stutter (1296x1600 32FC1 = ~8 MB/frame; a deep
    // queue would accumulate and the viewer would always lag behind).
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    depth_pub_         = node_->create_publisher<ImageMsg>(depth_topic,         qos);
    depth_sru_pub_ = node_->create_publisher<ImageMsg>(depth_sru_topic, qos);
    if (cfg_.raw_depth_preview_enabled) {
      auto preview_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                             .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                             .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      depth_preview_pub_ = node_->create_publisher<ImageMsg>(depth_preview_topic, preview_qos);
    }
    depth_color_pub_   = node_->create_publisher<ImageMsg>(depth_color_topic,   qos);
    depth_overlay_pub_ = node_->create_publisher<ImageMsg>(depth_overlay_topic, qos);

    depth_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / cfg_.depth_rate_hz), [this]() {
      auto cloud = std::move(pending_cloud_);
      auto image = std::move(pending_image_);
      if (cloud && image && std::chrono::steady_clock::now() - pending_time_ < std::chrono::milliseconds(500))
        processFrame(*cloud, image);
    });
    auto sub_qos = rclcpp::QoS(5)
                       .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                       .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    cloud_sub_.subscribe(node_, info_.prefix + cfg_.cloud_topic_suffix, sub_qos.get_rmw_qos_profile());
    image_sub_.subscribe(node_, info_.prefix + cfg_.image_topic_suffix, sub_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<Sync>(SyncPolicy(cfg_.sync_queue_size), cloud_sub_, image_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(cfg_.sync_slop_sec));
    sync_->registerCallback(
        std::bind(&DeviceWorker::syncCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
#else
    depth_pub_         = nh_.advertise<ImageMsg>(depth_topic,         10);
    depth_sru_pub_ = nh_.advertise<ImageMsg>(depth_sru_topic, 5);
    if (cfg_.raw_depth_preview_enabled)
      depth_preview_pub_ = nh_.advertise<ImageMsg>(depth_preview_topic, 1);
    depth_color_pub_   = nh_.advertise<ImageMsg>(depth_color_topic,   10);
    depth_overlay_pub_ = nh_.advertise<ImageMsg>(depth_overlay_topic, 10);

    cloud_sub_.subscribe(nh_, info_.prefix + cfg_.cloud_topic_suffix, 30);
    image_sub_.subscribe(nh_, info_.prefix + cfg_.image_topic_suffix, 30);
    sync_ = std::make_shared<Sync>(SyncPolicy(cfg_.sync_queue_size), cloud_sub_, image_sub_);
    sync_->registerCallback(boost::bind(&DeviceWorker::syncCallback, this, _1, _2));
#endif

    logInfo("Publishing: " + depth_topic);
    logInfo("Publishing adapted depth: " + depth_sru_topic);
    if (cfg_.raw_depth_preview_enabled)
      logInfo("Publishing full-resolution preview: " + depth_preview_topic);
    logInfo("Publishing: " + depth_color_topic);
    logInfo("Publishing: " + depth_overlay_topic);
  }

  // --------------------------------------------------------------------
  // Hot path
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  void syncCallback(const PointCloud2Msg::ConstSharedPtr& cloud,
                    const ImageMsg::ConstSharedPtr& image) {
    pending_cloud_ = cloud;
    pending_image_ = image;
    pending_time_ = std::chrono::steady_clock::now();
  }
#else
  void syncCallback(const PointCloud2Msg::ConstPtr& cloud,
                    const ImageMsg::ConstPtr& image) {
    processFrame(*cloud, image);
  }
#endif

  template <typename ImagePtr>
  void processFrame(const PointCloud2Msg& cloud, const ImagePtr& image_msg) {
    if (out_width_ <= 0 || out_height_ <= 0) return;  // not yet calibrated
    const bool want_depth_sru = hasSubscribers(depth_sru_pub_);
    // The full-resolution depth image is a primary data product.  Keep it
    // available for late-joining consumers instead of gating it on the
    // current subscription count.
    const bool want_depth = true;
    const bool want_preview = cfg_.raw_depth_preview_enabled;
    const bool want_color = hasSubscribers(depth_color_pub_);
    const bool want_overlay = hasSubscribers(depth_overlay_pub_);
    if (!want_depth_sru && !want_depth && !want_preview && !want_color && !want_overlay) return;

    // ---- Extract xyz array from PointCloud2 (assumes float32 x/y/z fields) ----
    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto& f : cloud.fields) {
      if      (f.name == "x") x_off = f.offset;
      else if (f.name == "y") y_off = f.offset;
      else if (f.name == "z") z_off = f.offset;
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) {
      logWarn("PointCloud2 missing x/y/z fields");
      return;
    }

    const size_t step = cloud.point_step;
    const size_t n_total = static_cast<size_t>(cloud.width) * cloud.height;
    if (cloud.is_bigendian || step == 0 ||
        static_cast<size_t>(std::max({x_off,y_off,z_off})+4) > step ||
        cloud.row_step < cloud.width*step || cloud.data.size() < static_cast<size_t>(cloud.row_step)*cloud.height) {
      logWarn("Invalid PointCloud2 layout"); return;
    }
    for (const auto& field : cloud.fields)
      if ((field.name == "x" || field.name == "y" || field.name == "z") && field.datatype != 7) {
        logWarn("Depth projection requires FLOAT32 xyz fields"); return;
      }
    std::vector<float> xyz;
    xyz.reserve(n_total*3);
    for (size_t row=0;row<cloud.height;++row) for(size_t col=0;col<cloud.width;++col) {
      const auto* p = cloud.data.data()+row*cloud.row_step+col*step;
      float x,y,z;
      std::memcpy(&x,p+x_off,4); std::memcpy(&y,p+y_off,4); std::memcpy(&z,p+z_off,4);
      xyz.push_back(x); xyz.push_back(y); xyz.push_back(z);
    }

    // ---- Decode RGB image for overlay color sampling ----
    cv::Mat rgb;
    try {
      rgb = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      logWarn(std::string("cv_bridge exception: ") + e.what());
      return;
    }

    // ---- Safety net for late-joining startup ----
    // The driver may publish resolution_change BEFORE this node subscribes
    // (even with TRANSIENT_LOCAL there are races at first launch). The
    // synchronized image is the ground truth for current resolution, so if
    // it disagrees with our internal state we resize the projection canvas.
    if (!rgb.empty() && !yaml_content_.empty() &&
        (rgb.cols != img_width_ || rgb.rows != img_height_)) {
      logInfo("Image dims differ from internal state (" +
              std::to_string(img_width_) + "x" + std::to_string(img_height_) +
              " -> " + std::to_string(rgb.cols) + "x" + std::to_string(rgb.rows) +
              "), rebuilding Kcl");
      img_width_  = rgb.cols;
      img_height_ = rgb.rows;
      rebuildKcl();
      if (out_width_ <= 0 || out_height_ <= 0) return;
    }

    if (rgb.empty()) return;

    // ---- Depth path: low-res depth map via plain-types core ----
    std::vector<float> depth_buf(static_cast<size_t>(out_width_) * out_height_, 0.0f);
    const uint32_t n_pts = static_cast<uint32_t>(n_total);
    depth_core::pointcloud_to_depth(
        xyz.data(), n_pts, Kcl_depth_,
        out_width_, out_height_, cfg_.dilate_radius,
        cfg_.z_min, cfg_.z_max, cfg_.use_z_buffer, depth_buf.data());

    cv::Mat depth_lowres(out_height_, out_width_, CV_32FC1, depth_buf.data());
    // Depth measures the cloud, even when the RGB pair has a nearby timestamp.
    // Keep the optical frame used by the projection, but preserve cloud provenance.
    auto depth_header = image_msg->header;
    depth_header.stamp = cloud.header.stamp;

    if (want_depth_sru) {
      cv::Mat depth_sru;
      cv::resize(depth_lowres, depth_sru, cv::Size(cfg_.output_width, cfg_.output_height),
                 0, 0, cv::INTER_NEAREST);

      // Clamp measured depth to the deployment range; missing data stays zero.
      for (int row = 0; row < depth_sru.rows; ++row) {
        float* values = depth_sru.ptr<float>(row);
        for (int col = 0; col < depth_sru.cols; ++col) {
          const float z = values[col];
          values[col] = std::isfinite(z) && z > 0.f
              ? std::max(0.2f, std::min(5.0f, z)) : 0.f;
        }
      }
      publishImage(depth_sru, "32FC1", depth_header, depth_sru_pub_);
    }

    if (want_depth || want_preview || want_color || want_overlay) {
      // Upsample only for optional full-resolution products.
      cv::Mat depth_full;
      cv::resize(depth_lowres, depth_full, cv::Size(rgb.cols, rgb.rows),
                 0, 0, cv::INTER_NEAREST);
      // ---- Step 4: Sobel edge suppression on FULL-RES depth ----
      // We run Sobel on the upsampled depth (not the low-res grid) on purpose:
      // doing it on low-res produces an edge mask that, after NN-upsample, is
      // ~scale pixels wide on the full image, which visibly over-erases depth
      // along discontinuities. Full-res Sobel keeps the wipeout band to ~2 px,
      // matching the visual fidelity the doc spec / customer expects.
      if (cfg_.edge_grad_threshold > 0.0f) {
        cv::Mat gx, gy, grad;
        cv::Sobel(depth_full, gx, CV_32F, 1, 0, 3);
        cv::Sobel(depth_full, gy, CV_32F, 0, 1, 3);
        cv::magnitude(gx, gy, grad);
        cv::Mat edge_mask = grad > cfg_.edge_grad_threshold;  // 8UC1, 0 or 255
        depth_full.setTo(0.0f, edge_mask);
      }
      if (want_depth) publishImage(depth_full, "32FC1", depth_header, depth_pub_);
      if (want_preview) {
        cv::Mat preview(depth_full.rows, depth_full.cols, CV_8UC1);
        const double max_depth = cfg_.vis_depth_max;
        for (int row = 0; row < depth_full.rows; ++row) {
          const float* depth_values = depth_full.ptr<float>(row);
          uint8_t* preview_values = preview.ptr<uint8_t>(row);
          for (int col = 0; col < depth_full.cols; ++col) {
            const float depth = depth_values[col];
            preview_values[col] = std::isfinite(depth) && depth > 0.0f
                ? static_cast<uint8_t>(1 + std::lround(254 * std::min(1.0, depth / max_depth)))
                : 0;
          }
        }
        publishImage(preview, "mono8", depth_header, depth_preview_pub_);
      }
      if (want_color || want_overlay) {
        cv::Mat jet_color, valid_mask;
        makeDepthJet(depth_full, jet_color, valid_mask);
        if (want_color) publishImage(jet_color, "bgr8", depth_header, depth_color_pub_);
        if (want_overlay) {
          cv::Mat depth_overlay = blendJetOnImage(jet_color, valid_mask, rgb);
          publishImage(depth_overlay, "bgr8", image_msg->header, depth_overlay_pub_);
        }
      }
    }

  }

  // Render the depth map as an OpenCV-style jet visualization where:
  //   - valid pixels (depth > 0) are mapped to the JET colormap by
  //     min-max-normalizing within the current frame
  //   - invalid pixels (depth == 0) stay solid black so the customer can
  //     immediately tell "no depth here" from "near object"
  // Outputs:
  //   color_out : BGR8 of the same size as depth, black where invalid
  //   mask_out  : 8UC1 (0 / 255) marking valid pixels for downstream blending
  void makeDepthJet(const cv::Mat& depth, cv::Mat& color_out, cv::Mat& mask_out) {
    color_out = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC3);
    mask_out  = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);
    if (depth.empty()) return;

    // 1) Valid mask (vectorized): mask_out is 8UC1, 255 where depth > 0.
    cv::compare(depth, 0.0f, mask_out, cv::CMP_GT);

    // 2) Pick the normalization range.
    //    - Default: fixed [cfg_.vis_depth_min, cfg_.vis_depth_max] (absolute coloring,
    //      a given physical depth always maps to the same color).
    //    - If cfg_.vis_depth_min < 0: per-frame auto-normalize (relative coloring;
    //      maximizes contrast but the same depth changes color across frames).
    float dmin, dmax;
    if (cfg_.vis_depth_min >= 0.0f && cfg_.vis_depth_max > cfg_.vis_depth_min) {
      dmin = cfg_.vis_depth_min;
      dmax = cfg_.vis_depth_max;
    } else {
      // cv::minMaxLoc is SIMD-accelerated and supports a mask, so we get the
      // per-frame valid-only [min, max] in one vectorized pass.
      double dmin_d = 0.0, dmax_d = 0.0;
      cv::minMaxLoc(depth, &dmin_d, &dmax_d, nullptr, nullptr, mask_out);
      if (dmax_d <= dmin_d) {
        // No valid pixels (or all identical) -> nothing meaningful to render.
        if (dmax_d == 0.0 && dmin_d == 0.0) return;
        dmax_d = dmin_d + 1e-3;
      }
      dmin = static_cast<float>(dmin_d);
      dmax = static_cast<float>(dmax_d);
    }

    // 3) Clamp depth into [dmin, dmax] then linearly map to 8U [1..255] via a
    //    single SIMD-accelerated convertTo (much faster than the per-pixel
    //    loop we used to do here).
    //
    //    The clamp leaves invalid pixels at dmin (from cv::max), so they would
    //    map to 1 (dark blue) after convertTo. We wipe those back to 0 using
    //    the valid mask so the final image keeps invalid pixels truly black.
    cv::Mat clamped;
    cv::max(depth, dmin, clamped);
    cv::min(clamped, dmax, clamped);
    const double alpha = 254.0 / (dmax - dmin);
    const double beta  = 1.0 - 254.0 * static_cast<double>(dmin) / (dmax - dmin);
    cv::Mat d8;
    clamped.convertTo(d8, CV_8UC1, alpha, beta);

    // 4) Apply JET colormap, then force invalid pixels to pure black so the
    //    customer can distinguish "no depth here" from "near object".
    cv::applyColorMap(d8, color_out, cv::COLORMAP_JET);
    cv::Mat invalid_mask;
    cv::bitwise_not(mask_out, invalid_mask);
    color_out.setTo(cv::Scalar(0, 0, 0), invalid_mask);
  }

  // Alpha-blend a jet-colored depth image onto a BGR camera image where the
  // mask is valid. Used by depth_overlay to verify alignment.
  cv::Mat blendJetOnImage(const cv::Mat& jet_color, const cv::Mat& mask,
                          const cv::Mat& rgb) {
    if (rgb.empty()) return cv::Mat();
    if (jet_color.empty() || mask.empty()) return rgb.clone();

    // Vectorized whole-image alpha blend, then keep the original RGB on
    // invalid pixels using the valid mask. This is ~5-10x faster than the
    // per-pixel scan it replaces because cv::addWeighted is SIMD-optimized.
    const double alpha = 0.55;
    cv::Mat blended;
    cv::addWeighted(jet_color, alpha, rgb, 1.0 - alpha, 0.0, blended);
    // Restore original RGB where the depth mask is invalid.
    rgb.copyTo(blended, ~mask);
    return blended;
  }

  // --------------------------------------------------------------------
  // Logging + publish helpers (ROS1/ROS2 thin wrappers)
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  void logInfo (const std::string& s) { RCLCPP_INFO (node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logWarn (const std::string& s) { RCLCPP_WARN (node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logError(const std::string& s) { RCLCPP_ERROR(node_->get_logger(), "[%s] %s", info_.sn.c_str(), s.c_str()); }

  bool hasSubscribers(const rclcpp::Publisher<ImageMsg>::SharedPtr& pub) const {
    return pub && (pub->get_subscription_count() + pub->get_intra_process_subscription_count() > 0);
  }

  template <typename HeaderT>
  void publishImage(const cv::Mat& img, const std::string& encoding,
                    const HeaderT& header,
                    rclcpp::Publisher<ImageMsg>::SharedPtr& pub) {
    if (!pub || img.empty()) return;
    cv_bridge::CvImage out;
    out.header   = header;
    out.encoding = encoding;
    out.image    = img;
    pub->publish(*out.toImageMsg());
  }
#else
  void logInfo (const std::string& s) { ROS_INFO ("[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logWarn (const std::string& s) { ROS_WARN ("[%s] %s", info_.sn.c_str(), s.c_str()); }
  void logError(const std::string& s) { ROS_ERROR("[%s] %s", info_.sn.c_str(), s.c_str()); }

  bool hasSubscribers(const ros::Publisher& pub) const {
    return pub.getNumSubscribers() > 0;
  }

  template <typename HeaderT>
  void publishImage(const cv::Mat& img, const std::string& encoding,
                    const HeaderT& header, ros::Publisher& pub) {
    if (img.empty()) return;
    cv_bridge::CvImage out;
    out.header   = header;
    out.encoding = encoding;
    out.image    = img;
    pub.publish(out.toImageMsg());
  }
#endif

  // --------------------------------------------------------------------
  // Members
  // --------------------------------------------------------------------
#if defined(ODIN_ROS2)
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ImageMsg>::SharedPtr depth_pub_, depth_sru_pub_, depth_preview_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr depth_color_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr depth_overlay_pub_;
  rclcpp::Client<odin_driver_interfaces::srv::GetCalibration>::SharedPtr calibration_client_;
  rclcpp::TimerBase::SharedPtr retry_timer_, depth_timer_;
  std::shared_ptr<std::atomic<bool>> alive_{std::make_shared<std::atomic<bool>>(true)};
  bool calibration_pending_ = false;
  int64_t calibration_request_id_ = 0;
  std::chrono::steady_clock::time_point calibration_sent_;
  PointCloud2Msg::ConstSharedPtr pending_cloud_;
  ImageMsg::ConstSharedPtr pending_image_;
  std::chrono::steady_clock::time_point pending_time_;

  message_filters::Subscriber<PointCloud2Msg> cloud_sub_;
  message_filters::Subscriber<ImageMsg> image_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg>;
  using Sync       = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;
#else
  ros::NodeHandle& nh_;
  ros::Publisher depth_pub_, depth_sru_pub_, depth_preview_pub_;
  ros::Publisher depth_color_pub_;
  ros::Publisher depth_overlay_pub_;

  message_filters::Subscriber<PointCloud2Msg> cloud_sub_;
  message_filters::Subscriber<ImageMsg> image_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg>;
  using Sync       = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;
#endif

  DeviceInfo info_;
  const Pointcloud2DepthConfig& cfg_;

  // Per-device state
  std::atomic<bool> initialized_{false};
  std::string yaml_content_;

  // Resolution (updated dynamically via resolution_change or YAML defaults)
  int img_width_  = 0;
  int img_height_ = 0;
  int out_width_  = 0;
  int out_height_ = 0;

  // Precomputed projection matrix (plain types): (K / scale) * Tcl
  float Kcl_depth_[16] = {0};
};

// ---------------------------------------------------------------------------
// Pointcloud2DepthNode: manager. Owns global parameters, listens for
// device_online / device_offline / resolution_change, and spawns / destroys
// per-device DeviceWorker instances.
// ---------------------------------------------------------------------------
class Pointcloud2DepthNode {
 public:
#if defined(ODIN_ROS2)
  explicit Pointcloud2DepthNode(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    declareParameters();
    init();
  }
#else
  Pointcloud2DepthNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    init();
  }
#endif

  ~Pointcloud2DepthNode() {
    std::lock_guard<std::mutex> lock(workers_mutex_);
    workers_.clear();
  }

 private:
#if defined(ODIN_ROS2)
  void declareParameters() {
    node_->declare_parameter<std::string>("topic_prefix", "manifold");
    node_->declare_parameter<double>("scale", 7.0);
    node_->declare_parameter<double>("depth_rate_hz", 10.0);
    node_->declare_parameter<int>("depth_width", 64);
    node_->declare_parameter<int>("depth_height", 40);
    node_->declare_parameter<double>("z_min", 0.1);
    node_->declare_parameter<double>("z_max", 50.0);
    node_->declare_parameter<int>("dilate_radius", 1);
    node_->declare_parameter<bool>("use_z_buffer", false);
    node_->declare_parameter<double>("edge_grad_threshold", 0.75);
    node_->declare_parameter<bool>("raw_depth_preview_enabled", false);
    node_->declare_parameter<double>("vis_depth_min", 0.1);
    node_->declare_parameter<double>("vis_depth_max", 5.0);
    node_->declare_parameter<std::string>("cloud_topic_suffix", "cloud/raw");
    node_->declare_parameter<std::string>("image_topic_suffix", "camera0/undistort");
    node_->declare_parameter<std::string>("output_topic_suffix", "pointcloud2depth");
    node_->declare_parameter<int>("sync_queue_size", 5);
    node_->declare_parameter<double>("sync_slop_sec", 0.1);
  }
#endif

  void loadParameters() {
#if defined(ODIN_ROS2)
    topic_prefix_base_       = node_->get_parameter("topic_prefix").as_string();
    cfg_.depth_rate_hz = node_->get_parameter("depth_rate_hz").as_double();
    cfg_.output_width = node_->get_parameter("depth_width").as_int();
    cfg_.output_height = node_->get_parameter("depth_height").as_int();
    if (!std::isfinite(cfg_.depth_rate_hz) || cfg_.depth_rate_hz <= 0 || cfg_.output_width <= 0 || cfg_.output_height <= 0)
      throw std::runtime_error("Depth rate and resolution must be positive");
    cfg_.scale               = static_cast<float>(node_->get_parameter("scale").as_double());
    cfg_.z_min               = static_cast<float>(node_->get_parameter("z_min").as_double());
    cfg_.z_max               = static_cast<float>(node_->get_parameter("z_max").as_double());
    cfg_.dilate_radius       = node_->get_parameter("dilate_radius").as_int();
    cfg_.use_z_buffer        = node_->get_parameter("use_z_buffer").as_bool();
    cfg_.edge_grad_threshold = static_cast<float>(node_->get_parameter("edge_grad_threshold").as_double());
    cfg_.raw_depth_preview_enabled = node_->get_parameter("raw_depth_preview_enabled").as_bool();
    cfg_.vis_depth_min       = static_cast<float>(node_->get_parameter("vis_depth_min").as_double());
    cfg_.vis_depth_max       = static_cast<float>(node_->get_parameter("vis_depth_max").as_double());
    cfg_.cloud_topic_suffix  = node_->get_parameter("cloud_topic_suffix").as_string();
    cfg_.image_topic_suffix  = node_->get_parameter("image_topic_suffix").as_string();
    cfg_.output_topic_suffix = node_->get_parameter("output_topic_suffix").as_string();
    cfg_.sync_queue_size     = node_->get_parameter("sync_queue_size").as_int();
    cfg_.sync_slop_sec       = node_->get_parameter("sync_slop_sec").as_double();
    if (cfg_.sync_queue_size <= 0 || !std::isfinite(cfg_.sync_slop_sec) || cfg_.sync_slop_sec < 0.0)
      throw std::runtime_error("Sync queue size must be positive and slop must be finite/nonnegative");
    if (cfg_.raw_depth_preview_enabled &&
        (!std::isfinite(cfg_.vis_depth_max) || cfg_.vis_depth_max <= 0.0f))
      throw std::runtime_error("vis_depth_max must be finite and positive for raw depth preview");
#else
    pnh_.param<std::string>("topic_prefix", topic_prefix_base_, "manifold");
    double scale_d = 7.0, zmin_d = 0.1, zmax_d = 50.0, edge_d = 0.75;
    double vmin_d = 0.1, vmax_d = 5.0;
    pnh_.param<double>("scale", scale_d, 7.0);
    pnh_.param<double>("z_min", zmin_d, 0.1);
    pnh_.param<double>("z_max", zmax_d, 50.0);
    pnh_.param<double>("edge_grad_threshold", edge_d, 0.75);
    pnh_.param<double>("vis_depth_min", vmin_d, 0.1);
    pnh_.param<double>("vis_depth_max", vmax_d, 5.0);
    cfg_.scale               = static_cast<float>(scale_d);
    cfg_.z_min               = static_cast<float>(zmin_d);
    cfg_.z_max               = static_cast<float>(zmax_d);
    cfg_.edge_grad_threshold = static_cast<float>(edge_d);
    cfg_.vis_depth_min       = static_cast<float>(vmin_d);
    cfg_.vis_depth_max       = static_cast<float>(vmax_d);
    pnh_.param<bool>("raw_depth_preview_enabled", cfg_.raw_depth_preview_enabled, false);
    if (cfg_.raw_depth_preview_enabled &&
        (!std::isfinite(cfg_.vis_depth_max) || cfg_.vis_depth_max <= 0.0f))
      throw std::runtime_error("vis_depth_max must be finite and positive for raw depth preview");
    pnh_.param<int>("dilate_radius", cfg_.dilate_radius, 1);
    pnh_.param<bool>("use_z_buffer", cfg_.use_z_buffer, false);
    pnh_.param<std::string>("cloud_topic_suffix", cfg_.cloud_topic_suffix, "cloud/raw");
    pnh_.param<std::string>("image_topic_suffix", cfg_.image_topic_suffix, "camera0/undistort");
    pnh_.param<std::string>("output_topic_suffix", cfg_.output_topic_suffix, "pointcloud2depth");
    pnh_.param<int>("sync_queue_size", cfg_.sync_queue_size, 5);
    pnh_.param<double>("sync_slop_sec", cfg_.sync_slop_sec, 0.1);
#endif
  }

  void init() {
    loadParameters();
    startDeviceDiscovery();
    logInfo("Pointcloud2DepthNode initialized. Waiting for devices...");
  }

  // Driver-side management topics use the unified /{prefix}/driver/<suffix> form.
  std::string buildDriverTopicName(const std::string& suffix) const {
    if (topic_prefix_base_.empty()) return "/driver/" + suffix;
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  void startDeviceDiscovery() {
#if defined(ODIN_ROS2)
    // TRANSIENT_LOCAL on all three subscriptions to mirror the driver-side
    // publishers, so this node correctly picks up latched device_online and
    // resolution_change messages even when it starts AFTER the driver.
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

  // ---- Hotplug callbacks ------------------------------------------------
  void onDeviceOnline(const std::string& json) {
    DeviceInfo info;
    if (!ParseDeviceInfoJson(json, info)) {
      logWarn("Failed to parse device_online: " + json);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(workers_mutex_);
      auto it = workers_.find(info.sn);
      if (it != workers_.end()) {
        // Already known (or pending). Re-announcements are common with
        // TRANSIENT_LOCAL latched topics; treat as no-op.
        return;
      }
      // Reserve the slot with a nullptr so concurrent device_online
      // re-announcements don't race to spawn duplicate workers.
      workers_[info.sn] = nullptr;
    }

    logInfo("New device online: sn=" + info.sn + " prefix=" + info.prefix);

    // ROS2 calibration is nonblocking. Construct on the executor so offline
    // and shutdown cannot race a detached constructor or resurrect a worker.
#if defined(ODIN_ROS2)
    auto worker = std::make_unique<DeviceWorker>(node_, info, cfg_);
#else
    auto worker = std::make_unique<DeviceWorker>(nh_, info, cfg_);
#endif
    std::lock_guard<std::mutex> lock(workers_mutex_);
    workers_[info.sn] = std::move(worker);
    logInfo("DeviceWorker registered for " + info.sn +
            " (total devices: " + std::to_string(workers_.size()) + ")");
  }

  void onDeviceOffline(const std::string& sn) {
    std::unique_ptr<DeviceWorker> dying;  // destruct outside the lock
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
    if (dying) {
      logInfo("DeviceWorker removed for " + sn);
    }
    // dying.reset() happens here, outside the lock, to avoid blocking other
    // hotplug callbacks while DDS tears down pubs/subs.
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
    if (it == workers_.end()) {
      // Resolution change arrived before device_online: ignore. The worker
      // will pick up the right resolution from the synchronized image via
      // the safety net inside processFrame.
      return;
    }
    if (it->second) {
      it->second->onResolutionChange(width, height);
    }
  }

  // ---- Logging (manager-level; workers prefix their own SN) ------------
#if defined(ODIN_ROS2)
  void logInfo (const std::string& s) { RCLCPP_INFO (node_->get_logger(), "%s", s.c_str()); }
  void logWarn (const std::string& s) { RCLCPP_WARN (node_->get_logger(), "%s", s.c_str()); }
  void logError(const std::string& s) { RCLCPP_ERROR(node_->get_logger(), "%s", s.c_str()); }
#else
  void logInfo (const std::string& s) { ROS_INFO ("%s", s.c_str()); }
  void logWarn (const std::string& s) { ROS_WARN ("%s", s.c_str()); }
  void logError(const std::string& s) { ROS_ERROR("%s", s.c_str()); }
#endif

  // ---- Members ----------------------------------------------------------
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

  Pointcloud2DepthConfig cfg_;
  std::string topic_prefix_base_ = "manifold";

  std::map<std::string, std::unique_ptr<DeviceWorker>> workers_;
  std::mutex workers_mutex_;
};

}  // namespace odin

// Locate package source directory to load config/control_command.yaml.
static std::string getPackageSourceDirectory() {
#if defined(ODIN_ROS2)
  try {
    return ament_index_cpp::get_package_share_directory(ODIN_ROS_PACKAGE_NAME);
  } catch (const std::exception& e) {
    return "";
  }
#else
  return ros::package::getPath(ODIN_ROS_PACKAGE_NAME);
#endif
}

// Read enable_pointcloud_to_depth from config/control_command.yaml.
// Returns true (enabled) by default when the config cannot be read.
static bool loadPointcloud2DepthEnabled() {
  std::string pkg_path = getPackageSourceDirectory();
  if (pkg_path.empty()) {
    std::cout << "[pointcloud2depth] package dir not found, using default (enable=1)" << std::endl;
    return true;
  }
  std::string config_file = pkg_path + "/config/control_command.yaml";
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    if (!config["register_keys"]) {
      std::cout << "[pointcloud2depth] register_keys missing, using default (enable=1)" << std::endl;
      return true;
    }
    YAML::Node keys = config["register_keys"];
    int enable = keys["enable_pointcloud_to_depth"]
                     ? keys["enable_pointcloud_to_depth"].as<int>()
                     : 1;
    std::cout << "[pointcloud2depth] enable_pointcloud_to_depth=" << enable << std::endl;
    return enable != 0;
  } catch (const std::exception& e) {
    std::cerr << "[pointcloud2depth] Failed to load " << config_file << ": " << e.what()
              << ", using default (enable=1)" << std::endl;
    return true;
  }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
#if defined(ODIN_ROS2)
  rclcpp::init(argc, argv);
  if (!loadPointcloud2DepthEnabled()) {
    std::cout << "[pointcloud2depth] disabled by config, exiting" << std::endl;
    rclcpp::shutdown();
    return 0;
  }
  auto node = std::make_shared<rclcpp::Node>("pointcloud2depth_node");
  odin::Pointcloud2DepthNode manager(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
#else
  ros::init(argc, argv, "pointcloud2depth_node");
  if (!loadPointcloud2DepthEnabled()) {
    ROS_INFO("[pointcloud2depth] disabled by config, exiting");
    return 0;
  }
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  odin::Pointcloud2DepthNode manager(nh, pnh);
  ros::spin();
#endif
  return 0;
}
