/**
 * @file post_process_node.cpp
 * @brief Unified ROS1/ROS2 post-processing node
 *
 * Uses ros_compat.hpp wrappers to minimize version-specific code.
 * Only service calls need #if/#endif due to different message generation.
 */

#include "odin_app_version.hpp"
#include "utility/ros_compat.hpp"

#if ROS_VERSION_MAJOR == 1
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include "service_types.h"
#else
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "service_types.h"
#endif

// cv_bridge header compatibility: Ubuntu 24.04 uses .hpp, earlier versions use .h
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ODIN_HAVE_LIBYUV
// libyuv fast colour conversion (YUV 4:2:0 → BGR), ~7x faster than
// cv::cvtColor on 1280x1088 frames (0.3 ms vs 2.2 ms). Optional dependency:
// enabled by CMake when libyuv-dev is present, see README.
#include <libyuv.h>
#endif

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <filesystem>

#include "color_undistort.hpp"
#include "h264_decoder.h"
#include "jpeg_decoder.h"
#include "rawcloud_render.h"
#include "yaml_config_loader.hpp"

using namespace ros_compat;

// Global Flags
static bool g_send_image_raw = false;
static bool g_send_cloud_render = false;
static bool g_send_undistort = false;  // manual camera0/undistort switch
static bool g_send_opencv_undistort = false;  // enable opencv_undistort_node processing

// Device info structure for multi-device support
struct DeviceInfo {
  std::string sn;
  std::string model;
  std::string prefix;
  std::string ip;
};

// Parse device info JSON: {"sn":"xxx","model":"Odin2Special","prefix":"/manifold/device0/","ip":"192.168.1.251"}
static bool ParseDeviceInfoJson(const std::string& json, DeviceInfo& info) {
  auto getValue = [&json](const std::string& key) -> std::string {
    std::string search = "\"" + key + "\":\"";
    size_t start = json.find(search);
    if (start == std::string::npos) return "";
    start += search.length();
    size_t end = json.find('"', start);
    if (end == std::string::npos) return "";
    return json.substr(start, end - start);
  };
  info.sn = getValue("sn");
  info.model = getValue("model");
  info.prefix = getValue("prefix");
  info.ip = getValue("ip");
  return !info.sn.empty() && !info.prefix.empty();
}

// Queue Item Types
struct ImageItem {
  Time stamp;
  CompressedImageConstPtr msg;
};

struct CloudItem {
  Time stamp;
  PointCloud2ConstPtr msg;
};

// Forward declaration
class PostProcessNode;

// DeviceProcessor - handles processing for a single device
class DeviceProcessor {
 public:
  DeviceProcessor(PostProcessNode* parent, const DeviceInfo& info);
  ~DeviceProcessor() {
    RC_LOG_INFO("DeviceProcessor destroying: %s, resetting pub/sub...", device_info_.sn.c_str());
    // Explicitly reset publishers and subscribers to ensure DDS cleanup
#if ROS_VERSION_MAJOR == 1
    mapping_active_sub_.shutdown();
#else
    mapping_active_sub_.reset();
#endif
    image_sub_ = Subscriber<CompressedImage>();
    cloud_sub_ = Subscriber<PointCloud2>();

    cloud_pub_ = Publisher<PointCloud2>();
    image_pub_ = Publisher<Image>();
    image_undistort_pub_ = Publisher<Image>();
    
    RC_LOG_INFO("DeviceProcessor destroyed: %s", device_info_.sn.c_str());
  }

  const std::string& sn() const { return device_info_.sn; }
  const std::string& prefix() const { return device_info_.prefix; }
  void SetupPubSub();

  void imageCallback(const CompressedImageConstPtr& img);
  void cloudCallback(const PointCloud2ConstPtr& cloud);

  // Reinitialize all resolution-dependent components with new image dimensions
  void ReinitComponents(int img_width, int img_height);

 private:
  bool FetchCalibrationFromService(std::string& yaml_content_out);
  void InitializeComponents(const std::string& yaml_content);

  template <typename T>
  void trimQueue(std::queue<T>& q) {
    while (static_cast<int>(q.size()) > max_queue_size_) q.pop();
  }
  void tryProcessPair();
  void renderCloud(const CompressedImageConstPtr& img, const PointCloud2ConstPtr& cloud);
  bool decodeCompressedImage(const CompressedImageConstPtr& img, cv::Mat& decoded);
  bool decodeNv12Image(const CompressedImageConstPtr& img, cv::Mat& decoded);

  PostProcessNode* parent_;
  DeviceInfo device_info_;

  // Publishers
  Publisher<PointCloud2> cloud_pub_;
  Publisher<Image> image_pub_;
  Publisher<Image> image_undistort_pub_;

  // Subscribers
#if ROS_VERSION_MAJOR == 1
  ros::Subscriber mapping_active_sub_;
#else
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mapping_active_sub_;
#endif
  bool mapping_active_ = false;
  Subscriber<CompressedImage> image_sub_;
  Subscriber<PointCloud2> cloud_sub_;

  // Processing components
  rawCloudRender render_;
  colorUndistort m_undistort_0_;

  bool render_initialized_ = false;
  bool undistort_0_initialized_ = false;

  std::string yaml_content_;  // Stored calibration YAML for reinit on resolution change
  int current_img_width_ = 0;
  int current_img_height_ = 0;

  int max_queue_size_ = 10;
  Duration sync_tol_;
  std::string output_frame_id_;

  std::queue<ImageItem> image_queue_;
  std::queue<CloudItem> cloud_queue_;
  std::mutex mutex_;

  // JPEG decoder (hardware or software)
  std::unique_ptr<odin_decoder::JpegDecoder> jpeg_decoder_;
  std::unique_ptr<odin_decoder::H264Decoder> h264_decoder_;

  // Logs the active decoder + image format once, on the first frame
  // actually received by imageCallback.
  bool logged_camera0_stream_info_ = false;
};

// PostProcessNode Class - manages multiple DeviceProcessors
class PostProcessNode : public Node {
 public:
  PostProcessNode() : Node("postprocess_node") {
    // Load global parameters
    max_queue_size_ = param<int>("max_queue_size", 10);
    sync_tolerance_ms_ = param<int>("sync_tolerance_ms", 10);
    output_frame_id_ = param<std::string>("output_frame_id", "");
    // Topic prefix base for driver management topics (default: "manifold")
    topic_prefix_base_ = param<std::string>("topic_prefix", "manifold");

    // Load post-processing flags from parameters
    // In ROS2, read from overrides to avoid double-declare conflict with SetupDynamicConfig()
#if ROS_VERSION_MAJOR == 2
    const auto &overrides = get_node_parameters_interface()->get_parameter_overrides();
    auto get_bool_override = [&overrides](const char *name, bool default_val) -> bool {
      auto it = overrides.find(name);
      if (it != overrides.end() &&
          it->second.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        return it->second.get<bool>();
      }
      return default_val;
    };
    g_send_image_raw    = get_bool_override("send_image_raw",    false);
    const bool send_image2_raw = get_bool_override("send_image2_raw",   false);
    g_send_cloud_render = get_bool_override("send_cloud_render", false);
    g_send_undistort    = get_bool_override("send_undistort",    false);
    const bool send_rectify = get_bool_override("send_rectify",      false);
    g_send_opencv_undistort = get_bool_override("send_opencv_undistort", false);
    post_process_       = get_bool_override("post_process",      false);
    undistort_mapping_only_ = get_bool_override("undistort_mapping_only", false);
#else
    g_send_image_raw   = param<bool>("send_image_raw", false);
    const bool send_image2_raw = param<bool>("send_image2_raw", false);
    g_send_cloud_render = param<bool>("send_cloud_render", false);
    g_send_undistort   = param<bool>("send_undistort", false);
    const bool send_rectify = param<bool>("send_rectify", false);
    g_send_opencv_undistort = param<bool>("send_opencv_undistort", false);
    post_process_      = param<bool>("post_process", false);
    undistort_mapping_only_ = param<bool>("undistort_mapping_only", false);
#endif

    if (send_image2_raw || send_rectify) {
      throw std::invalid_argument("This hardware has only camera0; send_image2_raw and send_rectify must be false");
    }

    SetupDynamicConfig();

    // Start listening for device_online messages only if enabled
    if (post_process_) {
      StartDeviceDiscovery();
      RC_LOG_INFO("PostProcessNode initialized, waiting for devices...");
      RC_LOG_INFO("Flags: image_raw=%d, cloud_render=%d, undistort=%d, opencv_undistort=%d",
               g_send_image_raw, g_send_cloud_render, g_send_undistort, g_send_opencv_undistort);
    } else {
      RC_LOG_INFO("PostProcessNode initialized but post_process=false");
    }
  }

  bool postProcessEnabled() const { return post_process_; }
  bool undistortMappingOnly() const { return undistort_mapping_only_; }

  // Accessor methods for DeviceProcessor
  int max_queue_size() const { return max_queue_size_; }
  int sync_tolerance_ms() const { return sync_tolerance_ms_; }
  const std::string& output_frame_id() const { return output_frame_id_; }

 private:
  void ForwardOpencvUndistortEnabled(bool enabled) {
#if ROS_VERSION_MAJOR == 2
    if (!opencv_undistort_param_client_) {
      opencv_undistort_param_client_ =
          std::make_shared<rclcpp::AsyncParametersClient>(this, "/opencv_undistort");
    }
    auto future = opencv_undistort_param_client_->set_parameters(
        {rclcpp::Parameter("send_opencv_undistort", enabled)});
    // Fire-and-forget: the opencv_undistort node's parameter callback
    // will update its processing_enabled_ flag asynchronously.
    (void)future;
    RC_LOG_INFO("Forwarded send_opencv_undistort=%d to /opencv_undistort", enabled);
#else
    ros::param::set("/opencv_undistort/send_opencv_undistort", enabled);
    RC_LOG_INFO("Forwarded send_opencv_undistort=%d to /opencv_undistort", enabled);
#endif
  }

  template <typename Response>
  void FillPostProcessState(Response &res) const {
    res.image_raw_enabled = g_send_image_raw;
    res.image2_raw_enabled = false;
    res.cloud_render_enabled = g_send_cloud_render;
    res.undistort_enabled = g_send_undistort;
    res.rectify_enabled = false;
    res.opencv_undistort_enabled = g_send_opencv_undistort;
  }

  void SetupDynamicConfig() {
#if ROS_VERSION_MAJOR == 2
    declare_parameter<bool>("send_image_raw", g_send_image_raw);
    rcl_interfaces::msg::ParameterDescriptor mono_desc;
    mono_desc.read_only = true;
    mono_desc.description = "Unsupported: this hardware has only camera0";
    declare_parameter<bool>("send_image2_raw", false, mono_desc);
    declare_parameter<bool>("send_cloud_render", g_send_cloud_render);
    declare_parameter<bool>("send_undistort", g_send_undistort);
    declare_parameter<bool>("send_rectify", false, mono_desc);
    rcl_interfaces::msg::ParameterDescriptor mapping_desc;
    mapping_desc.read_only = true;
    declare_parameter<bool>("undistort_mapping_only", undistort_mapping_only_, mapping_desc);
    declare_parameter<bool>("send_opencv_undistort", g_send_opencv_undistort);

    param_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &p : params) {
            if (p.get_name() == "send_image_raw")    g_send_image_raw    = p.as_bool();
            if (p.get_name() == "send_cloud_render") g_send_cloud_render = p.as_bool();
            if (p.get_name() == "send_undistort")    g_send_undistort    = p.as_bool();
            if (p.get_name() == "send_opencv_undistort") {
              g_send_opencv_undistort = p.as_bool();
              ForwardOpencvUndistortEnabled(g_send_opencv_undistort);
            }
          }
          RC_LOG_INFO("Post-process flags changed: image_raw=%d, cloud_render=%d, undistort=%d, opencv_undistort=%d",
                      g_send_image_raw, g_send_cloud_render, g_send_undistort, g_send_opencv_undistort);
          return result;
        });

    std::string topic_prefix = topic_prefix_base_.empty() ? "" : topic_prefix_base_ + "/";
    set_post_process_config_service_ = create_service<odin_driver_interfaces::srv::SetPostProcessConfig>(
        topic_prefix + "postprocess/set_post_process_config",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::SetPostProcessConfig::Request> req,
               std::shared_ptr<odin_driver_interfaces::srv::SetPostProcessConfig::Response> res) {
          if (req->send_image2_raw > 0 || req->send_rectify > 0) {
            res->success = false;
            res->message = "This hardware has only camera0; camera1 and stereo rectification are unsupported";
            FillPostProcessState(*res);
            return;
          }
          if (req->send_image_raw >= 0)    g_send_image_raw    = (req->send_image_raw != 0);
          if (req->send_cloud_render >= 0) g_send_cloud_render = (req->send_cloud_render != 0);
          if (req->send_undistort >= 0)    g_send_undistort    = (req->send_undistort != 0);
          if (req->send_opencv_undistort >= 0) {
            g_send_opencv_undistort = (req->send_opencv_undistort != 0);
            ForwardOpencvUndistortEnabled(g_send_opencv_undistort);
          }
          res->success = true;
          res->message = "Post-process config updated";
          FillPostProcessState(*res);
          RC_LOG_INFO("Post-process flags changed via service");
        });

    get_post_process_config_service_ = create_service<odin_driver_interfaces::srv::GetPostProcessConfig>(
        topic_prefix + "postprocess/get_post_process_config",
        [this](const std::shared_ptr<odin_driver_interfaces::srv::GetPostProcessConfig::Request> /*req*/,
               std::shared_ptr<odin_driver_interfaces::srv::GetPostProcessConfig::Response> res) {
          res->success = true;
          res->message = "Current post-process config";
          FillPostProcessState(*res);
        });
    RC_LOG_INFO("PostProcessConfig query service ready: %spostprocess/get_post_process_config", topic_prefix.c_str());
#else
    ddr_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(
        ros::NodeHandle("~"));

    ddr_->registerVariable<bool>("send_image_raw", g_send_image_raw,
        boost::bind(&PostProcessNode::onPostProcessFlagChanged, this, _1,
                    std::string("send_image_raw")),
        "Publish raw camera0 image", false, true);
    ddr_->registerVariable<bool>("send_cloud_render", g_send_cloud_render,
        boost::bind(&PostProcessNode::onPostProcessFlagChanged, this, _1,
                    std::string("send_cloud_render")),
        "Publish cloud-rendered image", false, true);
    ddr_->registerVariable<bool>("send_undistort", g_send_undistort,
        boost::bind(&PostProcessNode::onPostProcessFlagChanged, this, _1,
                    std::string("send_undistort")),
        "Publish undistorted images", false, true);
    ddr_->registerVariable<bool>("send_opencv_undistort", g_send_opencv_undistort,
        boost::bind(&PostProcessNode::onPostProcessFlagChanged, this, _1,
                    std::string("send_opencv_undistort")),
        "Enable OpenCV undistortion node processing", false, true);
    ddr_->publishServicesTopics();

    std::string topic_prefix = topic_prefix_base_.empty() ? "" : topic_prefix_base_ + "/";
    set_post_process_config_service_ = ros::NodeHandle().advertiseService(
        topic_prefix + "postprocess/set_post_process_config",
        &PostProcessNode::HandleSetPostProcessConfigService, this);
    RC_LOG_INFO("PostProcessConfig service ready: %spostprocess/set_post_process_config", topic_prefix.c_str());

    get_post_process_config_service_ = ros::NodeHandle().advertiseService(
        topic_prefix + "postprocess/get_post_process_config",
        &PostProcessNode::HandleGetPostProcessConfigService, this);
    RC_LOG_INFO("PostProcessConfig query service ready: %spostprocess/get_post_process_config", topic_prefix.c_str());
#endif
  }

#if ROS_VERSION_MAJOR == 1
  void onPostProcessFlagChanged(bool new_val, const std::string& name) {
    if (name == "send_image_raw")    g_send_image_raw    = new_val;
    if (name == "send_cloud_render") g_send_cloud_render = new_val;
    if (name == "send_undistort")    g_send_undistort    = new_val;
    if (name == "send_opencv_undistort") {
      g_send_opencv_undistort = new_val;
      ForwardOpencvUndistortEnabled(g_send_opencv_undistort);
    }
    RC_LOG_INFO("Post-process flag changed: %s=%d", name.c_str(), new_val);
  }

  bool HandleSetPostProcessConfigService(
      odin_driver_interfaces::srv::SetPostProcessConfig::Request &req,
      odin_driver_interfaces::srv::SetPostProcessConfig::Response &res) {
    if (req.send_image2_raw > 0 || req.send_rectify > 0) {
      res.success = false;
      res.message = "This hardware has only camera0; camera1 and stereo rectification are unsupported";
      FillPostProcessState(res);
      return true;
    }
    if (req.send_image_raw >= 0)    g_send_image_raw    = (req.send_image_raw != 0);
    if (req.send_cloud_render >= 0) g_send_cloud_render = (req.send_cloud_render != 0);
    if (req.send_undistort >= 0)    g_send_undistort    = (req.send_undistort != 0);
    if (req.send_opencv_undistort >= 0) {
      g_send_opencv_undistort = (req.send_opencv_undistort != 0);
      ForwardOpencvUndistortEnabled(g_send_opencv_undistort);
    }
    res.success = true;
    res.message = "Post-process config updated";
    FillPostProcessState(res);
    RC_LOG_INFO("Post-process flags changed via service");
    return true;
  }

  bool HandleGetPostProcessConfigService(
      odin_driver_interfaces::srv::GetPostProcessConfig::Request &/*req*/,
      odin_driver_interfaces::srv::GetPostProcessConfig::Response &res) {
    res.success = true;
    res.message = "Current post-process config";
    FillPostProcessState(res);
    return true;
  }
#endif

  // Helper to build driver topic name: "/{prefix}/driver/{suffix}" or "/driver/{suffix}" if prefix is empty
  // Unified with device topic format: /{prefix}/device{N}/...
  std::string BuildDriverTopicName(const std::string &suffix) const {
    if (topic_prefix_base_.empty()) {
      return "/driver/" + suffix;
    }
    return "/" + topic_prefix_base_ + "/driver/" + suffix;
  }

  void StartDeviceDiscovery() {
#if ROS_VERSION_MAJOR == 1
    // Subscribe to device_online topic for multi-device support
    // Use larger queue and transport hints to ensure all messages are received
    device_online_sub_ = nh().subscribe<std_msgs::String>(
        BuildDriverTopicName("device_online"), 100,
        [this](const boost::shared_ptr<std_msgs::String const>& msg) {
          OnDeviceInfoReceived(msg->data);
        });
    // Subscribe to device_offline topic for cleanup
    device_offline_sub_ = nh().subscribe<std_msgs::String>(
        BuildDriverTopicName("device_offline"), 10,
        [this](const boost::shared_ptr<std_msgs::String const>& msg) {
          OnDeviceOffline(msg->data);
        });
    // Subscribe to resolution_change topic for dynamic resolution switching
    resolution_change_sub_ = nh().subscribe<std_msgs::String>(
        BuildDriverTopicName("resolution_change"), 10,
        [this](const boost::shared_ptr<std_msgs::String const>& msg) {
          OnResolutionChange(msg->data);
        });
#else
    // Subscribe to device_online topic with transient_local QoS to receive latched messages
    device_online_sub_ = create_subscription<std_msgs::msg::String>(
        BuildDriverTopicName("device_online"), rclcpp::QoS(10).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { OnDeviceInfoReceived(msg->data); });
    // Disconnects are events. Replaying a previous disconnect can remove a
    // processor that just received the current device_online announcement.
    device_offline_sub_ = create_subscription<std_msgs::msg::String>(
        BuildDriverTopicName("device_offline"), rclcpp::QoS(10).reliable().durability_volatile(),
        [this](const std_msgs::msg::String::SharedPtr msg) { OnDeviceOffline(msg->data); });
    // Subscribe to resolution_change topic for dynamic resolution switching.
    // TRANSIENT_LOCAL matches the main driver publisher so a late-joining
    // post_process_node immediately receives the current resolution rather
    // than waiting for the next resolution event (which only fires when
    // the stream is restarted).
    resolution_change_sub_ = create_subscription<std_msgs::msg::String>(
        BuildDriverTopicName("resolution_change"),
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { OnResolutionChange(msg->data); });
#endif
  }

  void OnDeviceInfoReceived(const std::string& json) {
    RC_LOG_INFO("Received device_online: %s", json.c_str());

    DeviceInfo info;
    if (!ParseDeviceInfoJson(json, info)) {
      RC_LOG_WARN("Failed to parse device_online JSON: %s", json.c_str());
      return;
    }

    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(processors_mutex_);
      if (processors_.find(info.sn) != processors_.end()) {
        // Device already registered
        return;
      }
      // Mark as pending to avoid duplicate processing
      generation = ++next_device_generation_;
      processors_.emplace(info.sn, DeviceEntry{generation, nullptr});
    }

    RC_LOG_INFO("New device discovered: sn=%s, model=%s, prefix=%s, ip=%s", info.sn.c_str(),
                info.model.c_str(), info.prefix.c_str(), info.ip.c_str());

    // Create processor in a separate thread to avoid blocking callback
    std::thread([this, info, generation
#if ROS_VERSION_MAJOR == 2
                 , owner = shared_from_this()
#endif
    ]() {
#if ROS_VERSION_MAJOR == 2
      (void)owner;  // Keep the node alive while the calibration service is pending.
#endif
      try {
        auto processor = std::make_unique<DeviceProcessor>(this, info);
        std::lock_guard<std::mutex> lock(processors_mutex_);
        auto it = processors_.find(info.sn);
        if (it == processors_.end() || it->second.generation != generation || !ok()) {
          return;
        }
        processor->SetupPubSub();
        it->second.processor = std::move(processor);
        RC_LOG_INFO("DeviceProcessor created for %s, total devices: %zu", info.sn.c_str(),
                    processors_.size());
      } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(processors_mutex_);
        const auto it = processors_.find(info.sn);
        if (it != processors_.end() && it->second.generation == generation) {
          processors_.erase(it);
        }
        RC_LOG_ERROR("Failed to initialize camera0 processing for %s: %s", info.sn.c_str(), e.what());
      }
    }).detach();
  }

  void OnResolutionChange(const std::string& json) {
    // JSON format: {"sn":"P030300006","width":640,"height":544}
    auto getStr = [&json](const std::string& key) -> std::string {
      std::string search = "\"" + key + "\":\"";
      size_t start = json.find(search);
      if (start == std::string::npos) return "";
      start += search.length();
      size_t end = json.find('"', start);
      if (end == std::string::npos) return "";
      return json.substr(start, end - start);
    };
    auto getInt = [&json](const std::string& key) -> int {
      std::string search = "\"" + key + "\":";
      size_t start = json.find(search);
      if (start == std::string::npos) return 0;
      start += search.length();
      return std::atoi(json.c_str() + start);
    };

    std::string sn = getStr("sn");
    int width = getInt("width");
    int height = getInt("height");
    if (sn.empty() || width <= 0 || height <= 0) {
      RC_LOG_WARN("Invalid resolution_change JSON: %s", json.c_str());
      return;
    }

    RC_LOG_INFO("Resolution change for %s: %dx%d", sn.c_str(), width, height);
    std::lock_guard<std::mutex> lock(processors_mutex_);
    auto it = processors_.find(sn);
    if (it != processors_.end() && it->second.processor) {
      it->second.processor->ReinitComponents(width, height);
    }
  }

  void OnDeviceOffline(const std::string& payload) {
    // device_offline carries JSON {"sn":"...","model":"..."}; fall back to a
    // plain SN string for compatibility with older driver versions.
    std::string sn = payload;
    if (!payload.empty() && payload[0] == '{') {
      sn.clear();
      const std::string search = "\"sn\":\"";
      size_t start = payload.find(search);
      if (start != std::string::npos) {
        start += search.length();
        size_t end = payload.find('"', start);
        if (end != std::string::npos) sn = payload.substr(start, end - start);
      }
      if (sn.empty()) {
        RC_LOG_WARN("Invalid device_offline JSON: %s", payload.c_str());
        return;
      }
    }
    std::unique_ptr<DeviceProcessor> processor;

    {
      std::lock_guard<std::mutex> lock(processors_mutex_);
      auto it = processors_.find(sn);
      if (it == processors_.end()) {
        RC_LOG_WARN("Device offline notification for unknown device: %s", sn.c_str());
        return;
      }

      RC_LOG_INFO("Device offline: %s, removing DeviceProcessor", sn.c_str());
      processor = std::move(it->second.processor);
      processors_.erase(it);
      RC_LOG_INFO("Remaining devices: %zu", processors_.size());
    }

    // Processor destructor handles cleanup outside the mutex.
    processor.reset();
  }

  // Global parameters
  int max_queue_size_ = 10;
  int sync_tolerance_ms_ = 10;
  std::string output_frame_id_;
  std::string topic_prefix_base_ = "manifold";  // Topic prefix base for driver management topics
  bool post_process_ = false;
  bool undistort_mapping_only_ = false;

  // Device management
  struct DeviceEntry {
    uint64_t generation;
    std::unique_ptr<DeviceProcessor> processor;
  };
  uint64_t next_device_generation_ = 0;
  std::map<std::string, DeviceEntry> processors_;
  std::mutex processors_mutex_;

#if ROS_VERSION_MAJOR == 1
  ros::Subscriber device_online_sub_;
  ros::Subscriber device_offline_sub_;
  ros::Subscriber resolution_change_sub_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
  ros::ServiceServer set_post_process_config_service_;
  ros::ServiceServer get_post_process_config_service_;
#else
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr device_online_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr device_offline_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr resolution_change_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Service<odin_driver_interfaces::srv::SetPostProcessConfig>::SharedPtr set_post_process_config_service_;
  rclcpp::Service<odin_driver_interfaces::srv::GetPostProcessConfig>::SharedPtr get_post_process_config_service_;
  rclcpp::AsyncParametersClient::SharedPtr opencv_undistort_param_client_;
#endif
};

// ============================================================================
// DeviceProcessor Implementation
// ============================================================================

DeviceProcessor::DeviceProcessor(PostProcessNode* parent, const DeviceInfo& info)
    : parent_(parent), device_info_(info), sync_tol_(durationFromMs(parent->sync_tolerance_ms())) {
  max_queue_size_ = parent->max_queue_size();
  output_frame_id_ = parent->output_frame_id();

  // Initialize JPEG decoder (hardware or software)
  jpeg_decoder_ = odin_decoder::CreateDecoder();
  jpeg_decoder_->SetCameraLabel("camera0");
  RC_LOG_INFO("[%s] JPEG decoder: %s", device_info_.sn.c_str(), jpeg_decoder_->GetName().c_str());

  std::string yaml_content;
  bool calib_ok = FetchCalibrationFromService(yaml_content);

  if (calib_ok) {
    yaml_content_ = yaml_content;  // Store for reinit on resolution change
    InitializeComponents(yaml_content);
  } else {
    RC_LOG_WARN("No calibration for device %s, advanced features disabled",
                device_info_.sn.c_str());
  }

  RC_LOG_INFO("DeviceProcessor initialized for %s (calib=%s)", device_info_.sn.c_str(),
              calib_ok ? "ok" : "none");
}

bool DeviceProcessor::FetchCalibrationFromService(std::string& yaml_content_out) {
  std::string service_name = device_info_.prefix + "get_calibration";
  RC_LOG_INFO("Waiting for calibration service: %s", service_name.c_str());
#if ROS_VERSION_MAJOR == 1
  if (!ros::service::waitForService(service_name, ros::Duration(60.0))) {
    RC_LOG_ERROR("Calibration service not available after 60 seconds");
    return false;
  }
  RC_LOG_INFO("Calibration service available, sending request...");
  odin_driver_interfaces::srv::GetCalibration srv;
  if (!ros::service::call(service_name, srv)) {
    RC_LOG_ERROR("Failed to call calibration service");
    return false;
  }
  if (!srv.response.success) {
    RC_LOG_ERROR("Calibration service returned failure: %s", srv.response.message.c_str());
    return false;
  }
  RC_LOG_INFO("Received calibration data (%zu bytes)", srv.response.yaml_content.size());
  yaml_content_out = srv.response.yaml_content;
#else
  auto client = parent_->create_client<odin_driver_interfaces::srv::GetCalibration>(service_name);
  int wait_count = 0;
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) return false;
    if (++wait_count > 60) {
      RC_LOG_ERROR("Calibration service not available after 60 seconds");
      return false;
    }
    RC_LOG_INFO("Waiting for calibration service... (%d/60)", wait_count);
  }
  RC_LOG_INFO("Calibration service available, sending request...");
  auto request = std::make_shared<odin_driver_interfaces::srv::GetCalibration::Request>();
  auto future = client->async_send_request(request);
  // Use wait_for instead of spin_until_future_complete (works in separate thread)
  auto status = future.wait_for(std::chrono::seconds(30));
  if (status != std::future_status::ready) {
    RC_LOG_ERROR("Calibration service call timed out");
    return false;
  }
  auto response = future.get();
  if (!response->success) {
    RC_LOG_ERROR("Calibration service returned failure: %s", response->message.c_str());
    return false;
  }
  RC_LOG_INFO("Received calibration data (%zu bytes)", response->yaml_content.size());
  yaml_content_out = response->yaml_content;
#endif
  return true;
}

void DeviceProcessor::InitializeComponents(const std::string& yaml_content) {
  int w = current_img_width_;
  int h = current_img_height_;
  bool use_resolution = (w > 0 && h > 0);

  if (use_resolution) {
    render_initialized_ = render_.initFromString(yaml_content, w, h);
  } else {
    render_initialized_ = render_.initFromString(yaml_content);
  }
  if (!render_initialized_) {
    RC_LOG_WARN("[%s] Failed to init rawCloudRender", device_info_.sn.c_str());
  }

  if (use_resolution) {
    undistort_0_initialized_ = m_undistort_0_.initFromString(yaml_content, w, h, 0);
  } else {
    undistort_0_initialized_ = m_undistort_0_.initFromString(yaml_content);
  }
  if (!undistort_0_initialized_) {
    RC_LOG_WARN("[%s] Failed to init m_undistort_0", device_info_.sn.c_str());
  }
}

void DeviceProcessor::ReinitComponents(int img_width, int img_height) {
  if (yaml_content_.empty()) {
    RC_LOG_WARN("[%s] Cannot reinit: no calibration data available", device_info_.sn.c_str());
    return;
  }
  if (img_width == current_img_width_ && img_height == current_img_height_) {
    return;  // No change
  }
  RC_LOG_INFO("[%s] Resolution changed %dx%d -> %dx%d, reinitializing components...",
              device_info_.sn.c_str(), current_img_width_, current_img_height_, img_width,
              img_height);
  current_img_width_ = img_width;
  current_img_height_ = img_height;
  InitializeComponents(yaml_content_);
}

void DeviceProcessor::SetupPubSub() {
  const std::string& prefix = device_info_.prefix;

  cloud_pub_ = parent_->advertise<PointCloud2>(prefix + "cloud/render", 5);

  if (parent_->undistortMappingOnly()) {
#if ROS_VERSION_MAJOR == 1
    mapping_active_sub_ = parent_->nh().subscribe<std_msgs::Bool>(
        prefix + "mapping_active", 1,
        [this](const std_msgs::Bool::ConstPtr& msg) { mapping_active_ = msg->data; });
#else
    mapping_active_sub_ = parent_->create_subscription<std_msgs::msg::Bool>(
        prefix + "mapping_active", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) { mapping_active_ = msg->data; });
#endif
  }
  image_sub_ = parent_->subscribe<CompressedImage>(
      prefix + "camera0/compressed", 5,
      [this](const CompressedImageConstPtr& msg) { imageCallback(msg); });
  cloud_sub_ = parent_->subscribe<PointCloud2>(
      prefix + "cloud/raw", 5, [this](const PointCloud2ConstPtr& msg) { cloudCallback(msg); });
}

void DeviceProcessor::imageCallback(const CompressedImageConstPtr& img) {
  const bool send_undistort = parent_->undistortMappingOnly() ? mapping_active_ : g_send_undistort;
  if (!g_send_image_raw && !send_undistort) {
    // Skipping frames invalidates the H.264 reference chain. Resume at a keyframe.
    h264_decoder_.reset();
    return;
  }

  cv::Mat decoded;
  // Branch on CompressedImage.format to dispatch to the correct decoder.
  // MJPEG frames have format="jpeg"; H.264 Annex-B frames have format="h264";
  // raw NV12 frames have format="nv12; <width>x<height>".
  const bool is_h264 = (img->format == "h264");
  const bool is_nv12 = (img->format.rfind("nv12", 0) == 0);
  if (is_h264 && !h264_decoder_) {
    h264_decoder_ = odin_decoder::CreateH264Decoder("camera0");
  }

  if (is_h264) {
    // Stateful H.264 decode. Returns false while buffering until the first
    // IDR + SPS/PPS have been received — not an error, just skip this frame.
    if (!h264_decoder_->Decode(img->data.data(), img->data.size(), decoded)) {
      return;
    }
  } else if (is_nv12) {
    if (!decodeNv12Image(img, decoded)) {
      return;
    }
  } else {
    if (!decodeCompressedImage(img, decoded)) {
      return;
    }
  }

  // Log once after the first frame is actually decoded (resolution only
  // becomes known after a successful decode).
  if (!logged_camera0_stream_info_) {
    logged_camera0_stream_info_ = true;
    RC_LOG_INFO("[%s] [H264|camera0] [first-frame] stream format=%s, decoder=%s, resolution=%dx%d",
                device_info_.sn.c_str(),
                img->format.c_str(),
                is_h264 ? h264_decoder_->GetName().c_str()
                        : (is_nv12 ? "none (raw NV12)" : jpeg_decoder_->GetName().c_str()),
                decoded.cols, decoded.rows);
  }

  // Check for resolution change and reinitialize components if needed
  if (decoded.cols != current_img_width_ || decoded.rows != current_img_height_) {
    ReinitComponents(decoded.cols, decoded.rows);
  }

  // Clone to prevent decoder buffer reuse issues
  cv::Mat safe_decoded = decoded.clone();

  if (g_send_image_raw) {
    if (!image_pub_.valid()) {
      image_pub_ = parent_->advertise<Image>(device_info_.prefix + "camera0/raw", 5);
    }
    cv_bridge::CvImage cv_img;
    cv_img.header = img->header;
    cv_img.encoding = "bgr8";
    cv_img.image = safe_decoded;
    Image image_msg;
    cv_img.toImageMsg(image_msg);
    image_pub_.publish(image_msg);
  }

  if (send_undistort && undistort_0_initialized_) {
    if (!image_undistort_pub_.valid()) {
      image_undistort_pub_ = parent_->advertise<Image>(device_info_.prefix + "camera0/undistort", 5);
    }
    cv_bridge::CvImage undistort_img;
    undistort_img.header = img->header;

    m_undistort_0_.process(safe_decoded, undistort_img);

    Image undistort_msg;
    undistort_img.toImageMsg(undistort_msg);
    image_undistort_pub_.publish(undistort_msg);
  }

  if (g_send_image_raw && g_send_cloud_render && render_initialized_) {
    // Cloud render re-decodes the raw CompressedImage bytes internally.
    // H.264/NV12 raw bytes cannot be JPEG-decoded, so skip cloud render for them.
    if (!is_h264 && !is_nv12) {
      std::lock_guard<std::mutex> lock(mutex_);
      image_queue_.push({img->header.stamp, img});
      trimQueue(image_queue_);
      tryProcessPair();
    }
  }
}

void DeviceProcessor::cloudCallback(const PointCloud2ConstPtr& cloud) {
  if (!g_send_image_raw || !g_send_cloud_render || !render_initialized_) return;

  std::lock_guard<std::mutex> lock(mutex_);
  cloud_queue_.push({cloud->header.stamp, cloud});
  trimQueue(cloud_queue_);
  tryProcessPair();
}

void DeviceProcessor::tryProcessPair() {
  while (!image_queue_.empty() && !cloud_queue_.empty()) {
    auto& img_item = image_queue_.front();
    auto& cloud_item = cloud_queue_.front();
    Duration dt = absDiff(img_item.stamp, cloud_item.stamp);

    if (dt <= sync_tol_) {
      renderCloud(img_item.msg, cloud_item.msg);
      image_queue_.pop();
      cloud_queue_.pop();
    } else {
      if (img_item.stamp < cloud_item.stamp) {
        image_queue_.pop();
      } else {
        cloud_queue_.pop();
      }
    }
  }
}

void DeviceProcessor::renderCloud(const CompressedImageConstPtr& img,
                                  const PointCloud2ConstPtr& cloud) {
  cv::Mat decoded;
  if (!decodeCompressedImage(img, decoded)) return;

  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  cloud_xyz.reserve(cloud->width * cloud->height);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    cloud_xyz.emplace_back(*iter_x, *iter_y, *iter_z);
  }

  pcl::PointCloud<pcl::PointXYZRGBA> colored_cloud;
  render_.render(decoded, cloud_xyz, colored_cloud);
  if (colored_cloud.empty()) return;

  PointCloud2 output_msg;
  pcl::toROSMsg(colored_cloud, output_msg);
  output_msg.header = cloud->header;
  if (!output_frame_id_.empty()) output_msg.header.frame_id = output_frame_id_;
  cloud_pub_.publish(output_msg);
}

bool DeviceProcessor::decodeCompressedImage(const CompressedImageConstPtr& img, cv::Mat& decoded) {
  if (jpeg_decoder_->Decode(img->data.data(), img->data.size(), decoded)) {
    return true;
  }
  RC_LOG_ERROR("[%s] Failed to decode compressed image", device_info_.sn.c_str());
  return false;
}

bool DeviceProcessor::decodeNv12Image(const CompressedImageConstPtr& img, cv::Mat& decoded) {
  // The driver publishes NV12 frames with format "nv12; <width>x<height>"
  // because the raw byte stream itself carries no dimension information.
  int w = 0, h = 0;
  if (std::sscanf(img->format.c_str(), "nv12; %dx%d", &w, &h) != 2 || w <= 0 || h <= 0) {
    RC_LOG_ERROR("[%s] Invalid NV12 format string: '%s'", device_info_.sn.c_str(),
                 img->format.c_str());
    return false;
  }
  const size_t expected = static_cast<size_t>(w) * h * 3 / 2;
  if (img->data.size() != expected) {
    RC_LOG_ERROR("[%s] NV12 payload size mismatch: %zuB, expected %zuB for %dx%d",
                 device_info_.sn.c_str(), img->data.size(), expected, w, h);
    return false;
  }
#ifdef ODIN_HAVE_LIBYUV
  // Fast path: libyuv NV12 → BGR24. NB: libyuv's naming is counter-intuitive
  // here — the "...ToRGB24" family writes B,G,R byte order (identical to
  // OpenCV BGR24 / cv_bridge "bgr8"), while the "...ToRAW" family writes
  // R,G,B and would swap the red/blue channels (skin tones turn blue).
  // The driver's NV12 payload is tightly packed — the w*h Y plane immediately
  // followed by interleaved UV — so both strides are w.
  // Same escape hatch as the H.264 decoder: ODIN_DISABLE_LIBYUV=1.
  static const bool libyuv_forced_off = []() {
    const char* env = std::getenv("ODIN_DISABLE_LIBYUV");
    return env && env[0] == '1';
  }();
  if (!libyuv_forced_off && (w & 1) == 0) {
    decoded.create(h, w, CV_8UC3);
    if (libyuv::NV12ToRGB24(img->data.data(), w,
                            img->data.data() + static_cast<size_t>(w) * h, w,
                            decoded.data, static_cast<int>(decoded.step), w, h) == 0) {
      return true;
    }
  }
#endif
  // Wrap the Y+interleaved-UV planes as one (h*3/2 x w) 8-bit matrix and let
  // OpenCV do the colour conversion. cvtColor copies, so wrapping const data
  // without cloning is safe here.
  cv::Mat nv12(h * 3 / 2, w, CV_8UC1, const_cast<uint8_t*>(img->data.data()));
  cv::cvtColor(nv12, decoded, cv::COLOR_YUV2BGR_NV12);
  return true;
}

#if ROS_VERSION_MAJOR == 1

int main(int argc, char** argv) {
  odin::print_app_version("post_process_node");
  ros::init(argc, argv, "postprocess_node");

  try {
    PostProcessNode node;
    if (!node.postProcessEnabled()) {
      ROS_INFO("post_process disabled, exiting");
      return 0;
    }
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in PostProcessNode: %s", e.what());
    return 1;
  }
  return 0;
}

#else  // ROS2

int main(int argc, char** argv) {
  odin::print_app_version("post_process_node");
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<PostProcessNode>();
    if (node->postProcessEnabled()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception& e) {
    RC_LOG_ERROR("Exception in PostProcessNode: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#endif
