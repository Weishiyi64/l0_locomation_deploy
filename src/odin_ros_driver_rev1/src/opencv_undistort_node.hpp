/**
 * @file opencv_undistort_node.hpp
 * @brief OpenCV fisheye undistortion node for the Odin ROS driver.
 *
 * Subscribes to device_online to discover devices, fetches calibration via
 * the get_calibration service, converts the Odin FishPoly model to the
 * OpenCV Kannala-Brandt equidistant model, and publishes undistorted
 * images and CameraInfo messages.
 */

#pragma once

#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "utility/ros_compat.hpp"

#if ROS_VERSION_MAJOR == 1
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#endif

// ============================================================================
// Data Structures
// ============================================================================

/// Device info parsed from device_online JSON: {"sn":"...","model":"...","prefix":"...","ip":"..."}
struct UndistortDeviceInfo {
  std::string sn;
  std::string model;
  std::string prefix;
  std::string ip;
};

/// Odin FishPoly calibration parameters extracted from the calibration YAML.
struct FishPolyCalib {
  double k2 = 0.0, k3 = 0.0, k4 = 0.0, k5 = 0.0, k6 = 0.0, k7 = 0.0;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  double skew = 0.0;
  int width = 0;
  int height = 0;
};

/// OpenCV Kannala-Brandt equidistant fisheye model (4 distortion params).
struct KBModel {
  double k1 = 0.0, k2 = 0.0, k3 = 0.0, k4 = 0.0;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  int width = 0;
  int height = 0;
  cv::Mat K;      ///< 3x3 camera intrinsic matrix
  cv::Mat D;      ///< 4x1 distortion coefficients
};

// ============================================================================
// Forward Declarations
// ============================================================================

class OpenCVUndistortNode;

// ============================================================================
// UndistortDeviceProcessor - handles processing for a single device
// ============================================================================

class UndistortDeviceProcessor {
 public:
  UndistortDeviceProcessor(OpenCVUndistortNode* parent, const UndistortDeviceInfo& info);
  ~UndistortDeviceProcessor();

  const std::string& sn() const { return device_info_.sn; }
  bool isInitialized() const { return initialized_; }

  void imageCallback0(const ros_compat::CompressedImageConstPtr& img);

  /// Reinitialize undistortion maps when image resolution changes.
  void reinit(int width, int height);

 private:
  bool fetchCalibration(std::string& yaml_out);
  void initUndistortion(const std::string& yaml, int width, int height);
  void setupPubSub();
  bool decodeImage(const ros_compat::CompressedImageConstPtr& img, cv::Mat& decoded);
  void processImage(const ros_compat::CompressedImageConstPtr& img,
                    ros_compat::Publisher<ros_compat::Image>& img_pub,
                    ros_compat::Publisher<ros_compat::CameraInfo>& info_pub,
                    const KBModel& kb);

  OpenCVUndistortNode* parent_;
  UndistortDeviceInfo device_info_;

  // Subscribers
  ros_compat::Subscriber<ros_compat::CompressedImage> img0_sub_;

  // Publishers
  ros_compat::Publisher<ros_compat::Image> img0_pub_;
  // CameraInfo with KB fisheye model, published on camera0/camera_info.
  // Pairs with the raw compressed image on camera0/compressed.
  ros_compat::Publisher<ros_compat::CameraInfo> info0_pub_;

  // Undistortion models
  KBModel kb0_;
  bool kb0_ready_ = false;

  // State
  bool initialized_ = false;
  std::string yaml_content_;
  int current_width_ = 0;
  int current_height_ = 0;
};

// ============================================================================
// OpenCVUndistortNode - main node managing multiple devices
// ============================================================================

class OpenCVUndistortNode : public ros_compat::Node {
 public:
  OpenCVUndistortNode();

  bool processingEnabled() const { return processing_enabled_; }
  void setProcessingEnabled(bool v) { processing_enabled_ = v; }

 private:
  /// Build driver management topic: "/{base}/driver/{suffix}" or "/driver/{suffix}"
  std::string buildDriverTopicName(const std::string& suffix) const;

  void startDeviceDiscovery();
  void onDeviceInfoReceived(const std::string& json);
  void onDeviceOffline(const std::string& sn);

  std::string topic_prefix_base_ = "manifold";

  // Hot-switch flag: when false, images are not processed/published
  bool processing_enabled_ = false;

  // Device management
  std::map<std::string, std::unique_ptr<UndistortDeviceProcessor>> processors_;
  std::mutex processors_mutex_;

#if ROS_VERSION_MAJOR == 1
  ros::Subscriber device_online_sub_;
  ros::Subscriber device_offline_sub_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
#else
  rclcpp::Subscription<ros_compat::StringMsg>::SharedPtr device_online_sub_;
  rclcpp::Subscription<ros_compat::StringMsg>::SharedPtr device_offline_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
#endif
};
