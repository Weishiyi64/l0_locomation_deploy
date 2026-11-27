/**
 * @file opencv_undistort_node.cpp
 * @brief OpenCV fisheye undistortion node — implementation + main.
 *
 * Uses ros_compat.hpp wrappers to minimize version-specific code.
 * Only service calls and the device_online subscriber use #if/#endif.
 */

#include "odin_app_version.hpp"
#include "opencv_undistort_node.hpp"

#include "utility/ros_compat.hpp"
#include "utility/yaml_config_loader.hpp"

#if ROS_VERSION_MAJOR == 1
#include <std_msgs/String.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include "service_types.h"
#else
#include <std_msgs/msg/string.hpp>
#include "service_types.h"
#endif

// cv_bridge header compatibility: Ubuntu 24.04 uses .hpp, earlier versions use .h
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <sstream>
#include <thread>

using ros_compat::CameraInfo;
using ros_compat::CompressedImage;
using ros_compat::CompressedImageConstPtr;
using ros_compat::Image;
using ros_compat::Publisher;
using ros_compat::Subscriber;
using ros_compat::StringMsg;
using yaml_utils::safeGet;

// ============================================================================
// Anonymous namespace: JSON parsing + FishPoly -> KB conversion
// ============================================================================

namespace {

/// Parse a JSON string field value.
bool ParseJsonStringField(const std::string& json, const std::string& key,
                          std::string& out) {
  std::string search = "\"" + key + "\":\"";
  size_t start = json.find(search);
  if (start == std::string::npos) return false;
  start += search.length();
  size_t end = json.find('"', start);
  if (end == std::string::npos) return false;
  out = json.substr(start, end - start);
  return !out.empty();
}

/// Parse device info JSON: {"sn":"xxx","model":"Odin2Special","prefix":"/manifold/device0/","ip":"..."}
bool ParseDeviceInfoJson(const std::string& json, UndistortDeviceInfo& info) {
  ParseJsonStringField(json, "sn", info.sn);
  ParseJsonStringField(json, "model", info.model);
  ParseJsonStringField(json, "prefix", info.prefix);
  ParseJsonStringField(json, "ip", info.ip);
  return !info.sn.empty() && !info.prefix.empty();
}

// ---------------------------------------------------------------------------
// Odin FishPoly -> OpenCV Kannala-Brandt conversion
// ---------------------------------------------------------------------------

/**
 * @brief Compute the true incident angle theta from the distorted angle
 *        theta_d via fixed-point iteration (20 iterations).
 *
 * The Odin forward model is:  theta_d = theta + k2*theta^2 + ... + k7*theta^7
 * The inverse is:  theta = theta_d / (1 + k2*theta + k3*theta^2 + ... + k7*theta^6)
 * We iterate this fixed-point formula starting from theta = theta_d.
 *
 * @param theta_d  Distorted angle (rad)
 * @param k        Array of Odin coefficients; only indices [2..7] are used.
 * @param n_iter   Number of fixed-point iterations (default 20).
 * @return         Undistorted incident angle theta (rad).
 */
double ThetaFromThetaD(double theta_d, const double k[9], int n_iter = 20) {
  if (theta_d < 1e-12) return 0.0;
  double theta = theta_d;
  for (int i = 0; i < n_iter; ++i) {
    double t = theta;
    double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t, t6 = t5 * t;
    double denom = 1.0 + k[2] * t + k[3] * t2 + k[4] * t3 +
                   k[5] * t4 + k[6] * t5 + k[7] * t6;
    theta = theta_d / denom;
  }
  return theta;
}

/**
 * @brief Odin forward model: theta_d = theta + k2*theta^2 + ... + k7*theta^7
 */
double OdinThetaD(double theta, const double k[9]) {
  double t = theta;
  double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t,
         t6 = t5 * t, t7 = t6 * t;
  return t + k[2] * t2 + k[3] * t3 + k[4] * t4 + k[5] * t5 + k[6] * t6 +
         k[7] * t7;
}

/**
 * @brief Determine the maximum theta (FOV half-angle) by back-projecting
 *        all four image corners.
 *
 * For corner pixel (u, v):
 *   theta_d = sqrt(((u - cx) / fx)^2 + ((v - cy) / fy)^2)
 * Then invert using fixed-point iteration.
 *
 * @return Max theta across all 4 corners (radians).
 */
double ComputeMaxTheta(const FishPolyCalib& cal) {
  // Store Odin coefficients into the k[9] array format used by the helper.
  double k[9] = {};
  k[2] = cal.k2; k[3] = cal.k3; k[4] = cal.k4;
  k[5] = cal.k5; k[6] = cal.k6; k[7] = cal.k7;

  double max_theta = 0.0;
  // Four corners
  int corners[4][2] = {
      {0, 0},
      {cal.width - 1, 0},
      {0, cal.height - 1},
      {cal.width - 1, cal.height - 1}
  };

  for (int i = 0; i < 4; ++i) {
    double u = static_cast<double>(corners[i][0]);
    double v = static_cast<double>(corners[i][1]);
    double x_norm = (u - cal.cx) / cal.fx;
    double y_norm = (v - cal.cy) / cal.fy;
    double theta_d = std::sqrt(x_norm * x_norm + y_norm * y_norm);
    double theta = ThetaFromThetaD(theta_d, k);
    if (theta > max_theta) max_theta = theta;
  }
  return max_theta;
}

/**
 * @brief Convert Odin FishPoly distortion to OpenCV KB (Kannala-Brandt) model.
 *
 * Steps:
 * 1. Compute max theta from image corners.
 * 2. Sample N=1000 theta values in [0, max_theta].
 * 3. For each sample, compute target theta_d using the Odin model.
 * 4. Fit KB params [k1,k2,k3,k4] via linear least-squares (SVD):
 *    A * [k1,k2,k3,k4]^T = (target_theta_d - theta)
 *    where columns of A are [theta^3, theta^5, theta^7, theta^9].
 *
 * @param cal   Odin FishPoly calibration parameters.
 * @return      Fitted KB model (K, D matrices populated).
 */
KBModel ConvertFishPolyToKB(const FishPolyCalib& cal) {
  KBModel kb;
  kb.fx = cal.fx;
  kb.fy = cal.fy;
  kb.cx = cal.cx;
  kb.cy = cal.cy;
  kb.width = cal.width;
  kb.height = cal.height;

  // Build Odin coefficient array
  double k[9] = {};
  k[2] = cal.k2; k[3] = cal.k3; k[4] = cal.k4;
  k[5] = cal.k5; k[6] = cal.k6; k[7] = cal.k7;

  // Step 1: Determine max theta from image corners
  double max_theta = ComputeMaxTheta(cal);
  if (max_theta < 1e-6) {
    RC_LOG_WARN("Max theta is near zero, calibration may be invalid");
    kb.K = (cv::Mat_<double>(3, 3) << cal.fx, 0, cal.cx, 0, cal.fy, cal.cy, 0, 0, 1);
    kb.D = cv::Mat::zeros(4, 1, CV_64F);
    return kb;
  }

  // Step 2: Sample N=1000 theta values uniformly in [0, max_theta]
  const int N = 1000;
  cv::Mat A(N, 4, CV_64F);
  cv::Mat b(N, 1, CV_64F);

  for (int i = 0; i < N; ++i) {
    double theta = max_theta * static_cast<double>(i) / (N - 1);

    // Step 3: Compute target theta_d using the Odin forward model
    double target_theta_d = OdinThetaD(theta, k);

    // Step 4: Build least-squares system
    // KB model: theta_d = theta + k1*theta^3 + k2*theta^5 + k3*theta^7 + k4*theta^9
    // => k1*theta^3 + k2*theta^5 + k3*theta^7 + k4*theta^9 = target_theta_d - theta
    double t3 = theta * theta * theta;
    double t5 = t3 * theta * theta;
    double t7 = t5 * theta * theta;
    double t9 = t7 * theta * theta;

    A.at<double>(i, 0) = t3;
    A.at<double>(i, 1) = t5;
    A.at<double>(i, 2) = t7;
    A.at<double>(i, 3) = t9;
    b.at<double>(i, 0) = target_theta_d - theta;
  }

  // Solve A * [k1,k2,k3,k4]^T = b via SVD
  cv::Mat kb_params(4, 1, CV_64F);
  if (!cv::solve(A, b, kb_params, cv::DECOMP_SVD)) {
    RC_LOG_WARN("SVD solve failed for KB fitting, using zeros");
    kb_params = cv::Mat::zeros(4, 1, CV_64F);
  }

  kb.k1 = kb_params.at<double>(0, 0);
  kb.k2 = kb_params.at<double>(1, 0);
  kb.k3 = kb_params.at<double>(2, 0);
  kb.k4 = kb_params.at<double>(3, 0);

  // Build K and D matrices for cv::fisheye::undistortImage
  kb.K = (cv::Mat_<double>(3, 3) << cal.fx, 0.0, cal.cx,
                                       0.0, cal.fy, cal.cy,
                                       0.0, 0.0, 1.0);
  kb.D = (cv::Mat_<double>(4, 1) << kb.k1, kb.k2, kb.k3, kb.k4);

  RC_LOG_INFO("KB fit: k1=%.6e k2=%.6e k3=%.6e k4=%.6e (max_theta=%.4f rad = %.1f deg)",
              kb.k1, kb.k2, kb.k3, kb.k4, max_theta, max_theta * 180.0 / M_PI);

  return kb;
}

/**
 * @brief Extract FishPoly calibration from a YAML camera node.
 *
 * Uses yaml_utils::ResolveCamKey to find the correct camera section for
 * the given resolution, then reads k2-k7, A11 (fx), A22 (fy), u0 (cx),
 * v0 (cy), image dimensions.
 *
 * @param root       Root YAML node (entire calibration document).
 * @param cam_id     Camera index (0 or 1).
 * @param width      Desired image width (0 = use default).
 * @param height     Desired image height (0 = use default).
 * @param cal_out    Output FishPolyCalib.
 * @return           true on success.
 */
bool ExtractCalibration(const YAML::Node& root, int cam_id, int width, int height,
                        FishPolyCalib& cal_out) {
  std::string resolved = yaml_utils::ResolveCamKey(root, cam_id, width, height);
  if (resolved.empty()) {
    RC_LOG_WARN("No calibration for cam_%d at %dx%d", cam_id, width, height);
    return false;
  }

  YAML::Node cam = root[resolved];
  cal_out.k2 = safeGet<double>(cam, "k2", 0.0);
  cal_out.k3 = safeGet<double>(cam, "k3", 0.0);
  cal_out.k4 = safeGet<double>(cam, "k4", 0.0);
  cal_out.k5 = safeGet<double>(cam, "k5", 0.0);
  cal_out.k6 = safeGet<double>(cam, "k6", 0.0);
  cal_out.k7 = safeGet<double>(cam, "k7", 0.0);
  cal_out.fx = safeGet<double>(cam, "A11", 200.0);
  cal_out.fy = safeGet<double>(cam, "A22", 200.0);
  cal_out.cx = safeGet<double>(cam, "u0", 320.0);
  cal_out.cy = safeGet<double>(cam, "v0", 272.0);
  cal_out.skew = safeGet<double>(cam, "A12", 0.0);
  cal_out.width = safeGet<int>(cam, "image_width", width > 0 ? width : 640);
  cal_out.height = safeGet<int>(cam, "image_height", height > 0 ? height : 544);

  return true;
}

}  // anonymous namespace

// ============================================================================
// OpenCVUndistortNode Implementation
// ============================================================================

OpenCVUndistortNode::OpenCVUndistortNode() : ros_compat::Node("opencv_undistort_node") {
  topic_prefix_base_ = param<std::string>("topic_prefix", "manifold");

#if ROS_VERSION_MAJOR == 2
  declare_parameter<bool>("send_opencv_undistort", false);
  processing_enabled_ = get_parameter("send_opencv_undistort").as_bool();
  param_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
          if (p.get_name() == "send_opencv_undistort") {
            processing_enabled_ = p.as_bool();
            fprintf(stderr, "[opencv_undistort] send_opencv_undistort changed: %d\n", processing_enabled_);
            RCLCPP_INFO(this->get_logger(), "send_opencv_undistort changed: %d", processing_enabled_);
          }
        }
        return result;
      });
#else
  processing_enabled_ = param<bool>("send_opencv_undistort", false);

  // ROS1: register dynamic reconfigure for hot-switch support
  ddr_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(pnh());
  ddr_->registerVariable<bool>(
      "send_opencv_undistort", processing_enabled_,
      [this](bool new_val) {
        processing_enabled_ = new_val;
        RC_LOG_INFO("send_opencv_undistort changed: %d", processing_enabled_);
      },
      "Enable OpenCV undistortion processing", false, true);
  ddr_->publishServicesTopics();
#endif

  startDeviceDiscovery();
  RC_LOG_INFO("OpenCVUndistortNode initialized (processing_enabled=%d), waiting for devices...",
              processing_enabled_);
}

std::string OpenCVUndistortNode::buildDriverTopicName(const std::string& suffix) const {
  if (topic_prefix_base_.empty()) {
    return "/driver/" + suffix;
  }
  return "/" + topic_prefix_base_ + "/driver/" + suffix;
}

void OpenCVUndistortNode::startDeviceDiscovery() {
#if ROS_VERSION_MAJOR == 1
  device_online_sub_ = nh().subscribe<StringMsg>(
      buildDriverTopicName("device_online"), 100,
      [this](const boost::shared_ptr<StringMsg const>& msg) {
        onDeviceInfoReceived(msg->data);
      });
  device_offline_sub_ = nh().subscribe<StringMsg>(
      buildDriverTopicName("device_offline"), 10,
      [this](const boost::shared_ptr<StringMsg const>& msg) {
        onDeviceOffline(msg->data);
      });
#else
  device_online_sub_ = create_subscription<StringMsg>(
      buildDriverTopicName("device_online"), rclcpp::QoS(10).transient_local(),
      [this](const StringMsg::SharedPtr msg) { onDeviceInfoReceived(msg->data); });
  device_offline_sub_ = create_subscription<StringMsg>(
      buildDriverTopicName("device_offline"), rclcpp::QoS(10).reliable().durability_volatile(),
      [this](const StringMsg::SharedPtr msg) { onDeviceOffline(msg->data); });
#endif
}

void OpenCVUndistortNode::onDeviceInfoReceived(const std::string& json) {
  RC_LOG_INFO("Received device_online: %s", json.c_str());

  UndistortDeviceInfo info;
  if (!ParseDeviceInfoJson(json, info)) {
    RC_LOG_WARN("Failed to parse device_online JSON: %s", json.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    if (processors_.find(info.sn) != processors_.end()) {
      return;  // Already registered
    }
    // Mark as pending to avoid duplicate creation
    processors_[info.sn] = nullptr;
  }

  RC_LOG_INFO("New device discovered: sn=%s, model=%s, prefix=%s, ip=%s",
              info.sn.c_str(), info.model.c_str(), info.prefix.c_str(), info.ip.c_str());

  // Create processor in a separate thread to avoid blocking the callback
  std::thread([this, info]() {
    auto processor = std::make_unique<UndistortDeviceProcessor>(this, info);

    std::lock_guard<std::mutex> lock(processors_mutex_);
    processors_[info.sn] = std::move(processor);
    RC_LOG_INFO("UndistortDeviceProcessor created for %s, total devices: %zu",
                info.sn.c_str(), processors_.size());
  }).detach();
}

void OpenCVUndistortNode::onDeviceOffline(const std::string& payload) {
  // device_offline carries JSON {"sn":"...","model":"..."}; fall back to a
  // plain SN string for compatibility with older driver versions.
  std::string sn = payload;
  if (!payload.empty() && payload[0] == '{') {
    sn.clear();
    ParseJsonStringField(payload, "sn", sn);
    if (sn.empty()) {
      RC_LOG_WARN("Invalid device_offline JSON: %s", payload.c_str());
      return;
    }
  }
  std::unique_ptr<UndistortDeviceProcessor> processor;
  {
    std::lock_guard<std::mutex> lock(processors_mutex_);
    auto it = processors_.find(sn);
    if (it == processors_.end()) {
      RC_LOG_WARN("Device offline notification for unknown device: %s", sn.c_str());
      return;
    }
    RC_LOG_INFO("Device offline: %s, removing processor", sn.c_str());
    processor = std::move(it->second);
    processors_.erase(it);
  }
  processor.reset();
  RC_LOG_INFO("Processor removed for %s, remaining devices: %zu",
              sn.c_str(), processors_.size());
}

// ============================================================================
// UndistortDeviceProcessor Implementation
// ============================================================================

UndistortDeviceProcessor::UndistortDeviceProcessor(OpenCVUndistortNode* parent,
                                                     const UndistortDeviceInfo& info)
    : parent_(parent), device_info_(info) {
  RC_LOG_INFO("[%s] Initializing UndistortDeviceProcessor...", info.sn.c_str());

  std::string yaml_content;
  bool calib_ok = fetchCalibration(yaml_content);

  if (calib_ok) {
    yaml_content_ = yaml_content;
    // Initialize with default resolution (will be reinitialized when first
    // image arrives with actual dimensions)
    initUndistortion(yaml_content, 0, 0);
  } else {
    RC_LOG_WARN("[%s] No calibration available, undistortion disabled",
                info.sn.c_str());
  }

  // Always setup pub/sub - basic image passthrough works without calibration
  setupPubSub();

  initialized_ = true;
  RC_LOG_INFO("[%s] UndistortDeviceProcessor initialized (calib=%s)",
              info.sn.c_str(), calib_ok ? "ok" : "none");
}

UndistortDeviceProcessor::~UndistortDeviceProcessor() {
  RC_LOG_INFO("[%s] UndistortDeviceProcessor destroying...", device_info_.sn.c_str());
  img0_sub_ = Subscriber<CompressedImage>();
  img0_pub_ = Publisher<Image>();
  info0_pub_ = Publisher<CameraInfo>();
  RC_LOG_INFO("[%s] UndistortDeviceProcessor destroyed", device_info_.sn.c_str());
}

bool UndistortDeviceProcessor::fetchCalibration(std::string& yaml_out) {
  std::string service_name = device_info_.prefix + "get_calibration";
  RC_LOG_INFO("[%s] Waiting for calibration service: %s",
              device_info_.sn.c_str(), service_name.c_str());

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
    RC_LOG_ERROR("Calibration service returned failure: %s",
                 srv.response.message.c_str());
    return false;
  }
  RC_LOG_INFO("Received calibration data (%zu bytes)", srv.response.yaml_content.size());
  yaml_out = srv.response.yaml_content;
#else
  auto client = parent_->create_client<odin_driver_interfaces::srv::GetCalibration>(
      service_name);
  int wait_count = 0;
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (++wait_count > 60) {
      RC_LOG_ERROR("Calibration service not available after 60 seconds");
      return false;
    }
    RC_LOG_INFO("Waiting for calibration service... (%d/60)", wait_count);
  }
  RC_LOG_INFO("Calibration service available, sending request...");
  auto request = std::make_shared<odin_driver_interfaces::srv::GetCalibration::Request>();
  auto future = client->async_send_request(request);
  auto status = future.wait_for(std::chrono::seconds(30));
  if (status != std::future_status::ready) {
    RC_LOG_ERROR("Calibration service call timed out");
    return false;
  }
  auto response = future.get();
  if (!response->success) {
    RC_LOG_ERROR("Calibration service returned failure: %s",
                 response->message.c_str());
    return false;
  }
  RC_LOG_INFO("Received calibration data (%zu bytes)", response->yaml_content.size());
  yaml_out = response->yaml_content;
#endif
  return true;
}

void UndistortDeviceProcessor::initUndistortion(const std::string& yaml,
                                                 int width, int height) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const std::exception& e) {
    RC_LOG_ERROR("[%s] Failed to parse calibration YAML: %s",
                 device_info_.sn.c_str(), e.what());
    return;
  }

  // This hardware has only camera0.
  FishPolyCalib cal0;

  if (ExtractCalibration(root, 0, width, height, cal0)) {
    kb0_ = ConvertFishPolyToKB(cal0);
    kb0_ready_ = true;
    RC_LOG_INFO("[%s] cam0 KB model ready: %dx%d, D=[%.6e, %.6e, %.6e, %.6e]",
                device_info_.sn.c_str(), kb0_.width, kb0_.height,
                kb0_.k1, kb0_.k2, kb0_.k3, kb0_.k4);
  } else {
    kb0_ready_ = false;
    RC_LOG_WARN("[%s] Failed to extract calibration for cam0", device_info_.sn.c_str());
  }
}

void UndistortDeviceProcessor::reinit(int width, int height) {
  if (yaml_content_.empty()) {
    RC_LOG_WARN("[%s] Cannot reinit: no calibration data", device_info_.sn.c_str());
    return;
  }
  if (width == current_width_ && height == current_height_) {
    return;  // No change
  }
  RC_LOG_INFO("[%s] Resolution changed %dx%d -> %dx%d, reinitializing...",
              device_info_.sn.c_str(), current_width_, current_height_,
              width, height);
  current_width_ = width;
  current_height_ = height;
  initUndistortion(yaml_content_, width, height);
}

void UndistortDeviceProcessor::setupPubSub() {
  const std::string& prefix = device_info_.prefix;

  // Publishers for undistorted images and camera info
  img0_pub_ = parent_->advertise<Image>(prefix + "camera0/opencv_undistort", 5);
  // CameraInfo with KB fisheye model -- pairs with the undistorted image
  // on camera0/opencv_undistort so tools like RViz/Foxglove can use it.
  info0_pub_ = parent_->advertise<CameraInfo>(prefix + "camera0/camera_info", 5);

  // Subscribers for compressed images
  img0_sub_ = parent_->subscribe<CompressedImage>(
      prefix + "camera0/compressed", 5,
      [this](const CompressedImageConstPtr& msg) { imageCallback0(msg); });

  RC_LOG_INFO("[%s] PubSub setup: sub camera0/compressed, "
              "pub camera0/opencv_undistort + camera0/camera_info",
              device_info_.sn.c_str());
}

bool UndistortDeviceProcessor::decodeImage(const CompressedImageConstPtr& img,
                                           cv::Mat& decoded) {
  // Decode JPEG using OpenCV imdecode
  cv::Mat buf(1, static_cast<int>(img->data.size()), CV_8UC1,
              const_cast<uint8_t*>(img->data.data()));
  decoded = cv::imdecode(buf, cv::IMREAD_COLOR);
  if (decoded.empty()) {
    RC_LOG_ERROR("[%s] Failed to decode JPEG image", device_info_.sn.c_str());
    return false;
  }
  return true;
}

void UndistortDeviceProcessor::processImage(
    const CompressedImageConstPtr& img,
    Publisher<Image>& img_pub, Publisher<CameraInfo>& info_pub,
    const KBModel& kb) {
  // -----------------------------------------------------------------------
  // Always publish CameraInfo with the KB equidistant fisheye model.
  // Uses calibration resolution directly, no image decode needed.
  // -----------------------------------------------------------------------
  CameraInfo info_msg;
  info_msg.header = img->header;
  info_msg.width = kb.width > 0 ? static_cast<uint32_t>(kb.width) : 0;
  info_msg.height = kb.height > 0 ? static_cast<uint32_t>(kb.height) : 0;
  info_msg.distortion_model = "equidistant";

#if ROS_VERSION_MAJOR == 1
  auto& D_vec = info_msg.D;
  auto& K_arr = info_msg.K;
  auto& R_arr = info_msg.R;
  auto& P_arr = info_msg.P;
#else
  auto& D_vec = info_msg.d;
  auto& K_arr = info_msg.k;
  auto& R_arr = info_msg.r;
  auto& P_arr = info_msg.p;
#endif

  // D = [k1, k2, k3, k4]  (fitted KB distortion coefficients)
  D_vec.clear();
  D_vec.push_back(kb.k1);
  D_vec.push_back(kb.k2);
  D_vec.push_back(kb.k3);
  D_vec.push_back(kb.k4);

  // K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  K_arr[0] = kb.fx; K_arr[1] = 0.0;  K_arr[2] = kb.cx;
  K_arr[3] = 0.0;   K_arr[4] = kb.fy; K_arr[5] = kb.cy;
  K_arr[6] = 0.0;   K_arr[7] = 0.0;  K_arr[8] = 1.0;

  // R = identity
  R_arr[0] = 1.0; R_arr[1] = 0.0; R_arr[2] = 0.0;
  R_arr[3] = 0.0; R_arr[4] = 1.0; R_arr[5] = 0.0;
  R_arr[6] = 0.0; R_arr[7] = 0.0; R_arr[8] = 1.0;

  // P = projection matrix (original intrinsics, no new camera matrix).
  P_arr[0] = kb.fx;  P_arr[1] = 0.0;   P_arr[2] = kb.cx;  P_arr[3] = 0.0;
  P_arr[4] = 0.0;    P_arr[5] = kb.fy;  P_arr[6] = kb.cy;  P_arr[7] = 0.0;
  P_arr[8] = 0.0;    P_arr[9] = 0.0;   P_arr[10] = 1.0;   P_arr[11] = 0.0;

  info_pub.publish(info_msg);

  // Hot-switch: skip decoding and undistortion when disabled
  if (!parent_->processingEnabled()) {
    return;
  }

  // Decode the compressed image
  cv::Mat decoded;
  bool decode_ok = decodeImage(img, decoded);

  // Reinitialize undistortion maps if the resolution changed
  if (decode_ok &&
      (decoded.cols != current_width_ || decoded.rows != current_height_)) {
    reinit(decoded.cols, decoded.rows);
  }

  // Cannot undistort if decoding failed
  if (!decode_ok) {
    return;
  }

  // Clone to ensure safe data ownership
  cv::Mat safe_decoded = decoded.clone();

  // Undistort using OpenCV fisheye model
  cv::Mat undistorted;
  if (kb.K.empty() || kb.D.empty()) {
    // No calibration available, publish raw image
    undistorted = safe_decoded;
  } else {
    try {
      cv::fisheye::undistortImage(safe_decoded, undistorted, kb.K, kb.D, kb.K);
    } catch (const cv::Exception& e) {
      RC_LOG_ERROR("[%s] undistortImage failed: %s", device_info_.sn.c_str(), e.what());
      undistorted = safe_decoded;
    }
  }

  // Publish undistorted image
  cv_bridge::CvImage cv_img;
  cv_img.header = img->header;
  cv_img.encoding = "bgr8";
  cv_img.image = undistorted;
  Image image_msg;
  cv_img.toImageMsg(image_msg);
  img_pub.publish(image_msg);
}

void UndistortDeviceProcessor::imageCallback0(const CompressedImageConstPtr& img) {
  processImage(img, img0_pub_, info0_pub_, kb0_);
}

// ============================================================================
// Main
// ============================================================================

#if ROS_VERSION_MAJOR == 1

int main(int argc, char** argv) {
  odin::print_app_version("opencv_undistort_node");
  ros_compat::init(argc, argv, "opencv_undistort_node");
  try {
    OpenCVUndistortNode node;
    ros_compat::spin();
  } catch (const std::exception& e) {
    RC_LOG_ERROR("Exception in OpenCVUndistortNode: %s", e.what());
    return 1;
  }
  ros_compat::shutdown();
  return 0;
}

#else  // ROS2

int main(int argc, char** argv) {
  odin::print_app_version("opencv_undistort_node");
  ros_compat::init(argc, argv);
  try {
    auto node = std::make_shared<OpenCVUndistortNode>();
    ros_compat::spin(node);
  } catch (const std::exception& e) {
    RC_LOG_ERROR("Exception in OpenCVUndistortNode: %s", e.what());
    return 1;
  }
  ros_compat::shutdown();
  return 0;
}

#endif
