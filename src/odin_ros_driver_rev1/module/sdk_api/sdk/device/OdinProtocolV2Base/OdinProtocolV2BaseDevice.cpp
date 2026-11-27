#include "OdinProtocolV2BaseDevice.h"
#include "OdinCommand.hpp"
#include "OdinCaptureBase.hpp"

#include "logger.h"

#include <Eigen/Dense>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <thread>
#include <cstring>

namespace odin {
namespace sdk {
namespace {

uint64_t NowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

void RotationMatrixToQuaternion(const double R_data[9], double q_out[4]) {
  Eigen::Matrix3d R;
  R << R_data[0], R_data[1], R_data[2],
       R_data[3], R_data[4], R_data[5],
       R_data[6], R_data[7], R_data[8];
  Eigen::Quaterniond q(R);
  q.normalize();
  q_out[0] = q.x();
  q_out[1] = q.y();
  q_out[2] = q.z();
  q_out[3] = q.w();
}

std::vector<OdinLcPointXYZ> CopyXyzCloud(const float* xyz, size_t npts) {
  std::vector<OdinLcPointXYZ> points;
  if (!xyz || npts == 0) return points;
  points.reserve(npts);
  for (size_t i = 0; i < npts; ++i) {
    const float* p = xyz + i * 3;
    points.push_back({p[0], p[1], p[2]});
  }
  return points;
}

// 将回环算法领域错误码映射为对外 OdinResult，保留具体失败原因而非坍缩成 kUnknownError。
OdinResult MapLoopClosureStatus(odin::algo::LoopClosureStatus st) {
  switch (st) {
    case odin::algo::LoopClosureStatus::kOk:            return OdinResult::kOk;
    case odin::algo::LoopClosureStatus::kInvalidArg:    return OdinResult::kInvalidArgument;
    case odin::algo::LoopClosureStatus::kIoError:       return OdinResult::kIoError;
    case odin::algo::LoopClosureStatus::kMapLoadFailed: return OdinResult::kMapLoadFailed;
    case odin::algo::LoopClosureStatus::kInternal:      return OdinResult::kAlgoInternalError;
    default:                                            return OdinResult::kUnknownError;
  }
}

}  // namespace

// =============================================================================
// Loop-closure helpers
// =============================================================================

odin::algo::AlgoImageFormat OdinProtocolV2BaseDevice::ToLcImageFormat(OdinDataFormat fmt) {
  switch (fmt) {
    case OdinDataFormat::kImageMjpeg:
      return odin::algo::AlgoImageFormat::kMjpeg;
    case OdinDataFormat::kImageNv12:
      return odin::algo::AlgoImageFormat::kNv12;
    default:
      return odin::algo::AlgoImageFormat::kMjpeg;
  }
}

std::string OdinProtocolV2BaseDevice::FetchCalibrationToString() {
  // Download calibration to a temp file, read it, then delete the temp file.
  // Use the platform temp directory — "/tmp" only exists on POSIX; on Windows
  // both the 0x41-channel fopen and the HTTP-path rename would fail on it.
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::path temp_dir = fs::temp_directory_path(ec);
  if (ec) temp_dir = ".";
  const std::string temp_path =
      (temp_dir / ("odin_calib_" + discovered_device_.sn + ".yaml")).string();

  OdinResult result = GetFile(OdinFileType::kCalibrationFile, temp_path, nullptr, 5000);
  if (result != OdinResult::kOk) {
    LOG_WARN("FetchCalibrationToString: download failed (err=%d), skipping lc init\n",
             static_cast<int>(result));
    return {};
  }

  std::ifstream ifs(temp_path);
  if (!ifs.is_open()) {
    LOG_WARN("FetchCalibrationToString: cannot open %s\n", temp_path.c_str());
    std::remove(temp_path.c_str());
    return {};
  }

  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());
  ifs.close();
  std::remove(temp_path.c_str());

  if (content.empty()) {
    LOG_WARN("FetchCalibrationToString: empty calibration file, skipping lc init\n");
  }
  return content;
}

OdinProtocolV2BaseDevice::OdinProtocolV2BaseDevice(OdinDeviceHandle handle)
    : handle_(handle),
      command_(std::make_unique<OdinCommand>()),
      slam_odom_sync_(std::make_unique<SlamOdomSynchronizer>()) {
  // Set SLAM transform function for coordinate transformation using Odom pose
  // Input: float format (already converted from uint16 by OdinFrameAssembler)
  // Output: float format with rotation and translation applied
  if (slam_odom_sync_) {
    slam_odom_sync_->SetSlamTransformFunction(
        [](OdinSlamPacket& packet, const OdinOdomPacket& odom_pkt) {
          // Match found - transform points to world frame: pt_world = R * pt_imu + t
          if (odom_pkt.data.empty()) {
            return;
          }
          const OdinOdomData& odom_data = odom_pkt.data[0];

          // Extract quaternion and position from odom (wire is now float SI units:
          // position in meters, orientation a unit quaternion -> no scaling).
          constexpr double kPosScale = 1.0;
          constexpr double kOrientScale = 1.0;
          double tx = odom_data.pos[0] * kPosScale;
          double ty = odom_data.pos[1] * kPosScale;
          double tz = odom_data.pos[2] * kPosScale;
          double qx = odom_data.orient[0] * kOrientScale;
          double qy = odom_data.orient[1] * kOrientScale;
          double qz = odom_data.orient[2] * kOrientScale;
          double qw = odom_data.orient[3] * kOrientScale;

          // Build rotation matrix R^{w}_{i} from quaternion using Eigen
          Eigen::Quaterniond q(qw, qx, qy, qz);
          q.normalize();
          Eigen::Matrix3d R = q.toRotationMatrix();
          Eigen::Vector3d t(tx, ty, tz);

          // Transform each point: pt_world = R * pt_imu + t
          // Input is already float format (converted by OdinFrameAssembler)
          for (auto& pt : packet.points) {
            // Convert from millimeters to meters for transformation
            Eigen::Vector3d pt_imu(static_cast<double>(pt.x) * 0.001,
                                   static_cast<double>(pt.y) * 0.001,
                                   static_cast<double>(pt.z) * 0.001);

            // Apply rotation and translation: pt_world = R * pt_imu + t
            Eigen::Vector3d pt_world = R * pt_imu + t;

            // Store back as float (in millimeters)
            pt.x = static_cast<float>(pt_world.x() / 0.001);
            pt.y = static_cast<float>(pt_world.y() / 0.001);
            pt.z = static_cast<float>(pt_world.z() / 0.001);
            // r, g, b, a remain unchanged
          }
        });
  }
}

OdinProtocolV2BaseDevice::~OdinProtocolV2BaseDevice() {
  Disconnect();
}

bool OdinProtocolV2BaseDevice::Connect(const DiscoveredDevice& discovered_device) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (connected_.load()) return true;

  // Cache the descriptor and forward it down. The device layer never
  // inspects transport-specific fields (IP, host_ip, USB path, ...);
  // each component pulls the bits it needs from the descriptor itself.
  discovered_device_ = discovered_device;

  // The command layer is now the unified device-control surface; it pulls
  // out whatever transport-specific bits it needs from the descriptor and
  // brings up its own internal sub-protocols (TLV / HTTP / file ...).
  if (!command_->Connect(discovered_device_, handle_)) {
    LOG_ERROR("OdinProtocolV2BaseDevice::Connect: Command connect failed\n");
    return false;
  }

  connected_.store(true);
  LOG_INFO("Odin2Device connected: %s (SN: %s, Model: %s)\n",
           discovered_device_.network.ip_address.c_str(),
           discovered_device_.sn.c_str(),
           discovered_device_.model.c_str());

  StartHeartbeat();

  // Download calibration and initialize loop-closure session.
  // Failure is non-fatal: lc_handle_ stays empty and all algo calls
  // silently degrade.
  std::string calib_yaml = FetchCalibrationToString();
  if (!calib_yaml.empty()) {
    InitLcSession(odin::algo::AlgoMode::kIdle);
  }

  return true;
}

void OdinProtocolV2BaseDevice::Disconnect() {
  StopHeartbeat();

  // Destroy loop-closure handle
  {
    std::lock_guard<std::mutex> lock(lc_mutex_);
    lc_handle_ = odin::algo::AlgoHandle{};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_.load()) return;

  StopDataChannels();
  command_->Disconnect();

  connected_.store(false);
  LOG_INFO("Odin2Device disconnected: %s\n",  
           discovered_device_.network.ip_address.c_str());
}

bool OdinProtocolV2BaseDevice::IsConnected() const {
  return connected_.load();
}

OdinDeviceHandle OdinProtocolV2BaseDevice::GetHandle() const {
  return handle_;
}

std::string OdinProtocolV2BaseDevice::GetSerialNumber() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return discovered_device_.sn;
}

std::string OdinProtocolV2BaseDevice::GetModel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return discovered_device_.model;
}

MTConnectionType OdinProtocolV2BaseDevice::GetConnectionType() const {
  return MTConnectionType::kEthernet;
}

const DiscoveredDevice& OdinProtocolV2BaseDevice::GetDiscoveredDevice() const {
  return discovered_device_;
}

// =============================================================================
// Loop-closure: session management (plugin architecture)
// =============================================================================

odin::algo::ILoopClosureAlgo* OdinProtocolV2BaseDevice::GetLcAlgo() const {
  std::lock_guard<std::mutex> lock(lc_mutex_);
  if (!lc_handle_.instance) {
    return nullptr;
  }
  // QueryInterface returns the domain interface pointer (virtual inheritance safe)
  return static_cast<odin::algo::ILoopClosureAlgo*>(
      lc_handle_.instance->QueryInterface(odin::algo::kCapLoopClosure));
}

OdinResult OdinProtocolV2BaseDevice::InitLcSession(odin::algo::AlgoMode mode) {
  std::lock_guard<std::mutex> ctrl_lock(lc_control_mutex_);

  // Check if already loaded
  if (lc_handle_.instance) {
    return OdinResult::kOk;
  }

  std::string calib_yaml = FetchCalibrationToString();
  if (calib_yaml.empty()) {
    LOG_WARN("InitLcSession: no calibration, skipping\n");
    return OdinResult::kOk;
  }

  // Prepare load request
  // The plugin's odin_algo_name() reports "manifold_lc", so we request that
  // name. The actual library file follows the ABI-versioned naming convention
  // libmanifold_lc_sdk.so.<major>, built via the loader helper.
  odin::algo::AlgoLoadRequest req;
  req.name = "manifold_lc";
  req.required_capability = odin::algo::kCapLoopClosure;
  const std::string lc_plugin_path =
      odin::algo::MakeAbiVersionedPluginPath("manifold_lc_sdk");
  const char* env_path = getenv("ODIN_ALGO_LIB_PATH");
  req.explicit_path = env_path ? env_path : lc_plugin_path.c_str();

  // Prepare init config
  odin::algo::AlgoInitConfig cfg;
  cfg.device_type = (discovered_device_.model == "Odin2Special")
      ? odin::algo::AlgoDeviceType::kOdin2S
      : odin::algo::AlgoDeviceType::kOdin1L;
  cfg.calib_yaml = calib_yaml.c_str();
  cfg.calib_yaml_size = calib_yaml.size();  // explicit size; binary-safe
  cfg.output_dir = "./log";
  cfg.drain_interval_ms = 200;
  cfg.instance_tag = discovered_device_.sn.c_str();

  // Extension for loopclosure
  odin::algo::LoopClosureInitExt ext;
  ext.cloud_pipeline = 0;  // Raw
  cfg.extension = &ext;

  req.cfg = &cfg;

  // Load plugin
  std::string error;
  auto handle = odin::algo::AlgoLoader::Load(req, &error);
  if (!handle.instance) {
    const char* env_path = getenv("ODIN_ALGO_LIB_PATH");
    LOG_WARN("InitLcSession: loop-closure algorithm library not available.\n"
             "  requested plugin : %s\n"
             "  required caps    : 0x%x\n"
             "  resolved path    : %s\n"
             "  loader error     : %s\n"
             "  env ODIN_ALGO_LIB_PATH : %s\n"
             "  search hint      : ensure '%s' is installed and discoverable\n"
             "                     (e.g. /usr/lib/odin-algo/, /usr/lib/, or via LD_LIBRARY_PATH).\n"
             "                     If the file exists, run 'ldd /path/to/%s' to check\n"
             "                     missing dependencies and 'ldconfig' to refresh cache.\n",
             req.name ? req.name : "(null)",
             req.required_capability,
             req.explicit_path ? req.explicit_path : "(none)",
             error.empty() ? "(no error detail)" : error.c_str(),
             env_path ? env_path : "(not set)",
             lc_plugin_path.c_str(),
             lc_plugin_path.c_str());
    return OdinResult::kOk;  // degrade gracefully
  }

  LOG_INFO("InitLcSession: loaded %s v%s\n", handle.name.c_str(), handle.version.c_str());

  // Get the loopclosure interface
  auto* lc = static_cast<odin::algo::ILoopClosureAlgo*>(
      handle.instance->QueryInterface(odin::algo::kCapLoopClosure));
  if (!lc) {
    LOG_ERROR("InitLcSession: QueryInterface failed\n");
    return OdinResult::kAlgoInternalError;
  }

  // Register relocalization anchor callback: emit as TF (map -> odom)
  // Note: C function pointer callback - need a trampoline
  lc->SetRelocAnchorCallback(
      [](const odin::algo::AlgoRelocAnchor* anchor, void* user) {
        auto* device = static_cast<OdinProtocolV2BaseDevice*>(user);
        device->EmitRelocAnchorAsTf(*anchor);
      },
      this);
   lc->SetOptimizationStatusCallback(
      [](bool busy, void* user) {
        auto* device = static_cast<OdinProtocolV2BaseDevice*>(user);
        device->EmitLcOptimizationStatus(busy);
      },
      this);
  lc->SetFrameCallback(
      [](const odin::algo::AlgoFrameInfo* frame, void* user) {
        if (!frame) return;
        auto* device = static_cast<OdinProtocolV2BaseDevice*>(user);
        device->EmitLcFrame(*frame);
      },
      this);
  lc->SetOptimizationResultCallback(
      [](const odin::algo::AlgoOptimizationResult* result, void* user) {
        if (!result) return;
        auto* device = static_cast<OdinProtocolV2BaseDevice*>(user);
        device->EmitLcOptimizationResult(*result);
      },
      this);
  lc->SetMapCloudCallback(
      [](const float* xyz, size_t npts, void* user) {
        auto* device = static_cast<OdinProtocolV2BaseDevice*>(user);
        device->EmitLcMapCloud(xyz, npts);
      },
      this);
  // Store handle
  {
    std::lock_guard<std::mutex> lock(lc_mutex_);
    lc_handle_ = std::move(handle);
  }

  LOG_INFO("InitLcSession: loop-closure session initialized (mode=%d)\n",
           static_cast<int>(mode));

  // Switch to initial mode
  auto st = lc->SetMode(mode);
  if (st != odin::algo::AlgoStatus::kOk) {
    LOG_WARN("InitLcSession: SetMode returned %d\n", static_cast<int>(st));
  }

  return OdinResult::kOk;
}

void OdinProtocolV2BaseDevice::EmitRelocAnchorAsTf(const odin::algo::AlgoRelocAnchor& anchor) const {
  if (!tf_cb_) return;

  OdinTfPacket pkt;
  pkt.timestamp = static_cast<uint64_t>(anchor.stamp * 1e9);  // seconds -> ns
  pkt.frame_id       = "map";   // child frame
  pkt.child_frame_id = "odom";  // parent frame (device odometry)

  // Convert row-major rotation matrix to quaternion [x, y, z, w]
  Eigen::Matrix3d R;
  R << anchor.R[0], anchor.R[1], anchor.R[2],
       anchor.R[3], anchor.R[4], anchor.R[5],
       anchor.R[6], anchor.R[7], anchor.R[8];
  Eigen::Quaterniond q(R);
  q.normalize();

  // TF flip: anchor pose is map->odom, emit inverted transform (odom->map):
  // rotation: conjugate; translation: -(R^T * t)
  Eigen::Quaterniond q_inv = q.conjugate();
  Eigen::Vector3d t(anchor.t[0], anchor.t[1], anchor.t[2]);  // meters
  Eigen::Vector3d t_inv = -(q_inv * t);

  // OdinTfPacket.translation is float meters
  pkt.translation[0] = static_cast<float>(t_inv.x());
  pkt.translation[1] = static_cast<float>(t_inv.y());
  pkt.translation[2] = static_cast<float>(t_inv.z());
  pkt.rotation[0] = static_cast<float>(q_inv.x());
  pkt.rotation[1] = static_cast<float>(q_inv.y());
  pkt.rotation[2] = static_cast<float>(q_inv.z());
  pkt.rotation[3] = static_cast<float>(q_inv.w());

  tf_cb_(pkt, tf_user_data_);
}

void OdinProtocolV2BaseDevice::EmitLcOptimizationStatus(bool busy) const {
  if (!lc_optimization_status_cb_) return;
  OdinLcOptimizationStatusPacket pkt;
  pkt.timestamp = NowNs();
  pkt.busy = busy;
  lc_optimization_status_cb_(pkt, lc_optimization_status_user_data_);
}

void OdinProtocolV2BaseDevice::EmitLcFrame(const odin::algo::AlgoFrameInfo& frame) const {
  if (!lc_frame_cb_) return;
  OdinLcFramePacket pkt;
  pkt.timestamp = frame.stamp > 0.0 ? static_cast<uint64_t>(frame.stamp * 1e9) : NowNs();
  pkt.frame_id = frame.frame_id;
  std::memcpy(pkt.translation, frame.t_wi_adj, sizeof(pkt.translation));
  RotationMatrixToQuaternion(frame.R_wi_adj, pkt.rotation);
  pkt.is_keyframe = frame.is_keyframe;
  lc_frame_cb_(pkt, lc_frame_user_data_);
}

void OdinProtocolV2BaseDevice::EmitLcOptimizationResult(
    const odin::algo::AlgoOptimizationResult& result) const {
  if (!lc_optimization_result_cb_) return;
  OdinLcOptimizationResultPacket pkt;
  pkt.timestamp = NowNs();
  if (result.poses && result.pose_count > 0) {
    pkt.poses.reserve(result.pose_count);
    for (size_t i = 0; i < result.pose_count; ++i) {
      const auto& in = result.poses[i];
      OdinLcPoseStamped out;
      out.timestamp = in.stamp > 0.0 ? static_cast<uint64_t>(in.stamp * 1e9) : 0;
      std::memcpy(out.translation, in.t, sizeof(out.translation));
      RotationMatrixToQuaternion(in.R, out.rotation);
      pkt.poses.push_back(out);
      if (out.timestamp != 0) pkt.timestamp = out.timestamp;
    }
  }
  pkt.cloud = CopyXyzCloud(result.cloud_xyz, result.cloud_npts);
  lc_optimization_result_cb_(pkt, lc_optimization_result_user_data_);
}

void OdinProtocolV2BaseDevice::EmitLcMapCloud(const float* xyz, size_t npts) const {
  if (!lc_map_cloud_cb_) return;
  OdinLcMapCloudPacket pkt;
  pkt.timestamp = NowNs();
  pkt.points = CopyXyzCloud(xyz, npts);
  lc_map_cloud_cb_(pkt, lc_map_cloud_user_data_);
}

OdinResult OdinProtocolV2BaseDevice::GetFirmwareVersion(std::string& version, uint32_t timeout_ms) {
  OdinResult result = command_->GetFirmwareVersion(version, timeout_ms);
  if (result == OdinResult::kOk) {
    // Append algorithm version if available
    std::lock_guard<std::mutex> lock(lc_mutex_);
    if (lc_handle_.instance && !lc_handle_.version.empty()) {
      version += "|Algo:" + lc_handle_.version;
    }
  }
  return result;
}

OdinResult OdinProtocolV2BaseDevice::SetDeviceMode(OdinDeviceMode mode, uint32_t timeout_ms) {
   std::lock_guard<std::mutex> ctrl_lock(lc_control_mutex_);

  // Switch host-side algorithm mode first (per porting doc: algo before device)
  odin::algo::AlgoMode lc_mode = odin::algo::AlgoMode::kIdle;
  OdinDeviceMode device_mode = mode;
  switch (mode) {
    case OdinDeviceMode::kSlam:
      lc_mode = odin::algo::AlgoMode::kBuild;
      break;
    case OdinDeviceMode::KRelocation:
      lc_mode = odin::algo::AlgoMode::kReloc;
      break;
    case OdinDeviceMode::kReMap:
      lc_mode = odin::algo::AlgoMode::kReMap;
      device_mode = OdinDeviceMode::kSlam;
      break;
    default:
      lc_mode = odin::algo::AlgoMode::kIdle;
      break;
  }

  auto* lc = GetLcAlgo();
  if (lc) {
    auto st = lc->SetMode(lc_mode);
    if (st != odin::algo::AlgoStatus::kOk) {
      LOG_WARN("SetDeviceMode: SetMode returned %d\n", static_cast<int>(st));
    }
  } else if (lc_mode != odin::algo::AlgoMode::kIdle) {
    // Fail fast: entering SLAM/reloc/remap mode without the algorithm library would
    // leave the device in a mode with no host-side support. Report the error
    // immediately and skip the device command.
    const std::string& last_err = odin::algo::AlgoLoader::LastLoadError();
    LOG_WARN("SetDeviceMode: loop-closure algorithm library not available; "
             "refusing SLAM/relocalization/remap mode switch. last load error: %s\n",
             last_err.empty() ? "(none recorded - library may never have been loaded)"
                              : last_err.c_str());
    return OdinResult::kNotInitialized;
  }

  // Then send device mode command. ReMap is host-side only; keep the device data path in SLAM mode.
  return command_->SetDeviceMode(device_mode, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::SendAlgoData(const uint8_t* data, size_t len) {
  if (!IsConnected() || !command_) {
    return OdinResult::kNotInitialized;
  }
  return command_->SendAlgoData(data, len);
}

OdinResult OdinProtocolV2BaseDevice::GetDeviceMode(OdinDeviceMode& mode, uint32_t timeout_ms) {
  return command_->GetDeviceMode(mode, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::SetSensorMode(OdinSensorMode mode, uint32_t timeout_ms) {
  return command_->SetSensorMode(mode, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::GetSensorMode(OdinSensorMode& mode, uint32_t timeout_ms) {
  return command_->GetSensorMode(mode, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                             const std::vector<OdinDataChannel>& channels,
                                             uint32_t timeout_ms) {
  return command_->GetSensorCapability(capabilities, channels, timeout_ms);
}

void OdinProtocolV2BaseDevice::EnableSlamOdomSync(bool enabled, uint32_t max_frame_lag) {
  if (slam_odom_sync_) {
    slam_odom_sync_->SetEnabled(enabled);
    slam_odom_sync_->SetMaxFrameLag(max_frame_lag);
    LOG_INFO("Odin2Device[%d]: SLAM-Odom sync %s (max_lag=%u)\n", handle_,
             enabled ? "enabled" : "disabled", max_frame_lag);
  }
}

OdinResult OdinProtocolV2BaseDevice::StartStream(OdinDataChannel channel, OdinTransportMode transport,
                                     const OdinStreamCfg* mode, uint32_t timeout_ms) {
  // Reject kImage1 on single-camera devices
  if (channel == OdinDataChannel::kImage1 && GetCameraCount() < 2) {
    LOG_ERROR("StartStream: kImage1 not supported on this device\n");
    return OdinResult::kNotSupported;
  }

  // Validate: mode parameter only valid for Image channels
  bool is_image_channel = (channel == OdinDataChannel::kImage0 || 
                           channel == OdinDataChannel::kImage1);
  if (mode != nullptr && !is_image_channel) {
    LOG_ERROR("StartStream: mode parameter only valid for Image channels\n");
    return OdinResult::kInvalidArgument;
  }

  // Determine if H264 format is requested (affects callback routing)
  bool is_h264 = (mode != nullptr && mode->format == OdinDataFormat::kImageH264);
  // Width/height from the user-provided StreamCfg; applied as fallback in lambdas
  // (assembler emits 0 for both fields; the lambda fills them from config if still 0).
  uint16_t stream_width  = (mode != nullptr) ? mode->width  : 0;
  uint16_t stream_height = (mode != nullptr) ? mode->height : 0;

  // Step 1: Send start-stream command. The command layer owns local-side
  // endpoint allocation; the device never sees the underlying port.
  OdinResult result = command_->SendStartStream(channel, transport, mode, timeout_ms);
  if (result != OdinResult::kOk) {
    LOG_ERROR("StartStream: SendStartStream failed\n");
    return result;
  }

  // Step 2: Create capture and assembler. The transport mode is forwarded
  // verbatim; classification of sensor data vs datagram-style behaviour is
  // encapsulated inside each component.
  auto capture = std::make_unique<NetworkCapture>(transport);
  auto assembler = std::make_unique<OdinFrameAssembler>(transport);

  // Tell the assembler which format the device was configured to send so
  // emitted frames carry the correct OdinDataFormat.
  if (is_image_channel && is_h264) {
    assembler->SetCameraFormat(OdinDataFormat::kImageH264);
  } else if (is_image_channel && mode != nullptr) {
    assembler->SetCameraFormat(mode->format);
  }

  // Store image0 config for PushImage
  if (channel == OdinDataChannel::kImage0 && mode != nullptr) {
    image0_format_ = mode->format;
    image0_width_  = mode->width;
    image0_height_ = mode->height;
  }

  // Set up assembler callbacks based on channel
  switch (channel) {
    case OdinDataChannel::kRawPoint:
      assembler->SetPointCloudCallback([this](OdinPointCloudPacket pkt) {
        // Push to loop-closure algorithm
        auto* lc = GetLcAlgo();
        if (lc) {
          odin::algo::AlgoCloudMsg msg;
          msg.stamp    = static_cast<double>(pkt.timestamp) / 1e9;  // ns → s
          msg.lidar_id = 0;
          msg.is_dense = false;
          msg.already_deskewed = false;

          // Convert points: mm → m, NaN/Inf/|x|>100m → clamp to 0 (keep point count)
          std::vector<odin::algo::AlgoPoint> lc_points;
          lc_points.reserve(pkt.points.size());
          for (size_t i = 0; i < pkt.points.size(); ++i) {
            const auto& pt = pkt.points[i];
            odin::algo::AlgoPoint lp;
            float mx = pt.x * 0.001f;
            float my = pt.y * 0.001f;
            float mz = pt.z * 0.001f;
            if (std::isnan(mx) || std::isinf(mx) || std::fabs(mx) > 100.0f) mx = 0.0f;
            if (std::isnan(my) || std::isinf(my) || std::fabs(my) > 100.0f) my = 0.0f;
            if (std::isnan(mz) || std::isinf(mz) || std::fabs(mz) > 100.0f) mz = 0.0f;
            lp.x = mx;
            lp.y = my;
            lp.z = mz;
            lp.intensity   = static_cast<float>(pt.intensity);
            // group_idx × (1/328.5): 1536 points per group (256/row × 6 rows),
            // group interval ≈ 3.044 ms; unit: seconds relative to scan start
            lp.offset_time = static_cast<float>((i / 1536u) / 328.5);
            lc_points.push_back(lp);
          }
          msg.points = lc_points.data();
          msg.count  = lc_points.size();
          lc->PushCloud(&msg);
        }

        if (point_cloud_cb_) point_cloud_cb_(pkt, point_cloud_user_data_);
      });
      break;
    case OdinDataChannel::kSlamPoint:
      assembler->SetSlamCallback([this](const OdinSlamPacket& pkt) {
        // Data is already in float format (converted by OdinFrameAssembler)
        // Route through synchronizer for coordinate transformation if enabled
        bool sync_enabled = slam_odom_sync_ && slam_odom_sync_->IsEnabled();
        if (sync_enabled) {
          // Synchronizer will apply rotation/translation and call user callback
          slam_odom_sync_->ProcessSlam(pkt);
        } else {
          // No sync - call user callback directly (data already in float format)
          if (slam_cb_) {
            slam_cb_(pkt, slam_user_data_);
          }
        }
      });
      break;
    case OdinDataChannel::kImage0:
      if (is_h264) {
        assembler->SetCameraCallback([this, stream_width, stream_height](const OdinCameraPacket& pkt) {
          OdinCameraPacket p = pkt;
          if (p.width == 0) p.width = stream_width;
          if (p.height == 0) p.height = stream_height;
          if (camera_cb_) camera_cb_(p, camera_user_data_);
        });
      } else {
        assembler->SetCameraCallback([this, stream_width, stream_height](const OdinCameraPacket& pkt) {
          OdinCameraPacket p = pkt;
          if (p.width == 0) p.width = stream_width;
          if (p.height == 0) p.height = stream_height;

          // Push to loop-closure algorithm
          auto* lc = GetLcAlgo();
          if (lc && !p.payload.empty()) {
            odin::algo::AlgoImageMsg msg;
            msg.stamp    = static_cast<double>(p.timestamp) / 1e9;  // ns → s
            msg.camera_id = 0;
            msg.data     = p.payload.data();
            msg.size     = p.payload.size();
            msg.width    = p.width;
            msg.height   = p.height;
            msg.format   = ToLcImageFormat(image0_format_);
            lc->PushImage(&msg);
          }

          if (camera_cb_) camera_cb_(p, camera_user_data_);
        });
      }
      break;
    case OdinDataChannel::kImage1:
      if (is_h264) {
        assembler->SetCameraCallback([this, stream_width, stream_height](const OdinCameraPacket& pkt) {
          OdinCameraPacket p = pkt;
          if (p.width == 0) p.width = stream_width;
          if (p.height == 0) p.height = stream_height;
          if (camera2_cb_) camera2_cb_(p, camera2_user_data_);
        });
      } else {
        assembler->SetCameraCallback([this, stream_width, stream_height](const OdinCameraPacket& pkt) {
          OdinCameraPacket p = pkt;
          if (p.width == 0) p.width = stream_width;
          if (p.height == 0) p.height = stream_height;
          if (camera2_cb_) camera2_cb_(p, camera2_user_data_);
        });
      }
      break;
    case OdinDataChannel::kImu:
      assembler->SetImuCallback([this](const OdinImuPacket& pkt) {
        if (imu_cb_) imu_cb_(pkt, imu_user_data_);
      });
      break;
    // Odom Routing rules:
    //   - LF + sync enabled  -> SlamOdomSynchronizer (SLAM/odom alignment).
    //   - LF + sync disabled -> deliver to user callback directly.
    //   - HF (always)        -> deliver to user callback directly, bypassing
    //                            the synchronizer (HF odom must not be gated
    //                            by SLAM frame timing).
    case OdinDataChannel::kOdomLF:
      assembler->SetOdomCallback([this](const OdinOdomPacket& pkt, OdomSourceType type) {
        const bool sync_enabled = slam_odom_sync_ && slam_odom_sync_->IsEnabled();
        if(sync_enabled){
          slam_odom_sync_->ProcessOdom(pkt, type);
        }
        else{
          if (odom_cb_) odom_cb_(pkt, type, odom_user_data_);
        }
      });
      break;
    case OdinDataChannel::kOdomHF:
      assembler->SetOdomCallback([this](const OdinOdomPacket& pkt, OdomSourceType type) {
        // Push to loop-closure algorithm (only kOdom2ImuHigh)
        if (type == OdomSourceType::kOdom2ImuHigh) {
          auto* lc = GetLcAlgo();
          if (lc && !pkt.data.empty()) {
            const OdinOdomData& od = pkt.data[0];
            odin::algo::AlgoOdomMsg msg;
            msg.stamp = static_cast<double>(pkt.timestamp) / 1e9;  // ns → s
            // float → double, element-wise (no memcpy!) and reorder quaternion [x,y,z,w] → [w,x,y,z]
            msg.position[0] = static_cast<double>(od.pos[0]);
            msg.position[1] = static_cast<double>(od.pos[1]);
            msg.position[2] = static_cast<double>(od.pos[2]);
            msg.orientation[0] = static_cast<double>(od.orient[3]);  // w
            msg.orientation[1] = static_cast<double>(od.orient[0]);  // x
            msg.orientation[2] = static_cast<double>(od.orient[1]);  // y
            msg.orientation[3] = static_cast<double>(od.orient[2]);  // z
            lc->PushOdom(&msg);
          }
        }
        if (odom_cb_) odom_cb_(pkt, type, odom_user_data_);
      });
      break;
    case OdinDataChannel::kTf:
      assembler->SetTfCallback([this](const OdinTfPacket& pkt) {
        if (tf_cb_) tf_cb_(pkt, tf_user_data_);
      });
      break;
    default:
      LOG_ERROR("StartStream: Unknown channel 0x%02x\n", static_cast<uint8_t>(channel));
      command_->SendCloseStream(channel, timeout_ms);
      return OdinResult::kInvalidArgument;
  }

  // Create raw data callback that routes to assembler
  OdinFrameAssembler* asm_ptr = assembler.get();
  RawDataCallback raw_cb;
  switch (channel) {
    case OdinDataChannel::kRawPoint:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessPointData(data, len, false);
      };
      break;
    case OdinDataChannel::kSlamPoint:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessPointData(data, len, true);
      };
      break;
    case OdinDataChannel::kImage0:
    case OdinDataChannel::kImage1:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessCameraData(data, len);
      };
      break;
    case OdinDataChannel::kImu:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessImuData(data, len);
      };
      break;
    case OdinDataChannel::kOdomLF:
    case OdinDataChannel::kOdomHF:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessOdomData(data, len);
      };
      break;
    case OdinDataChannel::kTf:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessTfData(data, len);
      };
      break;
    default:
      break;
  }

  // Start capture against the local endpoint chosen by the command layer.
  // The reference is opaque to the device (typed as ITransportAddress) and
  // is forwarded straight to capture without inspecting its fields.
  if (!capture->Start(command_->GetLocalEndpoint(channel), raw_cb)) {
    LOG_ERROR("StartStream: StartCapture failed\n");
    command_->SendCloseStream(channel, timeout_ms);
    return OdinResult::kSocketError;
  }
  
  captures_[channel] = std::move(capture);
  assemblers_[channel] = std::move(assembler);

  LOG_INFO("StartStream: channel=0x%02x, local=%s, %s ok\n",
           static_cast<uint8_t>(channel),
           command_->GetLocalEndpoint(channel).ToString().c_str(),
           transport == OdinTransportMode::kTcp ? "TCP" : "UDP");
  return OdinResult::kOk;
}

OdinResult OdinProtocolV2BaseDevice::CloseStream(OdinDataChannel channel, uint32_t timeout_ms) {
  // Reject kImage1 on single-camera devices
  if (channel == OdinDataChannel::kImage1 && GetCameraCount() < 2) {
    LOG_ERROR("CloseStream: kImage1 not supported on this device\n");
    return OdinResult::kNotSupported;
  }

  // Step 1: Stop and remove capture instance
  auto it = captures_.find(channel);
  if (it != captures_.end()) {
    it->second->Stop();
    captures_.erase(it);
  }

  // Remove assembler
  auto asm_it = assemblers_.find(channel);
  if (asm_it != assemblers_.end()) {
    assemblers_.erase(asm_it);
  }

  // Step 2: Send close stream command
  OdinResult result = command_->SendCloseStream(channel, timeout_ms);
  if (result != OdinResult::kOk) {
    LOG_ERROR("CloseStream: SendCloseStream failed\n");
    return result;
  }

  LOG_INFO("CloseStream: channel=0x%02x ok\n", static_cast<uint8_t>(channel));
  return OdinResult::kOk;
}

OdinResult OdinProtocolV2BaseDevice::GetChannelConfig(OdinDataChannel channel, OdinChannelConfigInfo& info,
                                          uint32_t timeout_ms) {
  if (channel == OdinDataChannel::kImage1 && GetCameraCount() < 2) {
    LOG_ERROR("GetChannelConfig: kImage1 not supported on this device\n");
    return OdinResult::kNotSupported;
  }
  return command_->GetChannelConfig(channel, info, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::SetFilterParam(const OdinLidarFilterParams& params,
                                       uint32_t timeout_ms) {
  return command_->SetFilterParam(params, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::GetFilterParam(OdinLidarFilterParams& params,
                                       uint32_t timeout_ms) {
  return command_->GetFilterParam(params, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::RestoreDefaultConfig(uint32_t timeout_ms) {
  return command_->RestoreDefaultConfig(timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::SetDefaultSyncMode(OdinTimeSyncMode mode, uint32_t timeout_ms) {
  return command_->SetDefaultSyncMode(mode, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::GetDefaultSyncMode(OdinTimeSyncMode& mode, uint32_t timeout_ms) {
  return command_->GetDefaultSyncMode(mode, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::GetPtpSyncStatus(OdinPtpSyncStatus& status, uint32_t timeout_ms) {
  return command_->GetPtpSyncStatus(status, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::SetAdaptiveParam(const OdinAdaptiveFilterParam& param,
                                         uint32_t timeout_ms) {
  return command_->SetAdaptiveParam(param, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::GetAdaptiveParam(OdinAdaptiveFilterParam& param,
                                         uint32_t timeout_ms) {
  return command_->GetAdaptiveParam(param, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::ResetInitialPose(uint32_t timeout_ms) {
  return command_->ResetInitialPose(timeout_ms);
}

// =============================================================================
// Host-side loop-closure / relocalization public API
// =============================================================================

OdinResult OdinProtocolV2BaseDevice::SaveMap(const std::string& path) {
  std::lock_guard<std::mutex> ctrl_lock(lc_control_mutex_);

  auto* lc = GetLcAlgo();
  if (!lc) {
    LOG_WARN("SaveMap: loop-closure session not available\n");
    return OdinResult::kNotInitialized;
  }

  auto st = lc->SaveMap(path.c_str());
  if (st != odin::algo::LoopClosureStatus::kOk) {
    char detail[256] = {0};
    lc->GetLastError(detail, sizeof(detail));
    LOG_ERROR("SaveMap: SaveMap returned %s(%d): %s\n",
              odin::algo::LoopClosureStatusStr(st), static_cast<int>(st), detail);
    return MapLoopClosureStatus(st);
  }

  LOG_INFO("SaveMap: map saved to %s\n", path.c_str());
  return OdinResult::kOk;
}

OdinResult OdinProtocolV2BaseDevice::LoadMap(const std::string& path) {
  std::lock_guard<std::mutex> ctrl_lock(lc_control_mutex_);
  
  auto* lc = GetLcAlgo();
  if (!lc) {
    LOG_WARN("LoadMap: loop-closure session not available\n");
    return OdinResult::kNotInitialized;
  }

  auto st = lc->LoadMap(path.c_str());
  if (st != odin::algo::LoopClosureStatus::kOk) {
    char detail[256] = {0};
    lc->GetLastError(detail, sizeof(detail));
    LOG_ERROR("LoadMap: LoadMap returned %s(%d): %s\n",
              odin::algo::LoopClosureStatusStr(st), static_cast<int>(st), detail);
    return MapLoopClosureStatus(st);
  }

  LOG_INFO("LoadMap: map loaded from %s\n", path.c_str());
  return OdinResult::kOk;
}

OdinResult OdinProtocolV2BaseDevice::ExployMap(const std::string& path) {
  std::lock_guard<std::mutex> ctrl_lock(lc_control_mutex_);

  auto* lc = GetLcAlgo();
  if (!lc) {
    LOG_WARN("ExployMap: loop-closure session not available\n");
    return OdinResult::kNotInitialized;
  }

  auto st = lc->ExployMap(path.c_str());
  if (st != odin::algo::LoopClosureStatus::kOk) {
    char detail[256] = {0};
    lc->GetLastError(detail, sizeof(detail));
    LOG_ERROR("ExployMap: ExployMap returned %s(%d): %s\n",
              odin::algo::LoopClosureStatusStr(st), static_cast<int>(st), detail);
    return MapLoopClosureStatus(st);
  }

  LOG_INFO("ExployMap: map exported to %s\n", path.c_str());
  return OdinResult::kOk;
}

void OdinProtocolV2BaseDevice::RegisterPointCloudCallback(OdinPointCloudCallback cb, void* user_data) {
  point_cloud_cb_ = cb;
  point_cloud_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::RegisterSlamCallback(OdinSlamCallback cb, void* user_data) {
  slam_cb_ = cb;
  slam_user_data_ = user_data;
  // Update synchronizer callbacks
  if (slam_odom_sync_) {
    slam_odom_sync_->SetCallbacks(cb, user_data, odom_cb_, odom_user_data_);
  }
}

void OdinProtocolV2BaseDevice::RegisterCameraCallback(OdinCameraCallback cb, void* user_data) {
  camera_cb_ = cb;
  camera_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::RegisterCameraCallback2(OdinCameraCallback cb, void* user_data) {
  if (GetCameraCount() < 2) {
    LOG_WARN("RegisterCameraCallback2: second camera not supported on this device\n");
    return;
  }
  camera2_cb_ = cb;
  camera2_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::RegisterImuCallback(OdinImuCallback cb, void* user_data) {
  imu_cb_ = cb;
  imu_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::RegisterOdomCallback(OdinOdomCallback cb, void* user_data) {
  odom_cb_ = cb;
  odom_user_data_ = user_data;
  // Update synchronizer callbacks
  if (slam_odom_sync_) {
    slam_odom_sync_->SetCallbacks(slam_cb_, slam_user_data_, cb, user_data);
  }
}

namespace {
// Build a CommandProgressCallback that forwards progress to the public
// OdinUpgradeProgressCallback, carrying the device handle.
CommandProgressCallback MakeProgressAdapter(OdinDeviceHandle handle,
                                            OdinUpgradeProgressCallback cb) {
  if (!cb) return nullptr;
  return [cb, handle](float progress) {
    cb(handle, progress, nullptr);
  };
}
}  // namespace


void OdinProtocolV2BaseDevice::RegisterTelemetryCallback(OdinTelemetryCallback cb, void* user_data) {
  telemetry_cb_ = cb;
  telemetry_user_data_ = user_data;
  if (command_) {
    command_->RegisterTelemetryCallback(cb, user_data);
  }
}

OdinResult OdinProtocolV2BaseDevice::StartTelemetry() {
  if (!command_) return OdinResult::kNotInitialized;
  command_->StartTelemetry();
  return OdinResult::kOk;
}

void OdinProtocolV2BaseDevice::RegisterTfCallback(OdinTfCallback cb, void* user_data) {
  tf_cb_ = cb;
  tf_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::RegisterLcOptimizationStatusCallback(
    OdinLcOptimizationStatusCallback cb, void* user_data) {
  lc_optimization_status_cb_ = cb;
  lc_optimization_status_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::RegisterLcFrameCallback(OdinLcFrameCallback cb, void* user_data) {
  lc_frame_cb_ = cb;
  lc_frame_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::RegisterLcOptimizationResultCallback(
    OdinLcOptimizationResultCallback cb, void* user_data) {
  lc_optimization_result_cb_ = cb;
  lc_optimization_result_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::RegisterLcMapCloudCallback(OdinLcMapCloudCallback cb, void* user_data) {
  lc_map_cloud_cb_ = cb;
  lc_map_cloud_user_data_ = user_data;
}

void OdinProtocolV2BaseDevice::StopDataChannels() {
  for (auto& pair : captures_) {
    if (pair.second) {
      pair.second->Stop();
    }
  }
  captures_.clear();
  assemblers_.clear();
}

// =============================================================================
// File Transfer: delegated to ICommand (formerly IFileService)
// =============================================================================

OdinResult OdinProtocolV2BaseDevice::SendFile(OdinFileType type, const std::string& file_path,
                                  OdinUpgradeProgressCallback cb) {
  if (!IsConnected()) {
    return OdinResult::kNotInitialized;
  }
  if (type == OdinFileType::kFirmware) {
    return SendFileFirmwareOta(file_path, cb);
  }

  return command_->Upload(type, file_path, MakeProgressAdapter(handle_, cb));
}

OdinResult OdinProtocolV2BaseDevice::GetFile(OdinFileType type, const std::string& save_path,
                                 OdinUpgradeProgressCallback cb,
                                 uint32_t timeout_ms) {
  if (!IsConnected()) {
    return OdinResult::kNotInitialized;
  }

  if (type == OdinFileType::kCalibrationFile) {
    // Prefer the active TCP connection (0x41 channel) over HTTP — BLOCKING.
    // Use a shared_ptr<promise> so the lambda is safe even if this function
    // returns early (wait_for timeout) while the background thread is still live.
    auto done_flag = std::make_shared<std::promise<bool>>();
    std::future<bool> done_future = done_flag->get_future();
    // Adapt CommandProgressCallback: forward incremental progress to the outer
    // OdinUpgradeProgressCallback; trigger the promise only on the final value
    // (100.0f = success, < 0.0f = failure) so set_value is called exactly once.
    CommandProgressCallback calib_progress_cb =
        [done_flag, cb, this](float progress) {
          if (progress >= 100.0f) {
            if (cb) cb(handle_, 100.0f, nullptr);
            done_flag->set_value(true);
          } else if (progress < 0.0f) {
            done_flag->set_value(false);
          } else {
            // Incremental chunk progress (0x41 multi-chunk path only).
            if (cb) cb(handle_, progress, nullptr);
          }
        };

    OdinResult init_result = command_->FetchCalibrationFile(save_path, calib_progress_cb, timeout_ms);
    if (init_result == OdinResult::kOk) {
      // Block until the background thread signals or the extended deadline passes.
      constexpr uint32_t kExtraMarginMs = 2000;
      auto status = done_future.wait_for(
          std::chrono::milliseconds(timeout_ms + kExtraMarginMs));
      if (status == std::future_status::ready && done_future.get()) {
        // cb already forwarded inside calib_progress_cb at 100.0f
        return OdinResult::kOk;
      }
      LOG_WARN("OdinProtocolV2BaseDevice::GetFile: 0x41 transfer failed/timed out, falling back to HTTP\n");
    } else {
      LOG_WARN("OdinProtocolV2BaseDevice::GetFile: 0x41 channel init failed (err=%d), falling back to HTTP\n",
               static_cast<int>(init_result));
    }
  }

  // HTTP path: synchronous blocking (HttpFileBackend::Download returns after save).
  return command_->Download(type, save_path, MakeProgressAdapter(handle_, cb));
}

// =============================================================================
// Firmware OTA orchestration: device owns heartbeat/device-mode, service
// owns the file transfer + state polling.
// =============================================================================

OdinResult OdinProtocolV2BaseDevice::SendFileFirmwareOta(const std::string& firmware_path,
                                             OdinUpgradeProgressCallback cb) {
  if (!command_) {
    return OdinResult::kNotInitialized;
  }

  LOG_INFO("SendFileFirmwareOta: Stopping heartbeat for OTA\n");
  StopHeartbeat();

  OdinResult mode_result = command_->SetDeviceMode(OdinDeviceMode::kUpgrade, 3000);
  if (mode_result != OdinResult::kOk) {
    LOG_ERROR("SendFileFirmwareOta: Failed to set upgrade mode\n");
    StartHeartbeat();
    return mode_result;
  }

  OdinResult result = command_->UploadFirmware(firmware_path,
                                                MakeProgressAdapter(handle_, cb), 300);
  if (result == OdinResult::kOk) {
    // OTA succeeded — device has rebooted with new firmware.
    // Heartbeat was intentionally stopped before OTA, so it never "failed".
    // The caller is expected to call DisconnectDevice() to clean up the stale
    // session; do not synthesize any offline/heartbeat-failure event here.
    return OdinResult::kOk;
  }

  // On non-fatal failure (e.g. HTTPS-fallback path), restart heartbeat so the
  // device session keeps running.
  StartHeartbeat();
  return result;
}

// =============================================================================
// Heartbeat
// =============================================================================

void OdinProtocolV2BaseDevice::StartHeartbeat() {
  if (heartbeat_running_.load()) return;

  if (!command_->IsHeartbeatSupported()) {
    LOG_INFO("Odin2Device: Heartbeat not supported\n");
    return;
  }

  heartbeat_running_.store(true);
  heartbeat_thread_ = std::thread(&OdinProtocolV2BaseDevice::HeartbeatThread, this);
  LOG_DEBUG("Odin2Device: Heartbeat thread started\n");
}

void OdinProtocolV2BaseDevice::StopHeartbeat() {
  if (!heartbeat_running_.load()) return;

  heartbeat_running_.store(false);

  if (heartbeat_thread_.joinable()) {
    if (std::this_thread::get_id() == heartbeat_thread_.get_id()) {
      heartbeat_thread_.detach();
    } else {
      heartbeat_thread_.join();
    }
  }
  LOG_DEBUG("Odin2Device: Heartbeat thread stopped\n");
}

void OdinProtocolV2BaseDevice::HeartbeatThread() {
  int consecutive_failures = 0;

  while (heartbeat_running_.load()) {
    uint32_t elapsed = 0;
    while (heartbeat_running_.load() && elapsed < heartbeat_interval_ms_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      elapsed += 100;
    }

    if (!heartbeat_running_.load()) break;

    if (!command_->IsHeartbeatSupported()) {
      continue;
    }

    if (command_->SendHeartbeat(heartbeat_interval_ms_, 1000)) {
      consecutive_failures = 0;
      if (heartbeat_success_cb_) {
        // Carry the descriptor with the notification: discovered_device_ is
        // written once in Connect() before this thread starts and is read-only
        // afterwards, so the keep-alive path stays lock-free end to end.
        heartbeat_success_cb_(handle_, discovered_device_);
      }
    } else {
      consecutive_failures++;
      if (consecutive_failures < heartbeat_max_failures_) {
        LOG_WARN("Odin2Device: Heartbeat failed (%d/%d)\n", 
                 consecutive_failures, heartbeat_max_failures_);
      } else if (consecutive_failures == heartbeat_max_failures_) {
        LOG_ERROR("Odin2Device: Heartbeat failed %d times, device may be unreachable\n",
                  consecutive_failures);
        // Keep the heartbeat thread alive: offline detection is delegated to
        // the hotplug listener (no success callbacks -> last_seen expires).
        // If the network/device recovers, the next successful heartbeat will
        // refresh last_seen and the device stays online without a spurious
        // detach/attach cycle.
      }
      // Beyond max_failures: suppress further logs to avoid log flooding.
      // Recovery is handled by the hotplug listener or a successful heartbeat.
    }
  }
}

void OdinProtocolV2BaseDevice::SetHeartbeatSuccessCallback(HeartbeatSuccessCallback cb) {
  heartbeat_success_cb_ = cb;
}

void OdinProtocolV2BaseDevice::SetHeartbeatInterval(uint32_t interval_ms) {
  heartbeat_interval_ms_ = interval_ms;
  LOG_INFO("Odin2Device: Heartbeat interval set to %u ms\n", interval_ms);
}

void OdinProtocolV2BaseDevice::SetHeartbeatTimeout(uint32_t timeout_ms) {
  if (heartbeat_interval_ms_ == 0) {
    heartbeat_max_failures_ = 0;
    LOG_WARN("Odin2Device: Heartbeat interval is 0, cannot compute max failures\n");
    return;
  }
  heartbeat_max_failures_ = static_cast<int>((timeout_ms + heartbeat_interval_ms_ - 1) / heartbeat_interval_ms_);
  LOG_INFO("Odin2Device: Heartbeat timeout set to %u ms, max failures = %d\n", 
           timeout_ms, heartbeat_max_failures_);
}

// =============================================================================
// Control-plane: delegated to ICommand (formerly IConfigService)
// =============================================================================

OdinResult OdinProtocolV2BaseDevice::GetNetworkAttribute(NetworkAttribute& attr, uint32_t timeout_ms) {
  if (!connected_.load() || !command_) {
    return OdinResult::kNotInitialized;
  }
  return command_->GetNetworkAttribute(attr, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::SetNetworkAttribute(const NetworkAttribute& attr, uint32_t timeout_ms) {
  if (!connected_.load() || !command_) {
    return OdinResult::kNotInitialized;
  }
  return command_->SetNetworkAttribute(attr, timeout_ms);
}

OdinResult OdinProtocolV2BaseDevice::Reboot(uint32_t timeout_ms) {
  if (!connected_.load() || !command_) {
    return OdinResult::kNotInitialized;
  }
  // ICommand::Reboot tears down the underlying TLV/HTTP channels itself,
  // so we mirror that at the device layer to keep IsConnected() honest.
  OdinResult r = command_->Reboot(timeout_ms);
  StopHeartbeat();
  connected_.store(false);
  return r;
}

}  // namespace sdk
}  // namespace odin
