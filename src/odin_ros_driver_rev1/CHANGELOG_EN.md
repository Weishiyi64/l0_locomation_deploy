This project follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the version numbers follow [Semantic Versioning](https://semver.org/).
3.7.0_20261016
1. Added loop-closure visualization topics publishing the loaded map, corrected keyframe results, optimization results and odometry trajectory.
2. Exposed ReMap mode (device_mode=5, OdinDeviceMode gains kReMap): switches host-side loop_closure to ReMap, keeps firmware data-source handling consistent with SLAM, updated SaveMap/LoadMap documentation to support kReMap, and added corrected loop-closure cloud publishing.
3. Restored the map_cloud callback registration; RViz now distinguishes loaded, corrected and optimized clouds with three colour tiers and displays a continuous odometry trajectory.
4. Fixed the map_cloud frame id and the stale RViz trajectory decay time; restored the split between SDK algorithm control and callback mutexes.
5. ROS service responses and logs now use OdinResultStr enum names instead of raw error codes.
6. Added the ExployMap loop-closure map export service: the loopclosure plugin ABI, the SDK ExployMap API and the ROS1/ROS2 exploy_map service are wired through; the export format itself is implemented by the plugin.
7. Added static libraries for openssl and crypto.

## [3.6.0] - 2026-09-30

### Added
- Runtime hardware-acceleration detection and dynamic decoder discovery for the
  FFmpeg H.264 decoder (NVDEC / CUVID / VAAPI / VDPAU), with automatic fallback
  to software decoding when hardware decoding fails.
- VAAPI / VDPAU hardware decode support (Intel / AMD GPUs), with device nodes
  selectable via environment variables:
  ODIN_CUDA_DEVICE / ODIN_VA_DEVICE / ODIN_VDPAU_DEVICE.
- Optional libyuv integration for faster YUV to BGR conversion in the H.264 /
  JPEG decode and NV12 composite paths (~7x faster than libswscale / OpenCV);
  can be disabled at runtime with ODIN_DISABLE_LIBYUV=1.
- docs/CameraStreamDecoding.md documenting the H.264 decode pipeline, the
  hardware / software fallback logic and per-platform prerequisites.
- Device-specific launch files odin1Lite.launch / odin1Lite.launch.py (single
  camera) and odin2Special.launch / odin2Special.launch.py (dual camera).
- RViz configs split per device: odin1lite_driver_view_ros1/ros2.rviz (single
  camera) and odin2special_driver_view_ros1/ros2.rviz (dual camera).
- After connecting, the driver node auto-detects the camera count
  (GetSensorCapability query for kImage1); single-camera devices automatically
  disable the image1 channel, callback, queue and publisher.
- The camera_count launch parameter that automatically selects the matching RViz
  config.
- GetCameraCount() virtual function in the SDK layer; Odin1Lite returns 1 and
  Odin2Special returns 2.

### Changed
- **BREAKING**: topic / service naming no longer contains the device model
  segment: the per-device prefix changed from /{topic_prefix}/{model}/device{N}/
  to /{topic_prefix}/device{N}/ (e.g. /manifold/ODIN2/device0/ →
  /manifold/device0/). Devices of different models now share stable paths and
  topics / services no longer change with the device model; the TF frame prefix
  device{N}/ is unaffected.
- The device_online message JSON gained a model field:
  {"sn":"...","model":"...","prefix":"...","ip":"..."}.
- **BREAKING**: the device_offline message changed from a plain SN string to
  JSON: {"sn":"...","model":"..."}. Downstream nodes (post_process /
  pointcloud_to_depth / opencv_undistort / depth_to_pointcloud) accept both old
  and new formats, but external scripts subscribing to this topic must be
  adapted to the new format.
- Updated rviz configs, rosbag2_qos.yaml, launch-file comments and the example
  topic prefixes in README / docs accordingly
  (/manifold/ODIN2/device0/ → /manifold/device0/).
- Device discovery now checks model support: devices whose reported model has
  neither a builtin implementation nor a matching plugin in the search path are
  no longer added to the device list (previously they silently fell back to a
  mismatched implementation at connect time), and a WARN log is emitted. An
  empty model field is normalized to Odin1Lite.
- The bare model "Odin" / "ODIN" reported by legacy firmware (case-insensitive)
  is uniformly mapped to Odin1Lite, keeping such devices identifiable and
  connectable.
- Optimized decoder log output during initialization and on the first frame.
- The SDK layer rejects kImage1 stream start / stop / config query / callback2
  registration on single-camera devices, returning kNotSupported.
- SDK device implementations are now plugins: Odin2Special is split into the
  dlopen plugin library libodin-device-Odin2Special.so (lib/<arch>/), which must
  be shipped alongside libodin_sdk.so; Odin1Lite is the built-in default
  fallback device.

### Fixed
- The odometry_lf / odometry_hf topics now fill the device-reported 6x6 pose
  covariance (pose.covariance, row-major over (x, y, z, roll, pitch, yaw);
  generation-2 firmware only fills the first 18 position entries). It was
  previously always 0. The twist covariance is not reported by the device and
  remains 0.

### Removed
- The generic launch files driver.launch / driver.launch.py (renamed to
  odin2Special.launch / odin2Special.launch.py): startup commands or scripts
  referencing the old names must switch to the device-specific launch files.
- driver_view_ros1.rviz / driver_view_ros2.rviz

3.5.0_20260921
1. Added device-specific launch files odin1Lite.launch / odin1Lite.launch.py (single
   camera) and odin2Special.launch / odin2Special.launch.py (dual camera).
2. Split RViz configs per device: odin1lite_driver_view_ros1/ros2.rviz (single
   camera, topic prefix /manifold/Odin1Lite/) and odin2special_driver_view_ros1/ros2.rviz
   (dual camera, topic prefix /manifold/Odin2Special/); removed the old
   driver_view_ros1.rviz / driver_view_ros2.rviz.
3. The driver node auto-detects the camera count after connecting to a device
   (GetSensorCapability query for kImage1); single-camera devices automatically
   disable the image1 channel, callback, queue and publisher.
4. Added the camera_count launch parameter, which automatically selects the matching
   RViz config based on the camera count.
5. Added the GetCameraCount() virtual function in the SDK layer; Odin1Lite returns 1
   and Odin2Special returns 2.
6. The SDK layer rejects kImage1 stream start / stop / config query / callback2
   registration on single-camera devices, returning kNotSupported.
7. Changed the default device from Odin1Lite to Odin2Special, falling back to
   Odin2Special when no builtin device matches.

3.4.0_20260827
1. Added save_map / load_map loop-closure map save and load ROS services.
2. Added device_mode device operating-mode switch ROS service.

3.3.0_20260827
1. Added h264 hardware decoder support on Jetson platform.
2. Added h264 hardware decoder support on Rockchip platform.

3.2.0_20260822
1. Fixed hardware decoding not supported on Ubuntu 20.04 with Rockchip platform.
2. Added troubleshooting description for cv_bridge / OpenCV version mismatch crash <troubleshooting.md>.
3. Added FOV filtering for cloud_raw with parameters enable_row_filter and min_row for row filtering.
4. Fixed the point-cloud-transport publish plugin.
5. Removed some parameters from launch.py and their dynamic configuration.
6. Added set_channel_config and get_channel_config ROS services.
7. Added set_post_process_config and get_post_process_config ROS services.
8. Removed control_command.yaml; parameters moved to the corresponding ROS launch file.
9. Added the opencv_undistort undistortion node.
10. Removed odom_lidar2cam and odom_lidar2imu publishing.
11. Added dynamic switches and configuration for some parameters and nodes.

3.1.0_20260813

[BREAKING CHANGES] Existing launch / config files must be updated for this release.
B1. Launch parameter renamed: operating_mode -> device_mode (aligned with the SDK's
    OdinDeviceMode rename). An operating_mode entry in a custom launch file will no
    longer take effect and must be renamed manually.
B2. Allowed values of image_format changed: yuyv / nv21 / rgb24 removed, h264 added.
    Supported values are now mjpeg | h264 | nv12.
B3. Default of sensor_mode changed from 1 (High Peak) to -1 (do not send; keep the
    device's current mode). If you rely on forcing High Peak at startup, pass
    sensor_mode:=1 explicitly in your launch file.
B4. Default frame_id values changed from a single "odom" to semantic names:
    raw_frame_id: odom -> lidar; gray_image_frame_id: odom -> lidar;
    image_frame_id: odom -> camera0; image2_frame_id: odom -> camera1;
    imu_frame_id: odom -> imu.
    Downstream nodes, rviz configs and TF trees that rely on the old frame ids must
    be updated accordingly.
B5. The SDK dependency is now a shared library instead of a static one. Ship
    libodin_sdk.so* together with the driver.

[NEW FEATURES: ROS SERVICES]
1. Added services to set / get the time synchronization mode
   (set_sync_mode / get_sync_mode).
2. Added services to set / get adaptive mode parameters
   (set_adaptive_param / get_adaptive_param).
3. Added a service to reset the on-module algorithm state (resetInitialPose).
4. Added a service to restore the device's default persisted configuration
   (restore_default_config).
5. Added a service to query PTP synchronization status (get_ptp_sync_status),
   including sync state, master-slave offset, path delay and frequency adjustment.
6. Added a service to toggle diagnostics (set_diag_monitor). It enables / disables
   the 1 Hz diagnostic output at runtime and returns per-sensor frame rates and
   average latencies (raw / SLAM point clouds, image0 / image1, imu,
   odom_lf / odom_hf, tf).

[NEW FEATURES: LAUNCH PARAMETERS]
7. Added device-side filter launch parameters: filter_confidence_threshold,
   filter_adaptive_cdf_percentile, filter_adaptive_outdoor_thr,
   filter_range_min_depth (-1 means "do not send"; the device default is kept).
   Note: adaptive parameters are sent through the dedicated SetAdaptiveParam API,
   while confidence / range filters go through SetFilterParam.
8. Added the tf_publish_rate launch parameter to configure the TF republish rate
   (0 disables timer-based republishing).

[NEW FEATURES: DATA AND DIAGNOSTICS]
9. Added H.264 decoding with BGR8 output for rviz.
10. Added configuration of the device-side LiDAR filters.
11. Added a service reporting SDK-side callback frame rates and latencies.
12. Added the diag_subscriber node, which measures ROS-side publish frame rates and
    ROS-side sensor data latency.
13. Added per-sensor-mode lower bounds for the point cloud confidence filter
    (HighPeak >= 7, LowPeak >= 2, Adaptive disables the client-side filter and lets
    the device handle it, HDR has no lower bound). The rqt slider's lower bound
    follows sensor_mode dynamically.
14. Added additional TF transform relations.

[BEHAVIOR CHANGES]
15. Changed image data frame_id (see B4).
16. BuildTopicPrefix now logs more device information (SN / model / IP / MAC), and
    the queried device sensor mode log was lowered to debug level to reduce noise.

[BUG FIXES]
17. Fixed launch file parameters not taking effect: adaptive filter parameters were
    incorrectly sent via SetFilterParam and now use the dedicated SetAdaptiveParam
    API.
18. Fixed TF data transform failures in rviz on Ubuntu 24.04.

1.0.0_20260720
1. Fixed build failures on Ubuntu 24.04.
2. Added depth-to-dense-point-cloud conversion.
3. Added Rockchip RK3588 MJPEG hardware decoding.
4. Removed the ESS depth node.
5. Added feature switches in the config file (control_command.yaml) for
   cloud_raw-to-depth (pointcloud_to_depth), depth-to-dense-cloud
   (depth_to_pointcloud), undistortion (undistort) and stereo rectification (rect).
6. Fixed the cloud_raw confidence filter launch default not taking effect.
7. Added DHCP configuration instructions to README.md.

0.12.0_20260601
1. Added the point-cloud-to-depth node.
2. Added topic_prefix configuration in the launch file.

0.11.0_20260518
1. Added reprojection; the projection mode is configured in control_command.yaml
   ("slam" projection mode is not supported yet).
2. Fixed rviz stuttering.
3. Added lidar2camera and lidar2imu odometry publishing.

V0.10.0_20260513
1. Added the map -> odom -> base_link TF tree.
2. Added publisher QoS depth configuration.
3. Added a QoS strategy for rosbag recording to avoid packet loss.

V0.9.0_20260426
1. Fixed the ROS 1 resolution setting not taking effect.
2. Fixed ROS 1 build failures.
3. Added data stream configuration in the config file (control_command.yaml).
4. Added compatibility with 24.04 / Jazzy builds.

v0.8.0_20260421
1. Added resolution configuration in the launch file.
2. Fixed image decoding blocking the callback.
3. Added stream control.
4. Added capability query.

V0.7.0_20260415
1. Added multi-resolution calibration file reading.
2. Added LiDAR adaptive mode and HDR mode.

V0.6.0_20260326
1. Fixed flicker caused by the SLAM cloud timestamp callback.

V0.5.0_20260311
1. Added ROS 1 / ROS 2 services to switch long / short frame mode.
2. Added Jetson hardware decoding for MJPEG images (requires supported hardware).
3. Optimized the undistortion algorithm.
4. Added rqt control of the SDK log level.

V0.4.0_20260307
1. Fixed missing timestamps in the ROS 1 / ROS 2 header of undistorted images.
2. Moved data callback registration into the camera connection step.
3. Added high-frequency odometry topic publishing.
4. Changed device discovery from active discovery to hotplug.
5. Added per-point offset_time for the raw point cloud (distinct from the frame
   timestamp).
6. Changed the ROS 1 / ROS 2 parameter namespace.

V0.3.1_20260206
1. Fixed a startup crash caused by different calibration files on O1Lite / O2S.

V0.3.0_20260205
1. Added stereo undistorted / rectified image output (topic: xxx/image_rect).
2. Added reading of the module calibration file.
3. Behavior change: previously, raw cloud points above a confidence threshold were
   filtered out; now those points are additionally set to (0, 0, 0).
4. Added stereo disparity point cloud (NVIDIA GPU only).
   ### Requires ISAAC ROS ESS hardware.

V0.2.0_20250130
1. Added stereo undistorted image output.
2. Added high / low light mode switching configuration.
3. Added confidence-based point cloud filtering configuration.

v0.1.0
Initial release.
