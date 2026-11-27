# Changelog

This project follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the version numbers follow [Semantic Versioning](https://semver.org/).

V3.6.0_20261016
1. Exposed ReMap device mode: device_mode supports mode=5, switches host-side loop_closure to ReMap, and keeps firmware data-source handling consistent with SLAM.
2. Updated save_map / load_map documentation for loading an existing map and saving the incrementally extended ReMap map.
3. Fixed the stream frame-rate configuration failure.
4. Added warning logs for lost data frames.

V3.5.0_20260930

[BREAKING CHANGES] Upgrading to this version requires changes to existing integration code.
B1. Removed the is_keyframe field from OdinCameraPacket: keyframe detection moves
   to the consumer. For H.264, scan the Annex-B start codes and NAL unit types
   (IDR/SPS); every MJPEG frame is independently decodable.
B2. OdinOdomData covariance expanded from cov[18] to cov[36] (6x6, row-major over
   x/y/z/roll/pitch/yaw); struct size changes from 125 to 197 bytes. Generation-2
   firmware (cov[18]) fills only the first 18 position entries; the SDK zero-fills
   the orientation rows so users always see the full 6x6 layout.
B3. Device implementations are now plugins: Odin2Special is split out of
   libodin_sdk.so into a dlopen plugin libodin-device-Odin2Special.so (output to
   lib/<arch>/). Deployments must ship the plugin library alongside
   libodin_sdk.so. Odin1Lite is the built-in default fallback device.

[NEW FEATURES]
1. Added Windows 64-bit support: win64 library artifacts, MSVC build adaptation,
   and device plugin loading on Windows.
2. Added the device plugin loading framework (DeviceLoader): built-in device
   registration plus dlopen plugin search, IsKnownDevice support query, optional
   plugin version query, and plugin ABI mismatch downgraded to a warning.
3. Added odom generation-3 (V3) protocol support: V2/V3 structure compatibility,
   payload routing by the frame-header version byte, and UDP broadcast V2/V3
   compatibility.
4. Added the protocol_version field to DiscoveredDevice: identifies the device's
   protocol epoch from the DeviceQuery ACK, used by CreateOdinProtocol to pick
   the command dialect.
5. Added the kNotSupported return value to OdinResult (feature not supported by
   this device).
6. Added per-model camera count validation (GetCameraCount).
7. Added a device capability query (getcapabilities) to the StartStream flow.
8. Normalized legacy firmware model strings (ODIN) during discovery.
9. Added publishing of sensor messages to Foxglove on Windows.
10. Added eigen3 and openssl to 3rdparty.

[BEHAVIOR CHANGES]
11. The command layer is extracted from the device layer into a separate module;
    the protocol layer is internally refactored (OdinDataFrameView, the
    ChunkReassemblyState template, OdinPointDecoder, unified camera/point frame
    assembly). Public APIs are unchanged.

[BUG FIXES]
12. Fixed hotplug issues caused by a mismatch between the device online time and
    the module's sending IP when a NIC has multiple IPs.
13. Fixed a race between the heartbeat thread and the outer API.
14. Fixed an algorithm library load failure.
15. Fixed network cameras not being discoverable via broadcast on Windows.
16. Fixed the Windows setvbuf crash, shutdown failing to interrupt accept, and
    calib_thread not being joined.
17. Fixed the Foxglove download failure.


V3.4.0_20260827
1.Added mapping/relocalization modes: OdinDeviceMode gains kSlam (mapping) and
   KRelocation (relocalization) working with the host-side loop-closure algorithm.
   Added the SaveMap/LoadMap extended APIs. Relocalization anchors are published
   via the TF callback (frame_id=map, child_frame_id=odom).

V3.3.0_20260822
1. Removed kLidar2Camera, kLidar2Imu, kOdom2Map from the OdomSourceType enum.
2. Removed the GetImageEncoderParam extended API.
3. Updated GetChannelConfig comments.
4. Fixed a crash when the network camera disconnects.
5. Removed the SetImageEncoderParam extended API.
6. Removed the SendUserMessage extended API.

v3.2.0_20260813

[BREAKING CHANGES] Existing integration code must be updated for this release.
B1. Public header layering refactor: odin_lidar_api.h now contains the core API only;
    a new odin_lidar_api_ext.h provides the extended / tuning API. Likewise
    odin_lidar_def.h holds core type definitions and a new odin_lidar_def_ext.h
    holds extended type definitions.
    Users of basic functionality keep including odin_lidar_api.h. Users of advanced
    APIs (filters, image encoder, PTP sync, adaptive params, network config, ...)
    must switch to including odin_lidar_api_ext.h.
B2. Image callback renamed and merged: RegisterImageCallback / RegisterImageCallback2
    -> RegisterCameraCallback / RegisterCameraCallback2. Callback types
    OdinImageCallback / OdinImageCallback2 are merged into OdinCameraCallback, and
    the packet type OdinImagePacket is renamed to OdinCameraPacket.
    A single callback now delivers MJPEG / H.264 / NV12 and any other format; use
    the packet's `format` field to tell the codec apart.
B3. Device operating mode renamed: enum OdinOperatingMode -> OdinDeviceMode, and
    SetOperatingMode -> SetDeviceMode. A matching GetDeviceMode was added.
B4. SetSensorMode is now strongly typed: the mode argument changed from uint8_t to
    the new OdinSensorMode enum (kHDR=0 / kHighPeak=1 / kLowPeak=2 / kAdaptive=3).
    A matching GetSensorMode was added.
B5. IMU / Odom data structures reworked into typed packets: OdinImuPacket and
    OdinOdomPacket now carry frame_count (frame sequence), timestamp (frame time in
    ns) and a std::vector of samples. Protocol-level fields are no longer exposed.
B6. Removed the firmware_version field from the DiscoveredDevice struct. Use the
    GetFirmwareVersion API instead.
B7. Command IDs (cmd_id) were renumbered to align with protocol V2. A matching
    firmware version is required; older firmware is not compatible.
B8. GetNetworkAttribute / SetNetworkAttribute moved from the core API to the
    extended API (odin_lidar_api_ext.h).
B9. Supported image formats changed: yuyv / nv21 / rgb24 removed, h264 added.
B10. The SDK is now built as a shared library instead of a static one. Ship
    libodin_sdk.so* together with your application.

[NEW FEATURES]
1. Added SendUserMessage for application-defined downlink messages (requires
   firmware and algorithm support; non-standard feature).
2. Added SendAlgoData for algorithm data downlink (requires firmware and algorithm
   support; non-standard feature).
3. Added SetFilterParam / GetFilterParam for LiDAR point cloud filters.
4. Added the odomHf channel, separating high- and low-frequency odometry transport.
5. Added cross-subnet hotplug device discovery.
6. Added camera H.264 encoding capability reporting and output.
7. Added deb package generation for the SDK.
8. Added GetDefaultSyncMode to query the device time synchronization mode.
9. Added a mac address field to the DiscoveredDevice struct.
10. Added a warning log suggesting kernel receive-buffer tuning when UDP packet
    loss is detected.
11. Added GetChannelConfig to query the subscription state of a data stream channel.
12. Added SetAdaptiveParam / GetAdaptiveParam to read and write adaptive parameters
    used when sensorMode = adaptive.
13. Added ResetInitialPose to reset the on-module algorithm state.
14. Added RestoreDefaultConfig to restore the module's persisted configuration
    (NIC PTP state, IP, sync mode, ...).
15. Added TF transform data output: new OdinTfPacket struct, OdinTfCallback callback
    type and RegisterTfCallback registration API. Fields mirror ROS 2
    TransformStamped (frame_id / child_frame_id).
16. Added device telemetry output (channel 0x42, about 1 Hz): new OdinTelemetryPacket
    and OdinTelemetryData structs, OdinTelemetryCallback callback type, StartTelemetry
    and RegisterTelemetryCallback APIs. The SDK parses the device JSON internally and
    exposes structured fields (uptime, CPU / memory usage, system load).
17. Added GetPtpSyncStatus to query PTP synchronization status (sync state,
    master-slave offset, path delay, frequency adjustment).
18. Added SetImageEncoderParam / GetImageEncoderParam for image encoder tuning
    (JPEG quality, H.264 bitrate, H.264 QP min / max).
19. Added SetDefaultSyncMode to configure the device time synchronization mode.
20. Hotplug callback types (OnAttachCallback / OnDetachCallback / HotplugCallbacks)
    moved into the public header odin_lidar_def.h; users no longer need to include
    SDK-internal headers.

[BEHAVIOR CHANGES]
21. The default hotplug polling interval changed from 1000 ms to 500 ms
    (SetHotplugPollingInterval).

[BUG FIXES]
22. Fixed missing offline detection for network cameras that are discovered but not
    connected during hotplug events.
23. Fixed the OTA upgrade routine not returning after a successful upgrade.
24. Fixed a crash caused by network camera packet loss.
25. Fixed odom / SLAM transform synchronization: matching changed from keypoint
    matching to nearest-timestamp matching.
26. Fixed incomplete point counts caused by packet loss at the tail of a data frame
    (PadZerosForTailLoss).
27. Fixed intermittent failures of device discovery.

V2.0.3_20260518
1. Renamed the odom struct.
2. Added APIs to get and set the module's network attributes.
3. Added a device reboot API.

V2.0.2_20260513
1. Added the device map -> odom -> base_link TF tree.
2. Optimized IMU data transport.

V2.0.1_20260426
1. Added heartbeat detection (requires device support).
2. Reworked the network camera device mechanism to use heartbeats for offline
   detection.

V2.0.0_20260421
1. Added sensor capability query / configuration APIs.
2. Switched command transport from UDP to TCP.
3. Added image data transport channel configuration (UDP / TCP).
4. Optimized the transport protocol content.
5. Added StartStream / CloseStream flow-control APIs.

V1.0.2_20260326
1. Fixed flicker caused by the SLAM point cloud timestamp callback.

V1.0.1_20260311
1. Added image data validation to filter out incomplete images.
2. Enabled -O3 optimization and improved build speed.

V1.0.0_20260307
1. Fixed OTA failures.
2. Added network device hotplug support.
3. Added multi-instance support for long / short frame modes.
4. Added high-frequency odometry output.
5. Added the OdomSourceType odometry type enum.
6. Added multi-device support to the SLAM / odom synchronization API.
7. Added automatic data port adaptation (auto-discovery of free ports).
8. Added multi-NIC search to the device discovery API.

V0.3.0_20260205
1. Added calibration file retrieval.
2. Added an API to synchronize odometry and SLAM data output.

V0.2.0_20250130
1. Added ODIN2S stereo color image output.
2. Added OTA upgrade support.

V0.1.0_20251205
1. Initial release.
