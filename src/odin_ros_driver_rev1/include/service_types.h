#pragma once

// =============================================================================
// Service Type Definitions
// =============================================================================
// This header consolidates all ROS service includes and type aliases.
// When adding new services:
//   1. Add the srv file to srv/
//   2. Add the include here
//   3. Add a type alias if desired
// No changes needed in odin_ros_driver_node.cpp includes.
// =============================================================================

#if defined(ODIN_ROS2)
// ROS2 service headers
#include "odin_ros_driver_rev1/srv/get_calibration.hpp"
#include "odin_ros_driver_rev1/srv/set_sensor_mode.hpp"
#include "odin_ros_driver_rev1/srv/reset_initial_pose.hpp"
#include "odin_ros_driver_rev1/srv/get_adaptive_param.hpp"
#include "odin_ros_driver_rev1/srv/set_adaptive_param.hpp"
#include "odin_ros_driver_rev1/srv/get_sync_mode.hpp"
#include "odin_ros_driver_rev1/srv/set_sync_mode.hpp"
#include "odin_ros_driver_rev1/srv/get_ptp_sync_status.hpp"
#include "odin_ros_driver_rev1/srv/restore_default_config.hpp"
#include "odin_ros_driver_rev1/srv/set_diag_monitor.hpp"
#include "odin_ros_driver_rev1/srv/set_channel_config.hpp"
#include "odin_ros_driver_rev1/srv/get_channel_config.hpp"
#include "odin_ros_driver_rev1/srv/set_post_process_config.hpp"
#include "odin_ros_driver_rev1/srv/get_post_process_config.hpp"
#include "odin_ros_driver_rev1/srv/save_map.hpp"
#include "odin_ros_driver_rev1/srv/load_map.hpp"
#include "odin_ros_driver_rev1/srv/exploy_map.hpp"
#include "odin_ros_driver_rev1/srv/device_mode.hpp"

namespace odin_driver_interfaces {
namespace srv {
using GetCalibration = odin_ros_driver_rev1::srv::GetCalibration;
using SetSensorMode = odin_ros_driver_rev1::srv::SetSensorMode;
using ResetInitialPose = odin_ros_driver_rev1::srv::ResetInitialPose;
using GetAdaptiveParam = odin_ros_driver_rev1::srv::GetAdaptiveParam;
using SetAdaptiveParam = odin_ros_driver_rev1::srv::SetAdaptiveParam;
using GetSyncMode = odin_ros_driver_rev1::srv::GetSyncMode;
using SetSyncMode = odin_ros_driver_rev1::srv::SetSyncMode;
using GetPtpSyncStatus = odin_ros_driver_rev1::srv::GetPtpSyncStatus;
using RestoreDefaultConfig = odin_ros_driver_rev1::srv::RestoreDefaultConfig;
using SetDiagMonitor = odin_ros_driver_rev1::srv::SetDiagMonitor;
using SetChannelConfig = odin_ros_driver_rev1::srv::SetChannelConfig;
using GetChannelConfig = odin_ros_driver_rev1::srv::GetChannelConfig;
using SetPostProcessConfig = odin_ros_driver_rev1::srv::SetPostProcessConfig;
using GetPostProcessConfig = odin_ros_driver_rev1::srv::GetPostProcessConfig;
using SaveMap = odin_ros_driver_rev1::srv::SaveMap;
using LoadMap = odin_ros_driver_rev1::srv::LoadMap;
using ExployMap = odin_ros_driver_rev1::srv::ExployMap;
using DeviceMode = odin_ros_driver_rev1::srv::DeviceMode;
}  // namespace srv
}  // namespace odin_driver_interfaces

#else
// ROS1 service headers
#include "odin_ros_driver_rev1/GetCalibration.h"
#include "odin_ros_driver_rev1/SetSensorMode.h"
#include "odin_ros_driver_rev1/ResetInitialPose.h"
#include "odin_ros_driver_rev1/GetAdaptiveParam.h"
#include "odin_ros_driver_rev1/SetAdaptiveParam.h"
#include "odin_ros_driver_rev1/GetSyncMode.h"
#include "odin_ros_driver_rev1/SetSyncMode.h"
#include "odin_ros_driver_rev1/GetPtpSyncStatus.h"
#include "odin_ros_driver_rev1/RestoreDefaultConfig.h"
#include "odin_ros_driver_rev1/SetDiagMonitor.h"
#include "odin_ros_driver_rev1/SetChannelConfig.h"
#include "odin_ros_driver_rev1/GetChannelConfig.h"
#include "odin_ros_driver_rev1/SetPostProcessConfig.h"
#include "odin_ros_driver_rev1/GetPostProcessConfig.h"
#include "odin_ros_driver_rev1/SaveMap.h"
#include "odin_ros_driver_rev1/LoadMap.h"
#include "odin_ros_driver_rev1/ExployMap.h"
#include "odin_ros_driver_rev1/DeviceMode.h"

namespace odin_driver_interfaces {
namespace srv {
using GetCalibration = odin_ros_driver_rev1::GetCalibration;
using SetSensorMode = odin_ros_driver_rev1::SetSensorMode;
using ResetInitialPose = odin_ros_driver_rev1::ResetInitialPose;
using GetAdaptiveParam = odin_ros_driver_rev1::GetAdaptiveParam;
using SetAdaptiveParam = odin_ros_driver_rev1::SetAdaptiveParam;
using GetSyncMode = odin_ros_driver_rev1::GetSyncMode;
using SetSyncMode = odin_ros_driver_rev1::SetSyncMode;
using GetPtpSyncStatus = odin_ros_driver_rev1::GetPtpSyncStatus;
using RestoreDefaultConfig = odin_ros_driver_rev1::RestoreDefaultConfig;
using SetDiagMonitor = odin_ros_driver_rev1::SetDiagMonitor;
using SetChannelConfig = odin_ros_driver_rev1::SetChannelConfig;
using GetChannelConfig = odin_ros_driver_rev1::GetChannelConfig;
using SetPostProcessConfig = odin_ros_driver_rev1::SetPostProcessConfig;
using GetPostProcessConfig = odin_ros_driver_rev1::GetPostProcessConfig;
using SaveMap = odin_ros_driver_rev1::SaveMap;
using LoadMap = odin_ros_driver_rev1::LoadMap;
using ExployMap = odin_ros_driver_rev1::ExployMap;
using DeviceMode = odin_ros_driver_rev1::DeviceMode;
}  // namespace srv
}  // namespace odin_driver_interfaces

#endif
