/**
 * @file iloopclosure_def.h
 * @brief 回环检测插件需要的所有数据类型定义。
 *
 * 本文件包含回环领域的数据类型（不含接口和工厂函数）：
 *   - 传感器消息：AlgoImageFormat/AlgoImageMsg/AlgoPoint/AlgoCloudMsg/AlgoOdomMsg
 *   - 工作模式：AlgoMode
 *   - 领域错误码：LoopClosureStatus
 *   - 输出类型：AlgoLoopMatch/AlgoRelocAnchor
 *   - 初始化扩展：LoopClosureInitExt
 *
 * 依赖：ialgo.h（获取 AlgoStatus/AlgoDeviceType/AlgoInitConfig 等通用类型）。
 *
 * 开发者用法：不需要直接 include 本文件，include 领域入口头文件即可
 * （如 loopclosure_plugin.h 已包含本文件）。
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "ialgo.h"

namespace odin {
namespace algo {

// ============================================================================
// 传感器消息
// ============================================================================

/// 图像像素格式
enum class AlgoImageFormat : int32_t {
  kGray8   = 0,
  kBgr8    = 1,
  kMjpeg   = 2,
  kNv12    = 3,
  kH264    = 4,
  kUnknown = 5,
};

/// 图像帧（单目）。stamp 单位：秒。
struct AlgoImageMsg {
  double           stamp     = 0.0;        ///< 时间戳（秒）
  int32_t          camera_id = 0;
  const uint8_t*   data      = nullptr;    ///< 仅在 PushImage() 调用期间有效
  size_t           size      = 0;          ///< 字节数；Mjpeg 必填，raw 可选
  int32_t          width     = 0;
  int32_t          height    = 0;
  int32_t          step      = 0;          ///< 行步长（0 = 紧凑排列）
  AlgoImageFormat  format    = AlgoImageFormat::kGray8;
};

/// 单个点。坐标/强度单位：米 / 无量纲；offset_time 单位：秒。
struct AlgoPoint {
  float x           = 0.f;
  float y           = 0.f;
  float z           = 0.f;
  float intensity   = 0.f;
  float offset_time = 0.f;  ///< 相对扫描起点的秒数
};

/// 点云帧。stamp 单位：秒。
struct AlgoCloudMsg {
  double          stamp            = 0.0;   ///< 时间戳（秒）
  int32_t         lidar_id         = 0;
  const AlgoPoint* points          = nullptr;  ///< 仅在 PushCloud() 调用期间有效
  size_t          count            = 0;
  bool            is_dense         = false;
  bool            already_deskewed = false;
};

/// 里程计帧。position 单位：米；orientation 为四元数 [w, x, y, z]。
struct AlgoOdomMsg {
  double stamp          = 0.0;              ///< 时间戳（秒）
  double position[3]    = {0.0, 0.0, 0.0};  ///< 米
  double orientation[4] = {1.0, 0.0, 0.0, 0.0};  ///< w, x, y, z
};

// ============================================================================
// 回环工作模式
// ============================================================================

/// 回环工作模式。
enum class AlgoMode : int32_t {
  kIdle  = 0,  ///< 空闲：丢弃所有输入
  kBuild = 1,  ///< 建图
  kReloc = 2,  ///< 重定位
  kReMap = 3,  ///< 重定位 + 增量建图：加载地图重定位的同时把新帧追加进地图
};

// ============================================================================
// 回环领域错误码
// ============================================================================

/**
 * @brief 回环领域专属状态码（错误码分层第二层）。
 *
 * kOk/kInvalidArg 与公共层 AlgoStatus 数值直通（0/1）；
 * kIoError/kMapLoadFailed 为领域扩展。
 * 宿主 switch 必须带 default 分支（插件升级后新增错误码兼容）。
 */
enum class LoopClosureStatus : int32_t {
  kOk            = 0,
  kInvalidArg    = 1,
  kIoError       = 4,
  kMapLoadFailed = 5,
  kInternal      = 6,
};

/// 将 LoopClosureStatus 解码为可读字符串，便于日志定位具体失败原因。
inline const char* LoopClosureStatusStr(LoopClosureStatus s) {
  switch (s) {
    case LoopClosureStatus::kOk:            return "kOk";
    case LoopClosureStatus::kInvalidArg:    return "kInvalidArg";
    case LoopClosureStatus::kIoError:       return "kIoError";
    case LoopClosureStatus::kMapLoadFailed: return "kMapLoadFailed";
    case LoopClosureStatus::kInternal:      return "kInternal";
    default:                                return "kUnknown";
  }
}

// ============================================================================
// 回环输出类型
// ============================================================================

/// 回环匹配结果。平移单位：米；R_corr 为行主序 3x3 旋转矩阵。
struct AlgoLoopMatch {
  int32_t source   = 0;
  int32_t sequence = -1;
  double  stamp    = 0.0;
  double  t_corr[3]  = {0.0, 0.0, 0.0};
  double  R_corr[9]  = {1,0,0, 0,1,0, 0,0,1};
  double  fitness    = 0.0;
};

/// 重定位锚点：IMU-world 位姿在 map-world 坐标系下的表达。
struct AlgoRelocAnchor {
  double stamp = 0.0;
  double t[3]  = {0.0, 0.0, 0.0};
  double R[9]  = {1,0,0, 0,1,0, 0,0,1};
};

// ============================================================================
// 回环初始化扩展配置
// ============================================================================

/**
 * @brief 回环专属初始化参数（通过 AlgoInitConfig::extension 传入）。
 */
struct LoopClosureInitExt {
  uint32_t struct_size = sizeof(LoopClosureInitExt);
  int32_t  cloud_pipeline   = 0;     ///< 0=Raw, 1=Slam
  bool     enable_lidar_loop = true;
  bool     enable_image_loop = true;
  const char* mask_path = nullptr;
  // v2.0 追加：IMU←LiDAR 外参覆盖（R 行主序 3x3、t 单位米；Init 期间有效，插件需拷贝）。
  // 非 nullptr（两者同时）时覆盖设备查表默认值。
  const double* R_il = nullptr;
  const double* t_il = nullptr;
};

// ============================================================================
// ============================================================================

struct AlgoPoseStamped {
  double  stamp    = 0.0;
  double  t[3] = {0.0, 0.0, 0.0};
  double  R[9] = {1,0,0, 0,1,0, 0,0,1};
};

struct AlgoFrameInfo {
  double  stamp     = 0.0;
  int32_t frame_id  = -1;
  /// IMU(body) 在世界系中的位姿（经位姿图矫正）：R 行主序 3x3。
  double  t_wi_adj[3] = {0.0, 0.0, 0.0};
  double  R_wi_adj[9] = {1,0,0, 0,1,0, 0,0,1};
  const float* xyz  = nullptr;
  size_t       npts = 0;
  bool     is_keyframe = false;
};

struct AlgoOptimizationResult {
  const AlgoPoseStamped* poses = nullptr;
  size_t                 pose_count = 0;
  const float*           cloud_xyz = nullptr;
  size_t                 cloud_npts = 0;
};

}  // namespace algo
}  // namespace odin
