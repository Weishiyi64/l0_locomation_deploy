/**
 * @file iloopclosure_algo.h
 * @brief 回环检测领域接口 + 版本管控 + 工厂函数。
 *
 * 本文件是回环插件的【领域契约】，包含：
 *   - ODIN_ALGO_ABI_VERSION：回环领域 ABI 版本
 *   - AlgoPluginEntry + 6 个 extern "C" 工厂函数
 *   - ILoopClosureAlgo：回环领域接口（继承 IAlgo）
 *
 * 数据类型定义在 iloopclosure_def.h（本文件自动包含）。
 *
 * 版本链：
 *   插件 ABI 版本 → 对齐本文件的 ODIN_ALGO_ABI_VERSION
 *   本文件 → 保证兼容 IAlgo 基类（ialgo.h）
 *   宿主 → 只依赖 IAlgo 的稳定性 + 本文件的 ABI 版本校验
 *
 * 开发者用法：include loopclosure_plugin.h（已包含本文件）。
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "iloopclosure_def.h"

// ============================================================================
// 导出宏 + ABI 版本
// ============================================================================

/// 导出宏：插件编译时使用。
///   Linux/GCC   → visibility("default")（导出符号）
///   Windows/MSVC → 插件侧定义 ODIN_ALGO_PLUGIN 时 dllexport；
///                   宿主侧留空（宿主不链接插件导入库，运行期 GetProcAddress）
#ifndef ODIN_ALGO_EXPORT
#if defined(_WIN32)
#if defined(ODIN_ALGO_PLUGIN)
#define ODIN_ALGO_EXPORT __declspec(dllexport)
#else
#define ODIN_ALGO_EXPORT
#endif
#else
#define ODIN_ALGO_EXPORT __attribute__((visibility("default")))
#endif
#endif

/**
 * 回环领域 ABI 版本。插件 odin_algo_abi_version() 必须返回同源值。
 *
 * 版本规则：
 *   - 破坏性变更（ILoopClosureAlgo 虚函数表改动/枚举语义变化）→ MAJOR +1
 *   - 兼容性追加（新虚函数追加到末尾/新枚举值）→ MINOR +1
 *
 * 变更记录：
 *   1.0  初始版本（含 SetMode/GetMode/Push/Poll/SaveMap/LoadMap）
 *   2.0  位姿图（GBA）输出：ILoopClosureAlgo 虚函数表末尾追加
 *        SetOptimizationStatusCallback / SetFrameCallback /
 *        SetOptimizationResultCallback / SetMapCloudCallback
 *        （DEBUG_IMAGE 下另有 SetLoopMatchImagesCallback）。
 */
#define ODIN_ALGO_ABI_VERSION_MAJOR 2
#define ODIN_ALGO_ABI_VERSION_MINOR 1
#define ODIN_ALGO_ABI_VERSION \
  ((ODIN_ALGO_ABI_VERSION_MAJOR << 16) | ODIN_ALGO_ABI_VERSION_MINOR)

namespace odin {
namespace algo {

// ============================================================================
// 回环领域接口
// ============================================================================

/**
 * @brief 回环检测/重定位算法接口。
 *
 * 继承 IAlgo，扩展领域方法（模式控制 + 数据输入/输出 + 地图 I/O）。
 */
class ILoopClosureAlgo : public virtual IAlgo {
 public:
   /// 加载模型/资源。可重复调用。
  virtual AlgoStatus Init(const AlgoInitConfig* cfg) = 0;

  /// 完整复位：卸载资源（设备重连/驱动重启时使用）。
  virtual AlgoStatus Reset() = 0;

  // ---- 错误诊断 ----

  /// 最近一次失败的可读原因（人可读，用于日志/提示）。
  virtual AlgoStatus GetLastError(char* buf, size_t cap) const = 0;

  // ---- 接口查询 ----

  /**
   * @brief 按能力位查询领域接口指针（COM QueryInterface 风格）。
   *
   * 为什么必须用它而不是 cast：
   *   - 虚继承下 static_cast downcast 是 ill-formed；
   *   - dynamic_cast 依赖跨库 RTTI，脆弱；
   *   - 本方法在对象内部 upcast，编译器自动完成偏移调整。
   *
   * @param capability_bit AlgoCapabilityBit 中的某一位
   * @return 实现了该领域接口则返回指针；未实现返回 nullptr。
   */
  virtual void* QueryInterface(uint32_t capability_bit) = 0;

  // ---- 模式控制 ----

  /// 切换工作模式（kIdle ⇔ kBuild ⇔ kReloc ⇔ kReMap）。同模式切换为 no-op
  virtual AlgoStatus SetMode(AlgoMode mode) = 0;

  /// 查询当前模式（未 Init 时返回 kIdle）。
  virtual AlgoMode GetMode() const = 0;

  // ---- 数据输入 ----

  virtual AlgoStatus PushImage(const AlgoImageMsg* msg) = 0;
  virtual AlgoStatus PushCloud(const AlgoCloudMsg* msg) = 0;
  virtual AlgoStatus PushOdom(const AlgoOdomMsg* msg) = 0;
  virtual AlgoStatus Drain() = 0;

  // ---- 输出回调 ----

  using LoopMatchCb = void (*)(const AlgoLoopMatch* match, void* user);
  virtual void SetLoopMatchCallback(LoopMatchCb cb, void* user) = 0;

  using RelocAnchorCb = void (*)(const AlgoRelocAnchor* anchor, void* user);
  virtual void SetRelocAnchorCallback(RelocAnchorCb cb, void* user) = 0;

  virtual size_t PollLoopMatches(AlgoLoopMatch* out, size_t cap) = 0;
  virtual size_t PollRelocAnchors(AlgoRelocAnchor* out, size_t cap) = 0;

  // ---- 地图 I/O ----

  virtual LoopClosureStatus SaveMap(const char* path) = 0;
  virtual LoopClosureStatus LoadMap(const char* path) = 0;
  virtual LoopClosureStatus ExployMap(const char* path) = 0;

  using OptimizationStatusCb = void (*)(bool busy, void* user);
  virtual void SetOptimizationStatusCallback(OptimizationStatusCb cb, void* user) = 0;

  using FrameCb = void (*)(const AlgoFrameInfo* f, void* user);
  virtual void SetFrameCallback(FrameCb cb, void* user) = 0;

  using OptimizationResultCb = void (*)(const AlgoOptimizationResult* result,
                                        void* user);
  virtual void SetOptimizationResultCallback(OptimizationResultCb cb,
                                             void* user) = 0;

  using MapCloudCb = void (*)(const float* xyz, size_t npts, void* user);
  virtual void SetMapCloudCallback(MapCloudCb cb, void* user) = 0;
};

// ============================================================================
// 插件工厂函数（6 个 extern "C"）
// ============================================================================

/**
 * @brief 每个回环插件 .so 必须导出的 6 个 C 工厂函数类型。
 *
 * 实现要求：extern "C" + ODIN_ALGO_EXPORT；静态字符串；create 失败返 nullptr。
 * 宿主绝不 new/delete 插件对象。
 */
struct AlgoPluginEntry {
  using AbiVersionFn      = uint32_t (*)();
  using NameFn            = const char* (*)();
  using VersionFn         = const char* (*)();
  using CapabilityMaskFn  = uint32_t (*)();
  using CreateFn          = void* (*)(const AlgoInitConfig* cfg);
  using DestroyFn         = void (*)(void* algo);
};

extern "C" ODIN_ALGO_EXPORT uint32_t odin_algo_abi_version();
extern "C" ODIN_ALGO_EXPORT const char* odin_algo_name();
extern "C" ODIN_ALGO_EXPORT const char* odin_algo_version();
extern "C" ODIN_ALGO_EXPORT uint32_t odin_algo_capability_mask();
extern "C" ODIN_ALGO_EXPORT void* odin_algo_create(const AlgoInitConfig* cfg);
extern "C" ODIN_ALGO_EXPORT void odin_algo_destroy(void* algo);

}  // namespace algo
}  // namespace odin
