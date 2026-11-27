/**
 * @file ialgo.h
 * @brief 算法插件纯虚基类 IAlgo + 通用类型（宿主与插件共享的契约头）。
 *
 * 本文件包含：
 *   - AlgoStatus / AlgoDeviceType / AlgoCapabilityBit：通用枚举
 *   - AlgoInitConfig：初始化配置（通用字段 + extension 扩展指针）
 *   - IAlgo：纯虚基类（零领域假设）
 *
 * 领域类型不在本文件：
 *   - 传感器消息 + 回环专属类型 → iloopclosure_def.h
 *   - 回环领域接口 + 工厂函数 → iloopclosure_algo.h
 *
 * 开发者用法：不需要直接 include 本文件，include 领域入口头文件即可
 * （如 loopclosure_plugin.h 已包含本文件）。
 *
 * 关键决策：
 *   1. IAlgo 只放公共能力，不塞 Push/Poll/Mode 系列接口：
 *      不同算法输入输出差异极大，全塞基类会让每个插件实现一堆无关接口。
 *   2. 领域接口用 virtual 继承 IAlgo：
 *      一个插件可同时实现多个领域接口，virtual 继承保证 IAlgo 子对象唯一。
 *   3. 跨插件边界只用 C ABI 类型（见 ialgo.h 内 AlgoInitConfig 等）：
 *      无 std::string/vector/function。回调用 C 函数指针 + void* user。
 *   4. 宿主不 new/delete 插件对象：创建/销毁走工厂函数。
 *   5. 元信息不进 IAlgo 虚接口，只由工厂函数导出：
 *      create 之前就需要（ABI 门禁/缓存 key），实例级方法无法满足。
 *   6. 领域接口获取必须走 QueryInterface，禁止 cast downcast：
 *      虚继承下 static_cast downcast 是 ill-formed；QueryInterface 在
 *      对象内部 upcast，编译器自动完成偏移调整，零 RTTI 依赖。
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace odin {
namespace algo {

// ============================================================================
// 通用枚举
// ============================================================================

/**
 * @brief 公共层通用状态码（只留几乎不变的通用错误）
 *
 * 错误码分层策略：
 *   1. 公共层（本枚举）：所有算法绕不开的通用错误，几乎永不更新；
 *   2. 领域层：各领域接口在自己的头文件定义专属错误码
 *      （如 ILoopClosureAlgo 的 LoopClosureStatus），频繁新增只动领域文件；
 *   3. 细节层：GetLastError() 字符串承载所有具体原因（人读诊断）。
 */
enum class AlgoStatus : int32_t {
  kOk             = 0,  ///< 成功
  kInvalidArg     = 1,  ///< 参数非法（宿主传参错误）
  kNotInitialized = 2,  ///< 未初始化（Init 前调用数据/业务接口）
  kNotImplemented = 3,  ///< 插件未实现该能力
  kInternal       = 4,  ///< 兜底：未分类的内部错误；领域细节见 GetLastError
};

/// 设备类型
enum class AlgoDeviceType : int32_t {
  kOdin1   = 0,
  kOdin1L  = 1,
  kOdin2S  = 2,
  kUnknown = 3,
};

/// 插件支持的领域接口位图（IAlgo 的派生接口每加一个占一位）。
/// 只由插件级工厂函数 odin_algo_capability_mask() 导出。
enum AlgoCapabilityBit : uint32_t {
  kCapLoopClosure = 1u << 0,  ///< 实现 ILoopClosureAlgo（回环/重定位）
  kCapUnknown     = 0xff,        ///< 未知/未指定
};

// ============================================================================
// 初始化配置
// ============================================================================

/**
 * @brief 算法初始化参数（通用字段 + 领域扩展）。
 *
 * 设计要点：
 *   1. struct_size 前置——插件先校验 struct_size <= 自己编译时的
 *      sizeof(AlgoInitConfig)，实现向前兼容（宿主新增字段时旧插件仍可用）；
 *   2. 字符串用 const char*：Init() 为同步调用，指针在 Init 返回前
 *      保证有效；插件如需长期持有必须自行拷贝；
 *   3. extension 指针：领域专属配置（如回环的 cloud_pipeline/enable_lidar_loop）
 *      放在领域扩展结构体里，通过 extension 传入。插件按 capability 自行
 *      static_cast 到对应扩展类型。通用结构体不膨胀领域字段。
 *   4. 路径所有权分层（多实例隔离）：
 *      - <log_root>/<sn>/algo/  → 宿主所有（实例维度隔离）
 *      - <algo_name>/           → 插件所有（算法维度隔离）
 *      - 文件名与子目录         → 插件所有
 *      硬约束：instance_tag 只出现在文件名与行前缀，禁止参与目录路径拼接。
 */
struct AlgoInitConfig {
  uint32_t        struct_size     = sizeof(AlgoInitConfig);  ///< ABI 兼容哨兵
  AlgoDeviceType  device_type     = AlgoDeviceType::kUnknown;

  const char* instance_tag      = nullptr;  ///< 实例标签（日志文件名/前缀兜底）
  const char* output_dir        = nullptr;  ///< 输出根目录（插件须拷贝后再用）
  const char* resource_dir      = nullptr;  ///< 资源根目录（config/slam/ 等）
  const char* calib_yaml        = nullptr;  ///< 相机内参数据（可选，YAML 或二进制）
  size_t      calib_yaml_size   = 0;        ///< 字节数；0=按 strlen 推断（兼容文本）
  int32_t     drain_interval_ms = 0;        ///< 自动 Drain 间隔（ms）；0=仅手动

  /**
   * 领域扩展指针。
   *
   * 宿主传入领域专属配置结构体指针（如 LoopClosureInitExt*），
   * 插件在 Init() 内按 capability 自行 static_cast。
   * nullptr = 无扩展配置，插件使用默认值。
   *
   * 扩展结构体约定：
   *   - 第一个字段必须是 uint32_t struct_size（与 AlgoInitConfig 同机制）；
   *   - 指针在 Init() 返回前有效，插件需长期持有则自行拷贝。
   *
   * 示例（回环插件）：
   * @code
   *   struct LoopClosureInitExt {
   *     uint32_t struct_size = sizeof(LoopClosureInitExt);
   *     int32_t  cloud_pipeline = 0;   // 0=Raw, 1=Slam
   *     bool     enable_lidar_loop = true;
   *     bool     enable_image_loop = true;
   *   };
   *   // 宿主侧：
   *   LoopClosureInitExt ext;
   *   cfg.extension = &ext;
   *   // 插件侧：
   *   auto* lc_ext = static_cast<const LoopClosureInitExt*>(cfg.extension);
   * @endcode
   */
  const void* extension = nullptr;
};

// ============================================================================
// 纯虚基类
// ============================================================================

/**
 * @brief 算法插件统一抽象基类（纯虚，零领域假设）。
 *
 * 线程约定：
 *   - 控制类（Init/Reset）由宿主外部串行化；
 *   - 数据类（Push/Drain）由宿主保证单线程；
 *   - 输出回调在 Drain() 或插件内部线程中触发，回调内禁止重入。
 */
class IAlgo {
 public:
  virtual ~IAlgo() = default;

  // ---- 生命周期 ----

  /// 加载模型/资源。可重复调用。
  /// @param cfg 指针仅本次调用期间有效。
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
};

}  // namespace algo
}  // namespace odin
