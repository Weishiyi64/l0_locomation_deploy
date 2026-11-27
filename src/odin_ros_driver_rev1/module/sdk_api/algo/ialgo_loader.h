/**
 * @file ialgo_loader.h
 * @brief 算法插件加载器（宿主侧框架，实现见同目录 ialgo_loader.cpp，
 *        编译进 SDK；插件开发者不需要本文件）
 *
 * ─────────────────────────────────────────────────────────────────────────
 * 职责
 * ─────────────────────────────────────────────────────────────────────────
 * 1. 插件发现：多级路径策略（显式路径 → 环境变量 → 目录扫描 → 裸名回退）；
 * 2. dlopen + dlsym 6 个工厂符号（缺一即失败，报告缺失符号名）；
 * 3. 加载时校验：ABI 版本 / 插件名 / 能力位图，不匹配直接拒绝并给出
 *    明确错误原因（取代现有 "missing symbol" 的含糊提示）；
 * 4. 进程级缓存：插件 .so 只加载一次，实例可多份（沿用现有 Loader
 *    magic static 策略）；
 * 5. 缺失降级：加载失败返回错误码 + 原因字符串，宿主业务照常运行
 *    （对应现有 LoaderState.load_error + IsLibraryAvailable 行为）。
 *
 * ─────────────────────────────────────────────────────────────────────────
 * 与现有 loopclosure_loader.cpp 的对照
 * ─────────────────────────────────────────────────────────────────────────
 *   现有实现                          本设计
 *   dlopen("libmanifold_lc.so")      LoadRequest.path 多级解析
 *   ResolveAllSymbols（17 个 mangled） dlsym 6 个 C 工厂符号
 *   QueryVersion（构造临时 Session）  odin_algo_version() 直接获取
 *   LoaderState 单例                  AlgoLoader 单例（同策略）
 *   LoopClosureSession 封装           std::unique_ptr<IAlgo> + 自定义删除器
 * ─────────────────────────────────────────────────────────────────────────
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "ialgo.h"
#include "iloopclosure_algo.h"  // AlgoPluginEntry + 工厂函数声明 + ABI 版本

namespace odin {
namespace algo {

/**
 * @brief 插件加载请求。
 */
struct AlgoLoadRequest {
  /// 显式插件路径（如 "/opt/odin/libloopclosure.so"）。
  /// 非空时优先使用，跳过其余搜索路径。
  const char* explicit_path = nullptr;

  /// 期望的插件名（如 "loopclosure"）。
  /// 为空表示"任意插件，按 required_capability 搜索第一个匹配"。
  const char* name = nullptr;

  /// 需要的领域接口位图（AlgoCapabilityBit 按位或）。
  /// 加载时校验：插件 capability_mask 必须包含全部所需位。
  uint32_t required_capability = 0;

  /// 算法初始化参数（透传给 odin_algo_create）。
  /// 多实例场景：每个设备一次 Load，各自携带不同的 cfg——
  /// output_dir 按设备隔离（日志/地图目录），instance_tag 区分日志。
  /// nullptr = 使用插件默认配置；指针仅在 Load 调用期间有效（内部同步消费）。
  const AlgoInitConfig* cfg = nullptr;
};

/**
 * @brief 插件实例删除器（配合 unique_ptr）。
 * 内部调用该插件导出的 odin_algo_destroy——宿主绝不 delete 插件对象。
 * 删除器用 std::function 而非裸函数指针：destroy 符号因插件而异，
 * 需要把对应插件的函数指针带进闭包（裸函数指针无状态做不到）。
 */
using AlgoPtr = std::unique_ptr<IAlgo, std::function<void(IAlgo*)>>;

/**
 * @brief 加载结果：实例 + 插件级元信息。
 *
 * IAlgo 虚接口不再提供 GetName/GetVersion 等实例级元信息查询
 * （见 ialgo.h 关键决策 6），宿主需要元信息打日志/管理时从这里拿——
 * 元信息来自工厂函数（create 前已校验过），宿主加载时自然持有。
 */
struct AlgoHandle {
  AlgoPtr  instance;               ///< 算法实例（自带插件删除器）
  std::string name;                ///< 插件名（来自 odin_algo_name）
  std::string version;             ///< 算法版本（来自 odin_algo_version）
  uint32_t capability_mask = 0;    ///< 能力位图（来自 odin_algo_capability_mask）
};

/**
 * @brief 算法插件加载器（进程级单例）。
 *
 * 使用示例：
 * @code
 *   odin::algo::AlgoLoadRequest req;
 *   req.name = "loopclosure";
 *   req.required_capability = odin::algo::kCapLoopClosure;
 *   // 多实例：每个设备各自一份 cfg（日志目录按 SN 隔离，见 AlgoInitConfig 要点 4）
 *   odin::algo::AlgoInitConfig cfg;
 *   cfg.output_dir   = algo_dir.c_str();  // 如 ".../log/<sn>/algo"（Load 期间有效）
 *   cfg.instance_tag = dev_sn.c_str();   // 日志文件名/行前缀兜底
 *   req.cfg = &cfg;
 *   std::string err;
 *   auto h = odin::algo::AlgoLoader::Load(req, &err);
 *   if (!h.instance) {
 *     // 库未安装/版本不匹配：记录 err，主流程降级运行
 *     return;
 *   }
 *   LOG("loaded %s v%s", h.name.c_str(), h.version.c_str());  // 元信息来自句柄
 *   // 领域接口必须 QueryInterface 获取（虚继承禁 static_cast downcast）
 *   auto* lc = static_cast<odin::algo::ILoopClosureAlgo*>(
 *       h.instance->QueryInterface(odin::algo::kCapLoopClosure));
 *   lc->SetMode(odin::algo::AlgoMode::kBuild);
 * @endcode
 */
class AlgoLoader {
 public:
  // -------------------------------------------------------------------------
  // 静态查询（进程级，加载状态）
  // -------------------------------------------------------------------------

  /// 指定插件名/路径的库是否可用（已加载且校验通过）。
  static bool IsAvailable(const char* name);

  /// 最近一次加载失败的原因（空串表示成功）。
  static const std::string& LastLoadError();

  // -------------------------------------------------------------------------
  // 加载入口
  // -------------------------------------------------------------------------

  /**
   * @brief 加载插件并创建算法实例。
   *
   * 执行流程（对应 ialgo.h 的加载时序）：
   *   1. 路径解析（见 ResolvePath 注释）；
   *   2. dlopen(RTLD_NOW | RTLD_LOCAL)；
   *   3. dlsym 6 个工厂符号（缺一即失败并记录符号名）；
   *   4. ABI 版本校验：odin_algo_abi_version() 必须等于
   *      ODIN_ALGO_ABI_VERSION，主版本不一致立即拒绝；
   *   5. 名称/能力校验：与 LoadRequest 比对；
   *   6. odin_algo_create(req.cfg) 创建实例（cfg 可为 nullptr，用默认配置），
   *      失败返回空句柄；
   *   7. 把实例与插件级元信息（名字/版本/能力位图）打包为 AlgoHandle
   *      返回（宿主需要元信息时不再向实例查询）。
   *
   * @param req   加载请求（插件名/能力/显式路径）。
   * @param error 输出：失败原因（可读字符串）。
   * @return 成功返回句柄（instance 非空）；失败返回空句柄且 error 非空。
   *         加载失败不影响进程运行——宿主降级即可。
   */
  static AlgoHandle Load(const AlgoLoadRequest& req, std::string* error);

 private:
  /**
   * @brief 插件 .so 路径解析（按优先级）：
   *   1. req.explicit_path（显式指定）；
   *   2. 环境变量 ODIN_ALGO_LIB_PATH（如 /opt/odin/libloopclosure.so，
   *      或目录 /opt/odin/algo/ 则在该目录下按插件名拼 lib<name>.so）；
   *   3. 标准插件目录扫描：/usr/lib/odin-algo/lib<name>.so；
   *   4. 回退裸名：lib<name>.so（依赖 ldconfig，对应现有裸名策略）。
   */
  static bool ResolvePath(const AlgoLoadRequest& req, std::string* path,
                          std::string* error);

  // 进程级缓存（magic static，线程安全，仅加载一次）：
  //   map<string plugin_name, shared_ptr<PluginHandle>> loaded_;
  // PluginHandle 持有 dlopen 句柄 + 6 个工厂函数指针 + 校验结果。
};

/**
 * @brief 构造带 ABI 主版本号的插件库裸名。
 *
 * 输出格式：lib<base_name>.so.<ABI_MAJOR>，例如：
 *   MakeAbiVersionedPluginPath("manifold_lc_sdk")
 *     -> "libmanifold_lc_sdk.so.1"
 *
 * 返回的是裸文件名（不含目录），交给 dlopen 后由动态链接器按
 * LD_LIBRARY_PATH / ld.so.cache / 系统库路径搜索。
 */
std::string MakeAbiVersionedPluginPath(const char* base_name);

}  // namespace algo
}  // namespace odin
