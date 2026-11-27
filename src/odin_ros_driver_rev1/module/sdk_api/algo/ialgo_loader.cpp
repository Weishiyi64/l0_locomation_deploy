/**
 * @file  ialgo_loader.cpp
 * @brief AlgoLoader 实现：动态加载 / 符号解析 / 加载时校验 / 进程级缓存。
 *
 * 动态加载层按平台分流：POSIX 用 dlopen/dlsym，Windows 用
 * LoadLibraryA/GetProcAddress（见下方 ODIN_ALGO_WINDOWS 宏区）。
 * 编译进 SDK（见同目录 CMakeLists.txt）；插件开发者不需要本文件。
 * 设计契约见 ialgo_loader.h 与 docs/algo_plugin_design.md。
 */

#include "ialgo_loader.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <cstdio>
#include <map>
#include <mutex>
#include <utility>

namespace odin {
namespace algo {
namespace {

// ---------------------------------------------------------------------------
// 平台动态加载层：POSIX dlopen 系 vs Windows LoadLibrary 系。
// 统一错误信息格式，让上层日志两个平台一致。
// ---------------------------------------------------------------------------

#ifdef _WIN32

using ModuleHandle = HMODULE;

// LoadLibrary 失败原因（线程级 GetLastError，读后即清）。
std::string DlError() {
  DWORD code = GetLastError();
  char buf[256];
  snprintf(buf, sizeof(buf), "error %lu", static_cast<unsigned long>(code));
  return buf;
}

ModuleHandle DlOpen(const std::string& path) {
  return LoadLibraryA(path.c_str());
}

void* DlSym(ModuleHandle mod, const char* sym) {
  return reinterpret_cast<void*>(GetProcAddress(mod, sym));
}

void DlClose(ModuleHandle mod) {
  FreeLibrary(mod);
}

#else  // POSIX

using ModuleHandle = void*;

std::string DlError() {
  const char* raw = dlerror();
  return (raw != nullptr) ? std::string(raw) : std::string("unknown");
}

ModuleHandle DlOpen(const std::string& path) {
  return dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
}

void* DlSym(ModuleHandle mod, const char* sym) {
  return dlsym(mod, sym);
}

void DlClose(ModuleHandle mod) {
  dlclose(mod);
}

#endif

// 插件文件名：lib<name>.so（POSIX）或 <name>.dll（Windows）。
#ifdef _WIN32
inline std::string PluginFileName(const std::string& plugin_name) {
  return plugin_name + ".dll";
}
#else
inline std::string PluginFileName(const std::string& plugin_name) {
  return "lib" + plugin_name + ".so";
}
#endif

// ---------------------------------------------------------------------------
// 进程级插件缓存（magic static，线程安全）
// ---------------------------------------------------------------------------

/** @brief 已加载插件的进程级描述（模块句柄 + 6 个工厂函数 + 元信息）。 */
struct PluginHandle {
  ModuleHandle dl_handle        = nullptr;  ///< 动态库句柄；进程级缓存持有，永不卸载
  std::string path;                      ///< 解析出的插件绝对/搜索路径（缓存 key）
  std::string name;                      ///< odin_algo_name()（加载时拷贝）
  std::string version;                   ///< odin_algo_version()（加载时拷贝）
  uint32_t    capability_mask = 0;       ///< odin_algo_capability_mask()

  // 6 个工厂函数指针（类型见 ialgo.h 的 AlgoPluginEntry）
  AlgoPluginEntry::AbiVersionFn     abi_version = nullptr;
  AlgoPluginEntry::NameFn           name_fn     = nullptr;
  AlgoPluginEntry::VersionFn        version_fn  = nullptr;
  AlgoPluginEntry::CapabilityMaskFn caps_fn     = nullptr;
  AlgoPluginEntry::CreateFn         create      = nullptr;
  AlgoPluginEntry::DestroyFn        destroy     = nullptr;
};

using PluginCache = std::map<std::string, std::shared_ptr<PluginHandle>>;

PluginCache& CacheMap() {
  static PluginCache cache;  // magic static：首次调用初始化，线程安全
  return cache;
}

std::mutex& CacheMutex() {
  static std::mutex mutex;
  return mutex;
}

std::string& LastErrorStorage() {
  static std::string last_error;
  return last_error;
}

/** @brief 统一失败出口：记录最近错误 + 填充调用方 error，返回空句柄。 */
AlgoHandle Fail(std::string msg, std::string* error) {
  if (error) *error = msg;
  LastErrorStorage() = std::move(msg);
  return AlgoHandle{};
}

/** @brief 取符号并转为函数指针。
 *  POSIX 规定 dlsym 结果须经 *(void**)&fn 中转；Windows 的
 *  GetProcAddress 直接可用（共用同一双重转换写法，两边均合法）。 */
template <typename FnPtr>
bool Sym(ModuleHandle dl, const char* sym, FnPtr* out, std::string* err) {
  void* raw = DlSym(dl, sym);
  if (raw == nullptr) {
    *err = std::string("missing symbol: ") + sym;
    return false;
  }
  *(reinterpret_cast<void**>(out)) = raw;
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// 静态查询
// ---------------------------------------------------------------------------

const std::string& AlgoLoader::LastLoadError() {
  std::lock_guard<std::mutex> lock(CacheMutex());
  return LastErrorStorage();
}

bool AlgoLoader::IsAvailable(const char* name) {
  if (name == nullptr) return false;
  std::lock_guard<std::mutex> lock(CacheMutex());
  for (const auto& kv : CacheMap()) {
    if (kv.second && kv.second->name == name) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// 路径解析
// ---------------------------------------------------------------------------

bool AlgoLoader::ResolvePath(const AlgoLoadRequest& req, std::string* path,
                             std::string* error) {
  // 1. 显式路径优先（跳过其余搜索）
  if (req.explicit_path != nullptr && *req.explicit_path != '\0') {
    *path = req.explicit_path;
    return true;
  }

  const std::string plugin_name =
      (req.name != nullptr && *req.name != '\0') ? req.name : "";

  if (plugin_name.empty()) {
    *error = "ResolvePath: no explicit_path and no plugin name to search";
    return false;
  }

  const std::string file_name = PluginFileName(plugin_name);

  // 2. 环境变量：文件或目录
  if (const char* env = getenv("ODIN_ALGO_LIB_PATH")) {
    const std::string env_path = env;
    if (!env_path.empty()) {
      // 目录则拼接插件文件名；文件则直接使用（不校验后缀，交动态加载器判定）
#ifdef _WIN32
      const char kPathSep = '\\';
#else
      const char kPathSep = '/';
#endif
      *path = (env_path.back() == '/' || env_path.back() == '\\')
                  ? env_path + file_name
                  : env_path;
      (void)kPathSep;
      return true;
    }
  }

  // 3. 裸名回退（与旧 loopclosure_loader.cpp 一致：
  //    POSIX 依赖 ld.so.cache 与标准库路径，deb 安装到 /usr/lib 后由
  //    ldconfig 收录；部署到非标准目录时由 LD_LIBRARY_PATH 或
  //    rpath 接管。Windows 依赖标准 DLL 搜索路径（exe 目录、PATH 等）。
  //    不在本框架内写死路径，避免锁死部署布局）。
  *path = file_name;

  // 注：路径存在性不在此预检——加载失败时的错误信息更完整
  //（含依赖缺失原因），把"文件是否存在"的判定交给动态加载器。
  (void)error;
  return true;
}

// ---------------------------------------------------------------------------
// 加载入口
// ---------------------------------------------------------------------------

AlgoHandle AlgoLoader::Load(const AlgoLoadRequest& req, std::string* error) {
  // ---- 1. 路径解析 ----
  std::string path;
  {
    std::string resolve_err;
    if (!ResolvePath(req, &path, &resolve_err)) {
      return Fail("plugin path resolve failed: " + resolve_err, error);
    }
  }

  // ---- 2. 进程级缓存查找（按解析路径为 key，同路径只 dlopen 一次）----
  std::shared_ptr<PluginHandle> ph;
  {
    std::lock_guard<std::mutex> lock(CacheMutex());
    auto it = CacheMap().find(path);
    if (it != CacheMap().end()) ph = it->second;
  }

  // ---- 3. 首次加载：动态加载 + 符号解析 + ABI 校验 ----
  if (!ph) {
    // 加载即解析全部符号（对应 POSIX 的 RTLD_NOW；
    // RTLD_LOCAL 语义：插件符号不污染全局符号表，多插件同名符号互不干扰。
    // Windows 下 LoadLibrary 的默认行为即满足这两点）。
    ModuleHandle dl = DlOpen(path);
    if (dl == nullptr) {
      return Fail("load(" + path + ") failed: " + DlError(), error);
    }

    auto fresh = std::make_shared<PluginHandle>();
    fresh->dl_handle = dl;
    fresh->path      = path;

    std::string sym_err;
    do {
      if (!Sym(dl, "odin_algo_abi_version", &fresh->abi_version, &sym_err)) break;
      if (!Sym(dl, "odin_algo_name", &fresh->name_fn, &sym_err)) break;
      if (!Sym(dl, "odin_algo_version", &fresh->version_fn, &sym_err)) break;
      if (!Sym(dl, "odin_algo_capability_mask", &fresh->caps_fn, &sym_err)) break;
      if (!Sym(dl, "odin_algo_create", &fresh->create, &sym_err)) break;
      if (!Sym(dl, "odin_algo_destroy", &fresh->destroy, &sym_err)) break;
    } while (false);
    if (!sym_err.empty()) {
      // 缺工厂符号：不是合法插件。不卸载无妨（进程缓存策略），
      // 但保持资源干净：这里的句柄无引用方，直接关闭。
      DlClose(dl);
      return Fail(sym_err + " (in " + path + ")", error);
    }

    // ABI 版本门禁：主版本必须一致；MINOR 兼容（宿主旧/插件新可运行）。
    const uint32_t abi = fresh->abi_version();
    const uint32_t host_abi = ODIN_ALGO_ABI_VERSION;
    if ((abi >> 16) != (host_abi >> 16)) {
      DlClose(dl);
      char buf[128];
      snprintf(buf, sizeof(buf),
               "ABI major mismatch: plugin v%u.%u, host requires v%u.%u (%s)",
               abi >> 16, abi & 0xFFFF, host_abi >> 16, host_abi & 0xFFFF,
               path.c_str());
      return Fail(buf, error);
    }

    // 元信息立即拷贝（工厂返回的是插件静态字符串，拷贝后与句柄解耦）
    fresh->name            = fresh->name_fn();
    fresh->version         = fresh->version_fn();
    fresh->capability_mask = fresh->caps_fn();

    {
      std::lock_guard<std::mutex> lock(CacheMutex());
      ph = CacheMap().emplace(path, std::move(fresh)).first->second;
    }
    // 注意：进程级缓存持有加载句柄且永不卸载（实例析构也不关闭）——
    // 插件代码（vtable/静态数据）生命周期 = 进程生命周期。
  }

  // ---- 4. 名称 / 能力校验（create 之前，失败即拒绝）----
  if (req.name != nullptr && *req.name != '\0' && ph->name != req.name) {
    return Fail("plugin name mismatch: requested '" + std::string(req.name) +
                    "', loaded '" + ph->name + "'", error);
  }
  if ((ph->capability_mask & req.required_capability) !=
      req.required_capability) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "capability mismatch: required 0x%x, plugin '%s' provides 0x%x",
             req.required_capability, ph->name.c_str(), ph->capability_mask);
    return Fail(buf, error);
  }

  // ---- 5. 创建实例 ----
  // req.cfg 仅在本函数同步执行期间有效（约定），create 内部同步消费/拷贝。
  void* obj = ph->create(req.cfg);
  if (obj == nullptr) {
    std::string msg = "odin_algo_create returned null for plugin '" + ph->name + "'";
    if (req.cfg != nullptr) {
      char cfg_info[256];
      snprintf(cfg_info, sizeof(cfg_info),
               "; AlgoInitConfig: struct_size=%u, device_type=%d, extension=%p",
               req.cfg->struct_size,
               static_cast<int>(req.cfg->device_type),
               static_cast<const void*>(req.cfg->extension));
      msg += cfg_info;
    } else {
      msg += "; AlgoInitConfig is null";
    }
    msg += ". Common causes: struct_size mismatch between SDK and plugin, "
           "unsupported device_type, invalid extension pointer, missing "
           "resources, or Init() failure. Check plugin-side log.";
    return Fail(msg, error);
  }
  // create 返回的已是插件内调整好的 IAlgo* 视角指针（见 ialgo.h
  // AlgoPluginEntry::CreateFn 注释），static_cast 仅恢复类型不改值。
  IAlgo* algo = static_cast<IAlgo*>(obj);

  // ---- 6. 打包句柄 ----
  AlgoHandle handle;
  handle.name            = ph->name;
  handle.version         = ph->version;
  handle.capability_mask = ph->capability_mask;
  // 删除器闭包携带本插件的 destroy 指针；宿主绝不直接 delete。
  AlgoPluginEntry::DestroyFn destroy = ph->destroy;
  handle.instance = AlgoPtr(algo, [destroy](IAlgo* p) {
    if (p != nullptr) destroy(p);
  });
  return handle;
}

// -----------------------------------------------------------------------------
// ABI 版本化插件路径辅助函数
// -----------------------------------------------------------------------------

std::string MakeAbiVersionedPluginPath(const char* base_name) {
#ifdef _WIN32
  // Windows DLLs carry no ABI suffix in the file name; the loader resolves
  // <base>.dll via the standard DLL search order.
  return std::string(base_name) + ".dll";
#else
  return std::string("lib") + base_name + ".so." +
         std::to_string(ODIN_ALGO_ABI_VERSION_MAJOR);
#endif
}

}  // namespace algo
}  // namespace odin
