#include <cctype>
#include <fstream>
#include "DeviceLoader.h"
#include "DevicePluginEntry.h"

#include "logger.h"

#ifdef ODIN_HAS_DLOPEN
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#endif

namespace odin {
namespace sdk {

#ifdef ODIN_HAS_DLOPEN
namespace {

// ---------------------------------------------------------------------------
// Platform dynamic-loading layer: POSIX dlopen vs Windows LoadLibrary.
// Same split as algo/ialgo_loader.cpp.
// ---------------------------------------------------------------------------

#ifdef _WIN32

using ModuleHandle = HMODULE;

std::string DynError() {
  return "error " + std::to_string(static_cast<unsigned long>(GetLastError()));
}

ModuleHandle DynOpen(const std::string& path) {
  return LoadLibraryA(path.c_str());
}

void* DynSym(ModuleHandle mod, const char* sym) {
  return reinterpret_cast<void*>(GetProcAddress(mod, sym));
}

void DynClose(ModuleHandle mod) {
  FreeLibrary(mod);
}

#else  // POSIX

using ModuleHandle = void*;

std::string DynError() {
  const char* raw = dlerror();
  return (raw != nullptr) ? std::string(raw) : std::string("unknown");
}

ModuleHandle DynOpen(const std::string& path) {
  return dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
}

void* DynSym(ModuleHandle mod, const char* sym) {
  return dlsym(mod, sym);
}

void DynClose(ModuleHandle mod) {
  dlclose(mod);
}

#endif

// Candidate plugin file names for a device name, per platform.
// MSVC produces <name>.dll; MinGW prepends "lib". POSIX is lib<name>.so.
std::vector<std::string> PluginFileNames(const std::string& name) {
#ifdef _WIN32
  return {"odin-device-" + name + ".dll",
          "libodin-device-" + name + ".dll",
          "Odin" + name + "Device.dll",
          "libOdin" + name + "Device.dll"};
#else
  return {"libodin-device-" + name + ".so",
          "libOdin" + name + "Device.so"};
#endif
}

// Every location LoadFromSo probes for a plugin, in order. Shared with
// IsKnownDevice so the discovery-time support check sees exactly the same
// candidates the loader would.
std::vector<std::string> BuildPluginSearchPaths(const std::string& name,
                                                const std::string& explicit_path) {
  std::vector<std::string> search_paths;

  if (!explicit_path.empty()) {
    search_paths.push_back(explicit_path);
  }

  const auto file_names = PluginFileNames(name);

  // Compiled-in default: lib/<arch>/ (next to libodin_sdk.so)
#ifdef ODIN_DEVICE_PLUGIN_DIR
  for (const auto& f : file_names) {
    search_paths.push_back(std::string(ODIN_DEVICE_PLUGIN_DIR) + "/" + f);
  }
#endif

  // Environment variable override (ODIN_DEVICE_LIB_PATH)
  const char* env_path = getenv("ODIN_DEVICE_LIB_PATH");
  if (env_path) {
    std::string env_dir(env_path);
    for (const auto& f : file_names) {
      search_paths.push_back(env_dir + "/" + f);
    }
    // Also try arch subdir under env path
#ifdef ODIN_TARGET_ARCH
    for (const auto& f : file_names) {
      search_paths.push_back(env_dir + "/" + ODIN_TARGET_ARCH + "/" + f);
    }
#endif
  }

#ifndef _WIN32
  // System-wide fallback paths (POSIX only; Windows relies on the standard
  // DLL search order via the bare names below)
  for (const auto& f : file_names) {
    search_paths.push_back("/usr/lib/odin-device/" + f);
    search_paths.push_back("/usr/local/lib/odin-device/" + f);
  }
#endif

  // Bare file name: ld.so search path on POSIX, standard DLL search order
  // (exe dir, PATH, ...) on Windows.
  for (const auto& f : file_names) {
    search_paths.push_back(f);
  }

  return search_paths;
}

// True when a plugin candidate exists on disk. Paths containing a directory
// separator are checked as-is; bare file names (which LoadFromSo leaves to
// the system loader's search path) are probed in the working directory, the
// loader env path (LD_LIBRARY_PATH on POSIX, PATH on Windows) and the
// standard system lib dirs. Existence only — the file is never dlopen'ed.
bool PluginFileExists(const std::string& path) {
  auto readable = [](const std::string& p) {
    std::ifstream f(p.c_str(), std::ios::binary);
    return f.good();
  };

  const bool bare_name =
      path.find('/') == std::string::npos && path.find('\\') == std::string::npos;
  if (!bare_name) return readable(path);

  if (readable(path)) return true;  // working directory

#ifdef _WIN32
  const char* env = getenv("PATH");
  const char delim = ';';
#else
  const char* env = getenv("LD_LIBRARY_PATH");
  const char delim = ':';
#endif
  if (env != nullptr) {
    const std::string dirs(env);
    size_t start = 0;
    while (start <= dirs.size()) {
      const size_t end = dirs.find(delim, start);
      const std::string dir = (end == std::string::npos)
          ? dirs.substr(start)
          : dirs.substr(start, end - start);
      if (!dir.empty() && readable(dir + "/" + path)) return true;
      if (end == std::string::npos) break;
      start = end + 1;
    }
  }

#ifndef _WIN32
  // Approximate the ld.so.cache default dirs (incl. Debian multiarch).
  static const char* kSystemLibDirs[] = {"/lib", "/usr/lib", "/usr/local/lib"};
  for (const char* dir : kSystemLibDirs) {
    if (readable(std::string(dir) + "/" + path)) return true;
  }
#ifdef ODIN_TARGET_ARCH
  const std::string multiarch = std::string(ODIN_TARGET_ARCH) + "-linux-gnu";
  if (readable("/lib/" + multiarch + "/" + path) ||
      readable("/usr/lib/" + multiarch + "/" + path)) {
    return true;
  }
#endif
#endif
  return false;
}

}  // namespace
#endif  // ODIN_HAS_DLOPEN

namespace {
// ASCII case-insensitive equality for device names. Operates on unsigned
// char so non-ASCII bytes (e.g. from a raw firmware model field) are safe
// for std::tolower.
bool NameEqualsIgnoreCase(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(a[i])) !=
        std::tolower(static_cast<unsigned char>(b[i]))) {
      return false;
    }
  }
  return true;
}
}  // namespace

DeviceLoader& DeviceLoader::Instance() {
  static DeviceLoader instance;
  return instance;
}

void DeviceLoader::RegisterBuiltin(const std::string& name, DeviceCreateFn factory) {
  std::lock_guard<std::mutex> lock(mutex_);
  builtin_registry_[name] = factory;
  LOG_INFO("DeviceLoader: registered builtin device '%s'\n", name.c_str());
}

bool DeviceLoader::IsKnownDevice(const std::string& name) {
  // An empty name resolves to the builtin fallback inside Load().
  if (name.empty()) return true;

  // 1. Builtin registry — same exact + case-insensitive matching as Load().
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (builtin_registry_.find(name) != builtin_registry_.end()) return true;
    for (auto& kv : builtin_registry_) {
      if (NameEqualsIgnoreCase(kv.first, name)) return true;
    }
  }

#ifdef ODIN_HAS_DLOPEN
  // 2. A plugin candidate exists on the search paths. Existence check only —
  //    the file is never opened, so this stays free of plugin side effects
  //    (static initializers, ABI quirks) and is safe during discovery.
  for (const auto& path : BuildPluginSearchPaths(name, "")) {
    if (PluginFileExists(path)) return true;
  }
#endif

  return false;
}

std::unique_ptr<IDevice> DeviceLoader::Load(const DeviceLoadRequest& req,
                                             OdinDeviceHandle handle,
                                             std::string* error) {
  std::string name = req.name;
  if (name.empty()) name = "Odin2Special";

  // 1. Try builtin registry
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = builtin_registry_.find(name);
    if (it != builtin_registry_.end()) {
      return it->second(handle);
    }
    // Also try case-insensitive match for "ODIN1LITE" vs "Odin1Lite"
    for (auto& kv : builtin_registry_) {
      if (NameEqualsIgnoreCase(kv.first, name)) return kv.second(handle);
    }
  }

  // 2. Try dlopen plugin for the requested name
#ifdef ODIN_HAS_DLOPEN
  {
    auto device = LoadFromSo(name, req.explicit_path, handle, error);
    if (device) return device;
  }
#endif

  // 3. Default fallback: builtin Odin1Lite. Always available (compiled into
  //    libodin_sdk.so), so device creation never hard-fails just because a
  //    plugin .so is missing. Caveat: single-camera device — kImage1 and
  //    RegisterCameraCallback2 are inoperative on this fallback.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = builtin_registry_.find("Odin1Lite");
    if (it != builtin_registry_.end()) {
      LOG_INFO("DeviceLoader: falling back to builtin Odin1Lite for '%s'\n",
               name.c_str());
      return it->second(handle);
    }
  }

  if (error && error->empty()) {
    *error = "Device '" + name + "' not found in builtin registry"
#ifdef ODIN_HAS_DLOPEN
             " or plugin search paths"
#else
             " and dlopen is disabled"
#endif
             ", and Odin1Lite fallback is unavailable";
  }
  return nullptr;
}

std::unique_ptr<IDevice> DeviceLoader::LoadFromSo(const std::string& name,
                                                    const std::string& explicit_path,
                                                    OdinDeviceHandle handle,
                                                    std::string* error) {
#ifdef ODIN_HAS_DLOPEN
  const std::vector<std::string> search_paths =
      BuildPluginSearchPaths(name, explicit_path);

  ModuleHandle dl_handle = nullptr;
  std::string loaded_path;

  for (const auto& path : search_paths) {
    dl_handle = DynOpen(path);
    if (dl_handle) {
      loaded_path = path;
      break;
    }
  }

  if (!dl_handle) {
    if (error) *error = std::string("Failed to load device plugin '") + name + "': " + DynError();
    return nullptr;
  }

  LOG_INFO("DeviceLoader: loaded plugin '%s' from %s\n", name.c_str(), loaded_path.c_str());

  // ABI version check — advisory only. A mismatch does not reject the plugin:
  // if the required entry points still resolve below, calls can succeed.
  // Missing symbols remain the hard gate (odin_device_create/destroy).
  using AbiVersionFn = uint32_t (*)();
  auto abi_fn = reinterpret_cast<AbiVersionFn>(DynSym(dl_handle, "odin_device_abi_version"));
  if (abi_fn) {
    uint32_t plugin_abi = abi_fn();
    if (plugin_abi != ODIN_DEVICE_ABI_VERSION) {
      LOG_WARN("DeviceLoader: plugin '%s' ABI version %u != host %u, "
               "attempting to load anyway\n",
               name.c_str(), plugin_abi, ODIN_DEVICE_ABI_VERSION);
    }
  }

  // Check supported models
  using SupportedModelsFn = const char* (*)();
  auto models_fn = reinterpret_cast<SupportedModelsFn>(DynSym(dl_handle, "odin_device_supported_models"));
  if (models_fn) {
    const char* models = models_fn();
    if (models) {
      std::string models_str(models);
      bool found = false;
      size_t start = 0;
      while (start < models_str.size()) {
        size_t end = models_str.find(';', start);
        std::string token = (end == std::string::npos)
            ? models_str.substr(start)
            : models_str.substr(start, end - start);
        if (token == name) { found = true; break; }
        if (end == std::string::npos) break;
        start = end + 1;
      }
      if (!found) {
        if (error) *error = "Device '" + name + "' not in plugin's supported models: " + models_str;
        DynClose(dl_handle);
        return nullptr;
      }
    }
  }

  // Optional: plugin release version (diagnostics only, absence is fine)
  using VersionFn = const char* (*)();
  auto version_fn = reinterpret_cast<VersionFn>(DynSym(dl_handle, "odin_device_version"));
  if (version_fn) {
    const char* v = version_fn();
    if (v) LOG_INFO("DeviceLoader: plugin '%s' version %s\n", name.c_str(), v);
  }

  // Create device instance
  using CreateFn = IDevice* (*)(OdinDeviceHandle);
  auto create_fn = reinterpret_cast<CreateFn>(DynSym(dl_handle, "odin_device_create"));
  if (!create_fn) {
    if (error) *error = std::string("Symbol odin_device_create not found: ") + DynError();
    DynClose(dl_handle);
    return nullptr;
  }

  IDevice* raw_ptr = create_fn(handle);
  if (!raw_ptr) {
    if (error) *error = "odin_device_create returned nullptr";
    DynClose(dl_handle);
    return nullptr;
  }

  // Note: we leak dl_handle here because closing it would unmap the code.
  // In a production system we'd keep a handle->dl_handle map for cleanup on disconnect.
  // For now, the plugin stays loaded for the process lifetime.

  // The plugin's odin_device_destroy is intentionally not used: on Linux/glibc
  // the host's operator delete resolves to the same heap as the plugin's
  // operator new, so default delete is safe and simpler. The virtual destructor
  // of IDevice ensures correct cleanup regardless of the concrete type.
  return std::unique_ptr<IDevice>(raw_ptr);
#else
  if (error) *error = "dlopen support not compiled in";
  return nullptr;
#endif
}

}  // namespace sdk
}  // namespace odin
