#include "ControllerFileUtils.h"

#include <algorithm>
#include <cctype>

namespace legged {

std::string trimCopy(const std::string& s) {
  const auto first = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
  const auto last = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c); }).base();
  if (first >= last) return "";
  return std::string(first, last);
}

bool isAbsolutePath(const std::string& path) { return !path.empty() && path.front() == '/'; }

std::string joinPath(const std::string& base, const std::string& leaf) {
  if (base.empty()) return leaf;
  if (leaf.empty()) return base;
  if (base.back() == '/') return base + leaf;
  return base + "/" + leaf;
}

std::string resolvePackagePath(const std::string& raw_path, const std::string& package_share_dir) {
  if (raw_path.empty() || isAbsolutePath(raw_path)) return raw_path;
  return joinPath(package_share_dir, raw_path);
}

}  // namespace legged
