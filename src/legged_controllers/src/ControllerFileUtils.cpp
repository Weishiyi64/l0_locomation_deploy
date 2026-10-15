#include "legged_controllers/ControllerFileUtils.h"

#include <algorithm>
#include <cctype>
#include <fstream>

namespace legged {

std::string trimCopy(const std::string& s) {
  const auto first = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
  const auto last = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c); }).base();
  if (first >= last) return "";
  return std::string(first, last);
}

std::string toUpperCopy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
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

bool pathExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

}  // namespace legged
