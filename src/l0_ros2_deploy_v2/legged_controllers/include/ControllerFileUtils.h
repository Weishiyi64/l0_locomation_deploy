#pragma once

#include <string>

namespace legged {

std::string trimCopy(const std::string& s);
bool isAbsolutePath(const std::string& path);
std::string joinPath(const std::string& base, const std::string& leaf);
std::string resolvePackagePath(const std::string& raw_path, const std::string& package_share_dir);

}  // namespace legged
