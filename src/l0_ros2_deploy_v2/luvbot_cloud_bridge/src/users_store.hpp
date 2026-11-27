#pragma once

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>

#include "feature_validation.hpp"

namespace cloud {

inline void saveUsers(const Params& params, const std::string& path,
                      Params (*validate)(const Params&)) {
  using nlohmann::json;
  const auto directory = path.substr(0, path.rfind('/'));
  for (std::size_t end = 1; end <= directory.size(); ++end)
  {
    if (end != directory.size() && directory[end] != '/') continue;
    if (::mkdir(directory.substr(0, end).c_str(), 0755) != 0 && errno != EEXIST)
      throw std::runtime_error("cannot create biometric database directory");
  }

  json database = {{"users", json::array()}};
  struct stat status{};
  if (::lstat(path.c_str(), &status) == 0)
  {
    if (!S_ISREG(status.st_mode)) throw std::runtime_error("biometric database must be a regular file");
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot read existing biometric database");
    database = json::parse(input);
    if (input.bad()) throw std::runtime_error("failed reading biometric database");
  }
  else if (errno != ENOENT)
    throw std::runtime_error("cannot inspect existing biometric database");

  if (!database.is_object() || !database.contains("users") || !database["users"].is_array())
    throw std::runtime_error("biometric database requires a users array; existing file preserved");
  auto& users = database["users"];
  std::unordered_set<std::string> ids;
  std::size_t index = users.size();
  for (std::size_t i = 0; i < users.size(); ++i)
  {
    if (!users[i].is_object()) throw std::runtime_error("invalid biometric database member");
    const auto stored = validate(users[i].get<Params>());
    const auto& id = required(stored, "memberId");
    if (!ids.insert(id).second) throw std::runtime_error("duplicate biometric database memberId");
    if (id == required(params, "memberId")) index = i;
    users[i] = stored;
  }
  if (index == users.size()) users.push_back(params);
  else users[index] = params;
  const std::string bytes = database.dump(2) + "\n";

  // Keep the previous complete database until the replacement has reached disk.
  std::string pattern = path + ".tmp.XXXXXX";
  std::vector<char> temporary(pattern.begin(), pattern.end());
  temporary.push_back('\0');
  int fd = ::mkstemp(temporary.data());  // Owner read/write only: contains biometric features.
  if (fd < 0) throw std::runtime_error("cannot create temporary biometric database");
  try
  {
    std::size_t offset = 0;
    while (offset < bytes.size())
    {
      const auto written = ::write(fd, bytes.data() + offset, bytes.size() - offset);
      if (written < 0 && errno == EINTR) continue;
      if (written <= 0) throw std::runtime_error("cannot write biometric database");
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(fd) != 0) throw std::runtime_error("cannot sync biometric database");
    const int result = ::close(fd);
    fd = -1;
    if (result != 0) throw std::runtime_error("cannot close biometric database");
    if (::rename(temporary.data(), path.c_str()) != 0)
      throw std::runtime_error("cannot replace biometric database");
  }
  catch (...)
  {
    if (fd >= 0) ::close(fd);
    ::unlink(temporary.data());
    throw;
  }
}

}  // namespace cloud
