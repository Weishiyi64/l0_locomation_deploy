#pragma once
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <unordered_set>
#include <nlohmann/json.hpp>
#include "feature_validation.hpp"

namespace cloud {
namespace maps {
using Json = nlohmann::json;
constexpr std::size_t kMaxFileBytes = 16 * 1024 * 1024;

inline int slot(const std::string& index) {
  if (index == "1") return 1;
  if (index == "2") return 2;
  throw std::invalid_argument("params.mapIndex must be \"1\" or \"2\"");
}
inline std::string path(const std::string& cloud_id, const std::string& suffix = "") {
  if (cloud_id.empty() || cloud_id.size() > 128 ||
      cloud_id.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") != std::string::npos)
    throw std::invalid_argument("params.mapId must contain 1 to 128 ASCII letters, digits, hyphens or underscores");
  return "/userdata/forlinx/config/" + cloud_id + suffix + ".json";
}
inline struct stat snapshot(const std::string& file) {
  struct stat status{};
  if (::lstat(file.c_str(), &status) == 0) return status;
  if (errno == ENOENT) return {};
  throw std::runtime_error("cannot inspect map file");
}
inline bool sameFileVersion(const struct stat& left, const struct stat& right) {
#ifdef __APPLE__
  const auto left_mtime = left.st_mtimespec;
  const auto right_mtime = right.st_mtimespec;
#else
  const auto left_mtime = left.st_mtim;
  const auto right_mtime = right.st_mtim;
#endif
  return left.st_dev == right.st_dev && left.st_ino == right.st_ino &&
      left.st_size == right.st_size && left_mtime.tv_sec == right_mtime.tv_sec &&
      left_mtime.tv_nsec == right_mtime.tv_nsec;
}
inline std::string read(const std::string& file, const struct stat* previous = nullptr) {
  const auto status = snapshot(file);
  if (!S_ISREG(status.st_mode) ||
      status.st_size <= 0 || static_cast<uint64_t>(status.st_size) > kMaxFileBytes)
    throw std::runtime_error("map file missing, non-regular, empty or larger than 16 MiB");
  if (previous && sameFileVersion(status, *previous))
    throw std::runtime_error("map file was not updated for this task");
  std::ifstream input(file, std::ios::binary);
  if (!input) throw std::runtime_error("cannot read map file");
  std::string bytes(static_cast<std::size_t>(status.st_size), '\0');
  if (!input.read(&bytes[0], bytes.size()) || input.peek() != std::char_traits<char>::eof() ||
      !sameFileVersion(status, snapshot(file)))
    throw std::runtime_error("map file changed or read failed; producer must replace files atomically");
  return bytes;
}
inline void write(const std::string& file, const std::string& bytes) {
  const auto status = snapshot(file);
  if (status.st_mode && !S_ISREG(status.st_mode))
    throw std::runtime_error("map JSON destination is not a regular file");
  if (bytes.size() > kMaxFileBytes) throw std::runtime_error("map JSON exceeds 16 MiB");
  std::string pattern = file + ".tmp.XXXXXX";
  std::vector<char> temporary(pattern.begin(), pattern.end());
  temporary.push_back('\0');
  int fd = ::mkstemp(temporary.data());
  if (fd < 0) throw std::runtime_error("cannot create temporary map JSON");
  try {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto count = ::write(fd, bytes.data() + offset, bytes.size() - offset);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::runtime_error("cannot write map JSON");
      offset += static_cast<std::size_t>(count);
    }
    if (::fsync(fd) != 0) throw std::runtime_error("cannot sync map JSON");
    const int closed = ::close(fd);
    fd = -1;
    if (closed != 0) throw std::runtime_error("cannot close map JSON");
    if (::rename(temporary.data(), file.c_str()) != 0) throw std::runtime_error("cannot replace map JSON");
  } catch (...) {
    if (fd >= 0) ::close(fd);
    ::unlink(temporary.data());
    throw;
  }
}

inline std::vector<uint8_t> pngBytes(const std::string& text) {
  static const std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  if (text.empty() || text.size() % 4 != 0 || text.size() > kMaxFileBytes)
    throw std::invalid_argument("invalid PNG base64 length");
  std::vector<uint8_t> result;
  result.reserve(text.size() / 4 * 3);
  for (std::size_t i = 0; i < text.size(); i += 4) {
    unsigned v[4]{}, padding = 0;
    for (unsigned j = 0; j < 4; ++j) {
      if (text[i + j] == '=') {
        if (i + 4 != text.size() || j < 2) throw std::invalid_argument("invalid PNG base64 padding");
        ++padding;
      } else {
        const auto position = alphabet.find(text[i + j]);
        if (padding || position == std::string::npos) throw std::invalid_argument("invalid PNG base64 character");
        v[j] = static_cast<unsigned>(position);
      }
    }
    if ((padding == 1 && (v[2] & 3U)) || (padding == 2 && (v[1] & 15U)))
      throw std::invalid_argument("invalid PNG base64 padding bits");
    const uint32_t word = (v[0] << 18U) | (v[1] << 12U) | (v[2] << 6U) | v[3];
    result.push_back(static_cast<uint8_t>(word >> 16U));
    if (padding < 2) result.push_back(static_cast<uint8_t>(word >> 8U));
    if (!padding) result.push_back(static_cast<uint8_t>(word));
  }
  return result;
}
inline void finiteNumber(const Json& value) {
  if (!value.is_number() || !std::isfinite(value.get<double>())) throw std::invalid_argument("non-finite map coordinate");
}
inline void polygon(const Json& value) {
  if (!value.is_array() || value.size() < 3) throw std::invalid_argument("polygon requires at least three points");
  for (const auto& point : value) {
    if (point.is_array() && point.size() == 2) {
      finiteNumber(point[0]); finiteNumber(point[1]);
    } else if (point.is_object()) {
      finiteNumber(point.at("x")); finiteNumber(point.at("y"));
    } else throw std::invalid_argument("invalid polygon point");
  }
}
inline Json validate(const std::string& bytes, int map, const std::string& cloud_id) {
  auto document = Json::parse(bytes);
  const auto& data = document.at("payload");
  if (!document.at("cmd").is_string() ||
      data.at("mapId") != cloud_id ||
      !data.at("mapIndex").is_number_integer() || data.at("mapIndex") != map ||
      data.at("mapStatus") != "valid") throw std::invalid_argument("map identity/index/status mismatch");
  finiteNumber(data.at("resolution"));
  if (data.at("resolution").get<double>() <= 0) throw std::invalid_argument("map resolution must be positive");
  for (const auto* key : {"x", "y", "theta"}) finiteNumber(data.at("origin").at(key));
  for (const auto* key : {"x", "y", "yaw"}) finiteNumber(data.at("robotPose").at(key));
  for (const auto* key : {"pixelWidth", "pixelHeight"}) {
    if (!data.at(key).is_number_integer() || data.at(key) <= 0 || data.at(key) > 65535)
      throw std::invalid_argument("invalid map image dimensions");
  }
  const auto png = pngBytes(data.at("pngBinary").get<std::string>());
  const std::vector<uint8_t> signature{137, 80, 78, 71, 13, 10, 26, 10};
  auto word = [&png](std::size_t p) {
    return (uint32_t(png[p]) << 24U) | (uint32_t(png[p+1]) << 16U) | (uint32_t(png[p+2]) << 8U) | png[p+3];
  };
  if (png.size() < 45 || !std::equal(signature.begin(), signature.end(), png.begin()) || word(8) != 13 ||
      std::string(png.begin()+12, png.begin()+16) != "IHDR" ||
      word(16) != data.at("pixelWidth").get<uint32_t>() || word(20) != data.at("pixelHeight").get<uint32_t>() ||
      word(png.size()-12) != 0 || std::string(png.end()-8, png.end()-4) != "IEND")
    throw std::invalid_argument("PNG header/dimensions/end marker mismatch");
  const auto& rooms = data.at("roomList");
  if (!rooms.is_array()) throw std::invalid_argument("roomList must be an array");
  std::unordered_set<std::string> seen;
  for (const auto& room : rooms) {
    const auto room_id = room.at("roomId").get<std::string>();
    if (room_id.empty() || !seen.insert(room_id).second || !room.at("roomName").is_string())
      throw std::invalid_argument("invalid/duplicate roomId or roomName");
    polygon(room.at("polygonWorldPoints"));
  }
  return document;
}

inline void saveMetadata(const std::string& command, const Params& params, int map, bool fresh) {
  const auto& cloud_id = required(params, "mapId");
  const auto document = validate(read(path(cloud_id)), map, cloud_id);
  std::unordered_set<std::string> rooms;
  for (const auto& room : document.at("payload").at("roomList")) rooms.insert(room.at("roomId").get<std::string>());
  const bool rules = command == "map.zone.set";
  const std::string field = rules ? "rules" : "roomNameList";
  // The dot separates sidecars from map IDs, which cannot contain dots.
  const auto file = path(cloud_id, rules ? ".zone_rules" : ".room_names");
  Json database = {{"mapId", cloud_id}, {field, Json::array()}};
  struct stat status{};
  if (!fresh) {
    if (::lstat(file.c_str(), &status) == 0) database = Json::parse(read(file));
    else if (errno != ENOENT) throw std::runtime_error("cannot inspect previous map metadata");
  }
  if (!database.is_object() || database.at("mapId") != cloud_id || !database.at(field).is_array())
    throw std::invalid_argument("invalid existing map metadata; file preserved");
  Json updates;
  if (rules) {
    const auto points = Json::parse(required(params, "polygon"));
    polygon(points);
    updates = Json::array({{{"roomId", required(params, "roomId")}, {"zoneType", required(params, "zoneType")},
                            {"polygon", points}}});
  } else updates = Json::parse(required(params, "roomNameList"));
  if (!updates.is_array() || updates.empty()) throw std::invalid_argument("roomNameList requires a nonempty array");
  std::unordered_set<std::string> seen;
  for (const auto& update : updates) {
    const auto room = update.at("roomId").get<std::string>();
    if (!rooms.count(room) || !seen.insert(room).second) throw std::invalid_argument("unknown/duplicate roomId");
    if (!rules && (!update.at("roomName").is_string() || update.at("roomName").get<std::string>().empty()))
      throw std::invalid_argument("empty/invalid roomName");
    auto& entries = database[field];
    auto it = std::find_if(entries.begin(), entries.end(), [&](const Json& entry) {
      return entry.at("roomId") == room && (!rules || entry.at("zoneType") == update.at("zoneType"));
    });
    if (it == entries.end()) entries.push_back(update);
    else *it = update;
  }
  write(file, database.dump(2) + "\n");
}
}  // namespace maps
}  // namespace cloud
