#pragma once

#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace legged {
namespace motion {

using Json = nlohmann::json;
using Limits = std::vector<std::pair<double, double>>;
using Pose = std::array<float, 9>;
using Frames = std::vector<Pose>;
constexpr std::int64_t kGenerationNs = 3000000000LL;
constexpr std::size_t kJsonLimit = 4 * 1024 * 1024;
constexpr const char* kAction = "/control/motion/request";
constexpr const char* kTransaction = "/control/motion/transaction";
constexpr const char* kGeneration = "/txt2motion/generate";
constexpr const char* kFixedFrontend = "/wbc/frontend";

struct Cancelled : std::runtime_error { using std::runtime_error::runtime_error; };
void require(bool condition, const std::string& message);
std::int64_t nowMs();
std::int64_t steadyNs();
void checkDeadline(std::int64_t deadline);
std::int64_t integer(const Json& value, std::int64_t minimum, std::int64_t maximum);
double finiteNumber(const Json& value);
Json objectJson(const std::string& text);
std::string encode(const Json& value);
std::string readFile(const std::string& path, std::size_t maximum = kJsonLimit);
Json readJson(const std::string& path);
std::string parentPath(const std::string& path);
std::string assetPath(const std::string& root, const std::string& relative);
bool validId(const std::string& id);
std::string newId(const std::string& prefix);
void remember(std::deque<std::string>& ids, const std::string& id);
bool contains(const std::deque<std::string>& ids, const std::string& id);
std::vector<std::string> textCharacters(const std::string& text);
void validateText(const std::string& text);
std::uint32_t requestSeed(const Json& options);
std::int64_t requestDeadline(const Json& options, std::int64_t fallback);
Pose mirrorArms(const Pose& pose);

struct Assets {
  Json graph, actions = Json::object(), tracker;
  std::vector<std::string> names;
};
Assets loadAssets(const std::string& deployment);
Limits loadLimits(const std::string& mapping, const std::vector<std::string>& names);
Json checkSnapshot(Json snapshot, const std::vector<std::string>& names, const Limits& limits);
Json makeDocument(Json trajectory, const Json& snapshot, const Assets& assets);
std::string temporaryDirectory();
void saveDocument(const std::string& directory, const std::string& encoded, std::int64_t deadline);
void removeTemporaryDirectory(const std::string& directory);

}  // namespace motion
}  // namespace legged
