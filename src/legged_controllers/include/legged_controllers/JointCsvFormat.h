#pragma once

#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

namespace legged {

/// Trim leading/trailing whitespace (joint CSV lines).
std::string trimJointCsvWhitespace(std::string s);

/// Split a comma-separated joint CSV row into cells.
std::vector<std::string> splitJointCsvLine(const std::string& line);

/// Total playback duration for a joint CSV (`Frames:` / `Frame Time:` / header + data rows).
bool jointCsvPlaybackDurationFromFile(const std::string& path, double& out_duration, rclcpp::Logger logger);

}  // namespace legged
