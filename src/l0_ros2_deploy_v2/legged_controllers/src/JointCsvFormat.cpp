#include "legged_controllers/JointCsvFormat.h"

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace legged {

std::string trimJointCsvWhitespace(std::string s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r')) s.erase(0, 1);
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) s.pop_back();
  return s;
}

std::vector<std::string> splitJointCsvLine(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line)
  {
    if (c == ',')
    {
      out.push_back(trimJointCsvWhitespace(cur));
      cur.clear();
    }
    else
      cur += c;
  }
  out.push_back(trimJointCsvWhitespace(cur));
  return out;
}

bool jointCsvPlaybackDurationFromFile(const std::string& path, double& out_duration, rclcpp::Logger logger) {
  out_duration = 0.0;
  std::ifstream in(path);
  if (!in)
  {
    RCLCPP_ERROR(logger, "JointCsvFormat: cannot open csv for duration: %s", path.c_str());
    return false;
  }

  std::string line;
  if (!std::getline(in, line))
  {
    RCLCPP_ERROR(logger, "JointCsvFormat: empty csv: %s", path.c_str());
    return false;
  }
  {
    std::istringstream iss(trimJointCsvWhitespace(line));
    std::string tag;
    iss >> tag;
    if (tag != "Frames:")
    {
      RCLCPP_ERROR(logger, "JointCsvFormat: expected 'Frames:' first line in %s", path.c_str());
      return false;
    }
    int declared = -1;
    iss >> declared;
    if (declared < 0)
    {
      RCLCPP_ERROR(logger, "JointCsvFormat: bad Frames count in %s", path.c_str());
      return false;
    }
  }

  if (!std::getline(in, line))
  {
    RCLCPP_ERROR(logger, "JointCsvFormat: missing Frame Time in %s", path.c_str());
    return false;
  }
  double frame_time = 0.0;
  {
    std::istringstream iss(trimJointCsvWhitespace(line));
    std::string a, b;
    iss >> a >> b;
    if (a != "Frame" || b != "Time:")
    {
      RCLCPP_ERROR(logger, "JointCsvFormat: expected 'Frame Time:' second line in %s", path.c_str());
      return false;
    }
    iss >> frame_time;
    if (frame_time <= 0.0)
    {
      RCLCPP_ERROR(logger, "JointCsvFormat: bad frame time in %s", path.c_str());
      return false;
    }
  }

  if (!std::getline(in, line))
  {
    RCLCPP_ERROR(logger, "JointCsvFormat: missing header in %s", path.c_str());
    return false;
  }
  const std::vector<std::string> header = splitJointCsvLine(trimJointCsvWhitespace(line));
  if (header.empty())
  {
    RCLCPP_ERROR(logger, "JointCsvFormat: empty header in %s", path.c_str());
    return false;
  }
  const size_t expected_cols = header.size();

  size_t data_rows = 0;
  while (std::getline(in, line))
  {
    line = trimJointCsvWhitespace(line);
    if (line.empty()) continue;
    if (line.rfind("KP:", 0) == 0 || line.rfind("KD:", 0) == 0) continue;
    const std::vector<std::string> cells = splitJointCsvLine(line);
    if (cells.size() != expected_cols) continue;
    for (const std::string& c : cells)
    {
      try
      { std::stod(c); } catch (...)
      {
        RCLCPP_ERROR(logger, "JointCsvFormat: bad number in data row in %s", path.c_str());
        return false;
      }
    }
    ++data_rows;
  }

  if (data_rows == 0)
  {
    RCLCPP_ERROR(logger, "JointCsvFormat: no data rows in %s", path.c_str());
    return false;
  }

  out_duration = static_cast<double>(data_rows) * frame_time;
  return true;
}

}  // namespace legged
