#include "wbc_json_reference.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

// Board-side inspection of the exact parser/table consumed by RLWbc. No ROS
// initialization, model context, hardware transport or actuator publication.
int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "Usage: wbc_json_reference_probe JSON CONFIG MAPPING BASE_SHA256 OUTPUT\n";
    return 2;
  }
  try {
    std::ifstream source(argv[1], std::ios::binary);
    if (!source) throw std::runtime_error("cannot read JSON");
    const std::string payload{std::istreambuf_iterator<char>(source), std::istreambuf_iterator<char>()};
    const auto names = YAML::LoadFile(argv[2])["RobotCfg"]["joint_order_mj"].as<std::vector<std::string>>();
    const WbcJsonReference reference(payload, names, wbcJsonJointLimits(argv[3], names), argv[4]);
    std::ifstream existing(argv[5]);
    if (existing.good()) throw std::runtime_error("output already exists");
    std::ofstream out(argv[5]);
    if (!out) throw std::runtime_error("cannot write output");
    out << std::setprecision(17) << "{\"frame_count\":" << reference.size() << ",\"root_position\":[";
    for (std::size_t j = 0; j < 3; ++j) out << (j ? "," : "") << reference.rootPosition()[j];
    out << "],\"root_quaternion_wxyz\":[";
    for (std::size_t j = 0; j < 4; ++j) out << (j ? "," : "") << reference.rootOrientation()[j];
    out << "],\"joint_pos\":[";
    for (std::size_t i = 0; i < reference.size(); ++i) {
      out << (i ? ",[" : "[");
      const auto& row = reference.frame(static_cast<int>(i));
      for (std::size_t j = 0; j < 21; ++j) out << (j ? "," : "") << row[j];
      out << ']';
    }
    out << "]}\n";
    if (!out.good()) throw std::runtime_error("reference export failed");
    std::cout << "Validated reference: " << reference.size() << " frames; root + 21 joints; no actuator path\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
