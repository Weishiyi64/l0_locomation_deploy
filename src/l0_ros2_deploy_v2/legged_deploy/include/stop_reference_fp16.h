#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "rknn_model.h"

class StopReferenceFp16 {
public:
  using Row = std::array<double, 28>;
  using Entry = std::array<Row, 3>;
  struct Plan {
    std::vector<Row> rows;
    int decoder_frames;
    double max_leg_projection;
    double max_joint_step;
  };

  StopReferenceFp16(const std::string& model, const std::string& profile,
                    const std::vector<std::string>& joint_names);
  Plan plan(Entry entry, bool stationary);
  const std::array<double, 21>& standingJoints() const { return stand_; }

private:
  RknnModel decoder_;
  std::array<double, 21> stand_{};
  std::array<float, 21> target_{};
  int first_, last_, requested_, motion_, length_;
};
