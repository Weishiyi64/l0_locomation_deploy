#pragma once
#include <algorithm>
#include <array>
#include <vector>

// The student predicts six arm corrections for each reference offset.
// Negative offsets use the first correction issued on earlier control steps.
struct WbcIlaHistory {
  std::array<std::array<float, 6>, 8> past{};
  void correct(std::vector<float>& command, const std::vector<float>& window,
               const std::vector<int>& offsets, int frame, int count) const {
    for (std::size_t i = 0; i < offsets.size(); ++i) {
      const int offset = std::max(0, std::min(frame + offsets[i], count - 1)) - frame;
      for (int j = 0; j < 6; ++j)
        command[72 + 21 * i + 12 + j] += offset < 0 ? past[8 + offset][j] : window[6 * offset + j];
    }
  }
  void advance(const std::vector<float>& window) {
    std::move(past.begin() + 1, past.end(), past.begin());
    std::copy_n(window.begin(), 6, past.back().begin());
  }
};
