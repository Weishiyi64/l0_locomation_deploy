#pragma once

#include <array>

class FirstOrderLowPass3 {
 public:
  explicit FirstOrderLowPass3(double alpha = 0.8) : alpha_(alpha) {}

  void setAlpha(double alpha) {
    alpha_ = alpha;
    initialized_ = false;
  }

  const std::array<double, 3>& update(const std::array<double, 3>& input) {
    if (!initialized_)
    {
      value_ = input;
      initialized_ = true;
      return value_;
    }
    for (std::size_t i = 0; i < value_.size(); ++i) value_[i] = alpha_ * input[i] + (1.0 - alpha_) * value_[i];
    return value_;
  }

  void reset() { initialized_ = false; }

 private:
  double alpha_{0.8};
  bool initialized_{false};
  std::array<double, 3> value_{{0.0, 0.0, 0.0}};
};
