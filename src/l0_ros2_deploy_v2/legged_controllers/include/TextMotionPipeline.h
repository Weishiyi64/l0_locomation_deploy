#pragma once

#include "MotionData.h"
#include <functional>
#include <memory>

namespace legged {
namespace motion {
class TextMotionPipeline {
 public:
  TextMotionPipeline(const std::string& model_root, const std::vector<std::string>& names, const Limits& limits);
  ~TextMotionPipeline();
  Json generate(const Json& request, const Json& snapshot, std::int64_t deadline, const std::function<void()>& checkpoint);
  bool retired() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace motion
}  // namespace legged
