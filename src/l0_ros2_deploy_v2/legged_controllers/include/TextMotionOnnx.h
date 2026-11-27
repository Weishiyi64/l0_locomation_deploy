#pragma once
#include "MotionData.h"
#include <openvino/openvino.hpp>
#include <atomic>
#include <functional>
#include <memory>

namespace legged {
namespace motion {
struct TextOnnxApi {
  ov::Core core;
  std::atomic<bool> uncertain{false};
};
class TextOnnxSession {
 public:
  struct Input { const void* data; std::size_t elements; ov::element::Type type; };
  TextOnnxSession(std::shared_ptr<TextOnnxApi> api, const std::string& path, const Json& contract);
  void run(const std::vector<Input>& values, std::int64_t deadline, const std::function<void()>& checkpoint);
  const std::vector<float>& output(const std::string& name) const;
  bool retired() const { return !in_flight_.load(); }
 private:
  std::shared_ptr<TextOnnxApi> api_;
  ov::CompiledModel compiled_;
  ov::InferRequest request_;
  std::vector<std::size_t> input_indices_, input_elements_;
  std::vector<ov::element::Type> input_types_;
  std::vector<std::string> output_names_;
  std::vector<std::vector<float>> buffers_;
  std::atomic<bool> in_flight_{false};
};
}
}
