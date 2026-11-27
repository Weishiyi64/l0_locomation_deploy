#pragma once

#include "MotionData.h"
#include <rknn_api.h>
#include <atomic>
#include <functional>
#include <memory>

namespace legged {
namespace motion {

// Process-local C API loader. The backend and fixed frontend never instantiate it.
class TextRknnApi {
 public:
  explicit TextRknnApi(const std::string& library);
  ~TextRknnApi();
  TextRknnApi(const TextRknnApi&) = delete;
  TextRknnApi& operator=(const TextRknnApi&) = delete;
  decltype(&rknn_init) init;
  decltype(&rknn_destroy) destroy;
  decltype(&rknn_query) query;
  decltype(&rknn_set_core_mask) set_core_mask;
  decltype(&rknn_inputs_set) inputs_set;
  decltype(&rknn_run) run;
  decltype(&rknn_wait) wait;
  decltype(&rknn_outputs_get) outputs_get;
  decltype(&rknn_outputs_release) outputs_release;
  std::atomic<bool> uncertain{false};
 private:
  void* library_{nullptr};
};

class TextRknnSession {
 public:
  struct Input { const void* data; std::size_t elements; rknn_tensor_type type; };
  TextRknnSession(std::shared_ptr<TextRknnApi> api, const std::string& path, const Json& contract);
  ~TextRknnSession();
  TextRknnSession(const TextRknnSession&) = delete;
  TextRknnSession& operator=(const TextRknnSession&) = delete;
  void run(const std::vector<Input>& values, std::int64_t deadline, const std::function<void()>& checkpoint);
  const std::vector<float>& output(const std::string& name) const;
  bool retired() const { return !in_flight_.load(); }
 private:
  std::shared_ptr<TextRknnApi> api_;
  rknn_context context_{0};
  std::vector<rknn_input> inputs_;
  std::vector<rknn_output> outputs_;
  std::vector<std::size_t> input_elements_, input_indices_;
  std::vector<rknn_tensor_type> input_types_;
  std::vector<std::string> output_names_;
  std::vector<std::vector<float>> buffers_;
  std::atomic<bool> in_flight_{false};
};

}  // namespace motion
}  // namespace legged
