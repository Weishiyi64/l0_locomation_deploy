#pragma once

#include <openvino/openvino.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// Static FP32 ONNX backend for the existing WBC worker in x86 simulation.
// The worker retains observation ordering, deadlines and handoff validation.
class OpenVinoWbcModel {
 public:
  explicit OpenVinoWbcModel(const std::string& path) {
    ov::Core core;
    auto model = core.read_model(path);
    for (const auto& port : model->inputs()) validate(port);
    for (const auto& port : model->outputs()) validate(port);
    model_ = core.compile_model(model, "CPU", ov::inference_num_threads(2), ov::num_streams(1),
                                ov::hint::inference_precision(ov::element::f32));
    request_ = model_.create_infer_request();
    outputs_.resize(outputCount());
    for (std::size_t i = 0; i < outputCount(); ++i) outputs_[i].resize(outputElements(i));
  }
  std::size_t inputCount() const { return model_.inputs().size(); }
  std::size_t outputCount() const { return model_.outputs().size(); }
  std::size_t inputElements(std::size_t i = 0) const { return ov::shape_size(model_.input(i).get_shape()); }
  std::size_t outputElements(std::size_t i) const { return ov::shape_size(model_.output(i).get_shape()); }
  int inputIndex(const std::string& name) const {
    for (std::size_t i = 0; i < inputCount(); ++i)
      if (model_.input(i).get_names().count(name)) return static_cast<int>(i);
    return -1;
  }
  int outputIndex(const std::string& name) const {
    for (std::size_t i = 0; i < outputCount(); ++i)
      if (model_.output(i).get_names().count(name)) return static_cast<int>(i);
    return -1;
  }
  const std::vector<float>& output(std::size_t i) const { return outputs_.at(i); }
  const std::string& lastError() const { return error_; }
  bool run(const float* data, std::size_t count) { return run(std::vector<const float*>{data}, std::vector<std::size_t>{count}); }
  bool run(const std::vector<const float*>& data, const std::vector<std::size_t>& counts) {
    try {
      if (data.size() != inputCount() || counts.size() != inputCount())
        throw std::runtime_error("WBC OpenVINO input count mismatch");
      for (std::size_t i = 0; i < data.size(); ++i) {
        if (!data[i] || counts[i] != inputElements(i) ||
            !std::all_of(data[i], data[i] + counts[i], [](float v) { return std::isfinite(v); }))
          throw std::runtime_error("WBC OpenVINO invalid input shape/value");
        std::copy(data[i], data[i] + counts[i], request_.get_input_tensor(i).data<float>());
      }
      request_.infer();
      for (std::size_t i = 0; i < outputCount(); ++i) {
        auto tensor = request_.get_output_tensor(i);
        if (tensor.get_element_type() != ov::element::f32 || tensor.get_size() != outputs_[i].size())
          throw std::runtime_error("WBC OpenVINO output shape/type mismatch");
        const float* values = tensor.data<float>();
        if (!std::all_of(values, values + tensor.get_size(), [](float v) { return std::isfinite(v); }))
          throw std::runtime_error("WBC OpenVINO nonfinite output");
        std::copy(values, values + tensor.get_size(), outputs_[i].begin());
      }
      error_.clear();
      return true;
    } catch (const std::exception& exception) {
      error_ = exception.what();
      return false;
    }
  }
 private:
  template<class Port> static void validate(const Port& port) {
    if (port.get_partial_shape().is_dynamic() || port.get_element_type() != ov::element::f32)
      throw std::runtime_error("WBC OpenVINO requires static FP32 model inputs/outputs");
  }
  ov::CompiledModel model_;
  ov::InferRequest request_;
  std::vector<std::vector<float>> outputs_;
  std::string error_;
};
