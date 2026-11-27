#include "TextMotionOnnx.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>

namespace legged {
namespace motion {
namespace {
ov::Shape shape(const Json& spec) {
  require(spec.at("shape").is_array() && !spec.at("shape").empty() && spec.at("shape").size() <= 16,
          "Invalid ONNX text tensor rank");
  ov::Shape result;
  std::size_t count = 1;
  for (const auto& dim : spec.at("shape")) {
    const auto value = static_cast<std::size_t>(integer(dim, 1, 65536));
    require(count <= 16 * 1024 * 1024 / value, "ONNX text tensor too large");
    count *= value; result.push_back(value);
  }
  return result;
}
ov::element::Type type(const Json& spec) {
  const auto dtype = spec.at("dtype").get<std::string>();
  require(dtype == "int32" || dtype == "float32", "Unsupported ONNX text type");
  return dtype == "int32" ? ov::element::i32 : ov::element::f32;
}
void validate(const ov::Output<const ov::Node>& port, const Json& spec) {
  require(port.get_partial_shape().is_static() && port.get_shape() == shape(spec) &&
          port.get_element_type() == type(spec), "ONNX text shape/type differs from manifest");
}
}
TextOnnxSession::TextOnnxSession(std::shared_ptr<TextOnnxApi> api, const std::string& path, const Json& contract)
    : api_(std::move(api)) {
  auto model = api_->core.read_model(path);
  compiled_ = api_->core.compile_model(model, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
      ov::inference_num_threads(1), ov::hint::inference_precision(ov::element::f32));
  require(compiled_.inputs().size() == contract.at("inputs").size() &&
          compiled_.outputs().size() == contract.at("outputs").size(), "ONNX text I/O count mismatch");
  std::set<std::size_t> used;
  for (const auto& spec : contract.at("inputs")) {
    const auto name = spec.at("name").get<std::string>();
    std::size_t index = compiled_.inputs().size();
    for (std::size_t i = 0; i < compiled_.inputs().size(); ++i)
      if (compiled_.input(i).get_names().count(name)) index = i;
    require(index < compiled_.inputs().size() && used.insert(index).second, "ONNX text input name mismatch");
    validate(compiled_.input(index), spec);
    input_indices_.push_back(index); input_elements_.push_back(ov::shape_size(shape(spec))); input_types_.push_back(type(spec));
  }
  used.clear();
  for (std::size_t i = 0; i < compiled_.outputs().size(); ++i) {
    const auto port = compiled_.output(i);
    const Json* spec = nullptr;
    std::size_t index = 0;
    for (const auto& candidate : contract.at("outputs")) {
      if (port.get_names().count(candidate.at("name").get<std::string>())) {
        require(used.insert(index).second, "Duplicate ONNX text output"); spec = &candidate;
      }
      ++index;
    }
    require(spec != nullptr, "ONNX text output name mismatch"); validate(port, *spec);
    output_names_.push_back(spec->at("name").get<std::string>());
    buffers_.emplace_back(ov::shape_size(shape(*spec)), 0.0f);
  }
  request_ = compiled_.create_infer_request();
}
void TextOnnxSession::run(const std::vector<Input>& values, std::int64_t deadline, const std::function<void()>& checkpoint) {
  require(retired() && values.size() == input_indices_.size(), "ONNX text session busy or input count mismatch");
  checkpoint(); checkDeadline(deadline);
  for (std::size_t i = 0; i < values.size(); ++i) {
    const auto& value = values[i];
    require(value.data && value.elements == input_elements_[i] && value.type == input_types_[i], "Invalid ONNX text input");
    if (value.type == ov::element::f32) {
      const auto* data = static_cast<const float*>(value.data);
      require(std::all_of(data, data + value.elements, [](float x) { return std::isfinite(x); }), "Non-finite ONNX text input");
    }
    auto tensor = request_.get_input_tensor(input_indices_[i]);
    std::memcpy(tensor.data(), value.data, value.elements * value.type.size());
  }
  in_flight_.store(true);
  try { request_.infer(); } catch (...) { in_flight_.store(false); throw; }
  // Synchronous CPU inference has retired before checking cancellation/deadline.
  in_flight_.store(false);
  checkpoint(); checkDeadline(deadline);
  for (std::size_t i = 0; i < buffers_.size(); ++i) {
    const auto tensor = request_.get_output_tensor(i);
    require(tensor.get_size() == buffers_[i].size(), "ONNX text output size mismatch");
    if (tensor.get_element_type() == ov::element::f32)
      std::copy_n(tensor.data<const float>(), buffers_[i].size(), buffers_[i].begin());
    else {
      require(tensor.get_element_type() == ov::element::i32, "Unsupported ONNX text output type");
      const auto* data = tensor.data<const std::int32_t>();
      std::copy_n(data, buffers_[i].size(), buffers_[i].begin());
    }
    require(std::all_of(buffers_[i].begin(), buffers_[i].end(), [](float x) { return std::isfinite(x); }),
            "Non-finite ONNX text output");
  }
  checkpoint(); checkDeadline(deadline);
}
const std::vector<float>& TextOnnxSession::output(const std::string& name) const {
  const auto it = std::find(output_names_.begin(), output_names_.end(), name);
  require(it != output_names_.end(), "Unknown ONNX text output: " + name);
  return buffers_.at(static_cast<std::size_t>(it - output_names_.begin()));
}
}
}
