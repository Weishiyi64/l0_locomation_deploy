#include "openvino_wbc_model.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace {
int indexOf(const std::vector<std::string>& names, const std::string& name) {
  const auto it = std::find(names.begin(), names.end(), name);
  return it == names.end() ? -1 : static_cast<int>(it - names.begin());
}
void copyInput(ov::Tensor tensor, const float* data, std::size_t size) {
  if (!data || tensor.get_element_type() != ov::element::f32 || tensor.get_size() != size ||
      !std::all_of(data, data + size, [](float x) { return std::isfinite(x); }))
    throw std::runtime_error("OpenVINO WBC invalid input shape/type/value");
  std::copy(data, data + size, tensor.data<float>());
}
}
OpenVinoWbcModel::OpenVinoWbcModel(const std::string& path, InputMode mode) : mode_(mode) {
  if (mode == InputMode::AmpPreview) {
    // Match the AMP worker's CPU compilation settings at handoff.
    model_ = core_.compile_model(path, "CPU");
  } else {
    model_ = core_.compile_model(path, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                                ov::hint::inference_precision(ov::element::f32), ov::inference_num_threads(2));
  }
  request_ = model_.create_infer_request();
  for (const auto& port : model_.inputs()) {
    if (port.get_element_type() != ov::element::f32 || port.get_partial_shape().is_dynamic())
      throw std::runtime_error("OpenVINO WBC requires static FP32 inputs");
    input_names_.push_back(port.get_any_name());
  }
  for (const auto& port : model_.outputs()) {
    if (port.get_element_type() != ov::element::f32 || port.get_partial_shape().is_dynamic())
      throw std::runtime_error("OpenVINO WBC requires static FP32 outputs");
    output_names_.push_back(port.get_any_name());
    outputs_.emplace_back(ov::shape_size(port.get_shape()), 0.0f);
  }
  if (mode == InputMode::ResidentWbc && (inputCount() != 33 || inputIndex("policy") < 0 || inputIndex("command_short") < 0))
    throw std::runtime_error("OpenVINO WBC shared model ABI mismatch");
  for (auto& slot : parameters_) slot.resize(inputCount());
  stats_.context_initializations = 1;
}
OpenVinoWbcModel::~OpenVinoWbcModel() = default;
std::size_t OpenVinoWbcModel::inputCount() const { return input_names_.size(); }
std::size_t OpenVinoWbcModel::inputElements() const { return inputElements(0); }
std::size_t OpenVinoWbcModel::inputElements(std::size_t i) const { return ov::shape_size(model_.input(i).get_shape()); }
int OpenVinoWbcModel::inputIndex(const std::string& s) const { return indexOf(input_names_, s); }
const std::string& OpenVinoWbcModel::inputName(std::size_t i) const { return input_names_.at(i); }
std::vector<std::size_t> OpenVinoWbcModel::inputShape(std::size_t i) const { return model_.input(i).get_shape(); }
ov::element::Type OpenVinoWbcModel::inputType(std::size_t i) const { return model_.input(i).get_element_type(); }
std::string OpenVinoWbcModel::runtimeVersion() const { return "OpenVINO CPU (MuJoCo only)"; }
std::size_t OpenVinoWbcModel::outputElements(std::size_t i) const { return outputs_.at(i).size(); }
std::size_t OpenVinoWbcModel::outputCount() const { return outputs_.size(); }
int OpenVinoWbcModel::outputIndex(const std::string& s) const { return indexOf(output_names_, s); }
const std::string& OpenVinoWbcModel::outputName(std::size_t i) const { return output_names_.at(i); }
const std::vector<float>& OpenVinoWbcModel::output(std::size_t i) const { return outputs_.at(i); }
const std::string& OpenVinoWbcModel::lastError() const { return error_; }
bool OpenVinoWbcModel::run(const float* data, std::size_t size) { return run(std::vector<const float*>{data}, std::vector<std::size_t>{size}); }
bool OpenVinoWbcModel::run(const std::vector<const float*>& data, const std::vector<std::size_t>& sizes) {
  try {
    if (data.size() != inputCount() || sizes.size() != inputCount()) throw std::runtime_error("input count mismatch");
    for (std::size_t i = 0; i < data.size(); ++i) copyInput(request_.get_input_tensor(i), data[i], sizes[i]);
    request_.infer();
    for (std::size_t i = 0; i < outputs_.size(); ++i) {
      const auto* ptr = request_.get_output_tensor(i).data<const float>();
      if (!std::all_of(ptr, ptr + outputs_[i].size(), [](float x) { return std::isfinite(x); }))
        throw std::runtime_error("non-finite output");
      std::copy(ptr, ptr + outputs_[i].size(), outputs_[i].begin());
    }
    error_.clear();
    return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}
void OpenVinoWbcModel::stageResidentParameters(int slot, const std::vector<ParameterInput>& inputs) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != InputMode::ResidentWbc || slot < 0 || slot > 1 || slot == slot_ || inputs.size() != 31)
    throw std::runtime_error("invalid OpenVINO adapter stage");
  staged_[slot] = false;
  std::vector<bool> seen(inputCount(), false);
  for (const auto& p : inputs) {
    if (p.index >= inputCount() || seen[p.index] || inputName(p.index) == "policy" || inputName(p.index) == "command_short")
      throw std::runtime_error("duplicate/invalid adapter input");
    seen[p.index] = true;
    auto& tensor = parameters_[slot][p.index];
    if (!tensor) tensor = ov::Tensor(ov::element::f32, inputShape(p.index));
    copyInput(tensor, p.data, p.elements);
    stats_.parameter_write_bytes += p.elements * sizeof(float);
  }
  staged_[slot] = true;
  ++stats_.parameter_prepares;
}
bool OpenVinoWbcModel::commitResidentParameters(int slot) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (slot < 0 || slot > 1 || !staged_[slot]) throw std::runtime_error("adapter slot not staged");
    for (std::size_t i = 0; i < inputCount(); ++i)
      if (parameters_[slot][i]) request_.set_input_tensor(i, parameters_[slot][i]);
    slot_ = slot;
    ++stats_.commits;
    return true;
  } catch (const std::exception& e) { slot_ = -1; error_ = e.what(); return false; }
}
bool OpenVinoWbcModel::restoreResidentParameters(int slot) {
  if (slot == -1) { std::lock_guard<std::mutex> lock(mutex_); slot_ = -1; return true; }
  return commitResidentParameters(slot);
}
bool OpenVinoWbcModel::runResident(const float* policy, std::size_t np, const float* command, std::size_t nc) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (slot_ < 0) throw std::runtime_error("no committed adapter");
    copyInput(request_.get_input_tensor(inputIndex("policy")), policy, np);
    copyInput(request_.get_input_tensor(inputIndex("command_short")), command, nc);
    const auto start = std::chrono::steady_clock::now();
    request_.infer();
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    stats_.execution_ms += elapsed;
    stats_.execution_max_ms = std::max(stats_.execution_max_ms, elapsed);
    ++stats_.runs;
    for (std::size_t i = 0; i < outputs_.size(); ++i) {
      const auto* ptr = request_.get_output_tensor(i).data<const float>();
      if (!std::all_of(ptr, ptr + outputs_[i].size(), [](float x) { return std::isfinite(x); })) throw std::runtime_error("non-finite output");
      std::copy(ptr, ptr + outputs_[i].size(), outputs_[i].begin());
    }
    error_.clear();
    return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}
int OpenVinoWbcModel::residentSlot() const { std::lock_guard<std::mutex> lock(mutex_); return slot_; }
OpenVinoWbcModel::ResidentStats OpenVinoWbcModel::residentStats() const { std::lock_guard<std::mutex> lock(mutex_); return stats_; }
std::string OpenVinoWbcModel::residentLayout() const { return "CPU FP32 observations; two complete adapter slots; shared FP16-storage ONNX Base"; }
