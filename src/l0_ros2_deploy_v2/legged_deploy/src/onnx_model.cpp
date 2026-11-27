#include "onnx_model.h"
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <set>
#include <map>
#include <openvino/op/parameter.hpp>
#include <stdexcept>

namespace {
std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}
int indexOf(const std::vector<std::string>& names, const std::string& name) {
  for (std::size_t i = 0; i < names.size(); ++i) if (lower(names[i]) == lower(name)) return static_cast<int>(i);
  return -1;
}
void require(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
bool finite(const float* p, std::size_t n) {
  return p && std::all_of(p, p + n, [](float x) { return std::isfinite(x); });
}
}
OnnxModel::OnnxModel(const std::string& path, InputMode mode) : mode_(mode) {
  auto model = core_.read_model(path);
  if (mode == InputMode::Planner) {
    // The delivered ONNX also maps velocities. C++ already owns that mapping and its interlocks.
    ov::ParameterVector parameters;
    for (const auto* name : {"observations", "hidden_state"}) {
      const auto parameter = std::dynamic_pointer_cast<ov::op::v0::Parameter>(model->input(name).get_node_shared_ptr());
      require(bool(parameter), "Planner input is not a parameter");
      parameters.push_back(parameter);
    }
    model = std::make_shared<ov::Model>(
        ov::OutputVector{model->output("actions"), model->output("next_hidden_state")}, parameters);
    model->reshape(std::map<std::string, ov::PartialShape>{{"observations", {1, 2631}}, {"hidden_state", {1, 1, 128}}});
  }
  require(!model->inputs().empty() && !model->outputs().empty(), "ONNX model has no inputs/outputs: " + path);
  ov::preprocess::PrePostProcessor ppp(model);
  for (std::size_t i = 0; i < model->inputs().size(); ++i) {
    const auto port = model->input(i);
    require(port.get_partial_shape().is_static() && port.get_element_type().is_real(), "ONNX input must be static floating point");
    ppp.input(i).tensor().set_element_type(ov::element::f32);
    ppp.input(i).preprocess().convert_element_type(port.get_element_type());
  }
  for (std::size_t i = 0; i < model->outputs().size(); ++i) {
    require(model->output(i).get_partial_shape().is_static() && model->output(i).get_element_type().is_real(),
            "ONNX output must be static floating point");
    ppp.output(i).tensor().set_element_type(ov::element::f32);
  }
  compiled_ = core_.compile_model(ppp.build(), "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                                  ov::inference_num_threads(1), ov::hint::inference_precision(ov::element::f32));
  request_ = compiled_.create_infer_request();
  for (const auto& port : compiled_.inputs()) input_names_.push_back(port.get_any_name());
  for (const auto& port : compiled_.outputs()) {
    output_names_.push_back(port.get_any_name());
    outputs_.emplace_back(ov::shape_size(port.get_shape()), 0.0f);
  }
  if (mode == InputMode::ResidentWbc) {
    policy_ = inputIndex("policy"); command_ = inputIndex("command_short");
    require(policy_ >= 0 && command_ >= 0 && inputCount() == 33, "Invalid resident ONNX WBC ABI");
    for (auto& slot : parameters_) {
      slot.resize(inputCount());
      for (std::size_t i = 0; i < inputCount(); ++i)
        if (static_cast<int>(i) != policy_ && static_cast<int>(i) != command_) {
          slot[i].resize(inputElements(i)); stats_.allocated_bytes += slot[i].size() * sizeof(float);
        }
    }
    stats_.context_initializations = 1;
  }
}
std::size_t OnnxModel::inputElements(std::size_t i) const { return ov::shape_size(inputShape(i)); }
int OnnxModel::inputIndex(const std::string& name) const { return indexOf(input_names_, name); }
int OnnxModel::outputIndex(const std::string& name) const { return name.empty() ? 0 : indexOf(output_names_, name); }
std::string OnnxModel::runtimeVersion() const { return std::string("OpenVINO CPU ") + ov::get_openvino_version().buildNumber; }
bool OnnxModel::execute(const std::vector<const float*>& data, const std::vector<std::size_t>& counts) {
  error_.clear();
  try {
    require(data.size() == inputCount() && counts.size() == inputCount(), "ONNX input count mismatch");
    for (std::size_t i = 0; i < data.size(); ++i) {
      require(counts[i] == inputElements(i) && finite(data[i], counts[i]), "Invalid ONNX input: " + inputName(i));
      // Copy into request-owned buffers: no request may retain a caller's temporary buffer.
      auto tensor = request_.get_input_tensor(i);
      std::copy(data[i], data[i] + counts[i], tensor.data<float>());
    }
    request_.infer();
    for (std::size_t i = 0; i < outputs_.size(); ++i) {
      const auto tensor = request_.get_output_tensor(i);
      require(tensor.get_size() == outputs_[i].size() && finite(tensor.data<const float>(), tensor.get_size()),
              "Invalid ONNX output: " + outputName(i));
    }
    for (std::size_t i = 0; i < outputs_.size(); ++i) {
      const auto tensor = request_.get_output_tensor(i);
      std::copy_n(tensor.data<const float>(), outputs_[i].size(), outputs_[i].begin());
    }
    return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}
bool OnnxModel::run(const float* data, std::size_t count) { return run(std::vector<const float*>{data}, std::vector<std::size_t>{count}); }
bool OnnxModel::run(const std::vector<const float*>& data, const std::vector<std::size_t>& counts) {
  if (mode_ == InputMode::ResidentWbc) { error_ = "Copy run forbidden on resident WBC model"; return false; }
  return execute(data, counts);
}
void OnnxModel::stageResidentParameters(int slot, const std::vector<ParameterInput>& parameters) {
  std::lock_guard<std::mutex> lock(mutex_);
  require(mode_ == InputMode::ResidentWbc && slot >= 0 && slot < 2 && slot != (slot_ < 0 ? 0 : slot_),
          "Cannot stage active/unavailable WBC slot");
  staged_[slot] = false;
  require(parameters.size() == inputCount() - 2, "Incomplete WBC parameters");
  std::set<std::size_t> seen;
  for (const auto& p : parameters) {
    require(p.index < inputCount() && static_cast<int>(p.index) != policy_ && static_cast<int>(p.index) != command_ &&
            seen.insert(p.index).second && p.elements == inputElements(p.index) && finite(p.data, p.elements),
            "Invalid/duplicate WBC parameter");
    std::copy_n(p.data, p.elements, parameters_[slot][p.index].begin());
    stats_.parameter_write_bytes += p.elements * sizeof(float);
  }
  staged_[slot] = true; ++stats_.parameter_prepares;
}
bool OnnxModel::commitResidentParameters(int slot) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != InputMode::ResidentWbc || slot < 0 || slot > 1 || slot == slot_ || !staged_[slot]) {
    error_ = "WBC commit requires staged spare slot"; return false;
  }
  slot_ = slot; ++stats_.commits; return true;
}
bool OnnxModel::restoreResidentParameters(int slot) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != InputMode::ResidentWbc || slot < -1 || slot > 1 || (slot >= 0 && !staged_[slot])) {
    error_ = "Invalid WBC rollback slot"; return false;
  }
  slot_ = slot; ++stats_.rollbacks; return true;
}
bool OnnxModel::runResident(const float* policy, std::size_t policy_size, const float* command, std::size_t command_size) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != InputMode::ResidentWbc || slot_ < 0) { error_ = "No committed WBC parameters"; return false; }
  std::vector<const float*> data(inputCount());
  std::vector<std::size_t> counts(inputCount());
  for (std::size_t i = 0; i < inputCount(); ++i) {
    data[i] = parameters_[slot_][i].data(); counts[i] = parameters_[slot_][i].size();
  }
  data[policy_] = policy; counts[policy_] = policy_size;
  data[command_] = command; counts[command_] = command_size;
  const auto start = std::chrono::steady_clock::now();
  const bool ok = execute(data, counts);
  const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  ++stats_.runs; stats_.execution_ms += ms; stats_.execution_max_ms = std::max(stats_.execution_max_ms, ms);
  return ok;
}
int OnnxModel::residentSlot() const { std::lock_guard<std::mutex> lock(mutex_); return slot_; }
OnnxModel::ResidentStats OnnxModel::residentStats() const { std::lock_guard<std::mutex> lock(mutex_); return stats_; }
