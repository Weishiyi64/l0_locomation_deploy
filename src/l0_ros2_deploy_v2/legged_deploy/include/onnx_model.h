#pragma once
#include <openvino/openvino.hpp>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

// CPU execution of ONNX models; WBC parameters retain the same two-slot lifecycle.
class OnnxModel {
 public:
  enum class InputMode { Copy, ResidentWbc, Planner };
  explicit OnnxModel(const std::string& path, InputMode mode = InputMode::Copy);
  std::size_t inputCount() const { return input_names_.size(); }
  std::size_t outputCount() const { return outputs_.size(); }
  std::size_t inputElements() const { return inputElements(0); }
  std::size_t inputElements(std::size_t i) const;
  std::size_t outputElements(std::size_t i) const { return outputs_.at(i).size(); }
  std::vector<std::size_t> inputShape(std::size_t i) const { return compiled_.input(i).get_shape(); }
  std::vector<std::size_t> outputShape(std::size_t i) const { return compiled_.output(i).get_shape(); }
  int inputIndex(const std::string& name) const;
  int outputIndex(const std::string& name) const;
  const std::string& inputName(std::size_t i) const { return input_names_.at(i); }
  const std::string& outputName(std::size_t i) const { return output_names_.at(i); }
  const std::vector<float>& output(std::size_t i) const { return outputs_.at(i); }
  const std::string& lastError() const { return error_; }
  std::string runtimeVersion() const;
  bool run(const float* data, std::size_t count);
  bool run(const std::vector<const float*>& data, const std::vector<std::size_t>& counts);
  struct ParameterInput {
    std::size_t index;
    const float* data;
    std::size_t elements;
  };
  struct ResidentStats {
    std::uint64_t context_initializations{0}, allocated_bytes{0}, layout_bytes{0};
    std::uint64_t sdk_weight_bytes{0}, sdk_internal_bytes{0}, sdk_dma_bytes{0};
    int sdk_memory_query{0};
    std::uint64_t parameter_prepares{0}, parameter_write_bytes{0}, parameter_converted_elements{0};
    std::uint64_t parameter_bind_calls{0}, parameter_sync_calls{0}, commits{0}, rollbacks{0};
    std::uint64_t runs{0}, observation_sync_calls{0}, output_sync_calls{0};
    double pack_ms{0.0}, commit_ms{0.0};
    double observation_ms{0.0}, execution_ms{0.0}, output_ms{0.0};
    double observation_max_ms{0.0}, execution_max_ms{0.0}, output_max_ms{0.0};
  };
  void stageResidentParameters(int slot, const std::vector<ParameterInput>& parameters);
  bool commitResidentParameters(int slot);
  bool restoreResidentParameters(int slot);
  bool runResident(const float* policy, std::size_t policy_size, const float* command, std::size_t command_size);
  int residentSlot() const;
  ResidentStats residentStats() const;
 private:
  bool execute(const std::vector<const float*>& data, const std::vector<std::size_t>& counts);
  InputMode mode_;
  ov::Core core_;
  ov::CompiledModel compiled_;
  ov::InferRequest request_;
  std::vector<std::string> input_names_, output_names_;
  std::vector<std::vector<float>> outputs_;
  std::array<std::vector<std::vector<float>>, 2> parameters_;
  std::array<bool, 2> staged_{{false, false}};
  int slot_{-1}, policy_{-1}, command_{-1};
  mutable std::mutex mutex_;
  ResidentStats stats_;
  std::string error_;
};
