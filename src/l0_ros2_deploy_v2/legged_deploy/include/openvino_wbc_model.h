#pragma once
#include <openvino/openvino.hpp>
#include <array>
#include <mutex>
#include <string>
#include <vector>
class OpenVinoWbcModel {
 public:
  enum class InputMode { Copy, ResidentWbc, AmpPreview };
  explicit OpenVinoWbcModel(const std::string& model_path, InputMode mode = InputMode::Copy);
  ~OpenVinoWbcModel();

  OpenVinoWbcModel(const OpenVinoWbcModel&) = delete;
  OpenVinoWbcModel& operator=(const OpenVinoWbcModel&) = delete;

  std::size_t inputCount() const;
  std::size_t inputElements() const;
  std::size_t inputElements(std::size_t index) const;
  int inputIndex(const std::string& name) const;
  const std::string& inputName(std::size_t index) const;
  std::vector<std::size_t> inputShape(std::size_t index) const;
  ov::element::Type inputType(std::size_t index) const;
  std::string runtimeVersion() const;
  std::size_t outputElements(std::size_t index) const;
  std::size_t outputCount() const;
  int outputIndex(const std::string& name) const;
  const std::string& outputName(std::size_t index) const;
  const std::vector<float>& output(std::size_t index) const;
  const std::string& lastError() const;

  bool run(const float* input, std::size_t element_count);
  bool run(const std::vector<const float*>& inputs, const std::vector<std::size_t>& element_counts);

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
  // Caller keeps its prepare/inference lock for commit, validation and rollback.
  // stage does no SDK work; it can run in the loader after file/schema validation.
  void stageResidentParameters(int slot, const std::vector<ParameterInput>& parameters);
  bool commitResidentParameters(int slot);
  bool restoreResidentParameters(int slot);  // -1 restores the initial, inference-disabled binding.
  bool runResident(const float* policy, std::size_t policy_size,
                   const float* command, std::size_t command_size);
  int residentSlot() const;
  ResidentStats residentStats() const;
  std::string residentLayout() const;

 private:
  ov::Core core_;
  ov::CompiledModel model_;
  ov::InferRequest request_;
  std::vector<std::string> input_names_, output_names_;
  std::vector<std::vector<float>> outputs_;
  std::array<std::vector<ov::Tensor>, 2> parameters_;
  std::array<bool, 2> staged_{{false, false}};
  InputMode mode_;
  int slot_{-1};
  std::string error_;
  mutable std::mutex mutex_;
  ResidentStats stats_;
};
