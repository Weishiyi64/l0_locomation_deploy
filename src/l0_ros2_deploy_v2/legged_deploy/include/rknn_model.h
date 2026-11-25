#ifndef LEGGED_DEPLOY_RKNN_MODEL_H
#define LEGGED_DEPLOY_RKNN_MODEL_H

#include <rknn_api.h>

#include <cstddef>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

std::string rknnModelPath(const std::string& onnx_path);

class RknnModel {
 public:
  enum class InputMode { Copy, ResidentWbc };
  explicit RknnModel(const std::string& model_path, InputMode mode = InputMode::Copy);
  ~RknnModel();

  RknnModel(const RknnModel&) = delete;
  RknnModel& operator=(const RknnModel&) = delete;

  std::size_t inputCount() const;
  std::size_t inputElements() const;
  std::size_t inputElements(std::size_t index) const;
  int inputIndex(const std::string& name) const;
  const std::string& inputName(std::size_t index) const;
  std::vector<std::size_t> inputShape(std::size_t index) const;
  rknn_tensor_type inputType(std::size_t index) const;
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
  struct ResidentTensor {
    rknn_tensor_attr attr{};
    std::size_t elements{0}, width{0}, stride{0}, element_bytes{0};
    std::array<rknn_tensor_mem*, 2> memory{{nullptr, nullptr}};
  };
  void initializeResident();
  void releaseResident() noexcept;
  ResidentTensor residentTensor(const rknn_tensor_attr& logical, rknn_query_cmd query);
  void writeResident(ResidentTensor& tensor, int slot, const float* values, std::size_t count);
  bool bindResidentSlot(int slot);
  bool restoreResidentSlot(int slot);
  bool isObservation(std::size_t index) const;

  InputMode input_mode_;
  rknn_context context_ = 0;
  std::vector<rknn_tensor_attr> input_attrs_;
  std::vector<rknn_tensor_attr> output_attrs_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<std::vector<float>> output_buffers_;
  std::vector<rknn_output> outputs_;
  std::string last_error_;
  std::vector<ResidentTensor> resident_inputs_, resident_outputs_;
  int policy_index_{-1}, command_index_{-1}, resident_slot_{-1};
  std::array<bool, 2> staged_{{false, false}};
  bool resident_healthy_{true};
  mutable std::mutex resident_mutex_;
  ResidentStats resident_stats_;
};

#endif
