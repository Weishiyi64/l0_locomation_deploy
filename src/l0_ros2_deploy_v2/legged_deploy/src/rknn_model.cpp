#include "rknn_model.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <set>

namespace {
std::string lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

void checkRknn(int ret, const std::string& operation) {
  if (ret != RKNN_SUCC) throw std::runtime_error(operation + " failed, ret=" + std::to_string(ret));
}

double millisecondsSince(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

std::string describe(const rknn_tensor_attr& attr) {
  std::ostringstream out;
  out << attr.name << "[" << attr.index << "] dims=";
  for (std::uint32_t i = 0; i < attr.n_dims && i < RKNN_MAX_DIMS; ++i) out << (i ? "x" : "") << attr.dims[i];
  out << " fmt=" << get_format_string(attr.fmt) << " type=" << get_type_string(attr.type)
      << " qnt=" << get_qnt_type_string(attr.qnt_type) << " zp=" << attr.zp << " scale=" << attr.scale
      << " elements=" << attr.n_elems
      << " size=" << attr.size << " stride_size=" << attr.size_with_stride
      << " w_stride=" << attr.w_stride << " h_stride=" << attr.h_stride;
  return out.str();
}

bool supportedResidentFloat(const rknn_tensor_attr& attr) {
  // RKNN can tag FP16 I/O as AFFINE. Rockchip's rknpu2 examples explicitly
  // exclude FP16 from their AFFINE quantization check. Native FP16 bytes still
  // represent IEEE half values: do not apply integer scale/zp or rewrite attrs.
  // Keep FP32/NONE support; integer, DFP and unknown encodings remain rejected.
  if (attr.type == RKNN_TENSOR_FLOAT16)
    return attr.qnt_type == RKNN_TENSOR_QNT_NONE || attr.qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC;
  return attr.type == RKNN_TENSOR_FLOAT32 && attr.qnt_type == RKNN_TENSOR_QNT_NONE;
}

// IEEE 754 binary16, round-to-nearest, ties-to-even, including subnormals.
// No ARM-specific casts or FPCR-dependent rounding in the parameter loader.
std::uint16_t toHalf(float value) {
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  const auto sign = static_cast<std::uint16_t>((bits >> 16U) & 0x8000U);
  const int exponent = static_cast<int>((bits >> 23U) & 255U) - 127;
  const std::uint32_t fraction = bits & 0x7fffffU;
  if (exponent < -25) return sign;
  const unsigned shift = exponent < -14 ? static_cast<unsigned>(-exponent - 1) : 13U;
  const std::uint32_t significand = exponent < -14 ? fraction | 0x800000U : fraction;
  std::uint32_t rounded = significand >> shift;
  const std::uint32_t remainder = significand & ((1U << shift) - 1U);
  const std::uint32_t halfway = 1U << (shift - 1U);
  if (remainder > halfway || (remainder == halfway && (rounded & 1U))) ++rounded;
  return static_cast<std::uint16_t>(sign | (exponent < -14 ? rounded :
      (static_cast<std::uint32_t>(exponent + 15) << 10U) + rounded));
}

float fromHalf(std::uint16_t bits) {
  const int exponent = (bits >> 10U) & 31U;
  if (exponent == 31) return std::numeric_limits<float>::quiet_NaN();
  const int fraction = bits & 1023U;
  const float value = std::ldexp(static_cast<float>(exponent ? 1024 + fraction : fraction),
                                exponent ? exponent - 25 : -24);
  return bits & 0x8000U ? -value : value;
}
}  // namespace

std::string rknnModelPath(const std::string& onnx_path) {
  std::string path = onnx_path;
  constexpr char extension[] = ".onnx";
  if (path.size() >= sizeof(extension) - 1 &&
      path.compare(path.size() - (sizeof(extension) - 1), sizeof(extension) - 1, extension) == 0)
    path.replace(path.size() - (sizeof(extension) - 1), sizeof(extension) - 1, ".rknn");
  else
    path += ".rknn";
  return path;
}

RknnModel::RknnModel(const std::string& model_path, InputMode mode) : input_mode_(mode) {
  std::ifstream file(model_path, std::ios::binary | std::ios::ate);
  if (!file.good()) throw std::runtime_error("RKNN model not found: " + model_path);

  const std::streamsize size = file.tellg();
  if (size <= 0 || static_cast<std::uint64_t>(size) > std::numeric_limits<std::uint32_t>::max())
    throw std::runtime_error("RKNN model has invalid size: " + model_path);

  std::vector<char> data(static_cast<std::size_t>(size));
  file.seekg(0, std::ios::beg);
  if (!file.read(data.data(), size)) throw std::runtime_error("Failed to read RKNN model: " + model_path);

  // These flags belong only to the persistent context. Copy-mode AMP/static/reference
  // contexts retain the SDK's original cache and inputs_set/outputs_get behavior.
  const std::uint32_t flags = mode == InputMode::ResidentWbc ?
      RKNN_FLAG_DISABLE_FLUSH_INPUT_MEM_CACHE | RKNN_FLAG_DISABLE_FLUSH_OUTPUT_MEM_CACHE : 0;
  checkRknn(rknn_init(&context_, data.data(), static_cast<std::uint32_t>(data.size()), flags, nullptr),
            "rknn_init(" + model_path + ")");

  try
  {
    rknn_input_output_num io_num{};
    checkRknn(rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)), "RKNN query I/O count");
    if (io_num.n_input == 0 || io_num.n_output == 0)
      throw std::runtime_error("RKNN models must have at least one input and one output");

    input_attrs_.resize(io_num.n_input);
    input_names_.reserve(io_num.n_input);
    for (std::uint32_t i = 0; i < io_num.n_input; ++i)
    {
      input_attrs_[i].index = i;
      checkRknn(rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input_attrs_[i], sizeof(input_attrs_[i])),
                "RKNN query input " + std::to_string(i));
      input_names_.emplace_back(input_attrs_[i].name);
    }

    output_names_.reserve(io_num.n_output);
    output_buffers_.reserve(io_num.n_output);
    outputs_.resize(io_num.n_output);
    for (std::uint32_t i = 0; i < io_num.n_output; ++i)
    {
      rknn_tensor_attr output_attr{};
      output_attr.index = i;
      checkRknn(rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &output_attr, sizeof(output_attr)),
                "RKNN query output " + std::to_string(i));
      output_names_.emplace_back(output_attr.name);
      output_attrs_.push_back(output_attr);
      output_buffers_.emplace_back(output_attr.n_elems, 0.0f);
      outputs_[i].index = i;
      outputs_[i].want_float = 1;
    }
    if (mode == InputMode::ResidentWbc) initializeResident();
  } catch (...)
  {
    releaseResident();
    rknn_destroy(context_);
    context_ = 0;
    throw;
  }
}

RknnModel::~RknnModel() {
  releaseResident();
  if (context_ != 0) rknn_destroy(context_);
}

std::size_t RknnModel::inputCount() const { return input_attrs_.size(); }

std::size_t RknnModel::inputElements() const { return inputElements(0); }

std::size_t RknnModel::inputElements(std::size_t index) const { return input_attrs_.at(index).n_elems; }

int RknnModel::inputIndex(const std::string& name) const {
  const std::string wanted = lowercase(name);
  for (std::size_t i = 0; i < input_names_.size(); ++i)
  {
    if (lowercase(input_names_[i]) == wanted) return static_cast<int>(i);
  }
  return -1;
}

const std::string& RknnModel::inputName(std::size_t index) const { return input_names_.at(index); }

std::vector<std::size_t> RknnModel::inputShape(std::size_t index) const {
  const auto& attr = input_attrs_.at(index);
  return std::vector<std::size_t>(attr.dims, attr.dims + attr.n_dims);
}

rknn_tensor_type RknnModel::inputType(std::size_t index) const { return input_attrs_.at(index).type; }

std::string RknnModel::runtimeVersion() const {
  rknn_sdk_version version{};
  checkRknn(rknn_query(context_, RKNN_QUERY_SDK_VERSION, &version, sizeof(version)), "RKNN SDK version");
  return std::string("runtime=") + version.api_version + ", driver=" + version.drv_version;
}

std::size_t RknnModel::outputElements(std::size_t index) const { return output_buffers_.at(index).size(); }

std::size_t RknnModel::outputCount() const { return output_buffers_.size(); }

int RknnModel::outputIndex(const std::string& name) const {
  if (name.empty()) return 0;
  const std::string wanted = lowercase(name);
  for (std::size_t i = 0; i < output_names_.size(); ++i)
  {
    if (lowercase(output_names_[i]) == wanted) return static_cast<int>(i);
  }
  return -1;
}

const std::string& RknnModel::outputName(std::size_t index) const { return output_names_.at(index); }

const std::vector<float>& RknnModel::output(std::size_t index) const { return output_buffers_.at(index); }

const std::string& RknnModel::lastError() const { return last_error_; }

bool RknnModel::run(const float* input, std::size_t element_count) {
  if (inputCount() != 1)
  {
    last_error_ = "single-input run called for a model with " + std::to_string(inputCount()) + " inputs";
    return false;
  }
  return run(std::vector<const float*>{input}, std::vector<std::size_t>{element_count});
}

bool RknnModel::run(const std::vector<const float*>& inputs, const std::vector<std::size_t>& element_counts) {
  last_error_.clear();
  if (input_mode_ != InputMode::Copy) {
    last_error_ = "inputs_set is forbidden on a resident context";
    return false;
  }
  if (inputs.size() != inputCount() || element_counts.size() != inputCount())
  {
    last_error_ =
        "input count mismatch: expected " + std::to_string(inputCount()) + ", got " + std::to_string(inputs.size());
    return false;
  }

  std::vector<rknn_input> rknn_inputs(inputCount());
  for (std::size_t i = 0; i < inputCount(); ++i)
  {
    if (inputs[i] == nullptr || element_counts[i] != inputElements(i))
    {
      last_error_ = "input '" + inputName(i) + "' element count mismatch: expected " +
                    std::to_string(inputElements(i)) + ", got " + std::to_string(element_counts[i]);
      return false;
    }
    rknn_inputs[i].index = static_cast<std::uint32_t>(i);
    rknn_inputs[i].buf = const_cast<float*>(inputs[i]);
    rknn_inputs[i].size = static_cast<std::uint32_t>(element_counts[i] * sizeof(float));
    rknn_inputs[i].type = RKNN_TENSOR_FLOAT32;
    rknn_inputs[i].fmt = input_attrs_[i].fmt;
  }
  int ret = rknn_inputs_set(context_, static_cast<std::uint32_t>(rknn_inputs.size()), rknn_inputs.data());
  if (ret != RKNN_SUCC)
  {
    last_error_ = "rknn_inputs_set failed, ret=" + std::to_string(ret);
    return false;
  }

  ret = rknn_run(context_, nullptr);
  if (ret != RKNN_SUCC)
  {
    last_error_ = "rknn_run failed, ret=" + std::to_string(ret);
    return false;
  }

  for (std::size_t i = 0; i < outputs_.size(); ++i)
  {
    outputs_[i].index = static_cast<std::uint32_t>(i);
    outputs_[i].want_float = 1;
    outputs_[i].buf = nullptr;
    outputs_[i].size = 0;
    outputs_[i].is_prealloc = 0;
  }
  ret = rknn_outputs_get(context_, static_cast<std::uint32_t>(outputs_.size()), outputs_.data(), nullptr);
  if (ret != RKNN_SUCC)
  {
    last_error_ = "rknn_outputs_get failed, ret=" + std::to_string(ret);
    return false;
  }

  bool valid = true;
  for (std::size_t i = 0; i < outputs_.size(); ++i)
  {
    const std::size_t expected_bytes = output_buffers_[i].size() * sizeof(float);
    if (outputs_[i].buf == nullptr || outputs_[i].size < expected_bytes)
    {
      last_error_ = "RKNN output '" + output_names_[i] + "' has an invalid buffer size";
      valid = false;
      break;
    }
    std::memcpy(output_buffers_[i].data(), outputs_[i].buf, expected_bytes);
    if (!std::all_of(output_buffers_[i].begin(), output_buffers_[i].end(),
                     [](float value) { return std::isfinite(value); }))
    {
      last_error_ = "RKNN output '" + output_names_[i] + "' contains a non-finite value";
      valid = false;
      break;
    }
  }
  const int release_ret = rknn_outputs_release(context_, static_cast<std::uint32_t>(outputs_.size()), outputs_.data());
  if (valid && release_ret != RKNN_SUCC)
  {
    last_error_ = "rknn_outputs_release failed, ret=" + std::to_string(release_ret);
    valid = false;
  }
  return valid;
}

bool RknnModel::isObservation(std::size_t index) const {
  return index == static_cast<std::size_t>(policy_index_) || index == static_cast<std::size_t>(command_index_);
}

RknnModel::ResidentTensor RknnModel::residentTensor(const rknn_tensor_attr& logical, rknn_query_cmd query) {
  ResidentTensor tensor;
  auto& attr = tensor.attr;
  attr.index = logical.index;
  checkRknn(rknn_query(context_, query, &attr, sizeof(attr)), "RKNN native query " + std::string(logical.name));
  const auto reject = [&] {
    throw std::runtime_error("Unsupported resident WBC layout; logical: " + describe(logical) +
                             "; native: " + describe(attr));
  };
  // The exported WBC ABI consists solely of vectors and matrices. Do not interpret
  // a blocked/image layout (or padding) as a change to the manifest's logical ABI.
  // Such a model must be inspected on the board before adding an explicit mapping.
  if (attr.n_dims < 1 || attr.n_dims > 2 || logical.n_dims < 1 || logical.n_dims > RKNN_MAX_DIMS ||
      attr.fmt != RKNN_TENSOR_UNDEFINED || !supportedResidentFloat(attr) ||
      !supportedResidentFloat(logical) || attr.h_stride != 0)
    reject();
  std::vector<std::uint32_t> logical_shape, native_shape;
  for (std::uint32_t i = 0; i < logical.n_dims; ++i) {
    if (!logical.dims[i]) reject();
    if (logical.dims[i] != 1) logical_shape.push_back(logical.dims[i]);
  }
  std::uint64_t native_elements = 1;
  for (std::uint32_t i = 0; i < attr.n_dims; ++i) {
    if (!attr.dims[i]) reject();
    if (attr.dims[i] != 1) native_shape.push_back(attr.dims[i]);
    native_elements *= attr.dims[i];
  }
  if (logical_shape.empty() || logical_shape.size() > 2 || logical_shape.size() != native_shape.size()) reject();
  std::uint64_t elements = 1;
  for (std::size_t i = 0; i < logical_shape.size(); ++i) {
    elements *= logical_shape[i];
    // Only the last planar dimension may be padded. Never guess an axis permutation.
    if (i + 1 == logical_shape.size()) {
      if (native_shape[i] < logical_shape[i]) reject();
    } else if (native_shape[i] != logical_shape[i]) reject();
  }
  if (elements != logical.n_elems || attr.n_elems < elements) reject();
  tensor.elements = logical.n_elems;
  const std::size_t native_width = attr.dims[attr.n_dims - 1];
  tensor.width = native_width == 1 ? 1 : logical_shape.back();
  tensor.stride = attr.w_stride ? attr.w_stride : native_width;
  tensor.element_bytes = attr.type == RKNN_TENSOR_FLOAT16 ? 2 : sizeof(float);
  const std::uint64_t capacity = attr.size_with_stride ? attr.size_with_stride : attr.size;
  if (tensor.stride < native_width || attr.size < elements * tensor.element_bytes ||
      capacity < attr.size || capacity > 16U * 1024U * 1024U ||
      native_elements > capacity / tensor.element_bytes || attr.n_elems > capacity / tensor.element_bytes ||
      tensor.stride > capacity / tensor.element_bytes / (elements / tensor.width))
    reject();
  attr.size_with_stride = static_cast<std::uint32_t>(capacity);
  attr.pass_through = 1;  // Bytes written below are genuinely in this native dtype/layout.
  return tensor;
}

void RknnModel::initializeResident() {
  policy_index_ = inputIndex("policy");
  command_index_ = inputIndex("command_short");
  if (inputCount() != 33 || outputCount() != 1 || policy_index_ < 0 || command_index_ < 0 ||
      inputElements(policy_index_) != 399 || inputElements(command_index_) != 240 ||
      outputIndex("action") != 0 || outputElements(0) != 21)
    throw std::runtime_error("Resident mode requires the WBC 33-input / action[21] contract");
  // Query/validate every layout before allocating any device memory.
  for (const auto& attr : input_attrs_) resident_inputs_.push_back(residentTensor(attr, RKNN_QUERY_NATIVE_INPUT_ATTR));
  for (const auto& attr : output_attrs_) resident_outputs_.push_back(residentTensor(attr, RKNN_QUERY_NATIVE_OUTPUT_ATTR));
  const auto allocate = [&](ResidentTensor& tensor, int slots) {
    for (int slot = 0; slot < slots; ++slot) {
      auto*& memory = tensor.memory[slot];
      memory = rknn_create_mem2(context_, tensor.attr.size_with_stride, RKNN_FLAG_MEMORY_CACHEABLE);
      if (!memory || !memory->virt_addr || memory->size < tensor.attr.size_with_stride)
        throw std::runtime_error("RKNN persistent allocation failed: " + describe(tensor.attr));
      resident_stats_.allocated_bytes += memory->size;
      resident_stats_.layout_bytes += tensor.attr.size_with_stride;
      std::memset(memory->virt_addr, 0, memory->size);
    }
  };
  for (std::size_t i = 0; i < resident_inputs_.size(); ++i) allocate(resident_inputs_[i], isObservation(i) ? 1 : 2);
  for (auto& tensor : resident_outputs_) allocate(tensor, 1);
  for (auto& tensor : resident_inputs_) {
    checkRknn(rknn_mem_sync(context_, tensor.memory[0], RKNN_MEMORY_SYNC_TO_DEVICE), "RKNN initial input sync");
    checkRknn(rknn_set_io_mem(context_, tensor.memory[0], &tensor.attr), "RKNN initial input bind");
  }
  for (auto& tensor : resident_outputs_) {
    // Padding was CPU-initialized; flush it once before the device owns the output.
    checkRknn(rknn_mem_sync(context_, tensor.memory[0], RKNN_MEMORY_SYNC_TO_DEVICE), "RKNN initial output sync");
    checkRknn(rknn_set_io_mem(context_, tensor.memory[0], &tensor.attr), "RKNN persistent output bind");
  }
  // Slot 0 is a complete rollback binding, never a usable all-zero adapter.
  // resident_slot_ remains -1 until a validated parameter package is committed.
  resident_stats_.context_initializations = 1;
  rknn_mem_size sizes{};
  resident_stats_.sdk_memory_query = rknn_query(context_, RKNN_QUERY_MEM_SIZE, &sizes, sizeof(sizes));
  if (resident_stats_.sdk_memory_query == RKNN_SUCC) {
    resident_stats_.sdk_weight_bytes = sizes.total_weight_size;
    resident_stats_.sdk_internal_bytes = sizes.total_internal_size;
    resident_stats_.sdk_dma_bytes = sizes.total_dma_allocated_size;
  }
}

void RknnModel::releaseResident() noexcept {
  for (auto* tensors : {&resident_inputs_, &resident_outputs_}) {
    for (auto& tensor : *tensors) {
      for (auto*& memory : tensor.memory) {
        if (memory) rknn_destroy_mem(context_, memory);
        memory = nullptr;
      }
    }
  }
}

void RknnModel::writeResident(ResidentTensor& tensor, int slot, const float* values, std::size_t count) {
  if (!values || count != tensor.elements) throw std::runtime_error("Resident input size mismatch");
  // Validate before touching a buffer; FP16 overflow must never silently become Inf.
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(values[i]) || (tensor.attr.type == RKNN_TENSOR_FLOAT16 && std::abs(values[i]) > 65504.0f))
      throw std::runtime_error("Resident input not representable: " + std::string(tensor.attr.name));
  }
  auto* data = static_cast<unsigned char*>(tensor.memory[slot]->virt_addr);
  // Initialize all padding, including allocation alignment, on each prepare/write.
  std::memset(data, 0, tensor.memory[slot]->size);
  for (std::size_t i = 0; i < count; ++i) {
    const auto offset = (i / tensor.width * tensor.stride + i % tensor.width) * tensor.element_bytes;
    if (tensor.attr.type == RKNN_TENSOR_FLOAT16) {
      const auto half = toHalf(values[i]);
      std::memcpy(data + offset, &half, sizeof(half));
    } else std::memcpy(data + offset, values + i, sizeof(float));
  }
}

void RknnModel::stageResidentParameters(int slot, const std::vector<ParameterInput>& parameters) {
  std::lock_guard<std::mutex> lock(resident_mutex_);
  if (input_mode_ != InputMode::ResidentWbc || !resident_healthy_ || slot < 0 || slot > 1 ||
      slot == (resident_slot_ < 0 ? 0 : resident_slot_))
    throw std::runtime_error("Cannot write active/unavailable resident parameter slot");
  staged_[slot] = false;
  if (parameters.size() != inputCount() - 2) throw std::runtime_error("Incomplete resident parameter set");
  std::set<std::size_t> seen;
  for (const auto& value : parameters) {
    if (value.index >= inputCount() || isObservation(value.index) || !seen.insert(value.index).second ||
        !value.data || value.elements != inputElements(value.index))
      throw std::runtime_error("Invalid resident parameter mapping");
  }
  ++resident_stats_.parameter_prepares;
  const auto started = std::chrono::steady_clock::now();
  try {
    for (const auto& value : parameters) {
      auto& tensor = resident_inputs_[value.index];
      writeResident(tensor, slot, value.data, value.elements);
      resident_stats_.parameter_write_bytes += tensor.memory[slot]->size;
      if (tensor.attr.type == RKNN_TENSOR_FLOAT16) resident_stats_.parameter_converted_elements += value.elements;
    }
    staged_[slot] = true;
  } catch (...) {
    resident_stats_.pack_ms += millisecondsSince(started);
    throw;
  }
  resident_stats_.pack_ms += millisecondsSince(started);
}

bool RknnModel::bindResidentSlot(int slot) {
  bool success = true;
  // Attempt the whole set even after a failure, particularly during rollback.
  for (std::size_t i = 0; i < resident_inputs_.size(); ++i) {
    if (isObservation(i)) continue;
    auto& tensor = resident_inputs_[i];
    ++resident_stats_.parameter_bind_calls;
    const int ret = rknn_set_io_mem(context_, tensor.memory[slot], &tensor.attr);
    if (ret != RKNN_SUCC) {
      if (success) last_error_ = "Resident bind failed for " + inputName(i) + ", ret=" + std::to_string(ret);
      success = false;
    }
  }
  return success;
}

bool RknnModel::restoreResidentSlot(int slot) {
  ++resident_stats_.rollbacks;
  if (!bindResidentSlot(slot < 0 ? 0 : slot)) {
    resident_healthy_ = false;
    resident_slot_ = -1;
    last_error_ += "; complete rollback failed; resident inference disabled until restart";
    return false;
  }
  resident_slot_ = slot;
  return true;
}

bool RknnModel::commitResidentParameters(int slot) {
  std::lock_guard<std::mutex> lock(resident_mutex_);
  last_error_.clear();
  if (input_mode_ != InputMode::ResidentWbc || !resident_healthy_ || slot < 0 || slot > 1 ||
      !staged_[slot] || slot == (resident_slot_ < 0 ? 0 : resident_slot_)) {
    last_error_ = "Resident commit requires a staged spare slot and a healthy context";
    return false;
  }
  const auto started = std::chrono::steady_clock::now();
  // Sync all new tensors before changing any binding. No SDK operation runs in stage.
  for (std::size_t i = 0; i < resident_inputs_.size(); ++i) {
    if (isObservation(i)) continue;
    ++resident_stats_.parameter_sync_calls;
    const int ret = rknn_mem_sync(context_, resident_inputs_[i].memory[slot], RKNN_MEMORY_SYNC_TO_DEVICE);
    if (ret != RKNN_SUCC) {
      last_error_ = "Resident parameter sync failed for " + inputName(i) + ", ret=" + std::to_string(ret);
      resident_stats_.commit_ms += millisecondsSince(started);
      return false;  // Old complete binding is untouched.
    }
  }
  if (!bindResidentSlot(slot)) {
    const auto failure = last_error_;
    if (restoreResidentSlot(resident_slot_)) last_error_ = failure + "; previous complete binding restored";
    resident_stats_.commit_ms += millisecondsSince(started);
    return false;
  }
  resident_slot_ = slot;
  ++resident_stats_.commits;
  resident_stats_.commit_ms += millisecondsSince(started);
  return true;
}

bool RknnModel::restoreResidentParameters(int slot) {
  std::lock_guard<std::mutex> lock(resident_mutex_);
  if (input_mode_ != InputMode::ResidentWbc || !resident_healthy_ || slot < -1 || slot > 1 ||
      (slot >= 0 && !staged_[slot])) {
    last_error_ = "Resident rollback unavailable; inference disabled until restart";
    resident_healthy_ = false;
    return false;
  }
  return restoreResidentSlot(slot);
}

bool RknnModel::runResident(const float* policy, std::size_t policy_size,
                            const float* command, std::size_t command_size) {
  std::lock_guard<std::mutex> lock(resident_mutex_);
  last_error_.clear();
  if (input_mode_ != InputMode::ResidentWbc || !resident_healthy_ || resident_slot_ < 0) {
    last_error_ = "Resident inference requires a complete prepared parameter set";
    return false;
  }
  try {
    const auto observation_started = std::chrono::steady_clock::now();
    writeResident(resident_inputs_[policy_index_], 0, policy, policy_size);
    writeResident(resident_inputs_[command_index_], 0, command, command_size);
    for (int index : {policy_index_, command_index_}) {
      ++resident_stats_.observation_sync_calls;
      checkRknn(rknn_mem_sync(context_, resident_inputs_[index].memory[0], RKNN_MEMORY_SYNC_TO_DEVICE),
                "RKNN observation sync");
    }
    const double observation_ms = millisecondsSince(observation_started);
    resident_stats_.observation_ms += observation_ms;
    resident_stats_.observation_max_ms = std::max(resident_stats_.observation_max_ms, observation_ms);
    const auto run_started = std::chrono::steady_clock::now();
    ++resident_stats_.runs;
    // Synchronous set_io_mem API, no ASYNC_MASK, non-block or external fences.
    // rknn_run includes this frame's completion wait; never use outputs_get/release here.
    const int ret = rknn_run(context_, nullptr);
    const double execution_ms = millisecondsSince(run_started);
    resident_stats_.execution_ms += execution_ms;
    resident_stats_.execution_max_ms = std::max(resident_stats_.execution_max_ms, execution_ms);
    if (ret != RKNN_SUCC) {
      // No asynchronous submission is used. Treat any execution/driver error as
      // terminal, and report the extra wait result instead of retrying the frame.
      const int wait_ret = rknn_wait(context_, nullptr);
      resident_healthy_ = false;
      checkRknn(ret, "RKNN resident run (context disabled; wait_ret=" + std::to_string(wait_ret) + ")");
    }
    const auto output_started = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < resident_outputs_.size(); ++index) {
      const auto& tensor = resident_outputs_[index];
      ++resident_stats_.output_sync_calls;
      checkRknn(rknn_mem_sync(context_, tensor.memory[0], RKNN_MEMORY_SYNC_FROM_DEVICE), "RKNN output sync");
      const auto* bytes = static_cast<const unsigned char*>(tensor.memory[0]->virt_addr);
      for (std::size_t i = 0; i < tensor.elements; ++i) {
        const auto offset = (i / tensor.width * tensor.stride + i % tensor.width) * tensor.element_bytes;
        float value;
        if (tensor.attr.type == RKNN_TENSOR_FLOAT16) {
          std::uint16_t half;
          std::memcpy(&half, bytes + offset, sizeof(half));
          value = fromHalf(half);
        } else std::memcpy(&value, bytes + offset, sizeof(value));
        if (!std::isfinite(value)) throw std::runtime_error("Non-finite resident output");
        output_buffers_[index][i] = value;
      }
    }
    const double output_ms = millisecondsSince(output_started);
    resident_stats_.output_ms += output_ms;
    resident_stats_.output_max_ms = std::max(resident_stats_.output_max_ms, output_ms);
    return true;
  } catch (const std::exception& exception) {
    last_error_ = exception.what();
    return false;
  }
}

int RknnModel::residentSlot() const {
  std::lock_guard<std::mutex> lock(resident_mutex_);
  return resident_slot_;
}

RknnModel::ResidentStats RknnModel::residentStats() const {
  std::lock_guard<std::mutex> lock(resident_mutex_);
  return resident_stats_;
}

std::string RknnModel::residentLayout() const {
  std::ostringstream out;
  for (std::size_t i = 0; i < resident_inputs_.size(); ++i)
    out << "input logical " << describe(input_attrs_[i]) << "\ninput native " << describe(resident_inputs_[i].attr) << '\n';
  for (std::size_t i = 0; i < resident_outputs_.size(); ++i)
    out << "output logical " << describe(output_attrs_[i]) << "\noutput native " << describe(resident_outputs_[i].attr) << '\n';
  return out.str();
}
