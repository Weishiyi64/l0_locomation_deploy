#include "TextMotionRknn.h"

#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <limits>
#include <set>

namespace legged {
namespace motion {
namespace {
void checked(int code, const std::string& operation) {
  require(code == RKNN_SUCC, operation + " failed, RKNN code=" + std::to_string(code));
}
template <typename Function> Function symbol(void* library, const char* name) {
  auto result = reinterpret_cast<Function>(::dlsym(library, name));
  require(result != nullptr, std::string("Missing RKNN C API: ") + name);
  return result;
}
std::vector<std::size_t> dimensions(const Json& spec) {
  std::vector<std::size_t> result;
  for (const auto& value : spec.at("shape")) {
    const auto dim = static_cast<std::size_t>(integer(value, 1, 65536));
    if (dim != 1) result.push_back(dim);
  }
  return result;
}
std::size_t elements(const Json& spec) {
  std::size_t count = 1;
  require(spec.at("shape").is_array() && !spec.at("shape").empty() && spec.at("shape").size() <= RKNN_MAX_DIMS,
          "Invalid neural tensor rank");
  for (const auto& value : spec.at("shape")) {
    const auto dim = static_cast<std::size_t>(integer(value, 1, 65536));
    require(count <= 16 * 1024 * 1024 / dim, "Neural tensor is too large");
    count *= dim;
  }
  return count;
}
void validateAttribute(const rknn_tensor_attr& attr, const Json& spec, bool output) {
  std::vector<std::size_t> dims;
  require(attr.n_dims <= RKNN_MAX_DIMS, "Invalid RKNN tensor rank");
  for (unsigned i = 0; i < attr.n_dims; ++i) if (attr.dims[i] != 1) dims.push_back(attr.dims[i]);
  require(attr.n_elems == elements(spec) && dims == dimensions(spec), "RKNN tensor shape differs from manifest");
  const auto dtype = spec.at("dtype").get<std::string>();
  const bool floating = attr.type == RKNN_TENSOR_FLOAT32 || attr.type == RKNN_TENSOR_FLOAT16;
  const bool integral = attr.type == RKNN_TENSOR_INT32 || attr.type == RKNN_TENSOR_INT64;
  require((dtype == "float32" && floating) || (dtype == "int32" && (integral || (output && floating))),
          "RKNN tensor dtype differs from manifest");
}
}  // namespace
TextRknnApi::TextRknnApi(const std::string& library) {
  library_ = ::dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
  const char* error = library_ ? nullptr : ::dlerror();
  require(library_ != nullptr, "Cannot load shared RKNN runtime: " + std::string(error ? error : library));
  try {
    init = symbol<decltype(init)>(library_, "rknn_init");
    destroy = symbol<decltype(destroy)>(library_, "rknn_destroy");
    query = symbol<decltype(query)>(library_, "rknn_query");
    set_core_mask = symbol<decltype(set_core_mask)>(library_, "rknn_set_core_mask");
    inputs_set = symbol<decltype(inputs_set)>(library_, "rknn_inputs_set");
    run = symbol<decltype(run)>(library_, "rknn_run");
    wait = symbol<decltype(wait)>(library_, "rknn_wait");
    outputs_get = symbol<decltype(outputs_get)>(library_, "rknn_outputs_get");
    outputs_release = symbol<decltype(outputs_release)>(library_, "rknn_outputs_release");
  } catch (...) { ::dlclose(library_); library_ = nullptr; throw; }
}
TextRknnApi::~TextRknnApi() {
  // An unconfirmed device operation must never outlive its mapped C API library.
  if (library_ && !uncertain.load()) ::dlclose(library_);
}
TextRknnSession::TextRknnSession(std::shared_ptr<TextRknnApi> api, const std::string& path, const Json& contract)
    : api_(std::move(api)) {
  auto bytes = readFile(path, 128 * 1024 * 1024);
  checked(api_->init(&context_, &bytes[0], static_cast<std::uint32_t>(bytes.size()), 0, nullptr), "rknn_init");
  try {
    checked(api_->set_core_mask(context_, RKNN_NPU_CORE_0), "rknn_set_core_mask");
    rknn_input_output_num count{};
    checked(api_->query(context_, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count)), "RKNN I/O count");
    require(count.n_input == contract.at("inputs").size() && count.n_output == contract.at("outputs").size(),
            "RKNN input/output count differs from manifest");
    inputs_.resize(count.n_input);
    outputs_.resize(count.n_output);
    std::vector<rknn_tensor_attr> attributes(count.n_input);
    for (std::size_t i = 0; i < attributes.size(); ++i) {
      attributes[i].index = i;
      checked(api_->query(context_, RKNN_QUERY_INPUT_ATTR, &attributes[i], sizeof(attributes[i])), "RKNN input attribute");
    }
    std::set<std::size_t> used;
    for (const auto& spec : contract.at("inputs")) {
      const auto name = spec.at("name").get<std::string>();
      auto found = std::find_if(attributes.begin(), attributes.end(), [&](const rknn_tensor_attr& attr) { return name == attr.name; });
      require(found != attributes.end() && used.insert(found->index).second, "RKNN input name mismatch");
      validateAttribute(*found, spec, false);
      input_indices_.push_back(found->index);
      input_elements_.push_back(elements(spec));
      input_types_.push_back(spec.at("dtype") == "int32" ? RKNN_TENSOR_INT32 : RKNN_TENSOR_FLOAT32);
    }
    buffers_.resize(count.n_output);
    for (std::size_t i = 0; i < outputs_.size(); ++i) {
      rknn_tensor_attr attr{};
      attr.index = i;
      checked(api_->query(context_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)), "RKNN output attribute");
      const Json* spec = nullptr;
      for (const auto& candidate : contract.at("outputs")) if (candidate.at("name") == attr.name) spec = &candidate;
      require(spec != nullptr, "RKNN output name mismatch");
      validateAttribute(attr, *spec, true);
      output_names_.emplace_back(attr.name);
      buffers_[i].resize(elements(*spec));
      outputs_[i].index = i;
      outputs_[i].want_float = 1;
      outputs_[i].is_prealloc = 1;
      outputs_[i].buf = buffers_[i].data();
      outputs_[i].size = buffers_[i].size() * sizeof(float);
    }
  } catch (...) { api_->destroy(context_); context_ = 0; throw; }
}
TextRknnSession::~TextRknnSession() {
  if (context_ && retired()) api_->destroy(context_);
}
void TextRknnSession::run(const std::vector<Input>& values, std::int64_t deadline, const std::function<void()>& checkpoint) {
  require(retired() && !api_->uncertain.load(), "RKNN context retirement is unconfirmed");
  require(values.size() == inputs_.size(), "Neural input count mismatch");
  checkpoint();
  checkDeadline(deadline);
  for (std::size_t i = 0; i < values.size(); ++i) {
    const auto& value = values[i];
    require(value.data && value.type == input_types_[i] && value.elements == input_elements_[i], "Invalid neural input buffer");
    if (value.type == RKNN_TENSOR_FLOAT32) {
      const auto* data = static_cast<const float*>(value.data);
      require(std::all_of(data, data + value.elements, [](float x) { return std::isfinite(x); }), "Non-finite neural input");
    }
    auto& input = inputs_[i];
    input.index = input_indices_[i];
    input.buf = const_cast<void*>(value.data);
    input.size = value.elements * sizeof(std::int32_t);
    input.type = value.type;
    input.fmt = RKNN_TENSOR_NCHW;
    input.pass_through = 0;
  }
  checked(api_->inputs_set(context_, inputs_.size(), inputs_.data()), "rknn_inputs_set");
  checkpoint();
  checkDeadline(deadline);
  rknn_run_extend extend{};
  extend.timeout_ms = std::max<std::int64_t>(1, (deadline - steadyNs() + 999999) / 1000000);
  extend.fence_fd = -1;
  in_flight_.store(true);
  const int code = api_->run(context_, &extend);
  if (code != RKNN_SUCC) {
    // A run timeout is not permission to destroy/reuse a context. Wait for actual retirement.
    int retirement = code;
    do {
      extend.timeout_ms = 100;
      retirement = api_->wait(context_, &extend);
    } while (retirement == RKNN_ERR_TIMEOUT);
    if (retirement == RKNN_SUCC) in_flight_.store(false);
    else api_->uncertain.store(true);
    checked(code, "rknn_run (result discarded)");
  }
  in_flight_.store(false);
  for (std::size_t i = 0; i < outputs_.size(); ++i) {
    outputs_[i].index = i;
    outputs_[i].want_float = 1;
    outputs_[i].is_prealloc = 1;
    outputs_[i].buf = buffers_[i].data();
    outputs_[i].size = buffers_[i].size() * sizeof(float);
  }
  checked(api_->outputs_get(context_, outputs_.size(), outputs_.data(), nullptr), "rknn_outputs_get");
  const int released = api_->outputs_release(context_, outputs_.size(), outputs_.data());
  checked(released, "rknn_outputs_release");
  checkpoint();
  checkDeadline(deadline);
  for (const auto& buffer : buffers_)
    require(std::all_of(buffer.begin(), buffer.end(), [](float x) { return std::isfinite(x); }), "Non-finite neural output");
}
const std::vector<float>& TextRknnSession::output(const std::string& name) const {
  const auto found = std::find(output_names_.begin(), output_names_.end(), name);
  require(found != output_names_.end(), "Unknown neural output: " + name);
  return buffers_.at(static_cast<std::size_t>(found - output_names_.begin()));
}
}  // namespace motion
}  // namespace legged
