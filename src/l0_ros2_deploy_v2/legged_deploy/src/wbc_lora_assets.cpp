#include "wbc_lora_assets.h"

#include <openssl/evp.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
namespace fs = std::filesystem;
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error("WBC LoRA: " + message);
}
bool validId(const std::string& id) {
  return !id.empty() && id.size() <= 32 && std::all_of(id.begin(), id.end(), [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
}
bool isHash(const std::string& value) {
  return value.size() == 64 && value.find_first_not_of("0123456789abcdef") == std::string::npos;
}
std::string digest(const void* data, std::size_t size) {
  unsigned char out[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  require(EVP_Digest(data, size, out, &length, EVP_sha256(), nullptr) == 1, "SHA256 failed");
  std::ostringstream hex;
  for (unsigned i = 0; i < length; ++i) hex << std::hex << std::setw(2) << std::setfill('0') << unsigned(out[i]);
  return hex.str();
}
std::vector<unsigned char> readFile(const std::string& path, std::size_t maximum) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  require(stream.good(), "cannot open " + path);
  const auto length = stream.tellg();
  require(length > 0 && static_cast<std::uint64_t>(length) <= maximum, "invalid size: " + path);
  std::vector<unsigned char> data(static_cast<std::size_t>(length));
  stream.seekg(0);
  require(bool(stream.read(reinterpret_cast<char*>(data.data()), data.size())), "short read: " + path);
  return data;
}
std::string fileHash(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  require(stream.good(), "cannot hash " + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  require(ctx && EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) == 1, "SHA256 init failed");
  std::array<char, 65536> chunk{};
  while (stream.read(chunk.data(), chunk.size()) || stream.gcount())
    require(EVP_DigestUpdate(ctx.get(), chunk.data(), stream.gcount()) == 1, "SHA256 update failed");
  require(stream.eof(), "hash read failed: " + path);
  unsigned char out[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  require(EVP_DigestFinal_ex(ctx.get(), out, &length) == 1, "SHA256 final failed");
  std::ostringstream hex;
  for (unsigned i = 0; i < length; ++i) hex << std::hex << std::setw(2) << std::setfill('0') << unsigned(out[i]);
  return hex.str();
}
YAML::Node readManifest(const std::string& path) {
  const auto bytes = readFile(path, 4 * 1024 * 1024);
  const auto value = YAML::Load(std::string(bytes.begin(), bytes.end()));
  require(value.IsMap(), "manifest must be a map: " + path);
  return value;
}
std::string asset(const std::string& root, const std::string& relative) {
  require(!relative.empty() && !fs::path(relative).is_absolute(), "expected relative asset path");
  const auto base = fs::canonical(root);
  const auto path = fs::canonical(base / relative);
  const auto tail = path.lexically_relative(base);
  require(!tail.empty() && *tail.begin() != ".." && fs::is_regular_file(path), "asset escapes bundle: " + relative);
  return path.string();
}
std::vector<std::size_t> shape(const YAML::Node& node) {
  require(node.IsSequence() && node.size() > 0 && node.size() <= 4, "invalid tensor rank");
  std::vector<std::size_t> result;
  for (const auto& dim : node) {
    const auto value = dim.as<int>();
    require(value > 0 && value <= 65536, "invalid tensor dimension");
    result.push_back(static_cast<std::size_t>(value));
  }
  return result;
}
std::vector<std::size_t> withoutSingletons(std::vector<std::size_t> dims) {
  dims.erase(std::remove(dims.begin(), dims.end(), 1), dims.end());
  return dims;
}
std::string checkedRknn(const std::string& root, const std::string& relative, const std::string& source_hash) {
  require(isHash(source_hash), "invalid ONNX fingerprint");
  const auto path = asset(root, relative);
  const auto record = readManifest(asset(root, relative + ".json"));
  require(record["format"].as<std::string>() == "wbc_rknn_conversion_v1" &&
          record["source_sha256"].as<std::string>() == source_hash &&
          record["rknn_sha256"].as<std::string>() == fileHash(path) &&
          record["precision"].as<std::string>() == "float16" && !record["do_quantization"].as<bool>(),
          "model/conversion record mismatch: " + relative);
  return path;
}
float halfToFloat(std::uint16_t bits) {
  const int sign = (bits & 0x8000U) ? -1 : 1;
  const int exponent = (bits >> 10U) & 31U;
  const int fraction = bits & 1023U;
  require(exponent != 31, "non-finite FP16 parameter");
  return static_cast<float>(sign * std::ldexp(exponent ? 1024 + fraction : fraction,
                                             exponent ? exponent - 25 : -24));
}
}  // namespace

WbcLoraAssets::WbcLoraAssets(const std::string& manifest, const std::vector<std::string>& selected)
    : graph_(readManifest(manifest)), root_(fs::canonical(fs::path(manifest).parent_path()).string()) {
  require(graph_["format"].as<std::string>() == "wbc_lora_graph_v1" &&
          graph_["kind"].as<std::string>() == "policy" && graph_["rank"].as<int>() == 16 &&
          graph_["alpha"].as<int>() == 16 && graph_["target_precision"].as<std::string>() == "float16",
          "unsupported graph contract (WBC Base is independent of AMP)");
  for (const char* key : {"base_checkpoint_sha256", "source_onnx_sha256", "normalization_sha256", "parameter_schema_sha256"})
    require(isHash(graph_[key].as<std::string>()), std::string("invalid fingerprint: ") + key);
  base_hash_ = graph_["base_checkpoint_sha256"].as<std::string>();
  model_path_ = checkedRknn(root_, graph_["model"]["rknn"].as<std::string>(), graph_["model"]["sha256"].as<std::string>());
  require(readManifest(model_path_ + ".json")["graph_manifest_sha256"].as<std::string>() == fileHash(manifest),
          "shared model was converted with another graph manifest");

  std::map<std::string, std::vector<std::size_t>> expected;
  const std::array<std::size_t, 7> widths{{639, 1024, 1024, 1423, 1024, 1024, 1024}};
  for (std::size_t i = 0; i < widths.size(); ++i) {
    const auto prefix = "lora_" + std::to_string(i);
    expected[prefix + "_a"] = {16, widths[i]};
    expected[prefix + "_b"] = {i == 6 ? 21U : 1024U, 16};
    expected[prefix + "_bias"] = {i == 6 ? 21U : 1024U};
  }
  for (int i = 0; i < 5; ++i) {
    expected["norm_" + std::to_string(i) + "_gamma"] = {1024};
    expected["norm_" + std::to_string(i) + "_beta"] = {1024};
  }
  const auto specs = graph_["parameters"];
  require(specs.IsSequence() && specs.size() == 31, "expected 31 complete adapter tensors");
  std::set<std::string> names;
  std::ostringstream canonical;
  canonical << '[';
  for (const auto& spec : specs) {
    Tensor tensor;
    tensor.name = spec["name"].as<std::string>();
    tensor.actor_key = spec["actor_key"].as<std::string>();
    tensor.shape = shape(spec["shape"]);
    require(expected.count(tensor.name) && tensor.shape == expected.at(tensor.name) && names.insert(tensor.name).second,
            "wrong or duplicate parameter: " + tensor.name);
    require(!tensor.actor_key.empty() && tensor.actor_key.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.") == std::string::npos, "invalid actor key");
    if (!tensors_.empty()) canonical << ',';
    canonical << "{\"actor_key\":\"" << tensor.actor_key << "\",\"name\":\"" << tensor.name << "\",\"shape\":[";
    tensor.elements = 1;
    for (std::size_t i = 0; i < tensor.shape.size(); ++i) {
      if (i) canonical << ',';
      canonical << tensor.shape[i];
      tensor.elements *= tensor.shape[i];
    }
    canonical << "]}";
    tensors_.push_back(std::move(tensor));
  }
  canonical << ']';
  const auto schema = canonical.str();
  require(digest(schema.data(), schema.size()) == graph_["parameter_schema_sha256"].as<std::string>(),
          "parameter schema fingerprint mismatch");
  require(!selected.empty() && std::set<std::string>(selected.begin(), selected.end()).size() == selected.size(),
          "empty/duplicate action selection");
  const auto actions = graph_["actions"];
  require(actions.IsSequence(), "missing action catalog");
  std::set<std::string> ids;
  for (const auto& item : actions) {
    const auto id = item["action_id"].as<std::string>();
    require(validId(id) && ids.insert(id).second, "invalid/duplicate action ID");
    if (std::find(selected.begin(), selected.end(), id) == selected.end()) continue;
    Action action;
    action.id = id;
    action.parameter_source = item["parameter_source"].as<std::string>("lora");
    require(action.parameter_source == "base" || action.parameter_source == "lora", "invalid parameter source");
    action.config = asset(root_, item["config"].as<std::string>());
    require(fileHash(action.config) == item["config_sha256"].as<std::string>(), "config hash mismatch: " + id);
    action.reference = checkedRknn(root_, item["reference_rknn"].as<std::string>(),
                                   item["reference_onnx_sha256"].as<std::string>());
    action.adapter = asset(root_, item["adapter_manifest"].as<std::string>());
    action.adapter_hash = fileHash(action.adapter);
    require(readManifest(action.adapter)["parameter_source"].as<std::string>("lora") == action.parameter_source,
            "graph/adapter parameter source mismatch");
    require(readManifest(action.adapter)["action_id"].as<std::string>() == id &&
            readManifest(action.config)["WbcCfg"]["motion_id"].as<std::string>() == id, "action binding mismatch: " + id);
    actions_.emplace(id, std::move(action));
  }
  require(actions_.size() == selected.size(), "selected action absent from manifest");
}

void WbcLoraAssets::bindModel(const RknnModel& model) {
  const auto inputs = graph_["inputs"];
  require(inputs.IsSequence() && inputs.size() == 33 && model.inputCount() == 33 && model.outputCount() == 1 &&
          model.outputIndex("action") == 0 && model.outputElements(0) == 21, "WBC shared model must have 33 inputs / action[21]");
  std::set<std::string> seen;
  for (const auto& item : inputs) {
    const auto name = item["name"].as<std::string>();
    const auto dims = shape(item["shape"]);
    const auto role = item["role"].as<std::string>();
    const int index = model.inputIndex(name);
    require(seen.insert(name).second && index >= 0 && item["dtype"].as<std::string>() == "float32", "invalid input: " + name);
    require(withoutSingletons(model.inputShape(index)) == withoutSingletons(dims) &&
            (model.inputType(index) == RKNN_TENSOR_FLOAT16 || model.inputType(index) == RKNN_TENSOR_FLOAT32),
            "RKNN input shape/type mismatch: " + name);
    if (name == "policy" || name == "command_short") {
      require(role == "observation" && dims == std::vector<std::size_t>{name == "policy" ? 399U : 240U}, "wrong WBC observation ABI");
    } else {
      auto tensor = std::find_if(tensors_.begin(), tensors_.end(), [&](const Tensor& spec) { return spec.name == name; });
      require(role == "adapter" && tensor != tensors_.end() && tensor->shape == dims, "unknown adapter input: " + name);
      tensor->input_index = index;
    }
  }
  require(seen.count("policy") && seen.count("command_short"), "missing WBC observations");
  for (const auto& tensor : tensors_) require(tensor.input_index >= 0, "unbound parameter: " + tensor.name);
}

void WbcLoraAssets::allocate(Buffer& buffer) const {
  buffer.parameters.resize(tensors_.size());
  for (std::size_t i = 0; i < tensors_.size(); ++i) buffer.parameters[i].resize(tensors_[i].elements);
}

void WbcLoraAssets::loadAdapter(const Action& action, Buffer& buffer) const {
  require(fileHash(action.adapter) == action.adapter_hash, "adapter manifest changed since startup");
  const auto manifest = readManifest(action.adapter);
  require(manifest["format"].as<std::string>() == "wbc_lora_adapter_v1" &&
          manifest["action_id"].as<std::string>() == action.id, "adapter identity mismatch");
  for (const char* key : {"base_checkpoint_sha256", "normalization_sha256", "parameter_schema_sha256", "rank", "alpha"})
    require(manifest[key].as<std::string>() == graph_[key].as<std::string>(), std::string("adapter mismatch: ") + key);
  const auto weights = manifest["weights"];
  const auto path = asset(fs::path(action.adapter).parent_path().string(), weights["path"].as<std::string>());
  const auto payload = readFile(path, 4 * 1024 * 1024);
  require(weights["dtype"].as<std::string>() == "float16-le" && weights["size_bytes"].as<std::size_t>() == payload.size() &&
          digest(payload.data(), payload.size()) == weights["sha256"].as<std::string>(), "invalid adapter payload");
  const auto records = manifest["tensors"];
  require(records.IsSequence() && records.size() == tensors_.size() && buffer.parameters.size() == tensors_.size(),
          "incomplete adapter buffer/schema");
  std::size_t end = 0;
  for (std::size_t i = 0; i < tensors_.size(); ++i) {
    const auto& spec = tensors_[i];
    const auto record = records[i];
    const auto offset = record["offset_bytes"].as<std::size_t>();
    const auto bytes = spec.elements * 2;
    require(record["name"].as<std::string>() == spec.name && record["actor_key"].as<std::string>() == spec.actor_key &&
            shape(record["shape"]) == spec.shape && record["size_bytes"].as<std::size_t>() == bytes &&
            offset >= end && offset % 64 == 0 && offset <= payload.size() && bytes <= payload.size() - offset &&
            buffer.parameters[i].size() == spec.elements, "invalid adapter tensor: " + spec.name);
    for (std::size_t j = 0; j < spec.elements; ++j)
      buffer.parameters[i][j] = halfToFloat(static_cast<std::uint16_t>(payload[offset + j * 2]) |
                                           (static_cast<std::uint16_t>(payload[offset + j * 2 + 1]) << 8U));
    end = offset + bytes;
  }
  require(end == payload.size(), "trailing adapter payload");
}

std::vector<RknnModel::ParameterInput> WbcLoraAssets::parameterInputs(const Buffer& buffer) const {
  require(buffer.parameters.size() == tensors_.size(), "incomplete staged adapter");
  std::vector<RknnModel::ParameterInput> inputs;
  inputs.reserve(tensors_.size());
  for (std::size_t i = 0; i < tensors_.size(); ++i) {
    require(tensors_[i].input_index >= 0 && buffer.parameters[i].size() == tensors_[i].elements,
            "unbound or incomplete parameter: " + tensors_[i].name);
    const auto index = static_cast<std::size_t>(tensors_[i].input_index);
    inputs.push_back({index, buffer.parameters[i].data(), buffer.parameters[i].size()});
  }
  return inputs;
}
