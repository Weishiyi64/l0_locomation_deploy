#include "wbc_lora_assets.h"

#include <openssl/evp.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <sys/stat.h>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>

namespace {
std::string canonicalPath(const std::string& path) {
  std::unique_ptr<char, decltype(&std::free)> resolved(::realpath(path.c_str(), nullptr), std::free);
  if (!resolved) throw std::runtime_error("WBC LoRA: cannot resolve asset: " + path);
  return resolved.get();
}
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error("WBC LoRA: " + message);
}
bool validId(const std::string& id) {
  return !id.empty() && id.size() <= 32 && std::all_of(id.begin(), id.end(), [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
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
YAML::Node readManifest(const std::string& path) {
  const auto bytes = readFile(path, 4 * 1024 * 1024);
  const auto value = YAML::Load(std::string(bytes.begin(), bytes.end()));
  require(value.IsMap(), "manifest must be a map: " + path);
  return value;
}
std::string asset(const std::string& root, const std::string& relative) {
  require(!relative.empty() && relative.front() != '/', "expected relative asset path");
  const auto base = canonicalPath(root);
  const auto path = canonicalPath(base + "/" + relative);
  const auto prefix = base == "/" ? base : base + "/";
  struct stat info{};
  require(path.compare(0, prefix.size(), prefix) == 0 &&
          ::stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode), "asset escapes bundle: " + relative);
  return path;
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
bool isHash(const std::string& value) {
  return value.size() == 64 && value.find_first_not_of("0123456789abcdef") == std::string::npos;
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
std::string checkedRknn(const std::string& root, const std::string& relative, const std::string& source_hash) {
  require(isHash(source_hash), "invalid ONNX fingerprint");
  const auto path = asset(root, relative);
  const auto record = readManifest(asset(root, relative + ".json"));
  require(record["format"].as<std::string>() == "wbc_rknn_conversion_v1" &&
          record["source_sha256"].as<std::string>() == source_hash &&
          record["rknn_sha256"].as<std::string>() == fileHash(path) &&
          record["target_platform"].as<std::string>() == "rk3588" && record["precision"].as<std::string>() == "float16" && !record["do_quantization"].as<bool>(),
          "model/conversion record mismatch: " + relative);
  return path;
}

std::string fixedModel(const std::string& root, const YAML::Node& model,
                       const std::string& onnx_key, const std::string& rknn_key,
                       const std::string& hash_key) {
  const auto hash = model[hash_key].as<std::string>();
#ifdef USE_RKNN
  (void)onnx_key;
  return checkedRknn(root, model[rknn_key].as<std::string>(), hash);
#else
  (void)rknn_key;
  const auto path = asset(root, model[onnx_key].as<std::string>());
  require(fileHash(path) == hash, "ONNX hash mismatch: " + path);
  return path;
#endif
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

std::string wbcFileHash(const std::string& path) { return fileHash(path); }

WbcBaseAssets::WbcBaseAssets(const std::string& manifest, const std::vector<std::string>& selected) {
  const auto graph = readManifest(manifest);
  const auto path = canonicalPath(manifest);
  const auto root = path.substr(0, path.find_last_of('/'));
  require(graph["format"].as<std::string>() == "wbc_base_graph_v1" &&
          graph["kind"].as<std::string>() == "policy" &&
          graph["target_precision"].as<std::string>() == "float16", "unsupported base-only graph contract");
  model_path_ = fixedModel(root, graph["model"], "onnx", "rknn", "sha256");
  if (const auto ila = graph["ila_adapter"]) {
    require(ila["format"].as<std::string>() == "sonic_ila_arm_window_v1", "unsupported ILA adapter");
    ila_path_ = fixedModel(root, ila, "onnx", "rknn", "sha256");
  }
  require(!selected.empty() && std::set<std::string>(selected.begin(), selected.end()).size() == selected.size(),
          "empty/duplicate base action selection");
  const auto actions = graph["actions"];
  require(actions.IsSequence(), "missing base action catalog");
  std::set<std::string> ids;
  YAML::Node first_robot, first_wbc;
  for (const auto& item : actions) {
    const auto id = item["action_id"].as<std::string>();
    require(validId(id) && ids.insert(id).second, "invalid/duplicate base action ID");
    if (std::find(selected.begin(), selected.end(), id) == selected.end()) continue;
    Action action;
    if (item["model"]) {
      const auto model = item["model"];
      action.model = fixedModel(root, model, "onnx", "rknn", "sha256");
    }
    action.config = asset(root, item["config"].as<std::string>());
    require(fileHash(action.config) == item["config_sha256"].as<std::string>(), "base config hash mismatch: " + id);
    action.reference = fixedModel(root, item, "reference_onnx", "reference_rknn", "reference_onnx_sha256");
    const auto config = readManifest(action.config);
    require(config["WbcCfg"]["motion_id"].as<std::string>() == id, "base action binding mismatch: " + id);
    // One fixed actor must use the same actuation and input encoding for every clip.
    if (actions_.empty()) {
      first_robot = config["RobotCfg"];
      first_wbc = config["WbcCfg"];
    } else {
      for (const char* key : {"joint_order_mj", "default_joint_angles", "action_scale", "kps", "kds"})
        require(YAML::Dump(first_robot[key]) == YAML::Dump(config["RobotCfg"][key]),
                std::string("base action RobotCfg mismatch: ") + key);
      for (const char* key : {"control_dt", "history_steps", "future_steps", "previous_action_steps"})
        require(YAML::Dump(first_wbc[key]) == YAML::Dump(config["WbcCfg"][key]),
                std::string("base action WbcCfg mismatch: ") + key);
    }
    actions_.emplace(id, std::move(action));
  }
  require(actions_.size() == selected.size(), "selected base action absent from manifest");
}

WbcLoraAssets::WbcLoraAssets(const std::string& manifest, const std::vector<std::string>& selected)
    : graph_(readManifest(manifest)), root_(canonicalPath(manifest)) {
  root_.resize(root_.find_last_of('/') + 1);
  std::string model_manifest = manifest;
  std::vector<std::pair<YAML::Node, std::string>> catalogs;
  const bool deployment = graph_["format"].as<std::string>() == "wbc_motion_deployment_v1";
  if (deployment) {
    const auto layout = graph_;
    model_manifest = asset(root_, layout["model_manifest"].as<std::string>());
    for (const auto& relative : layout["catalogs"]) {
      const auto path = asset(root_, relative.as<std::string>());
      catalogs.emplace_back(readManifest(path), path.substr(0, path.find_last_of('/') + 1));
    }
    graph_ = readManifest(model_manifest);
    root_ = model_manifest.substr(0, model_manifest.find_last_of('/') + 1);
  } else {
    catalogs.emplace_back(graph_, root_);
  }
  require(graph_["format"].as<std::string>() == "wbc_lora_graph_v1" &&
          graph_["kind"].as<std::string>() == "policy" && graph_["rank"].as<int>() == 16 &&
          graph_["alpha"].as<int>() == 16 && graph_["target_precision"].as<std::string>() == "float16",
          "unsupported graph contract (WBC Base is independent of AMP)");
  model_path_ = asset(root_, graph_["model"][inferenceAssetKey()].as<std::string>());

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
  for (const auto& spec : specs) {
    Tensor tensor;
    tensor.name = spec["name"].as<std::string>();
    tensor.actor_key = spec["actor_key"].as<std::string>();
    tensor.shape = shape(spec["shape"]);
    require(expected.count(tensor.name) && tensor.shape == expected.at(tensor.name) && names.insert(tensor.name).second,
            "wrong or duplicate parameter: " + tensor.name);
    require(!tensor.actor_key.empty() && tensor.actor_key.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.") == std::string::npos, "invalid actor key");
    tensor.elements = 1;
    for (const auto dim : tensor.shape) tensor.elements *= dim;
    tensors_.push_back(std::move(tensor));
  }
  require((deployment || !selected.empty()) &&
          std::set<std::string>(selected.begin(), selected.end()).size() == selected.size(), "empty/duplicate action selection");
  std::set<std::string> ids;
  for (const auto& catalog : catalogs) {
    const auto& data = catalog.first;
    const auto& root = catalog.second;
    const auto format = data["format"].as<std::string>();
    require(format == "wbc_motion_catalog_v1" ||
            (format == "wbc_lora_graph_v1" && data["kind"].as<std::string>() == "policy"),
            "unsupported action catalog");
    const auto actions = data["actions"];
    require(actions.IsSequence(), "missing action catalog");
    for (const auto& item : actions) {
      const auto id = item["action_id"].as<std::string>();
      require(validId(id) && ids.insert(id).second, "invalid/duplicate action ID");
      const bool json_only = item["json_only"].as<bool>(false);
      if ((!deployment || !selected.empty()) && !json_only &&
          std::find(selected.begin(), selected.end(), id) == selected.end()) continue;
      Action action;
      action.id = id;
      action.parameter_source = item["parameter_source"].as<std::string>("lora");
      action.json_only = json_only;
      require(action.parameter_source == "base" || action.parameter_source == "lora", "invalid parameter source");
      action.config = asset(root, item["config"].as<std::string>());
      if (!action.json_only) {
        action.reference_root = root;
        action.reference = item[std::string("reference_") + inferenceAssetKey()].as<std::string>();
        // Resolve fixed references only when preparing the action.
      }
      action.adapter = asset(root, item["adapter_manifest"].as<std::string>());
      actions_.emplace(id, std::move(action));
    }
  }
  for (const auto& id : selected) require(actions_.count(id), "selected action absent from catalog: " + id);
  require(!actions_.empty(), "empty deployment catalog");
}

std::vector<std::string> WbcLoraAssets::actionIds() const {
  std::vector<std::string> ids;
  for (const auto& item : actions_) ids.push_back(item.first);
  return ids;
}

std::string WbcLoraAssets::referencePath(const Action& action) const {
  require(!action.json_only, "text tracker has no reference model");
  return asset(action.reference_root, action.reference);
}

void WbcLoraAssets::bindModel(const InferenceModel& model) {
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
            isFloatingInput(model, index),
            "inference input shape/type mismatch: " + name);
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
  const auto manifest = readManifest(action.adapter);
  require(manifest["format"].as<std::string>() == "wbc_lora_adapter_v1" &&
          manifest["action_id"].as<std::string>() == action.id, "adapter identity mismatch");
  const auto weights = manifest["weights"];
  const auto path = asset(action.adapter.substr(0, action.adapter.find_last_of('/')), weights["path"].as<std::string>());
  const auto payload = readFile(path, 4 * 1024 * 1024);
  require(weights["dtype"].as<std::string>() == "float16-le" && weights["size_bytes"].as<std::size_t>() == payload.size(), "invalid adapter payload");
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

std::vector<InferenceModel::ParameterInput> WbcLoraAssets::parameterInputs(const Buffer& buffer) const {
  require(buffer.parameters.size() == tensors_.size(), "incomplete staged adapter");
  std::vector<InferenceModel::ParameterInput> inputs;
  inputs.reserve(tensors_.size());
  for (std::size_t i = 0; i < tensors_.size(); ++i) {
    require(tensors_[i].input_index >= 0 && buffer.parameters[i].size() == tensors_[i].elements,
            "unbound or incomplete parameter: " + tensors_[i].name);
    const auto index = static_cast<std::size_t>(tensors_[i].input_index);
    inputs.push_back({index, buffer.parameters[i].data(), buffer.parameters[i].size()});
  }
  return inputs;
}
