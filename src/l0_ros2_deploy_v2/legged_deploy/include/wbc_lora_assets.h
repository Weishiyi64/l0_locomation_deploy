#pragma once

#include "rknn_model.h"
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <vector>

// A fixed two-input actor plus a catalog of references. No adapter tensors.
class WbcBaseAssets {
 public:
  struct Action { std::string config, reference; };
  WbcBaseAssets(const std::string& manifest, const std::vector<std::string>& selected);
  const Action& action(const std::string& id) const { return actions_.at(id); }
  const std::string& modelPath() const { return model_path_; }

 private:
  std::string model_path_;
  std::map<std::string, Action> actions_;
};

// Shared by the deployed worker in both HPM and isolated runs. No ROS or training dependencies.
class WbcLoraAssets {
 public:
  struct Action {
    std::string id, config, reference, adapter, adapter_hash;
  };
  struct Buffer {
    std::vector<std::vector<float>> parameters;
  };
  WbcLoraAssets(const std::string& manifest, const std::vector<std::string>& selected);
  const Action& action(const std::string& id) const { return actions_.at(id); }
  const std::string& modelPath() const { return model_path_; }
  const std::string& baseHash() const { return base_hash_; }
  void bindModel(const RknnModel& model);
  void allocate(Buffer& buffer) const;
  void loadAdapter(const Action& action, Buffer& buffer) const;
  // Complete parameter mapping, used only by prepare; no device ownership here.
  std::vector<RknnModel::ParameterInput> parameterInputs(const Buffer& buffer) const;

 private:
  struct Tensor {
    std::string name, actor_key;
    std::vector<std::size_t> shape;
    std::size_t elements{};
    int input_index{-1};
  };
  YAML::Node graph_;
  std::string root_, model_path_, base_hash_;
  std::vector<Tensor> tensors_;
  std::map<std::string, Action> actions_;
};
