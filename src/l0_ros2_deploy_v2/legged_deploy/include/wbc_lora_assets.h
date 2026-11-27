#pragma once

#include "inference_model.h"
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <vector>

std::string wbcFileHash(const std::string& path);

// A fixed two-input actor plus a catalog of references. No adapter tensors.
class WbcBaseAssets {
 public:
  struct Action { std::string config, reference, model; };
  WbcBaseAssets(const std::string& manifest, const std::vector<std::string>& selected);
  const std::string& ilaPath() const { return ila_path_; }
  const Action& action(const std::string& id) const { return actions_.at(id); }
  const std::string& modelPath() const { return model_path_; }

 private:
  std::string model_path_, ila_path_;
  std::map<std::string, Action> actions_;
};

// Shared by the deployed worker in both HPM and isolated runs. No ROS or training dependencies.
class WbcLoraAssets {
 public:
  struct Action {
    std::string id, config, reference, reference_root, adapter;
    std::string parameter_source;
    bool json_only{false};
  };
  struct Buffer {
    std::vector<std::vector<float>> parameters;
  };
  WbcLoraAssets(const std::string& manifest, const std::vector<std::string>& selected);
  const Action& action(const std::string& id) const { return actions_.at(id); }
  const std::string& modelPath() const { return model_path_; }
  std::vector<std::string> actionIds() const;
  std::string referencePath(const Action& action) const;
  void bindModel(const InferenceModel& model);
  void allocate(Buffer& buffer) const;
  void loadAdapter(const Action& action, Buffer& buffer) const;
  // Complete parameter mapping, used only by prepare; no device ownership here.
  std::vector<InferenceModel::ParameterInput> parameterInputs(const Buffer& buffer) const;

 private:
  struct Tensor {
    std::string name, actor_key;
    std::vector<std::size_t> shape;
    std::size_t elements{};
    int input_index{-1};
  };
  YAML::Node graph_;
  std::string root_, model_path_;
  std::vector<Tensor> tensors_;
  std::map<std::string, Action> actions_;
};
