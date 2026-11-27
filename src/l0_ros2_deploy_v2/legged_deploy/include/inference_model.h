#pragma once
#include <memory>

#ifdef USE_RKNN
#include "rknn_model.h"
using InferenceModel = RknnModel;
inline std::string inferenceModelPath(const std::string& path) { return rknnModelPath(path); }
inline const char* inferenceAssetKey() { return "rknn"; }
inline std::unique_ptr<InferenceModel> loadPlannerModel(const std::string& path) {
  return std::make_unique<InferenceModel>(inferenceModelPath(path));
}
inline bool isFloatingTensor(const InferenceModel& model, std::size_t i, bool input) {
  const auto type = input ? model.inputAttribute(i).type : model.outputAttribute(i).type;
  return type == RKNN_TENSOR_FLOAT32 || type == RKNN_TENSOR_FLOAT16;
}
inline std::vector<std::size_t> inferenceOutputShape(const InferenceModel& model, std::size_t i) {
  const auto& attr = model.outputAttribute(i);
  return {attr.dims, attr.dims + attr.n_dims};
}
#else
#include "onnx_model.h"
using InferenceModel = OnnxModel;
inline std::string inferenceModelPath(const std::string& path) { return path; }
inline const char* inferenceAssetKey() { return "onnx"; }
inline std::unique_ptr<InferenceModel> loadPlannerModel(const std::string& path) {
  return std::make_unique<InferenceModel>(path, InferenceModel::InputMode::Planner);
}
inline bool isFloatingTensor(const InferenceModel&, std::size_t, bool) { return true; }
inline std::vector<std::size_t> inferenceOutputShape(const InferenceModel& model, std::size_t i) {
  return model.outputShape(i);
}
#endif
inline bool isFloatingInput(const InferenceModel& model, std::size_t i) {
  return isFloatingTensor(model, i, true);
}
