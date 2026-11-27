#pragma once
#ifdef USE_RKNN
#include "TextMotionRknn.h"
namespace legged { namespace motion {
using TextInferenceApi = TextRknnApi;
using TextInferenceSession = TextRknnSession;
constexpr auto kTextInt32 = RKNN_TENSOR_INT32;
constexpr auto kTextFloat32 = RKNN_TENSOR_FLOAT32;
constexpr const char* kTextBackend = "rknn-c-api";
} }
#else
#include "TextMotionOnnx.h"
namespace legged { namespace motion {
using TextInferenceApi = TextOnnxApi;
using TextInferenceSession = TextOnnxSession;
const auto kTextInt32 = ov::element::i32;
const auto kTextFloat32 = ov::element::f32;
constexpr const char* kTextBackend = "openvino-cpu-onnx";
} }
#endif
