#pragma once

#include "feature_validation.hpp"

namespace cloud {

inline Params faceParameters(const Params& params) {
  required(params, "memberId");
  if (required(params, "faceDim") != "512" || required(params, "faceEmotionDim") != "5")
    throw std::invalid_argument("face requires 512 dimensions and emotion requires 5 dimensions");
  validateVector(required(params, "faceFeature"), 512, true);
  validateVector(required(params, "faceEmotion"), 5, false);
  return {{"memberId", required(params, "memberId")},
          {"faceFeature", required(params, "faceFeature")},
          {"faceDim", required(params, "faceDim")},
          {"faceEmotion", required(params, "faceEmotion")},
          {"faceEmotionDim", required(params, "faceEmotionDim")}};
}

}  // namespace cloud
