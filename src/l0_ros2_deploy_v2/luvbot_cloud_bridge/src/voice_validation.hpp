#pragma once

#include "feature_validation.hpp"

namespace cloud {

inline Params voiceParameters(const Params& params) {
  required(params, "memberId");
  if (required(params, "voiceDim") != "192")
    throw std::invalid_argument("voice requires 192 dimensions");
  const auto& need_train = required(params, "needTrain");
  if (need_train != "0" && need_train != "1")
    throw std::invalid_argument("needTrain must be 0 or 1");
  const auto task = params.find("taskId");
  const std::string task_id = task == params.end() ? "" : task->second;
  if (need_train == "1" && task_id.empty())
    throw std::invalid_argument("needTrain=1 requires taskId");
  validateVector(required(params, "voiceFeature"), 192, true);
  return {{"memberId", required(params, "memberId")},
          {"taskId", task_id},
          {"needTrain", need_train},
          {"voiceFeature", required(params, "voiceFeature")},
          {"voiceDim", required(params, "voiceDim")}};
}

}  // namespace cloud
