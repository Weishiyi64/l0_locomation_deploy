#pragma once
#ifdef USE_WBC_OPENVINO
#include "openvino_wbc_model.h"
using WbcModel = OpenVinoWbcModel;
inline std::string wbcModelPath(const std::string& path) { return path; }
#else
#include "rknn_model.h"
using WbcModel = RknnModel;
inline std::string wbcModelPath(const std::string& path) { return rknnModelPath(path); }
#endif
