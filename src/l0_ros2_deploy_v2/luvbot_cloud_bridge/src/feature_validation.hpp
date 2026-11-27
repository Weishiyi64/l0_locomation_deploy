#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "luvbot.pb.h"

namespace cloud {

using Params = std::map<std::string, std::string>;

inline Params parameters(const luvbot::v1::Command& command) {
  Params result;
  for (const auto& param : command.params())
    if (!result.emplace(param.key(), param.value()).second)
      throw std::invalid_argument("duplicate parameter: " + param.key());
  return result;
}

inline const std::string& required(const Params& params, const std::string& key) {
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) throw std::invalid_argument("missing/empty " + key);
  return it->second;
}

inline void validateVector(const std::string& text, std::size_t dimensions, bool require_nonzero) {
  static const std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const std::size_t bytes_expected = dimensions * 4;
  if (text.size() != ((bytes_expected + 2) / 3) * 4)
    throw std::invalid_argument("incorrect base64 vector length");
  std::vector<uint8_t> bytes;
  bytes.reserve(bytes_expected + 2);
  for (std::size_t i = 0; i < text.size(); i += 4)
  {
    unsigned values[4]{};
    unsigned padding = 0;
    for (unsigned j = 0; j < 4; ++j)
    {
      if (text[i + j] == '=')
      {
        if (i + 4 != text.size() || j < 2) throw std::invalid_argument("invalid base64 padding");
        ++padding;
      }
      else
      {
        const auto value = alphabet.find(text[i + j]);
        if (padding || value == std::string::npos) throw std::invalid_argument("invalid base64 character");
        values[j] = static_cast<unsigned>(value);
      }
    }
    if ((padding == 1 && (values[2] & 3U)) || (padding == 2 && (values[1] & 15U)))
      throw std::invalid_argument("noncanonical base64 padding bits");
    const uint32_t word = (values[0] << 18U) | (values[1] << 12U) | (values[2] << 6U) | values[3];
    bytes.push_back(static_cast<uint8_t>(word >> 16U));
    if (padding < 2) bytes.push_back(static_cast<uint8_t>(word >> 8U));
    if (padding == 0) bytes.push_back(static_cast<uint8_t>(word));
  }
  if (bytes.size() != bytes_expected) throw std::invalid_argument("incorrect decoded vector length");
  bool nonzero = false;
  static_assert(sizeof(float) == 4, "features require float32");
  for (std::size_t i = 0; i < bytes.size(); i += 4)
  {
    const uint32_t bits = uint32_t(bytes[i]) | (uint32_t(bytes[i + 1]) << 8U) |
                          (uint32_t(bytes[i + 2]) << 16U) | (uint32_t(bytes[i + 3]) << 24U);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    if (!std::isfinite(value)) throw std::invalid_argument("vector contains NaN/Inf");
    nonzero = nonzero || value != 0.0f;
  }
  if (require_nonzero && !nonzero) throw std::invalid_argument("zero feature vector");
}

}  // namespace cloud
