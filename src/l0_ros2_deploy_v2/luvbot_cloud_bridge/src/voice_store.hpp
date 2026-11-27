#pragma once

#include "voice_validation.hpp"
#include "users_store.hpp"

namespace cloud {

constexpr const char* kVoiceUsersPath = "/userdata/forlinx/config/voice_db/users.json";

inline void saveVoice(const Params& params) {
  saveUsers(params, kVoiceUsersPath, voiceParameters);
}

}  // namespace cloud
