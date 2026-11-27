#pragma once

#include "face_validation.hpp"
#include "users_store.hpp"

namespace cloud {

constexpr const char* kFaceUsersPath = "/userdata/forlinx/config/face_db/users.json";

inline void saveFace(const Params& params) {
  saveUsers(params, kFaceUsersPath, faceParameters);
}

}  // namespace cloud
