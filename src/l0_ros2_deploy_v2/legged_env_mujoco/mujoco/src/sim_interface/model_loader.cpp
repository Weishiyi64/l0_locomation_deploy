#include "sim_interface/model_loader.h"

#include <cstdio>
#include <cstring>

#include "main.h"
#include "simulate/array_safety.h"
#include "simulate/simulate.h"

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

constexpr int kErrorLengthLocal = 1024;
}  // namespace

mjModel* LoadModel(const char* file, mujoco::Simulate& sim) {
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  char filename[mujoco::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  if (!filename[0])
  {
    return nullptr;
  }

  char loadError[kErrorLengthLocal] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename) > 4 && !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                                                     mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
  {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew)
    {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  }
  else
  {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLengthLocal);
    if (loadError[0])
    {
      int error_length = static_cast<int>(std::strlen(loadError));
      if (error_length > 0 && loadError[error_length - 1] == '\n')
      {
        loadError[error_length - 1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew)
  {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  if (loadError[0])
  {
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

void LoadInitKeyframeIfAny(const mjModel* m, mjData* d) {
  if (!m || !d) return;
  if (m->nkey > 0)
  {
    mj_resetDataKeyframe(m, d, 0);
  }
  mj_forward(m, d);
}
