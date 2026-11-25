#pragma once

#include <mujoco/mujoco.h>

namespace mujoco {
class Simulate;
}

mjModel* LoadModel(const char* file, mujoco::Simulate& sim);
void LoadInitKeyframeIfAny(const mjModel* m, mjData* d);
