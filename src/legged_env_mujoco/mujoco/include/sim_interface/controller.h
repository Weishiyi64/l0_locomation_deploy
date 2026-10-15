#pragma once

#include <mujoco/mujoco.h>

void mycontroller(const mjModel* m, mjData* d);
void init_cmd(mjData* d);
