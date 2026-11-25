#pragma once

#include <mujoco/mujoco.h>

#include <vector>

// Written by mjcb_control and read by PublishState on the same physics thread.
struct SimAppliedTargets {
  bool valid{false};
  std::vector<double> positions;
};
extern thread_local SimAppliedTargets trialAppliedTargets;

void mycontroller(const mjModel* m, mjData* d);
void init_cmd(mjData* d);
