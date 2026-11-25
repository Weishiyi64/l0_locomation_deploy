#include "sim_interface/controller.h"

#include <algorithm>
#include <chrono>

#include "main.h"

extern mjModel* m;
thread_local SimAppliedTargets trialAppliedTargets;

void mycontroller(const mjModel* m, mjData* d) {
  const int64_t now_ros_ns = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
  const int64_t now_steady_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  pthread_mutex_lock(&recvMutex_);
  const bool fresh =
      commandReceived && now_ros_ns <= commandValidUntilNs && now_steady_ns - commandReceivedSteadyNs <= watchdogNs;
  if (commandReceived && !fresh)
  {
    std::fill(motorEnabled.begin(), motorEnabled.end(), false);
    std::fill(recvCmd.begin(), recvCmd.end(), SimJointCommand{});
    commandReceived = false;
  }
  const std::vector<SimJointCommand> current_cmd = recvCmd;
  const std::vector<bool> enabled = motorEnabled;
  pthread_mutex_unlock(&recvMutex_);

  if (!trialAppliedTargets.positions.empty()) {
    trialAppliedTargets.valid = fresh && current_cmd.size() == trialAppliedTargets.positions.size() &&
        enabled.size() == current_cmd.size() && std::all_of(enabled.begin(), enabled.end(), [](bool value) { return value; });
    if (trialAppliedTargets.valid)
      for (size_t j = 0; j < current_cmd.size(); ++j) trialAppliedTargets.positions[j] = current_cmd[j].q;
  }

  for (int i = 0; i < m->nu; i++)
  {
    if (!fresh || i >= static_cast<int>(current_cmd.size()) || i >= static_cast<int>(enabled.size()) || !enabled[i])
    {
      d->ctrl[simActuator[i]] = 0.0;
      continue;
    }
    const double ff_tau = current_cmd[i].tau;
    const double kp = current_cmd[i].kp;
    const double kd = current_cmd[i].kd;
    const double joint_pos = current_cmd[i].q;
    const double joint_vel = current_cmd[i].dq;
    d->ctrl[simActuator[i]] = ff_tau + kp * (joint_pos - d->qpos[simQpos[i]]) + kd * (joint_vel - d->qvel[simDof[i]]);
  }
}

void init_cmd(mjData* d) {
  for (int i = 0; i < m->nu; i++)
  {
    recvCmd[i] = SimJointCommand{};
    d->ctrl[i] = 0;
  }
}
