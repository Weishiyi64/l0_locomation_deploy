#include "sim_interface/controller.h"

#include <algorithm>
#include <chrono>

#include "main.h"

extern mjModel* m;

void mycontroller(const mjModel* m, mjData* d) {
  (void)m;
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

  for (int i = 0; i < static_cast<int>(mappingToActuator.size()); i++)
  {
    const int actuator = mappingToActuator[static_cast<std::size_t>(i)];
    const int qpos = mappingToQpos[static_cast<std::size_t>(i)];
    const int dof = mappingToDof[static_cast<std::size_t>(i)];
    if (!fresh || i >= static_cast<int>(current_cmd.size()) || i >= static_cast<int>(enabled.size()) || !enabled[i])
    {
      d->ctrl[actuator] = 0.0;
      continue;
    }
    const double ff_tau = current_cmd[i].tau;
    const double kp = current_cmd[i].kp;
    const double kd = current_cmd[i].kd;
    const double joint_pos = current_cmd[i].q;
    const double joint_vel = current_cmd[i].dq;
    d->ctrl[actuator] = ff_tau + kp * (joint_pos - d->qpos[qpos]) + kd * (joint_vel - d->qvel[dof]);
  }
}

void init_cmd(mjData* d) {
  for (int i = 0; i < m->nu; i++)
  {
    recvCmd[i] = SimJointCommand{};
    d->ctrl[i] = 0;
  }
}
