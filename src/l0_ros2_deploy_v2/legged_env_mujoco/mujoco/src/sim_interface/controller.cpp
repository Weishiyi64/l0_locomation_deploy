#include "sim_interface/controller.h"

#include <algorithm>
#include <chrono>

#include "main.h"
#include "sim_interface/communication.h"

extern mjModel* m;

void mycontroller(const mjModel* m, mjData* d) {
  const int64_t now_steady_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  pthread_mutex_lock(&recvMutex_);
  ExpireMotorCommand(now_steady_ns);
  std::fill(d->ctrl, d->ctrl + m->nu, 0.0);

  for (std::size_t i = 0; i < simJointAddresses.size(); ++i)
  {
    if (!commandReceived || !motorEnabled[i] || motorCommandFault[i])
      continue;
    const auto& address = simJointAddresses[i];
    const double ff_tau = recvCmd[i].tau;
    const double kp = recvCmd[i].kp;
    const double kd = recvCmd[i].kd;
    const double joint_pos = recvCmd[i].q;
    const double joint_vel = recvCmd[i].dq;
    d->ctrl[address.actuator] =
        ff_tau + kp * (joint_pos - d->qpos[address.qpos]) + kd * (joint_vel - d->qvel[address.dof]);
  }
  pthread_mutex_unlock(&recvMutex_);
}

void init_cmd(mjData* d) {
  // Called before callbacks start, or with recvMutex_ held during model reload.
  for (std::size_t i = 0; i < recvCmd.size(); ++i)
  {
    recvCmd[i] = SimJointCommand{};
    motorCommandFault[i] = motorCommandFault[i] || motorEnabled[i];
    motorEnabled[i] = false;
  }
  commandReceived = false;
  commandWatchdogArmed = false;
  std::fill(d->ctrl, d->ctrl + m->nu, 0.0);
}
