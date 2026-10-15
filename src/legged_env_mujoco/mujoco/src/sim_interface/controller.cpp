#include "sim_interface/controller.h"

#include "main.h"

extern mjModel* m;

void mycontroller(const mjModel* m, mjData* d) {
  pthread_mutex_lock(&recvMutex_);
  legged_msgs::msg::LowCmd currentCmd = recvCmd;
  pthread_mutex_unlock(&recvMutex_);

  for (size_t i = 0; i < m->nu; i++)
  {
    double ff_tau = currentCmd.motor_cmd[i].tau;
    double kp = currentCmd.motor_cmd[i].kp;
    double kd = currentCmd.motor_cmd[i].kd;
    double joint_pos = currentCmd.motor_cmd[i].q;
    double joint_vel = currentCmd.motor_cmd[i].dq;
    d->ctrl[i] = ff_tau + kp * (joint_pos - d->qpos[7 + i]) + kd * (joint_vel - d->qvel[6 + i]);
  }
}

void init_cmd(mjData* d) {
  for (size_t i = 0; i < m->nu; i++)
  {
    recvCmd.motor_cmd[i].tau = 0;
    recvCmd.motor_cmd[i].kp = 0;
    recvCmd.motor_cmd[i].kd = 0;
    recvCmd.motor_cmd[i].q = 0;
    recvCmd.motor_cmd[i].dq = 0;
    d->ctrl[i] = 0;
  }
}
