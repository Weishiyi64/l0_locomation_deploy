// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "main.h"

#include <mujoco/mujoco.h>

#include <memory>
#include <stdexcept>
#include <thread>

#include "glfw_adapter.h"
#include "sim_interface/physics_loop.h"
#include "sim_interface/plugin_loader.h"
#include "simulate.h"
// #include <ros/ros.h>

// Global variable definitions
mjModel* m = nullptr;
mjData* d = nullptr;

std::vector<SimJointCommand> recvCmd;
std::vector<bool> motorEnabled;
std::unique_ptr<system_command_arbiter::MotorMapping> motorMapping;
int64_t commandValidUntilNs = 0;
int64_t commandReceivedSteadyNs = 0;
int64_t watchdogNs = 100000000LL;
uint64_t lastSequenceId = 0;
bool commandReceived = false;
uint64_t stateSequenceId = 0;
pthread_mutex_t recvMutex_;
pthread_mutex_t sendMutex_;

// Constants
// 1. 同步误差容忍度
const double syncMisalign = 0.1;
// 含义：当 CPU 时间和仿真时间差异超过 0.1 秒时，重新同步
// 影响：
//   - 值越大：容忍更大的时间偏差，less频繁重同步
//   - 值越小：更严格的实时性，more频繁重同步
// 推荐：0.05 - 0.2 秒
// 调整建议：
//   - 如果看到频繁的时间跳变 → 增大此值
//   - 如果需要更严格的实时性 → 减小此值

// 2. 物理循环时间分配
const double simRefreshFraction = 0.7;
// 含义：每个刷新周期中，物理循环最多占用 70% 时间
// 计算：refreshTime = 0.7 / refresh_rate
//       例如 60Hz → 0.7/60 = 11.7ms 给物理循环
// 影响：
//   - 值越大：物理可执行更多步，但可能阻塞渲染
//   - 值越小：渲染更流畅，但物理步数受限
// 推荐：0.6 - 0.8
// 调整建议：
//   - 如果渲染卡顿 → 减小此值 (如 0.5-0.6)
//   - 如果需要更高物理频率 → 增大此值 (如 0.8-0.9)
//   - 无头模式可设为 1.0

// 3. 错误消息缓冲区大小
const int kErrorLength = 1024;
// 含义：用于存储模型加载和仿真错误信息的缓冲区大小（字节）
// 用途：
//   - 模型加载失败时的错误描述（如 XML 解析错误）
//   - 物理引擎运行时错误信息
//   - 插件加载错误提示
// 影响：
//   - 值越大：可以捕获更详细的错误信息，但占用更多内存
//   - 值越小：节省内存，但可能截断长错误消息
// 推荐：512 - 2048 字节
// 调整建议：
//   - 如果错误信息经常被截断（显示不完整）→ 增大此值（如 2048）
//   - 如果内存受限且错误信息足够 → 可减小（如 512）
//   - 通常默认 1024 字节足够显示完整的错误堆栈
// 注意：这是 MuJoCo 内部使用的缓冲区，不影响仿真性能

std::vector<int> simActuator, simQpos, simDof;

int main(int argc, char** argv) {
  // ros::init(argc, argv, "mujoco_simulate");
  rclcpp::init(argc, argv);

  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &cam, &opt, &pert,
                                                /* is_passive = */ false);

  const char *filename = nullptr, *body_name = nullptr, *mapping_file = nullptr;
  if (argc > 1)
  {
    filename = argv[1];
  }
  if (argc > 2)
  {
    body_name = argv[2];
  }
  if (argc > 3) mapping_file = argv[3];
  if (mapping_file == nullptr)
  {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "A motor mapping file is required");
    return 1;
  }
  try
  {
    motorMapping = std::make_unique<system_command_arbiter::MotorMapping>(mapping_file);
  } catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Failed to load motor mapping: %s", error.what());
    return 1;
  }

  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename, body_name);
  sim->RenderLoop();
  physicsthreadhandle.join();

  rclcpp::shutdown();
  return 0;
}
