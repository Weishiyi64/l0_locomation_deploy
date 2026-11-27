#include "sim_interface/physics_loop.h"

#include <mujoco/mujoco.h>

#include "main.h"
#include "sim_interface/communication.h"
#include "sim_interface/controller.h"
#include "sim_interface/model_loader.h"
#include "simulate/array_safety.h"
#include "simulate/simulate.h"
// #include <ros/ros.h>
#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using Seconds = std::chrono::duration<double>;

bool validateModelMapping(const mjModel* model, std::vector<SimJointAddress>& addresses) {
  if (!model || !motorMapping || motorMapping->size() != static_cast<std::size_t>(model->nu))
  {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Motor mapping size (%zu) does not match MuJoCo actuators (%d)",
                 motorMapping ? motorMapping->size() : 0U, model ? model->nu : 0);
    return false;
  }
  if (motorMapping->size() > 32U)
  {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Motor mapping exceeds the 32-bit feedback mask");
    return false;
  }
  if (model->nq != model->nu + 7 || model->nv != model->nu + 6 || model->njnt != model->nu + 1 ||
      model->jnt_type[0] != mjJNT_FREE || model->jnt_qposadr[0] != 0 || model->jnt_dofadr[0] != 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco"),
                 "The simulator bridge requires one free base and one scalar joint per actuator "
                 "(nq=%d nv=%d nu=%d)",
                 model->nq, model->nv, model->nu);
    return false;
  }
  std::vector<SimJointAddress> next(motorMapping->size());
  for (int i = 0; i < model->nu; ++i)
  {
    if (model->actuator_trntype[i] != mjTRN_JOINT)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Actuator %d is not a joint transmission", i);
      return false;
    }
    const int joint_id = model->actuator_trnid[2 * i];
    const char* joint_name = mj_id2name(model, mjOBJ_JOINT, joint_id);
    const int index = joint_name ? motorMapping->indexOfJoint(joint_name) : -1;
    if (index < 0 || next[static_cast<std::size_t>(index)].actuator >= 0 ||
        (model->jnt_type[joint_id] != mjJNT_HINGE && model->jnt_type[joint_id] != mjJNT_SLIDE))
    {
      RCLCPP_FATAL(rclcpp::get_logger("mujoco"),
                   "Actuator %d targets unknown, duplicate, or non-scalar mapping joint '%s'", i,
                   joint_name ? joint_name : "<unnamed>");
      return false;
    }
    auto& address = next[static_cast<std::size_t>(index)];
    address.actuator = i;
    address.qpos = model->jnt_qposadr[joint_id];
    address.dof = model->jnt_dofadr[joint_id];
  }
  addresses = std::move(next);
  return true;
}
}  // namespace

extern const double syncMisalign;
extern const double simRefreshFraction;

void PhysicsLoop(mujoco::Simulate& sim, const char* body_name) {
  std::chrono::time_point<mujoco::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  pthread_mutex_init(&recvMutex_, NULL);
  pthread_mutex_init(&sendMutex_, NULL);

  auto node = rclcpp::Node::make_shared("mujoco_simulate");
  node->declare_parameter<double>("command_watchdog_ms", 100.0);
  const double watchdog_ms = node->get_parameter("command_watchdog_ms").as_double();
  if (!std::isfinite(watchdog_ms) || watchdog_ms <= 0.0 ||
      watchdog_ms >= static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e6)
    throw std::runtime_error("command_watchdog_ms must be finite, positive, and fit in nanoseconds");
  watchdogNs = static_cast<int64_t>(std::max(1.0, watchdog_ms) * 1e6);

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto state_qos = rclcpp::SensorDataQoS();
  auto cmd_subscriber = node->create_subscription<luvbot_hardware_interfaces::msg::MotorCommand>(
      "actuator/motor/command", command_qos, HandleMotorCommand);
  auto enable_subscriber = node->create_subscription<luvbot_hardware_interfaces::msg::MotorEnableCommand>(
      "actuator/motor/enable", command_qos, HandleMotorEnable);
  auto motor_publisher =
      node->create_publisher<luvbot_hardware_interfaces::msg::MotorStateArray>("actuator/motor/state", state_qos);
  auto imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
  auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("odom", state_qos);

  // Use executor so we can cancel() before joining the thread to avoid
  // std::terminate() when PhysicsLoop returns with a still-joinable thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner_thread([&executor]() { executor.spin(); });

  mjcb_control = mycontroller;
  sim.run = 1;

  while (!sim.exitrequest.load())
  {
    if (sim.droploadrequest.load())
    {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      std::vector<SimJointAddress> new_addresses;
      if (mnew && validateModelMapping(mnew, new_addresses)) dnew = mj_makeData(mnew);
      if (dnew)
      {
        sim.Load(mnew, dnew, sim.dropfilename);
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        pthread_mutex_lock(&recvMutex_);
        simJointAddresses = std::move(new_addresses);
        init_cmd(d);
        pthread_mutex_unlock(&recvMutex_);
        LoadInitKeyframeIfAny(m, d);
      }
      else
      {
        mj_deleteModel(mnew);
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load())
    {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      std::vector<SimJointAddress> new_addresses;
      if (mnew && validateModelMapping(mnew, new_addresses)) dnew = mj_makeData(mnew);
      if (dnew)
      {
        sim.Load(mnew, dnew, sim.filename);
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        pthread_mutex_lock(&recvMutex_);
        simJointAddresses = std::move(new_addresses);
        init_cmd(d);
        pthread_mutex_unlock(&recvMutex_);
        LoadInitKeyframeIfAny(m, d);
      }
      else
      {
        mj_deleteModel(mnew);
        sim.LoadMessageClear();
      }
    }

    if (sim.run && sim.busywait)
    {
      std::this_thread::yield();
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      if (m)
      {
        if (sim.run)
        {
          bool stepped = false;
          const auto startCPU = mujoco::Simulate::Clock::now();
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];
          bool misaligned = mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim.speed_changed)
          {
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            mj_step(m, d);
            PublishState(m, d, body_name, motor_publisher, imu_publisher, odom_publisher);
            stepped = true;
          }
          else
          {
            bool measured = false;
            mjtNum prevSim = d->time;
            double refreshTime = simRefreshFraction / sim.refresh_rate;

            while (Seconds((d->time - syncSim) * slowdown) < mujoco::Simulate::Clock::now() - syncCPU &&
                   mujoco::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
            {
              if (!measured && elapsedSim)
              {
                sim.measured_slowdown = std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              mj_step(m, d);
              PublishState(m, d, body_name, motor_publisher, imu_publisher, odom_publisher);
              stepped = true;

              if (d->time < prevSim)
              {
                break;
              }
            }
          }

          if (stepped)
          {
            sim.AddToHistory();
          }
        }
        else
        {
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }
  }

  // Stop the executor so the spinner thread can exit cleanly.
  // Without this, spinner_thread destructor is called on a joinable thread,
  // which triggers std::terminate().
  executor.cancel();
  if (spinner_thread.joinable())
  {
    spinner_thread.join();
  }
}

void PhysicsThread(mujoco::Simulate* sim, const char* filename, const char* body_name) {
  RCLCPP_INFO(rclcpp::get_logger("mujoco"), "Loading MuJoCo scene: %s", filename ? filename : "<none>");
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) d = mj_makeData(m);
    if (d)
    {
      if (!validateModelMapping(m, simJointAddresses))
      {
        sim->LoadMessageClear();
        sim->exitrequest.store(1);
        mj_deleteData(d);
        mj_deleteModel(m);
        d = nullptr;
        m = nullptr;
        return;
      }
      recvCmd.resize(m->nu);
      motorEnabled.assign(m->nu, false);
      motorCommandFault.assign(m->nu, false);
      init_cmd(d);
      sim->Load(m, d, filename);
      LoadInitKeyframeIfAny(m, d);
      RCLCPP_INFO(rclcpp::get_logger("mujoco"),
                  "MuJoCo model loaded: %d actuators, %zu enabled motors; joints bound by mapping names", m->nu,
                  motorMapping->enabledSize());

      if (body_name != nullptr)
      {
        int body_id = mj_name2id(m, mjOBJ_BODY, body_name);
        if (body_id > 0)
        {
          sim->cam.type = mjCAMERA_TRACKING;
          sim->cam.trackbodyid = body_id;
          sim->cam.distance = 3.0;
          sim->cam.elevation = -20.0;
          sim->cam.azimuth = -45.0;
          std::printf("Camera set to track body '%s' (ID: %d)\n", body_name, body_id);
        }
        else
        {
          std::printf("Warning: Body '%s' not found, using default camera\n", body_name);
        }
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("mujoco"), "Could not load MuJoCo scene: %s", filename);
      sim->LoadMessageClear();
      sim->exitrequest.store(1);
      mj_deleteModel(m);
      m = nullptr;
      return;
    }
  }

  PhysicsLoop(*sim, body_name);

  pthread_mutex_destroy(&recvMutex_);
  pthread_mutex_destroy(&sendMutex_);

  mj_deleteData(d);
  mj_deleteModel(m);
}
