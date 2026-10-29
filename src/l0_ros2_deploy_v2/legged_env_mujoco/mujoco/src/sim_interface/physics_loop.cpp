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
#include <luvbot_hardware_interfaces/srv/set_motor_enabled.hpp>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <string>
#include <thread>

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using Seconds = std::chrono::duration<double>;

bool validateModelMapping(const mjModel* model) {
  if (!model || !motorMapping || motorMapping->size() != static_cast<std::size_t>(model->nu))
  {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Motor mapping size (%zu) does not match MuJoCo actuators (%d)",
                 motorMapping ? motorMapping->size() : 0U, model ? model->nu : 0);
    return false;
  }
  if (model->nq != model->nu + 7 || model->nv != model->nu + 6)
  {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco"),
                 "The simulator bridge requires one free base and one scalar joint per actuator "
                 "(nq=%d nv=%d nu=%d)",
                 model->nq, model->nv, model->nu);
    return false;
  }
  for (int i = 0; i < model->nu; ++i)
  {
    if (model->actuator_trntype[i] != mjTRN_JOINT)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Actuator %d is not a joint transmission", i);
      return false;
    }
    const int joint_id = model->actuator_trnid[2 * i];
    const char* joint_name = mj_id2name(model, mjOBJ_JOINT, joint_id);
    const std::string expected = motorMapping->entries()[static_cast<std::size_t>(i)].joint_name;
    if (!joint_name || expected != joint_name)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Actuator %d targets joint '%s', but mapping entry %d is '%s'", i,
                   joint_name ? joint_name : "<unnamed>", i, expected.c_str());
      return false;
    }
  }
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
  watchdogNs = static_cast<int64_t>(std::max(1.0, node->get_parameter("command_watchdog_ms").as_double()) * 1e6);

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
  auto cmd_subscriber = node->create_subscription<luvbot_hardware_interfaces::msg::MotorCommand>(
      "actuator/motor/command", command_qos, HandleMotorCommand);
  auto motor_publisher =
      node->create_publisher<luvbot_hardware_interfaces::msg::MotorStateArray>("actuator/motor/state", state_qos);
  auto imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
  auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("odom", state_qos);
  auto enable_service = node->create_service<luvbot_hardware_interfaces::srv::SetMotorEnabled>(
      "actuator/motor/set_enabled",
      [](const std::shared_ptr<luvbot_hardware_interfaces::srv::SetMotorEnabled::Request> request,
         std::shared_ptr<luvbot_hardware_interfaces::srv::SetMotorEnabled::Response> response) {
        const int index = motorMapping ? motorMapping->indexOfRoute(request->motor) : -1;
        if (index < 0)
        {
          response->success = false;
          response->error_code = response->ERROR_INVALID_ROUTE;
          response->diagnostic_message = "unknown motor route";
          return;
        }
        pthread_mutex_lock(&recvMutex_);
        motorEnabled[static_cast<std::size_t>(index)] = request->enabled;
        if (!request->enabled) recvCmd[static_cast<std::size_t>(index)] = SimJointCommand{};
        pthread_mutex_unlock(&recvMutex_);
        response->success = true;
        response->error_code = response->ERROR_NONE;
        response->diagnostic_message = request->enabled ? "enabled" : "disabled";
      });

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
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew)
      {
        sim.Load(mnew, dnew, sim.dropfilename);
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        LoadInitKeyframeIfAny(m, d);
      }
      else
      {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load())
    {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew)
      {
        sim.Load(mnew, dnew, sim.filename);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        init_cmd(d);
        LoadInitKeyframeIfAny(m, d);
      }
      else
      {
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
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) d = mj_makeData(m);
    if (d)
    {
      if (!validateModelMapping(m))
      {
        sim->LoadMessageClear();
        return;
      }
      recvCmd.resize(m->nu);
      motorEnabled.assign(m->nu, false);
      init_cmd(d);
      sim->Load(m, d, filename);
      LoadInitKeyframeIfAny(m, d);

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
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim, body_name);

  pthread_mutex_destroy(&recvMutex_);
  pthread_mutex_destroy(&sendMutex_);

  mj_deleteData(d);
  mj_deleteModel(m);
}
