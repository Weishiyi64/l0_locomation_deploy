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
#include <cstdlib>
#include <cmath>
#include <robot_interfaces/msg/motor_enable_command.hpp>
#include <robot_interfaces/srv/motor_set_zero.hpp>
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
  std::vector<int> actuators(model->nu, -1), positions(model->nu, -1), velocities(model->nu, -1);
  for (int i = 0; i < model->nu; ++i) {
    const auto& name = motorMapping->entries()[i].joint_name;
    const int joint = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (joint < 0 || (model->jnt_type[joint] != mjJNT_HINGE && model->jnt_type[joint] != mjJNT_SLIDE)) return false;
    for (int a = 0; a < model->nu; ++a) {
      if (model->actuator_trntype[a] == mjTRN_JOINT && model->actuator_trnid[2 * a] == joint) {
        if (actuators[i] != -1) return false;
        actuators[i] = a;
      }
    }
    if (actuators[i] < 0) return false;
    positions[i] = model->jnt_qposadr[joint];
    velocities[i] = model->jnt_dofadr[joint];
  }
  simActuator = std::move(actuators);
  simQpos = std::move(positions);
  simDof = std::move(velocities);
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

  node->declare_parameter<double>("joint_limit_time_constant", 0.0);
  const double limit_time = node->get_parameter("joint_limit_time_constant").as_double();
  if (!m || !std::isfinite(limit_time) || limit_time < 0 || (limit_time > 0 && limit_time < 2 * m->opt.timestep)) {
    RCLCPP_FATAL(node->get_logger(), "joint_limit_time_constant must be zero (XML default) or >= 2 * physics timestep");
    sim.exitrequest.store(true);
    return;
  }
  if (limit_time > 0) {
    // MuJoCo joint limits are compliant constraints, not hard position clips.
    // Change only their response time; retain ranges, damping ratio and feedback.
    for (int j = 0; j < m->njnt; ++j)
      if (m->jnt_limited[j] && m->jnt_solref[2 * j] > 0)
        m->jnt_solref[2 * j] = limit_time;
    RCLCPP_INFO(node->get_logger(), "MuJoCo joint limit constraint time constant %.6f s; angle ranges unchanged", limit_time);
  }

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
  auto cmd_subscriber = node->create_subscription<robot_interfaces::msg::MotorCommand>(
      "actuator/motor/command", command_qos, HandleMotorCommand);
  auto motor_publisher =
      node->create_publisher<robot_interfaces::msg::MotorStateArray>("actuator/motor/state", state_qos);
  auto imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
  auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("odom", state_qos);
  rclcpp::Publisher<luvbot_control_interfaces::msg::SimSupportState>::SharedPtr support_pub;
  const char* trial = std::getenv("L0_MUJOCO_STOP_PROFILE");
  if (trial && *trial) {
    trialAppliedTargets.positions.resize(m->nu);
    support_pub = node->create_publisher<luvbot_control_interfaces::msg::SimSupportState>(
        "experiment/stop_switch/support", state_qos);
  }
  auto enable_subscriber = node->create_subscription<robot_interfaces::msg::MotorEnableCommand>(
      "actuator/motor/enable", command_qos,
      [](const robot_interfaces::msg::MotorEnableCommand::ConstSharedPtr msg) {
        if (msg->motor_types.size() != msg->enabled.size()) return;
        std::vector<bool> seen(motorEnabled.size(), false);
        for (auto index : msg->motor_types) {
          if (index >= seen.size() || seen[index]) return;
          seen[index] = true;
        }
        pthread_mutex_lock(&recvMutex_);
        for (std::size_t i = 0; i < msg->motor_types.size(); ++i) {
          const auto index = msg->motor_types[i];
          motorEnabled[index] = msg->enabled[i];
          if (!msg->enabled[i]) recvCmd[index] = SimJointCommand{};
        }
        pthread_mutex_unlock(&recvMutex_);
      });
  auto zero_service = node->create_service<robot_interfaces::srv::MotorSetZero>(
      "actuator/motor/set_zero",
      [](const std::shared_ptr<robot_interfaces::srv::MotorSetZero::Request> req,
         std::shared_ptr<robot_interfaces::srv::MotorSetZero::Response> res) {
        res->motor_types = req->motor_types;
        res->results.assign(req->motor_types.size(), res->REJECTED);
        res->messages.assign(req->motor_types.size(), "MuJoCo does not perform motor calibration");
      });

  // Use executor so we can cancel() before joining the thread to avoid
  // std::terminate() when PhysicsLoop returns with a still-joinable thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner_thread([&executor]() { executor.spin(); });

  mjcb_control = mycontroller;
  sim.run = 1;
  bool physics_released = false;

  while (!sim.exitrequest.load())
  {
    if (sim.droploadrequest.load())
    {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew && !validateModelMapping(mnew)) {
        mj_deleteData(dnew);
        mj_deleteModel(mnew);
        dnew = nullptr;
      }
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
      if (dnew && !validateModelMapping(mnew)) {
        mj_deleteData(dnew);
        mj_deleteModel(mnew);
        dnew = nullptr;
      }
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
        if (!physics_released) {
          pthread_mutex_lock(&recvMutex_);
          physics_released = !motorEnabled.empty() && std::all_of(motorEnabled.begin(), motorEnabled.end(), [](bool v) { return v; });
          pthread_mutex_unlock(&recvMutex_);
          // Startup support only: publish the actual initial pose until the
          // controller enables motors. Never freeze physics after a fault.
          if (!physics_released) {
            mj_forward(m, d);
            PublishState(m, d, body_name, motor_publisher, imu_publisher, odom_publisher, support_pub);
            continue;
          }
        }
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
            PublishState(m, d, body_name, motor_publisher, imu_publisher, odom_publisher, support_pub);
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
              PublishState(m, d, body_name, motor_publisher, imu_publisher, odom_publisher, support_pub);
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
