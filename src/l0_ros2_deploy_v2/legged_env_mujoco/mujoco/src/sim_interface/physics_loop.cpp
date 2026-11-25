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
#include <atomic>
#include <chrono>
#include <luvbot_hardware_interfaces/srv/set_motor_enabled.hpp>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <robot_interfaces/msg/motor_enable_command.hpp>
#include <std_srvs/srv/trigger.hpp>
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
  mappingToActuator.assign(motorMapping->size(), -1);
  mappingToQpos.assign(motorMapping->size(), -1);
  mappingToDof.assign(motorMapping->size(), -1);
  for (int actuator = 0; actuator < model->nu; ++actuator)
  {
    if (model->actuator_trntype[actuator] != mjTRN_JOINT)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Actuator %d is not a joint transmission", actuator);
      return false;
    }
    const int joint_id = model->actuator_trnid[2 * actuator];
    const char* joint_name = mj_id2name(model, mjOBJ_JOINT, joint_id);
    const int mapping_index = joint_name ? motorMapping->indexOfJoint(joint_name) : -1;
    if (mapping_index < 0 || model->jnt_type[joint_id] == mjJNT_FREE || model->jnt_type[joint_id] == mjJNT_BALL)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Actuator %d targets unknown or non-scalar joint '%s'", actuator,
                   joint_name ? joint_name : "<unnamed>");
      return false;
    }
    if (mappingToActuator[static_cast<std::size_t>(mapping_index)] >= 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "Multiple actuators target mapped joint '%s'", joint_name);
      return false;
    }
    mappingToActuator[static_cast<std::size_t>(mapping_index)] = actuator;
    mappingToQpos[static_cast<std::size_t>(mapping_index)] = model->jnt_qposadr[joint_id];
    mappingToDof[static_cast<std::size_t>(mapping_index)] = model->jnt_dofadr[joint_id];
  }
  if (std::find(mappingToActuator.begin(), mappingToActuator.end(), -1) != mappingToActuator.end())
  {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco"), "At least one mapped joint has no MuJoCo actuator");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("mujoco"), "Validated %zu mapped joints independent of MJCF actuator order",
              motorMapping->size());
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
  // ROS 2 forbids two types on the same topic within one node.  Keep the
  // compatibility endpoints on a separate node while sharing the simulator state.
  rclcpp::NodeOptions legacy_node_options;
  legacy_node_options.use_global_arguments(false);
  auto legacy_node = rclcpp::Node::make_shared("mujoco_topic_adapter_compat", legacy_node_options);
  node->declare_parameter<double>("command_watchdog_ms", 100.0);
  watchdogNs = static_cast<int64_t>(std::max(1.0, node->get_parameter("command_watchdog_ms").as_double()) * 1e6);
  node->declare_parameter<bool>("manual_start_required", false);
  const bool manual_start_required = node->get_parameter("manual_start_required").as_bool();
  std::atomic<bool> manual_start_requested{false};

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
  auto cmd_subscriber = node->create_subscription<luvbot_hardware_interfaces::msg::MotorCommand>(
      "actuator/motor/command", command_qos, HandleMotorCommand);
  auto legacy_cmd_subscriber = legacy_node->create_subscription<robot_interfaces::msg::MotorCommand>(
      "sim/actuator/motor/command", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      HandleLegacyMotorCommand);
  auto motor_publisher =
      node->create_publisher<luvbot_hardware_interfaces::msg::MotorStateArray>("actuator/motor/state", state_qos);
  auto legacy_motor_publisher = legacy_node->create_publisher<robot_interfaces::msg::MotorStateArray>(
      "sim/actuator/motor/state", rclcpp::SensorDataQoS());
  auto imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
  auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("odom", state_qos);
  std::atomic<int64_t> last_feedback_publish_ns{0};
  bool feedback_gap_reported = false;
  auto feedback_watch_timer = node->create_wall_timer(std::chrono::milliseconds(100), [&]() {
    const int64_t published_ns = last_feedback_publish_ns.load();
    if (published_ns == 0) return;
    const int64_t now_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    const int64_t gap_ms = (now_ns - published_ns) / 1000000;
    if (gap_ms > 100 && !feedback_gap_reported)
    {
      RCLCPP_WARN(node->get_logger(), "MuJoCo feedback publisher stalled for %ld ms", static_cast<long>(gap_ms));
      feedback_gap_reported = true;
    }
    else if (gap_ms <= 100 && feedback_gap_reported)
    {
      RCLCPP_INFO(node->get_logger(), "MuJoCo feedback publisher recovered");
      feedback_gap_reported = false;
    }
  });
  auto publish_feedback = [&]() {
    PublishState(m, d, body_name, motor_publisher, legacy_motor_publisher, imu_publisher, odom_publisher);
    last_feedback_publish_ns.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
  };
  auto start_dynamics_service = node->create_service<std_srvs::srv::Trigger>(
      "/sim/mujoco/start_dynamics",
      [&manual_start_requested](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const int64_t now_ros_ns = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
        const int64_t now_steady_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count();
        pthread_mutex_lock(&recvMutex_);
        const std::size_t enabled_count = static_cast<std::size_t>(
            std::count(motorEnabled.begin(), motorEnabled.end(), true));
        const std::size_t motor_count = motorEnabled.size();
        const bool all_enabled = !motorEnabled.empty() &&
                                 std::all_of(motorEnabled.begin(), motorEnabled.end(), [](bool enabled) { return enabled; });
        const bool fresh_command = commandReceived && now_ros_ns <= commandValidUntilNs &&
                                   now_steady_ns - commandReceivedSteadyNs <= watchdogNs;
        const bool command_seen = commandReceived;
        const int64_t command_age_ms = command_seen ? (now_steady_ns - commandReceivedSteadyNs) / 1000000 : -1;
        const int64_t command_ttl_ms = command_seen ? (commandValidUntilNs - now_ros_ns) / 1000000 : -1;
        pthread_mutex_unlock(&recvMutex_);
        // Accept the operator's request once the motors are enabled. The physics
        // loop still requires a fresh command before the first mj_step; latching
        // the request avoids a one-instant service/command timing race.
        response->success = all_enabled;
        if (all_enabled) manual_start_requested.store(true);
        response->message = std::string(all_enabled
                                            ? (fresh_command ? "Start armed; fresh command available"
                                                             : "Start armed; waiting for next fresh command")
                                            : "Start refused; motors are not all enabled") +
                            "; enabled=" + std::to_string(enabled_count) + "/" + std::to_string(motor_count) +
                            "; command_seen=" + (command_seen ? "true" : "false") +
                            "; command_age_ms=" + std::to_string(command_age_ms) +
                            "; command_ttl_ms=" + std::to_string(command_ttl_ms);
      });
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
  auto legacy_enable_subscriber = legacy_node->create_subscription<robot_interfaces::msg::MotorEnableCommand>(
      "sim/actuator/motor/enable", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      [](const robot_interfaces::msg::MotorEnableCommand::ConstSharedPtr message) {
        if (message->motor_types.size() != message->enabled.size())
        {
          RCLCPP_WARN(rclcpp::get_logger("mujoco"), "Rejected malformed MotorEnableCommand");
          return;
        }
        pthread_mutex_lock(&recvMutex_);
        for (std::size_t i = 0; i < message->motor_types.size(); ++i)
        {
          const std::size_t index = message->motor_types[i];
          if (index >= motorEnabled.size()) continue;
          motorEnabled[index] = message->enabled[i];
          if (!message->enabled[i]) recvCmd[index] = SimJointCommand{};
        }
        pthread_mutex_unlock(&recvMutex_);
      });

  // Use executor so we can cancel() before joining the thread to avoid
  // std::terminate() when PhysicsLoop returns with a still-joinable thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(legacy_node);
  std::thread spinner_thread([&executor]() { executor.spin(); });

  mjcb_control = mycontroller;
  sim.run = 1;
  bool simulation_started = false;
  bool pause_reported = false;

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

    if (!simulation_started)
    {
      const int64_t now_ros_ns = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
      const int64_t now_steady_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count();
      pthread_mutex_lock(&recvMutex_);
      const bool all_enabled = !motorEnabled.empty() && std::all_of(motorEnabled.begin(), motorEnabled.end(),
                                                                    [](bool enabled) { return enabled; });
      std::size_t support_joint_count = 0;
      bool support_gains_ready = recvCmd.size() == motorEnabled.size();
      if (!manual_start_required && motorMapping && support_gains_ready)
      {
        for (std::size_t i = 0; i < motorMapping->size(); ++i)
        {
          const std::string& name = motorMapping->entries()[i].joint_name;
          const bool support_joint = name.find("hip_") != std::string::npos ||
                                     name.find("knee_") != std::string::npos ||
                                     name.find("ankle_") != std::string::npos;
          if (!support_joint) continue;
          ++support_joint_count;
          support_gains_ready = support_gains_ready && recvCmd[i].kp > 1e-6;
        }
      }
      support_gains_ready = support_gains_ready && support_joint_count == 12U;
      const bool fresh_command = commandReceived && now_ros_ns <= commandValidUntilNs &&
                                 now_steady_ns - commandReceivedSteadyNs <= watchdogNs;
      pthread_mutex_unlock(&recvMutex_);
      if ((manual_start_requested.load() || (!manual_start_required && support_gains_ready)) && all_enabled &&
          fresh_command)
      {
        simulation_started = true;
        if (manual_start_requested.load())
          RCLCPP_INFO(node->get_logger(), "Starting MuJoCo dynamics on manual request with all motors enabled and a "
                                          "fresh motor command");
        else
          RCLCPP_INFO(node->get_logger(), "Starting MuJoCo dynamics after all motors are enabled and the first "
                                          "fresh stabilizing command arrived");
      }
    }

    if (sim.run && simulation_started && sim.busywait)
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
        if (simulation_started && !sim.run && !pause_reported)
        {
          RCLCPP_WARN(node->get_logger(), "MuJoCo dynamics paused; publishing static motor feedback until resumed");
          pause_reported = true;
        }
        else if (pause_reported && sim.run)
        {
          RCLCPP_INFO(node->get_logger(), "MuJoCo dynamics resumed");
          pause_reported = false;
        }
        if (sim.run && simulation_started)
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
            publish_feedback();
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
              publish_feedback();
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
          // Before the first complete command, hold the initial pose against gravity while still
          // publishing feedback so the controller can finish its enable/turn-confirmation handshake.
          mj_forward(m, d);
          // The adapter's feedback watchdog keeps running when the MuJoCo UI is
          // paused. Publish the held state so a pause alone cannot latch an
          // emergency stop on every motor at once.
          publish_feedback();
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
