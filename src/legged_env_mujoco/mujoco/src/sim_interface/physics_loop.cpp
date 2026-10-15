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

#include <chrono>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using Seconds = std::chrono::duration<double>;
}  // namespace

extern const double syncMisalign;
extern const double simRefreshFraction;

void PhysicsLoop(mujoco::Simulate& sim, const char* body_name) {
  std::chrono::time_point<mujoco::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  pthread_mutex_init(&recvMutex_, NULL);
  pthread_mutex_init(&sendMutex_, NULL);

  /*
  ros::NodeHandle nh;
  ros::Subscriber cmdSubscriber_ = nh.subscribe("/LOWCMD", 100, HandleLowCmd,
  ros::TransportHints().unreliable()); ros::Publisher statePublisher_ =
  nh.advertise<legged_msgs::LowState>("/LOWSTATE", 100);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  */
  auto node = rclcpp::Node::make_shared("mujoco_simulate");

  // QoS: unreliable
  rclcpp::QoS qos_unreliable(100);
  qos_unreliable.best_effort();

  // Subscriber
  auto cmdSubscriber_ = node->create_subscription<legged_msgs::msg::LowCmd>("/LOWCMD", qos_unreliable, HandleLowCmd);

  // Publisher
  auto statePublisher_ = node->create_publisher<legged_msgs::msg::LowState>("/LOWSTATE", rclcpp::QoS(100));

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
            PublishState(m, d, body_name, statePublisher_);
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
              PublishState(m, d, body_name, statePublisher_);
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
      recvCmd.motor_cmd.resize(m->nu);
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
