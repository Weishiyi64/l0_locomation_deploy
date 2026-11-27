// Copyright (c) 2026 HPMicro
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "hpm_motor/motor.hpp"

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

class MotorCanLog {
public:
  MotorCanLog(std::filesystem::path directory, size_t shard_bytes, size_t total_bytes,
              std::vector<hpmMotor::MotorDefinition> motors, rclcpp::Logger logger);
  ~MotorCanLog();
  void tx(const frameTransport::Frame &frame, uint32_t target_id);
  void tx(const frameTransport::Frame &frame, uint32_t target_id, hpmMotor::TxTopic topic);
  void rx(const frameTransport::Frame &frame);
  void snapshot(const frameTransport::MotorSnapshot &snapshot, hpmMotor::HpmMotor *motor);

private:
  struct Published {
    std::string topic;
    luvbot_hardware_interfaces::msg::MotorStateArray message;
  };
  using Topic = std::variant<std::monostate, hpmMotor::TxTopic, std::shared_future<Published>>;
  struct Record {
    frameTransport::Frame frame;
    bool tx;
    uint32_t target_id;
    int slot;
    int64_t unix_us;
    int64_t elapsed_us;
    uint64_t sequence;
    Topic topic;
  };
  void enqueue(const frameTransport::Frame &frame, bool tx, uint32_t id, int slot, Topic topic = {});
  std::string row(const Record &record) const;
  std::string topic_fields(const Record &record) const;
  void run();

  const std::filesystem::path directory_;
  const size_t shard_bytes_, total_bytes_;
  const std::vector<hpmMotor::MotorDefinition> motors_;
  std::unordered_map<std::string, std::string> name_column_;
  const rclcpp::Logger logger_;
  const std::chrono::steady_clock::time_point start_{std::chrono::steady_clock::now()};
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_{false};
  uint64_t sequence_{0}, dropped_{0};
  std::vector<Record> pending_;
  std::thread thread_;
};
