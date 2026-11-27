/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_motor/motor.hpp"
#include "hpm_transport/udp_wire.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <future>
#include <thread>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <unistd.h>

using namespace std::chrono_literals;
using SetZero = luvbot_hardware_interfaces::srv::MotorSetZero;

TEST(MotorSetZero, ServiceValidatesRequestsAndReportsSendFailure)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  const auto path = std::filesystem::temp_directory_path() /
    ("hpm_set_zero_" + std::to_string(getpid()) + ".yaml");
  const auto routes = YAML::Load(
    "motors:\n"
    "  dm: {index: 4, motor_type: DM, bus_index: 0, motor_id: 1}\n"
    "  rs: {index: 0, motor_type: RS, bus_index: 1, motor_id: 2}\n"
    "  enc: {index: 3, motor_type: ENC, bus_index: 2, motor_id: 3}\n"
    "  mr: {index: 1, motor_type: MR, model: PA43-10-20.25, bus_index: 3, motor_id: 4}\n"
    "  ls: {index: 2, motor_type: LS, bus_index: 4, motor_id: 5}\n");
  auto yaml = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
  yaml["motors"] = routes["motors"];
  std::ofstream(path) << yaml;

  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({rclcpp::Parameter("motor_config_file", path.string())});
  auto node = std::make_shared<rclcpp::Node>("set_zero_test", options);
  frameTransport::HpmFrameTransport::Config config;
  config.enable_tcp = false;
  frameTransport::HpmFrameTransport transport(config);
  hpmMotor::HpmMotor motor(*node, transport);
  EXPECT_EQ(node->get_parameter("motor_set_zero_timeout_ms").as_int(), 1000);
  auto client = node->create_client<SetZero>("/actuator/motor/set_zero");
  ASSERT_TRUE(client->wait_for_service(3s));
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::MultiThreadedExecutor executor(executor_options, 2U);
  executor.add_node(node);

  const auto call = [&](std::vector<uint8_t> types) {
      auto request = std::make_shared<SetZero::Request>();
      request->motor_types = std::move(types);
      auto future = client->async_send_request(request);
      EXPECT_EQ(executor.spin_until_future_complete(future, 3s),
        rclcpp::FutureReturnCode::SUCCESS);
      if (future.wait_for(0s) != std::future_status::ready) {
        return SetZero::Response::SharedPtr{};
      }
      return future.get();
    };

  auto response = call({});
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->motor_types.empty());
  EXPECT_TRUE(response->results.empty());
  EXPECT_TRUE(response->messages.empty());

  response = call({3, 3, 1});
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->motor_types, (std::vector<uint8_t>{3, 3, 1}));
  EXPECT_EQ(response->results, (std::vector<uint8_t>(3, SetZero::Response::REJECTED)));

  response = call({3, 255, 2, 4, 0, 1});
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->motor_types, (std::vector<uint8_t>{3, 255, 2, 4, 0, 1}));
  EXPECT_EQ(response->results, (std::vector<uint8_t>{
    SetZero::Response::SEND_FAILED, SetZero::Response::REJECTED,
    SetZero::Response::SEND_FAILED, SetZero::Response::SEND_FAILED,
    SetZero::Response::SEND_FAILED, SetZero::Response::SEND_FAILED}));
  ASSERT_EQ(response->messages.size(), 6U);
  EXPECT_NE(response->messages[0].find("Failed to send"), std::string::npos);
  EXPECT_NE(response->messages[5].find("Failed to send"), std::string::npos);

  // A failed write has an unknown outcome. Late feedback must not clear the
  // quarantine and allow the same motor to be resent on a later call.
  frameTransport::Frame frame;
  frame.msg_id = 0x15;
  frame.port = 4;
  frame.ide = frameTransport::kFrameIdeStandard;
  frame.can_id = 0x7FF;
  frame.dlc = 4;
  for (const auto status_opcode : {0x0103U, 0x0100U, 0x0003U, 0x0101U}) {
    frame.data = {0, 5, static_cast<uint8_t>(status_opcode >> 8U),
      static_cast<uint8_t>(status_opcode), 0, 0, 0, 0};
    motor.handle_frame(frame);
  }
  frame.dlc = 8;
  frame.can_id = 5;
  frame.data = {};
  motor.handle_frame(frame);
  response = call({2});
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->results, (std::vector<uint8_t>{SetZero::Response::REJECTED}));

  for (const auto timeout : {0, -1}) {
    options.parameter_overrides({rclcpp::Parameter("motor_config_file", path.string()),
      rclcpp::Parameter("motor_set_zero_timeout_ms", timeout)});
    auto invalid_node = std::make_shared<rclcpp::Node>("invalid_timeout_test", options);
    EXPECT_THROW(hpmMotor::HpmMotor(*invalid_node, transport), std::invalid_argument);
  }
  std::filesystem::remove(path);
}

// All transport traffic stays on loopback; there is no physical motor/node.
TEST(MotorSetZero, LsAckTimeoutCanRetryButOtherProtocolsStayQuarantined)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  const auto path = std::filesystem::temp_directory_path() /
    ("hpm_ls_retry_" + std::to_string(getpid()) + ".yaml");
  auto yaml = YAML::LoadFile(HPM_TEST_MOTOR_CONFIG);
  yaml["motors"] = YAML::Load(
    "ls: {index: 2, motor_type: LS, bus_index: 4, motor_id: 5}\n"
    "enc: {index: 3, motor_type: ENC, bus_index: 2, motor_id: 3}\n");
  std::ofstream(path) << yaml;
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({rclcpp::Parameter("motor_config_file", path.string()),
    rclcpp::Parameter("motor_set_zero_timeout_ms", 100)});
  auto node = std::make_shared<rclcpp::Node>("ls_retry_test", options);

  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(fd, 0);
  sockaddr_in peer{};
  peer.sin_family = AF_INET;
  peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::bind(fd, reinterpret_cast<sockaddr *>(&peer), sizeof(peer)), 0);
  socklen_t peer_size = sizeof(peer);
  ASSERT_EQ(::getsockname(fd, reinterpret_cast<sockaddr *>(&peer), &peer_size), 0);
  const timeval timeout{2, 0};
  ASSERT_EQ(::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)), 0);
  frameTransport::HpmFrameTransport::Config config;
  config.board_ip = "127.0.0.1";
  config.port = ntohs(peer.sin_port);
  config.enable_udp = true;
  frameTransport::HpmFrameTransport transport(config);
  hpmMotor::HpmMotor motor(*node, transport);
  transport.start();
  uint8_t packet[UDP_WIRE_MAX_DATAGRAM_SIZE]{};
  sockaddr_in client_addr{};
  socklen_t client_size = sizeof(client_addr);
  const auto n = ::recvfrom(fd, packet, sizeof(packet), 0,
    reinterpret_cast<sockaddr *>(&client_addr), &client_size);
  ASSERT_GT(n, 0);
  udp_wire_header_t header{};
  ASSERT_TRUE(udp_wire_header_decode(packet, n, &header));
  ASSERT_EQ(header.type, UDP_WIRE_HELLO);
  header.type = UDP_WIRE_HELLO_ACK;
  header.payload_len = 0;
  ASSERT_TRUE(udp_wire_header_encode(packet, sizeof(packet), &header));
  ASSERT_EQ(::sendto(fd, packet, UDP_WIRE_HEADER_SIZE, 0,
    reinterpret_cast<sockaddr *>(&client_addr), client_size), UDP_WIRE_HEADER_SIZE);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (transport.udp_stats_get().state != frameTransport::UdpLinkState::Ready &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(transport.udp_stats_get().state, frameTransport::UdpLinkState::Ready);
  auto client = node->create_client<SetZero>("/actuator/motor/set_zero");
  ASSERT_TRUE(client->wait_for_service(2s));
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::MultiThreadedExecutor executor(executor_options, 2U);
  executor.add_node(node);
  const auto call = [&](uint8_t index) {
      auto request = std::make_shared<SetZero::Request>();
      request->motor_types = {index};
      auto future = client->async_send_request(request);
      EXPECT_EQ(executor.spin_until_future_complete(future, 2s),
        rclcpp::FutureReturnCode::SUCCESS);
      return future.wait_for(0s) == std::future_status::ready ? future.get() : nullptr;
    };
  for (int attempt = 0; attempt < 2; ++attempt) {
    auto response = call(2);
    ASSERT_NE(response, nullptr);
    EXPECT_EQ(response->results, (std::vector<uint8_t>{SetZero::Response::TIMEOUT}));
  }
  auto response = call(3);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->results, (std::vector<uint8_t>{SetZero::Response::TIMEOUT}));
  response = call(3);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->results, (std::vector<uint8_t>{SetZero::Response::REJECTED}));
  auto ack = std::async(std::launch::async, [&]() {
      frameTransport::Frame reply;
      reply.msg_id = 0x15;
      reply.port = 4;
      reply.ide = frameTransport::kFrameIdeStandard;
      reply.can_id = 0x7FF;
      reply.dlc = 4;
      reply.data = {0, 5, 1, 3, 0, 0, 0, 0};
      for (int i = 0; i < 100; ++i) {
        std::this_thread::sleep_for(2ms);
        motor.handle_frame(reply);
      }
    });
  response = call(2);
  ack.get();
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->results, (std::vector<uint8_t>{SetZero::Response::CONFIRMED}));
  transport.stop();
  ::close(fd);
  std::filesystem::remove(path);
}
