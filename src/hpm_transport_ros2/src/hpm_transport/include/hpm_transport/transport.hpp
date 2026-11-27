#ifndef HPM_TRANSPORT__TRANSPORT_HPP_
#define HPM_TRANSPORT__TRANSPORT_HPP_

#include "hpm_transport/frame.hpp"
#include "hpm_transport/motor.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/* UDP transport is compiled in; enable_udp controls runtime startup. */
#ifndef ENABLE_UDP
#define ENABLE_UDP 1
#endif

namespace frameTransport
{

enum class HeartbeatState : uint8_t
{
  Unknown = 0,
  Alive,
  Stale,
};

enum class TcpLinkEvent : uint8_t
{
  Connecting = 0,
  Connected,
  ConnectFailed,
  Busy,
  Disconnected,
};

enum class UdpLinkState : uint8_t
{
  Disabled = 0,
  Registering,
  Ready,
  Busy,
  Stale,
  OtaBusy,
  SocketError,
};

struct UdpTransportStats
{
  UdpLinkState state{UdpLinkState::Disabled};
  uint64_t rx_datagrams{0U};
  uint64_t tx_datagrams{0U};
  uint64_t rx_frames{0U};
  uint64_t tx_frames{0U};
  uint64_t rx_malformed{0U};
  uint64_t rx_gaps{0U};
  uint64_t rx_duplicates{0U};
  uint64_t rx_reordered{0U};
  uint64_t tx_errors{0U};
};

struct HeartbeatStatus
{
  HeartbeatState state{HeartbeatState::Unknown};
  uint16_t last_tx_seq{0U};
  uint16_t last_rx_seq{0U};
  uint32_t lost_count{0U};
  uint32_t duplicate_count{0U};
  uint32_t reorder_count{0U};
};

class HpmFrameTransport
{
public:
  using FrameCallback = std::function<void(const Frame &)>;
  using MotorTxCallback = std::function<void(const Frame &, uint32_t target_id)>;
  using BatchTxCallback = std::function<void(size_t, const Frame &, uint32_t)>;
  using MotorSnapshotCallback = std::function<void(const MotorSnapshot &)>;
  using BatterySnapshotCallback = std::function<void(const BatterySnapshot &)>;
  using TcpLinkCallback = std::function<void(TcpLinkEvent, int err)>;
  using UdpStateCallback = std::function<void(UdpLinkState, int err)>;
  using FrameGapCallback = std::function<void()>;

  struct Config
  {
    std::string board_ip{"192.168.1.10"};
    uint16_t port{5001U};
    bool enable_tcp{false};  // retained for source compatibility; business TCP is disabled
    bool enable_udp{false};
    int reconnect_ms{1000};
    int heartbeat_timeout_ms{6000};
  };

  explicit HpmFrameTransport(const Config & config);
  ~HpmFrameTransport();

  HpmFrameTransport(const HpmFrameTransport &) = delete;
  HpmFrameTransport & operator=(const HpmFrameTransport &) = delete;

  // Configure before start(); all motor operations fail until configured.
  void set_motor_protocol_config(const MotorProtocolConfig & config);
  // Register before start(). Called on the sending thread after a successful send.
  void set_motor_tx_callback(MotorTxCallback callback);
  void start();
  void stop();

  struct MotorControl {
    MotorType type;
    uint32_t can_id;
    uint8_t can_port;
    mitctrl_t params;
  };
  // One control array is one UDP DATA packet and one board feedback window.
  bool send_motor_controls(const std::vector<MotorControl> & commands);
  bool send_motor_controls(const std::vector<MotorControl> & commands, const BatchTxCallback & on_sent);

  struct MotorEnable {
    MotorType type;
    uint32_t can_id;
    uint8_t can_port;
    bool enabled;
  };
  // One ROS enable/disable array is one UDP DATA packet; no snapshot boundary.
  bool send_motor_enables(const std::vector<MotorEnable> & commands);
  bool send_motor_enables(const std::vector<MotorEnable> & commands, const BatchTxCallback & on_sent);

  bool send_motorcmd(MotorType type, MotorCommand command, uint32_t can_id,
                     uint8_t can_port, const uint8_t * data = nullptr,
                     const mitctrl_t & params = mit_dummy);
  bool unpack_motorcmd(MotorType type, MotorCommand command, const Frame & frame,
                       MotorRx & out, MotorModel model = MotorModel::UNKNOWN);
  // ID write only: the caller must confirm the reply before saving at the new ID.
  bool send_motor_id_write(MotorType type, uint16_t current_id, uint16_t target_id,
                           uint8_t can_port);
  HeartbeatStatus heartbeat_get() const;
  UdpTransportStats udp_stats_get() const;
  void set_rcvcallback(FrameCallback callback);
  void set_motor_snapshot_callback(MotorSnapshotCallback callback);
  void set_battery_snapshot_callback(BatterySnapshotCallback callback);
  void set_tcplink_callback(TcpLinkCallback callback);
  void set_udp_state_callback(UdpStateCallback callback);
  void set_frame_gap_callback(FrameGapCallback callback);
  void set_rcvcallback_udp(FrameCallback callback);

private:
  bool send(const Frame & frame, uint32_t target_id);
  bool send_canframe(uint32_t target_id, uint8_t msg_id, uint32_t can_id, uint8_t can_port,
                     uint8_t ide, uint8_t dlc, const uint8_t * data);

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace frameTransport

#endif  // HPM_TRANSPORT__TRANSPORT_HPP_
