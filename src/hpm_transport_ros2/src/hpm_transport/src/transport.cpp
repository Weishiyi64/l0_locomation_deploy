#include "hpm_transport/transport.hpp"
#include "hpm_transport/motor_config.hpp"
#include "hpm_transport/udp_wire.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <random>
#include <stdexcept>

namespace frameTransport {

namespace {

constexpr char kBusinessModeMagic[] = "HPMBUS1\n";
constexpr size_t kUdpReceiveBatchSize = 8U;

uint32_t monotonic_ms() {
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint64_t random_session_id() {
  static std::random_device source;
  uint64_t id = (static_cast<uint64_t>(source()) << 32U) | source();
  return id == 0U ? 1U : id;
}

bool send_all(int fd, const uint8_t *data, size_t len) {
  size_t sent = 0U;
  while (sent < len) {
    const ssize_t n = ::send(fd, data + sent, len - sent, 0);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

constexpr uint16_t heartbeat_keepalive(uint16_t current) {
  return (current == 0U || current == 65535U)
             ? 1U
             : static_cast<uint16_t>(current + 1U);
}

static_assert(heartbeat_keepalive(0U) == 1U, "first seq");
static_assert(heartbeat_keepalive(65535U) == 1U, "wrap");

sockaddr_in make_board_addr(const std::string &ip, uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    throw std::invalid_argument("invalid board_ip: " + ip);
  }
  return addr;
}

bool mr_param_frame_valid(const Frame &frame) {
  return frame.ide == kFrameIdeStandard && frame.dlc == 8U &&
         frame.data[6] == 0x00U && frame.data[7] == 0xFFU &&
         frame.can_id == 0x600U + frame.data[0];
}

} // namespace

struct MotorTxFrame {
  Frame frame;
  uint32_t target_id;
};

struct HpmFrameTransport::Impl {
  MotorTxCallback motor_tx_callback;
  Config config;
  std::shared_ptr<const MotorProtocolConfig> motor_protocols;
  std::atomic<bool> running{false};

  std::mutex callback_mutex;
  FrameCallback tcp_callback;
  MotorSnapshotCallback motor_snapshot_callback;
  BatterySnapshotCallback battery_snapshot_callback;
  TcpLinkCallback tcp_link_callback;
  UdpStateCallback udp_state_callback;
  FrameGapCallback frame_gap_callback;

  std::mutex tcp_send_mutex;
  int tcp_fd{-1};

#if ENABLE_UDP
  std::mutex udp_send_mutex;
  int udp_fd{-1};
#endif

  std::thread tcp_thread;
  std::thread udp_thread;

  std::mutex udp_status_mutex;
  UdpTransportStats udp_stats;
  uint64_t udp_session_id{0U};
  uint32_t udp_tx_sequence{0U};
  uint32_t udp_rx_sequence{0U};
  bool udp_have_rx_sequence{false};
  std::chrono::steady_clock::time_point udp_last_rx_time{};
  std::chrono::steady_clock::time_point udp_last_control_tx{};

  std::mutex hb_mutex;
  std::atomic<uint16_t> hb_tx_seq{0U};
  HeartbeatStatus hb_status;
  std::chrono::steady_clock::time_point hb_last_rx_time;

  uint16_t alloc_hb_seq() {
    uint16_t cur = hb_tx_seq.load(std::memory_order_relaxed);
    uint16_t nxt;
    do {
      nxt = heartbeat_keepalive(cur);
    } while (!hb_tx_seq.compare_exchange_weak(
        cur, nxt, std::memory_order_relaxed, std::memory_order_relaxed));
    return nxt;
  }

  bool send_internal(const MotorTxFrame *frames, size_t count, const BatchTxCallback &on_sent = {}) {
    if (frames == nullptr || count == 0U || count > UDP_WIRE_MAX_FRAMES) {
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(udp_send_mutex);
      if (!running.load()) {
        return false;
      }
      UdpLinkState state;
      {
        std::lock_guard<std::mutex> status_lock(udp_status_mutex);
        state = udp_stats.state;
      }
      if (udp_fd < 0 || state != UdpLinkState::Ready || udp_session_id == 0U) {
        return false;
      }

      std::array<uint8_t, UDP_WIRE_MAX_DATAGRAM_SIZE> datagram{};
      const size_t datagram_size = UDP_WIRE_HEADER_SIZE + count * UDP_WIRE_FRAME_SIZE;
      const uint32_t timestamp = monotonic_ms();
      for (size_t i = 0U; i < count; ++i) {
        Frame frame = frames[i].frame;
        frame.timestamp = timestamp;
        frame.heartbeat = alloc_hb_seq();
        {
          std::lock_guard<std::mutex> heartbeat_lock(hb_mutex);
          hb_status.last_tx_seq = frame.heartbeat;
        }
        std::array<uint8_t, kFrameWireSize> wire{};
        if (frame_serialize(frame, wire) != FrameStatus::Ok) {
          return false;
        }
        std::memcpy(datagram.data() + UDP_WIRE_HEADER_SIZE + i * UDP_WIRE_FRAME_SIZE,
                    wire.data(), wire.size());
      }
      udp_wire_header_t header{};
      header.type = UDP_WIRE_DATA;
      header.frame_count = static_cast<uint8_t>(count);
      header.payload_len = static_cast<uint16_t>(count * UDP_WIRE_FRAME_SIZE);
      header.session_id = udp_session_id;
      header.sequence = ++udp_tx_sequence;
      if (!udp_wire_header_encode(datagram.data(), datagram_size, &header)) {
        return false;
      }
      const ssize_t n = ::send(udp_fd, datagram.data(), datagram_size, MSG_DONTWAIT);
      std::lock_guard<std::mutex> status_lock(udp_status_mutex);
      if (n == static_cast<ssize_t>(datagram_size)) {
        udp_stats.tx_datagrams++;
        udp_stats.tx_frames += count;
      } else {
        udp_stats.tx_errors++;
        return false;
      }
    }
    if (on_sent) {
      for (size_t i = 0; i < count; ++i) on_sent(i, frames[i].frame, frames[i].target_id);
    } else if (motor_tx_callback) {
      for (size_t i = 0; i < count; ++i) {
        motor_tx_callback(frames[i].frame, frames[i].target_id);
      }
    }
    return true;
  }

  void record_hpm_seq(uint16_t seq) {
    hb_last_rx_time = std::chrono::steady_clock::now();
    if (seq == 0U) {
      return;
    }
    if (hb_status.last_rx_seq == 0U) {
      hb_status.last_rx_seq = seq;
      hb_status.state = HeartbeatState::Alive;
      return;
    }
    const uint16_t prev = hb_status.last_rx_seq;
    const uint16_t next = heartbeat_keepalive(prev);
    if (seq == prev) {
      hb_status.duplicate_count++;
    } else if (seq != next) {
      if ((seq > prev && (uint32_t)(seq - prev) <= 32768UL) ||
          (seq < prev && (uint32_t)(prev - seq) > 32768UL)) {
        hb_status.lost_count += (seq > prev) ? (seq - prev - 1U) : 0U;
      } else {
        hb_status.reorder_count++;
      }
    }
    hb_status.last_rx_seq = seq;
    hb_status.state = HeartbeatState::Alive;
  }

  void notify_tcp(TcpLinkEvent event, int err = 0) {
    TcpLinkCallback callback;
    {
      std::lock_guard<std::mutex> lock(callback_mutex);
      callback = tcp_link_callback;
    }
    if (callback) {
      callback(event, err);
    }
  }

  void notify_udp(UdpLinkState state, int err = 0) {
    UdpStateCallback callback;
    bool changed;
    {
      std::lock_guard<std::mutex> lock(udp_status_mutex);
      changed = udp_stats.state != state;
      udp_stats.state = state;
    }
    {
      std::lock_guard<std::mutex> lock(callback_mutex);
      callback = udp_state_callback;
    }
    if (changed && callback) {
      callback(state, err);
    }
    {
      std::lock_guard<std::mutex> lock(hb_mutex);
      if (state == UdpLinkState::Ready) {
        hb_status.state = HeartbeatState::Alive;
        hb_last_rx_time = std::chrono::steady_clock::now();
      } else if (state == UdpLinkState::Stale || state == UdpLinkState::Busy ||
                 state == UdpLinkState::OtaBusy || state == UdpLinkState::SocketError) {
        hb_status.state = HeartbeatState::Stale;
      } else {
        hb_status.state = HeartbeatState::Unknown;
      }
    }
  }

  void notify_frame_gap() {
    FrameGapCallback callback;
    {
      std::lock_guard<std::mutex> lock(callback_mutex);
      callback = frame_gap_callback;
    }
    if (callback) {
      callback();
    }
  }

  void update_udp_stats(const std::function<void(UdpTransportStats &)> &update) {
    std::lock_guard<std::mutex> lock(udp_status_mutex);
    update(udp_stats);
  }

  void sleep_reconnect() {
    auto left = std::chrono::milliseconds(config.reconnect_ms);
    const auto step = std::chrono::milliseconds(50);
    while (running.load() && left > std::chrono::milliseconds(0)) {
      const auto n = (left < step) ? left : step;
      std::this_thread::sleep_for(n);
      left -= n;
    }
  }

  static void abort_socket(std::mutex & mu, int & fd) {
    std::lock_guard<std::mutex> lock(mu);
    if (fd >= 0) {
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
      fd = -1;
    }
  }

  bool own_socket(std::mutex & mu, int & slot, int fd) {
    std::lock_guard<std::mutex> lock(mu);
    if (!running.load()) {
      return false;
    }
    if (slot >= 0 && slot != fd) {
      ::close(slot);
    }
    slot = fd;
    return true;
  }

  void drop_socket(std::mutex & mu, int & slot, int fd) {
    bool should_close = false;
    {
      std::lock_guard<std::mutex> lock(mu);
      if (slot == fd) {
        slot = -1;
        should_close = true;
      }
    }
    if (should_close) {
      ::close(fd);
    }
  }

  void tcp_loop() {
    notify_tcp(TcpLinkEvent::Connecting);
    while (running.load()) {
      const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
      if (fd < 0) {
        notify_tcp(TcpLinkEvent::ConnectFailed, errno);
        sleep_reconnect();
        continue;
      }

      int yes = 1;
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

      if (!own_socket(tcp_send_mutex, tcp_fd, fd)) {
        ::close(fd);
        break;
      }

      const sockaddr_in addr = make_board_addr(config.board_ip, config.port);
      if (::connect(fd, reinterpret_cast<const sockaddr *>(&addr),
                    sizeof(addr)) != 0) {
        const int err = errno;
        drop_socket(tcp_send_mutex, tcp_fd, fd);
        notify_tcp(TcpLinkEvent::ConnectFailed, err);
        sleep_reconnect();
        continue;
      }

      /* The board shares TCP/5001 between business traffic and OTA.  Send a
       * business-mode preamble so receive-only nodes can be selected as the
       * feedback host without emitting a dummy CAN frame. */
      if (!send_all(fd, reinterpret_cast<const uint8_t *>(kBusinessModeMagic),
                    sizeof(kBusinessModeMagic) - 1U)) {
        const int err = errno;
        drop_socket(tcp_send_mutex, tcp_fd, fd);
        notify_tcp(TcpLinkEvent::Busy, err);
        sleep_reconnect();
        continue;
      }

      notify_tcp(TcpLinkEvent::Connected);

      std::array<uint8_t, 4096U> buffer{};
      size_t pending = 0U;
      int recv_err = 0;
      bool got_frame = false;
      while (running.load()) {
        const ssize_t n = ::recv(fd, buffer.data() + pending,
                                 buffer.size() - pending, 0);
        if (n <= 0) {
          const int err = (n < 0) ? errno : 0;
          if (err == EINTR) {
            continue;
          }
          recv_err = err;
          break;
        }

        pending += static_cast<size_t>(n);
        const size_t complete = pending - pending % kFrameWireSize;
        for (size_t offset = 0U; offset < complete && running.load();
             offset += kFrameWireSize) {
          Frame frame;
          if (frame_deserialize(frame, buffer.data() + offset, kFrameWireSize) !=
              FrameStatus::Ok) {
            continue;
          }
          got_frame = true;

          {
            std::lock_guard<std::mutex> lock(hb_mutex);
            record_hpm_seq(frame.heartbeat);
          }

          FrameCallback callback;
          {
            std::lock_guard<std::mutex> lock(callback_mutex);
            callback = tcp_callback;
          }
          if (callback) {
            callback(frame);
          }
        }
        // Carry only the incomplete frame into the next ordinary recv().
        pending -= complete;
        if (pending != 0U) {
          std::memmove(buffer.data(), buffer.data() + complete, pending);
        }
      }

      drop_socket(tcp_send_mutex, tcp_fd, fd);

      if (running.load()) {
        notify_tcp(got_frame ? TcpLinkEvent::Disconnected : TcpLinkEvent::Busy,
                   recv_err);
        sleep_reconnect();
      }
    }
  }

  void udp_loop() {
    while (running.load()) {
      const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
      if (fd < 0) {
        notify_udp(UdpLinkState::SocketError, errno);
        sleep_reconnect();
        continue;
      }

      const timeval timeout{0, 100000};
      (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      const sockaddr_in addr = make_board_addr(config.board_ip, config.port);
      if (::connect(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0 ||
          !own_socket(udp_send_mutex, udp_fd, fd)) {
        const int err = errno;
        ::close(fd);
        notify_udp(UdpLinkState::SocketError, err);
        sleep_reconnect();
        continue;
      }

      uint64_t session_id = random_session_id();
      uint32_t rx_sequence = 0U;
      bool have_rx_sequence = false;
      auto last_rx = std::chrono::steady_clock::now();
      auto last_control_tx = last_rx;
      auto last_hello_tx = last_rx;
      bool ready = false;
      {
        std::lock_guard<std::mutex> lock(udp_send_mutex);
        udp_session_id = session_id;
        udp_tx_sequence = 0U;
      }
      notify_udp(UdpLinkState::Registering);

      const auto send_control = [this, fd](udp_wire_type_t type,
                                           uint64_t session,
                                           uint32_t host_ms) {
        uint8_t packet[UDP_WIRE_HEADER_SIZE + 4U]{};
        udp_wire_header_t header{};
        header.type = type;
        header.session_id = session;
        header.payload_len = (type == UDP_WIRE_HELLO || type == UDP_WIRE_PING) ? 4U : 0U;
        if (!udp_wire_header_encode(packet, sizeof(packet), &header)) {
          return false;
        }
        if (header.payload_len != 0U) {
          udp_wire_put_be32(packet + UDP_WIRE_HEADER_SIZE, host_ms);
        }
        std::lock_guard<std::mutex> lock(udp_send_mutex);
        if (udp_fd != fd) {
          return false;
        }
        const size_t len = UDP_WIRE_HEADER_SIZE + header.payload_len;
        const ssize_t n = ::send(fd, packet, len, MSG_DONTWAIT);
        std::lock_guard<std::mutex> stats_lock(udp_status_mutex);
        if (n == static_cast<ssize_t>(len)) {
          udp_stats.tx_datagrams++;
          return true;
        }
        udp_stats.tx_errors++;
        return false;
      };

      (void)send_control(UDP_WIRE_HELLO, session_id, monotonic_ms());
      last_hello_tx = std::chrono::steady_clock::now();
      std::array<std::array<uint8_t, UDP_WIRE_MAX_DATAGRAM_SIZE + 1U>,
                 kUdpReceiveBatchSize>
          packets{};
      std::array<iovec, kUdpReceiveBatchSize> iovecs{};
      std::array<mmsghdr, kUdpReceiveBatchSize> messages{};
      for (size_t i = 0U; i < kUdpReceiveBatchSize; ++i) {
        iovecs[i].iov_base = packets[i].data();
        iovecs[i].iov_len = packets[i].size();
        messages[i].msg_hdr.msg_iov = &iovecs[i];
        messages[i].msg_hdr.msg_iovlen = 1U;
      }
      while (running.load()) {
        for (size_t i = 0U; i < kUdpReceiveBatchSize; ++i) {
          messages[i].msg_len = 0U;
          messages[i].msg_hdr.msg_flags = 0;
        }
        const int received = ::recvmmsg(
            fd, messages.data(), static_cast<unsigned int>(messages.size()),
            MSG_WAITFORONE, nullptr);
        if (received > 0) {
          for (int i = 0; i < received && running.load(); ++i) {
            const size_t packet_len = messages[static_cast<size_t>(i)].msg_len;
            const uint8_t *packet = packets[static_cast<size_t>(i)].data();
            update_udp_stats([](UdpTransportStats &s) { s.rx_datagrams++; });
            if (packet_len > UDP_WIRE_MAX_DATAGRAM_SIZE ||
                (messages[static_cast<size_t>(i)].msg_hdr.msg_flags & MSG_TRUNC) != 0) {
              update_udp_stats([](UdpTransportStats &s) { s.rx_malformed++; });
              notify_frame_gap();
              continue;
            }
            udp_wire_header_t header{};
            if (!udp_wire_header_decode(packet, packet_len, &header) ||
                header.session_id != session_id) {
              update_udp_stats([](UdpTransportStats &s) { s.rx_malformed++; });
              continue;
            }

            const auto now = std::chrono::steady_clock::now();
            if (header.type == UDP_WIRE_HELLO_ACK || header.type == UDP_WIRE_PONG) {
              if (header.type == UDP_WIRE_HELLO_ACK || ready) {
                ready = true;
                last_rx = now;
                notify_udp(UdpLinkState::Ready);
                std::lock_guard<std::mutex> lock(hb_mutex);
                hb_last_rx_time = now;
                hb_status.state = HeartbeatState::Alive;
              }
              continue;
            }
            if (header.type == UDP_WIRE_BUSY) {
              ready = false;
              notify_udp(UdpLinkState::Busy);
              continue;
            }
            if (header.type == UDP_WIRE_OTA_BUSY) {
              ready = false;
              notify_udp(UdpLinkState::OtaBusy);
              continue;
            }
            if (header.type == UDP_WIRE_RESET) {
              ready = false;
              session_id = random_session_id();
              rx_sequence = 0U;
              have_rx_sequence = false;
              {
                std::lock_guard<std::mutex> lock(udp_send_mutex);
                udp_session_id = session_id;
                udp_tx_sequence = 0U;
              }
              notify_frame_gap();
              notify_udp(UdpLinkState::Registering);
              (void)send_control(UDP_WIRE_HELLO, session_id, monotonic_ms());
              last_hello_tx = std::chrono::steady_clock::now();
              continue;
            }
            if ((header.type != UDP_WIRE_DATA &&
                 header.type != UDP_WIRE_MOTOR_SNAPSHOT &&
                 header.type != UDP_WIRE_BATTERY_SNAPSHOT) || !ready) {
              update_udp_stats([](UdpTransportStats &s) { s.rx_malformed++; });
              continue;
            }

            const uint8_t *payload = packet + UDP_WIRE_HEADER_SIZE;
            MotorSnapshot snapshot;
            BatterySnapshot battery_snapshot{};
            if (header.type == UDP_WIRE_BATTERY_SNAPSHOT &&
                (frame_crc16_modbus(packet, packet_len - 2U) !=
                     udp_wire_get_be16(packet + packet_len - 2U) ||
                 !battery_payload_decode(payload, header.payload_len, &battery_snapshot))) {
              update_udp_stats([](UdpTransportStats &s) { s.rx_malformed++; });
              continue;
            }
            if (header.type == UDP_WIRE_MOTOR_SNAPSHOT) {
              uint32_t mask = 0U;
              bool valid = packet_len == UDP_WIRE_MOTOR_SNAPSHOT_SIZE &&
                  payload[0] == 1U && payload[1] == UDP_WIRE_MOTOR_SNAPSHOT_SLOTS &&
                  payload[2] <= 3U && payload[3] == 0U &&
                  frame_crc16_modbus(packet, packet_len - 2U) ==
                    udp_wire_get_be16(packet + packet_len - 2U);
              if (valid) {
                mask = udp_wire_get_be32(payload + 16U);
                valid = (mask & ~0x001FFFFFU) == 0U;
              }
              if (valid) {
                snapshot.control_sequence = udp_wire_get_be32(payload + 4U);
                snapshot.window_start_us = udp_wire_get_be32(payload + 8U);
                snapshot.snapshot_us = udp_wire_get_be32(payload + 12U);
                snapshot.feedback_mask = mask;
                snapshot.finish_reason = payload[2];
                const uint8_t *slot_bytes = payload + UDP_WIRE_MOTOR_SNAPSHOT_META_SIZE;
                for (size_t slot_index = 0U;
                     slot_index < UDP_WIRE_MOTOR_SNAPSHOT_SLOTS && valid;
                     ++slot_index) {
                  const uint8_t *wire = slot_bytes +
                      slot_index * UDP_WIRE_MOTOR_SNAPSHOT_SLOT_SIZE;
                  const bool present = (mask & (1UL << slot_index)) != 0U;
                  if (!present) {
                    for (size_t i = 0U; i < UDP_WIRE_MOTOR_SNAPSHOT_SLOT_SIZE; ++i) {
                      if (wire[i] != 0U) {
                        valid = false;
                        break;
                      }
                    }
                    continue;
                  }
                  Frame &frame = snapshot.slots[slot_index];
                  frame.msg_id = wire[0];
                  frame.heartbeat = udp_wire_get_be16(wire + 1U);
                  frame.timestamp = udp_wire_get_be32(wire + 3U);
                  frame.reserved = wire[7];
                  frame.port = wire[8];
                  frame.ide = wire[9];
                  frame.can_id = udp_wire_get_be32(wire + 10U);
                  frame.dlc = wire[14];
                  if (frame.msg_id < 0x10U || frame.msg_id > 0x1FU ||
                      frame.ide > 1U || frame.dlc > 8U) {
                    valid = false;
                    break;
                  }
                  std::memcpy(frame.data.data(), wire + 15U, frame.data.size());
                  for (size_t i = frame.dlc; i < frame.data.size(); ++i) {
                    if (frame.data[i] != 0U) {
                      valid = false;
                      break;
                    }
                  }
                }
              }
              if (!valid) {
                update_udp_stats([](UdpTransportStats &s) { s.rx_malformed++; });
                continue;
              }
            }

            // A malformed snapshot must not change sequence, gap or liveness state.
            const uint32_t delta = header.sequence - rx_sequence;
            if (have_rx_sequence && delta == 0U) {
              update_udp_stats([](UdpTransportStats &s) { s.rx_duplicates++; });
              continue;
            }
            if (have_rx_sequence && delta >= 0x80000000U) {
              update_udp_stats([](UdpTransportStats &s) { s.rx_reordered++; });
              continue;
            }
            if (have_rx_sequence && delta > 1U) {
              update_udp_stats([delta](UdpTransportStats &s) { s.rx_gaps += delta - 1U; });
              notify_frame_gap();
            }
            rx_sequence = header.sequence;
            have_rx_sequence = true;
            last_rx = now;

            if (header.type == UDP_WIRE_BATTERY_SNAPSHOT) {
              BatterySnapshotCallback callback;
              {
                std::lock_guard<std::mutex> lock(callback_mutex);
                callback = battery_snapshot_callback;
              }
              if (callback) callback(battery_snapshot);
              update_udp_stats([](UdpTransportStats &s) { s.rx_frames++; });
              continue;
            }
            if (header.type == UDP_WIRE_MOTOR_SNAPSHOT) {
              const uint32_t mask = snapshot.feedback_mask;
              MotorSnapshotCallback snapshot_callback;
              {
                std::lock_guard<std::mutex> lock(callback_mutex);
                snapshot_callback = motor_snapshot_callback;
              }
              if (snapshot_callback) {
                snapshot_callback(snapshot);
              }
              unsigned int present_count = 0U;
              for (uint32_t bits = mask; bits != 0U; bits &= bits - 1U) {
                present_count++;
              }
              update_udp_stats([present_count](UdpTransportStats &s) {
                s.rx_frames += present_count;
              });
              for (size_t slot_index = 0U;
                   slot_index < UDP_WIRE_MOTOR_SNAPSHOT_SLOTS; ++slot_index) {
                if ((mask & (1UL << slot_index)) != 0U) {
                  std::lock_guard<std::mutex> lock(hb_mutex);
                  record_hpm_seq(snapshot.slots[slot_index].heartbeat);
                }
              }
              continue;
            }
            for (uint8_t i = 0U; i < header.frame_count && running.load(); ++i) {
              Frame frame;
              const uint8_t *wire = payload + static_cast<size_t>(i) * kFrameWireSize;
              if (frame_deserialize(frame, wire, kFrameWireSize) != FrameStatus::Ok) {
                update_udp_stats([](UdpTransportStats &s) { s.rx_malformed++; });
                notify_frame_gap();
                continue;
              }
              update_udp_stats([](UdpTransportStats &s) { s.rx_frames++; });
              {
                std::lock_guard<std::mutex> lock(hb_mutex);
                record_hpm_seq(frame.heartbeat);
              }
              FrameCallback callback;
              {
                std::lock_guard<std::mutex> lock(callback_mutex);
                callback = tcp_callback;
              }
              if (callback) {
                callback(frame);
              }
            }
            continue;
          }
        }

        const int recv_error = errno;
        if (received < 0 && recv_error != EINTR && recv_error != EAGAIN &&
            recv_error != EWOULDBLOCK) {
          notify_udp(UdpLinkState::SocketError, recv_error);
          break;
        }

        // Keep control traffic running even when every receive returns data.
        const auto now = std::chrono::steady_clock::now();
        if (ready && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx).count() >=
                         config.heartbeat_timeout_ms) {
          (void)send_control(UDP_WIRE_CLOSE, session_id, 0U);
          ready = false;
          notify_frame_gap();
          notify_udp(UdpLinkState::Stale);
          session_id = random_session_id();
          rx_sequence = 0U;
          have_rx_sequence = false;
          {
            std::lock_guard<std::mutex> lock(udp_send_mutex);
            udp_session_id = session_id;
            udp_tx_sequence = 0U;
          }
          notify_udp(UdpLinkState::Registering);
          (void)send_control(UDP_WIRE_HELLO, session_id, monotonic_ms());
          last_hello_tx = now;
          last_control_tx = now;
        } else if (ready &&
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - last_control_tx).count() >= 1000) {
          (void)send_control(UDP_WIRE_PING, session_id, monotonic_ms());
          last_control_tx = now;
        } else if (!ready &&
                   std::chrono::duration_cast<std::chrono::milliseconds>(now - last_hello_tx).count() >=
                       config.reconnect_ms) {
          (void)send_control(UDP_WIRE_HELLO, session_id, monotonic_ms());
          last_hello_tx = now;
        }
      }

      bool close_fd = false;
      {
        std::lock_guard<std::mutex> lock(udp_send_mutex);
        if (udp_fd == fd) {
          udp_fd = -1;
          udp_session_id = 0U;
          udp_tx_sequence = 0U;
          close_fd = true;
        }
      }
      if (close_fd) {
        ::close(fd);
      }

      if (running.load()) {
        notify_udp(UdpLinkState::Stale);
        notify_frame_gap();
        sleep_reconnect();
      }
    }
  }
};

HpmFrameTransport::HpmFrameTransport(const Config &config)
    : impl_(std::make_unique<Impl>()) {
  impl_->config = config;
}

HpmFrameTransport::~HpmFrameTransport() { stop(); }

void HpmFrameTransport::set_motor_protocol_config(const MotorProtocolConfig &config) {
  if (impl_->running.load()) {
    throw std::logic_error("motor protocol configuration must be loaded before transport start");
  }
  validate_motor_protocol_config(config);
  std::atomic_store(&impl_->motor_protocols,
      std::make_shared<const MotorProtocolConfig>(config));
}

void HpmFrameTransport::set_motor_tx_callback(MotorTxCallback callback) {
  if (impl_->running.load()) {
    throw std::logic_error("set motor TX callback before transport start");
  }
  impl_->motor_tx_callback = std::move(callback);
}

void HpmFrameTransport::start() {
  if (impl_->running.exchange(true)) {
    return;
  }

#if ENABLE_UDP
  if (impl_->config.enable_udp) {
    impl_->udp_thread = std::thread([this]() { impl_->udp_loop(); });
  }
#endif
}

void HpmFrameTransport::stop() {
  if (!impl_->running.exchange(false)) {
    return;
  }

#if ENABLE_UDP
  {
    std::lock_guard<std::mutex> lock(impl_->udp_send_mutex);
    if (impl_->udp_fd >= 0 && impl_->udp_session_id != 0U) {
      uint8_t packet[UDP_WIRE_HEADER_SIZE]{};
      udp_wire_header_t header{};
      header.type = UDP_WIRE_CLOSE;
      header.session_id = impl_->udp_session_id;
      if (udp_wire_header_encode(packet, sizeof(packet), &header)) {
        (void)::send(impl_->udp_fd, packet, sizeof(packet), MSG_DONTWAIT);
      }
    }
  }
#endif

  /* close() before join: shutdown() cannot abort an in-progress connect() */
  Impl::abort_socket(impl_->tcp_send_mutex, impl_->tcp_fd);
#if ENABLE_UDP
  Impl::abort_socket(impl_->udp_send_mutex, impl_->udp_fd);
#endif

#if ENABLE_UDP
  if (impl_->udp_thread.joinable()) {
    impl_->udp_thread.join();
  }
#endif
  impl_->notify_udp(UdpLinkState::Disabled);
}

bool HpmFrameTransport::send(const Frame &frame, uint32_t target_id) {
  const MotorTxFrame tx{frame, target_id};
  return impl_->send_internal(&tx, 1U);
}

bool HpmFrameTransport::send_canframe(uint32_t target_id, uint8_t msg_id, uint32_t can_id,
                                      uint8_t can_port, uint8_t ide,
                                      uint8_t dlc, const uint8_t *data) {
  Frame f;
  frame_init(f);
  f.msg_id = msg_id;
  f.port = can_port;
  f.can_id = can_id;
  f.ide = ide;
  f.dlc = (dlc > kFrameDataLen) ? kFrameDataLen : dlc;
  for (uint8_t i = 0U; i < f.dlc; ++i) {
    f.data[i] = data[i];
  }
  return send(f, target_id);
}

bool HpmFrameTransport::send_motor_id_write(
    MotorType type, uint16_t current_id, uint16_t target_id, uint8_t can_port) {
  const auto protocols = std::atomic_load(&impl_->motor_protocols);
  if (!protocols) {
    return false;
  }

  if (can_port > 7U || current_id == 0U || target_id == 0U ||
      current_id > (type == MotorType::MOTOR_TYPE_LS ? 0x7FFU : 0xFFU) ||
      target_id > (type == MotorType::MOTOR_TYPE_LS ? 0x7FFU : 0xFFU)) {
    return false;
  }
  if (type != MotorType::MOTOR_TYPE_RS && type != MotorType::MOTOR_TYPE_MR &&
      type != MotorType::MOTOR_TYPE_LS) {
    return false;
  }
  std::array<uint8_t, 8> data{};
  if (motor_pack_setid(type, current_id, target_id, data) != 0) {
    return false;
  }
  const uint32_t can_id = type == MotorType::MOTOR_TYPE_MR ? 0x600U + current_id :
      type == MotorType::MOTOR_TYPE_RS ?
      motor_rs_pack_canid_setid(static_cast<uint8_t>(current_id), static_cast<uint8_t>(target_id)) :
      0x7FFU;
  return send_canframe(current_id, static_cast<uint8_t>(0x10U + (current_id & 0x0FU)), can_id,
      can_port, type == MotorType::MOTOR_TYPE_RS ? kFrameIdeExtended : kFrameIdeStandard,
      type == MotorType::MOTOR_TYPE_LS ? 6U : 8U, data.data());
}

static bool encode_motor_control(MotorType type, uint32_t can_id, uint8_t can_port,
                                  const mitctrl_t &params, Frame &frame,
                                  const MotorProtocolConfig &protocols) {
  frame_init(frame);
  frame.msg_id = static_cast<uint8_t>(0x10U + (can_id & 0x0FU));
  frame.port = can_port;
  frame.can_id = can_id;
  frame.ide = kFrameIdeStandard;
  switch (type) {
  case MotorType::MOTOR_TYPE_RS:
    frame.ide = kFrameIdeExtended;
    frame.can_id = motor_rs_pack_canid_mit(protocols, static_cast<uint8_t>(can_id & 0xFFU), params.t_ff);
    break;
  case MotorType::MOTOR_TYPE_MR:
    if (can_id == 0U || can_id > 0xFFU) {
      return false;
    }
    frame.can_id = 0x200U + can_id;
    break;
  case MotorType::MOTOR_TYPE_DM:
  case MotorType::MOTOR_TYPE_ENC:
  case MotorType::MOTOR_TYPE_LS:
    break;
  default:
    return false;
  }
  std::array<uint8_t, 8> data{};
  if (motor_pack_mit(protocols, type, params, data) != 0) {
    return false;
  }
  frame.dlc = 8U;
  for (size_t i = 0U; i < data.size(); ++i) {
    frame.data[i] = data[i];
  }
  return true;
}

bool HpmFrameTransport::send_motor_controls(const std::vector<MotorControl> &commands) {
  return send_motor_controls(commands, {});
}

bool HpmFrameTransport::send_motor_controls(const std::vector<MotorControl> &commands, const BatchTxCallback &on_sent) {
  const auto protocols = std::atomic_load(&impl_->motor_protocols);
  if (!protocols) {
    return false;
  }

  if (commands.empty() || commands.size() > UDP_WIRE_MAX_FRAMES) {
    return false;
  }
  std::vector<MotorTxFrame> frames(commands.size());
  for (size_t i = 0U; i < commands.size(); ++i) {
    const auto &command = commands[i];
    frames[i].target_id = command.can_id;
    if (!encode_motor_control(command.type, command.can_id, command.can_port,
                               command.params, frames[i].frame, *protocols)) {
      return false;
    }
  }
  return impl_->send_internal(frames.data(), frames.size(), on_sent);
}

static bool encode_motor_enable(MotorType type, bool enabled, uint32_t can_id,
                                 uint8_t can_port, Frame &frame) {
  frame_init(frame);
  frame.msg_id = static_cast<uint8_t>(0x10U + (can_id & 0x0FU));
  frame.port = can_port;
  frame.can_id = can_id;
  frame.ide = kFrameIdeStandard;
  frame.dlc = 8U;
  switch (type) {
  case MotorType::MOTOR_TYPE_RS:
    frame.ide = kFrameIdeExtended;
    frame.can_id = enabled ? motor_rs_pack_canid_enable(static_cast<uint8_t>(can_id)) :
        motor_rs_pack_canid_disable(static_cast<uint8_t>(can_id));
    break;
  case MotorType::MOTOR_TYPE_MR:
    if (can_id == 0U || can_id > 0xFFU) {
      return false;
    }
    frame.can_id = 0x100U + can_id;
    break;
  case MotorType::MOTOR_TYPE_LS:
    frame.can_id = 0x7FFU;
    frame.dlc = 4U;
    break;
  case MotorType::MOTOR_TYPE_ENC:
    frame.dlc = 3U;
    break;
  case MotorType::MOTOR_TYPE_DM:
    break;
  default:
    return false;
  }
  std::array<uint8_t, 8> data{};
  const int status = enabled ? motor_pack_enable(type, static_cast<uint16_t>(can_id), data) :
      motor_pack_disable(type, static_cast<uint16_t>(can_id), data);
  if (status != 0) {
    return false;
  }
  for (size_t i = 0U; i < data.size(); ++i) {
    frame.data[i] = data[i];
  }
  return true;
}

bool HpmFrameTransport::send_motor_enables(const std::vector<MotorEnable> &commands) {
  return send_motor_enables(commands, {});
}

bool HpmFrameTransport::send_motor_enables(const std::vector<MotorEnable> &commands, const BatchTxCallback &on_sent) {
  const auto protocols = std::atomic_load(&impl_->motor_protocols);
  if (!protocols) {
    return false;
  }

  if (commands.empty() || commands.size() > UDP_WIRE_MAX_FRAMES) {
    return false;
  }
  std::vector<MotorTxFrame> frames(commands.size());
  for (size_t i = 0U; i < commands.size(); ++i) {
    const auto &command = commands[i];
    frames[i].target_id = command.can_id;
    if (!encode_motor_enable(command.type, command.enabled, command.can_id,
                              command.can_port, frames[i].frame)) {
      return false;
    }
  }
  return impl_->send_internal(frames.data(), frames.size(), on_sent);
}

bool HpmFrameTransport::send_motorcmd(MotorType type, MotorCommand command,
                                      uint32_t can_id,
                                      uint8_t can_port, const uint8_t *data,
                                      const mitctrl_t &params) {
  const auto protocols = std::atomic_load(&impl_->motor_protocols);
  if (!protocols) {
    return false;
  }

  if (command == MotorCommand::MOTOR_CMD_CTRL) {
    Frame frame;
    return encode_motor_control(type, can_id, can_port, params, frame, *protocols) && send(frame, can_id);
  }
  if (command == MotorCommand::MOTOR_CMD_ENABLE || command == MotorCommand::MOTOR_CMD_DISABLE) {
    Frame frame;
    return encode_motor_enable(type, command == MotorCommand::MOTOR_CMD_ENABLE,
                                can_id, can_port, frame) && send(frame, can_id);
  }
  std::array<uint8_t, 8> _data{};
  uint8_t _dlc = 0;
  uint8_t _ide = kFrameIdeStandard;
  uint32_t _can_id = can_id;

  if (type == MotorType::MOTOR_TYPE_MR) {
    if (can_id == 0U || can_id > 0xFFU) {
      return false;
    }
    if (command == MotorCommand::MOTOR_CMD_SETZERO ||
               command == MotorCommand::MOTOR_CMD_RESET) {
      _can_id = 0x100U + can_id;
    }
  }

  const uint8_t msg_id = static_cast<uint8_t>(0x10U + (can_id & 0x0FU));



  switch (type) {
  case MotorType::MOTOR_TYPE_DM:
    _ide = kFrameIdeStandard;
    break;
  case MotorType::MOTOR_TYPE_RS:
    _ide = kFrameIdeExtended;
    break;
  case MotorType::MOTOR_TYPE_ENC:
    _ide = kFrameIdeStandard;
    break;
  case MotorType::MOTOR_TYPE_MR:
    _ide = kFrameIdeStandard;
    break;
  case MotorType::MOTOR_TYPE_LS:
    _ide = kFrameIdeStandard;
    break;
  default:
    return false;
  }

  switch (command) {
  case MotorCommand::MOTOR_CMD_READ_LS_ERROR:
    if (type != MotorType::MOTOR_TYPE_LS || can_port > 7U || can_id == 0U || can_id > 0x7FEU) {
      return false;
    }
    _data[0] = 0xE0U;
    _data[1] = kLsAlarmQuery;
    _dlc = 2U;
    break;
  case MotorCommand::MOTOR_CMD_SETZERO:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      _dlc = 8;
      if (motor_pack_setzero(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    case MotorType::MOTOR_TYPE_RS:
      _dlc = 8;
      if (motor_pack_setzero(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      _can_id =
          motor_rs_pack_canid_setzero(static_cast<uint8_t>(can_id & 0xFFu));
      break;
    case MotorType::MOTOR_TYPE_ENC:
      _can_id = 0x7FFU;
      _dlc = 4;
      if (motor_pack_setzero(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    case MotorType::MOTOR_TYPE_MR:
      _dlc = 8;
      if (motor_pack_setzero(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      _can_id = 0x7FFU;
      _dlc = 4;
      if (motor_pack_setzero(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    default:
      return false;
    }
    break;
  case MotorCommand::MOTOR_CMD_RESET:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM: {
      _dlc = 8;
      motor_pack_reset(type, static_cast<uint16_t>(can_id), _data);
      break;
    }
    case MotorType::MOTOR_TYPE_RS:
      _dlc = 8;
      if (motor_pack_reset(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      _can_id = motor_rs_pack_canid_reset(static_cast<uint8_t>(can_id & 0xFFu));
      break;
    case MotorType::MOTOR_TYPE_ENC:
      _dlc = 3;
      if (motor_pack_reset(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    case MotorType::MOTOR_TYPE_MR:
      _dlc = 8;
      if (motor_pack_reset(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      _can_id = 0x7FFU;
      _dlc = 4;
      if (motor_pack_reset(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    default:
      return false;
    }
    break;
  case MotorCommand::MOTOR_CMD_WRITEREG:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      if (data == nullptr) {
        return false;
      }
      _can_id = 0x7FFu;
      _dlc = 8U;
      for (uint8_t i = 0U; i < _dlc; ++i) {
        _data[i] = data[i];
      }
      break;
    case MotorType::MOTOR_TYPE_RS: {
      if (data == nullptr) {
        return false;
      }
      uint16_t addr = 0;
      uint32_t value = 0;
      std::memcpy(&addr, data, sizeof(addr));
      std::memcpy(&value, data + sizeof(addr), sizeof(value));
      _dlc = 8;
      if (motor_pack_regw(type, static_cast<uint8_t>(can_id & 0xFFu), addr,
                          value, _data) != 0) {
        return false;
      }
      _can_id = motor_rs_pack_canid_regw(static_cast<uint8_t>(can_id & 0xFFu));
      break;
    }
    case MotorType::MOTOR_TYPE_ENC:
      return false;
    case MotorType::MOTOR_TYPE_MR: {
      if (data == nullptr) {
        return false;
      }
      uint16_t addr = 0;
      uint32_t value = 0;
      std::memcpy(&addr, data, sizeof(addr));
      std::memcpy(&value, data + sizeof(addr), sizeof(value));
      _dlc = 8;
      if (motor_pack_regw(type, static_cast<uint8_t>(can_id & 0xFFu), addr,
                          value, _data) != 0) {
        return false;
      }
      _can_id = 0x600U + (can_id & 0xFFu);
      break;
    }
    case MotorType::MOTOR_TYPE_LS:
      return false;
    default:
      return false;
    }
    break;
  case MotorCommand::MOTOR_CMD_READREG:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:

      _dlc = 8;
      for (uint8_t i = 0U; i < _dlc; ++i) {
        _data[i] = data[i];
      }
      _can_id = 0x7FFU;
      break;
    case MotorType::MOTOR_TYPE_RS: {
      if (data == nullptr) {
        return false;
      }
      uint16_t addr = 0;
      std::memcpy(&addr, data, sizeof(addr));
      _dlc = 8;
      if (motor_pack_regr(type, static_cast<uint8_t>(can_id & 0xFFu), addr,
                          _data) != 0) {
        return false;
      }
      _can_id = motor_rs_pack_canid_regr(static_cast<uint8_t>(can_id & 0xFFu));
      break;
    }
    case MotorType::MOTOR_TYPE_ENC:
      return false;
    case MotorType::MOTOR_TYPE_MR: {
      if (data == nullptr) {
        return false;
      }
      uint16_t addr = 0;
      std::memcpy(&addr, data, sizeof(addr));
      _dlc = 8;
      if (motor_pack_regr(type, static_cast<uint8_t>(can_id & 0xFFu), addr,
                          _data) != 0) {
        return false;
      }
      _can_id = 0x600U + (can_id & 0xFFu);
      break;
    }
    case MotorType::MOTOR_TYPE_LS:
      return false;
    default:
      return false;
    }
    break;
  case MotorCommand::MOTOR_CMD_SAVEREG:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:

      _can_id = 0x7FFu;
      _dlc = 8U;
      for (uint8_t i = 0U; i < _dlc; ++i) {
        _data[i] = data[i];
      }
      break;
    case MotorType::MOTOR_TYPE_RS:
      _dlc = 8;
      if (motor_pack_regs(type, static_cast<uint8_t>(can_id & 0xFFu), 0,
                          _data) != 0) {
        return false;
      }
      _can_id = motor_rs_pack_canid_regs(static_cast<uint8_t>(can_id & 0xFFu));
      break;
    case MotorType::MOTOR_TYPE_ENC:
      return false;
    case MotorType::MOTOR_TYPE_MR:
      _dlc = 8;
      if (motor_pack_regs(type, static_cast<uint8_t>(can_id & 0xFFu), 0,
                          _data) != 0) {
        return false;
      }
      _can_id = 0x600U + (can_id & 0xFFu);
      break;
    case MotorType::MOTOR_TYPE_LS:
      return false;
    default:
      return false;
    }
    break;
  case MotorCommand::MOTOR_CMD_GETID:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      return false;
    case MotorType::MOTOR_TYPE_RS:
      _dlc = 8;
      if (motor_pack_getid(type, _data) != 0) {
        return false;
      }
      _can_id = motor_rs_pack_canid_getid(static_cast<uint8_t>(can_id & 0xFFu));
      break;
    case MotorType::MOTOR_TYPE_ENC:
      _can_id = 0x7FFU;
      _dlc = 4;
      if (motor_pack_getid(type, _data) != 0) {
        return false;
      }
      break;
    case MotorType::MOTOR_TYPE_MR:
      _dlc = 8;
      if (motor_pack_getid(type, _data) != 0) {
        return false;
      }
      _can_id = 0x600U + (can_id & 0xFFU);
      break;
    case MotorType::MOTOR_TYPE_LS:
      _can_id = 0x7FFU;
      _dlc = 4;
      if (motor_pack_getid(type, _data) != 0) {
        return false;
      }
      break;
    default:
      return false;
    }
    break;
  case MotorCommand::MOTOR_CMD_SETID:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM: {
      if (data == nullptr) {
        return false;
      }
      uint16_t new_id = 0;
      std::memcpy(&new_id, data, sizeof(new_id));
      const uint16_t kDmIdRegAddr = 0x08U;
      const uint8_t motor_id = static_cast<uint8_t>(can_id & 0xFFU);

      std::array<uint8_t, 8> wr{};
      if (motor_pack_regw(MotorType::MOTOR_TYPE_DM, motor_id, kDmIdRegAddr,
                          new_id, wr) != 0) {
        return false;
      }
      const bool ok_write =
          send_canframe(can_id, msg_id, can_id, can_port, _ide, 8, wr.data());

      std::array<uint8_t, 8> sv{};
      if (motor_pack_regs(MotorType::MOTOR_TYPE_DM, motor_id, kDmIdRegAddr,
                          sv) != 0) {
        return false;
      }
      const bool ok_save =
          send_canframe(can_id, msg_id, can_id, can_port, _ide, 8, sv.data());
      return ok_write && ok_save;
    }
    case MotorType::MOTOR_TYPE_RS: {
      if (data == nullptr) {
        return false;
      }
      uint16_t new_id = 0;
      std::memcpy(&new_id, data, sizeof(new_id));
      _dlc = 8;
      if (motor_pack_setid(type, static_cast<uint16_t>(can_id), new_id,
                           _data) != 0) {
        return false;
      }
      _can_id = motor_rs_pack_canid_setid(static_cast<uint8_t>(can_id & 0xFFu),
                                          static_cast<uint8_t>(new_id & 0xFFu));
      break;
    }
    case MotorType::MOTOR_TYPE_ENC: {
      if (data == nullptr) {
        return false;
      }
      uint16_t new_id = 0;
      std::memcpy(&new_id, data, sizeof(new_id));
      _can_id = 0x7FFU;
      _dlc = 6;
      if (motor_pack_setid(type, static_cast<uint16_t>(can_id), new_id,
                           _data) != 0) {
        return false;
      }
      break;
    }
    case MotorType::MOTOR_TYPE_MR: {
      if (data == nullptr) {
        return false;
      }
      uint16_t new_id = 0;
      std::memcpy(&new_id, data, sizeof(new_id));
      const uint8_t motor_id = static_cast<uint8_t>(can_id & 0xFFU);
      const uint32_t old_param_can_id = 0x600U + motor_id;

      std::array<uint8_t, 8> wr{};
      if (motor_pack_setid(MotorType::MOTOR_TYPE_MR, motor_id, new_id, wr) !=
          0) {
        return false;
      }
      const bool ok_write =
          send_canframe(can_id, msg_id, old_param_can_id, can_port, _ide, 8, wr.data());

      std::array<uint8_t, 8> sv{};
      if (motor_pack_regs(MotorType::MOTOR_TYPE_MR, motor_id, 0, sv) != 0) {
        return false;
      }
      /* MR switches to the new ID immediately after Index 36 is written. */
      const uint32_t new_param_can_id = 0x600U + new_id;
      const bool ok_save =
          send_canframe(new_id, msg_id, new_param_can_id, can_port, _ide, 8, sv.data());
      return ok_write && ok_save;
    }
    case MotorType::MOTOR_TYPE_LS: {
      if (data == nullptr) {
        return false;
      }
      uint16_t new_id = 0;
      std::memcpy(&new_id, data, sizeof(new_id));
      _can_id = 0x7FFU;
      _dlc = 6;
      if (motor_pack_setid(type, static_cast<uint16_t>(can_id), new_id,
                           _data) != 0) {
        return false;
      }
      break;
    }
    default:
      return false;
    }
    break;
  case MotorCommand::MOTOR_CMD_RESETID:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM: {

      const uint16_t kDmIdRegAddr = 0x08U;
      const uint8_t motor_id = static_cast<uint8_t>(can_id & 0xFFU);

      std::array<uint8_t, 8> wr{};
      if (motor_pack_regw(MotorType::MOTOR_TYPE_DM, motor_id, kDmIdRegAddr,
                          0x01U, wr) != 0) {
        return false;
      }
      const bool ok_write =
          send_canframe(can_id, msg_id, can_id, can_port, _ide, 8, wr.data());

      std::array<uint8_t, 8> sv{};
      if (motor_pack_regs(MotorType::MOTOR_TYPE_DM, motor_id, kDmIdRegAddr,
                          sv) != 0) {
        return false;
      }
      const bool ok_save =
          send_canframe(can_id, msg_id, can_id, can_port, _ide, 8, sv.data());
      return ok_write && ok_save;
    }
    case MotorType::MOTOR_TYPE_RS:
      _dlc = 8;
      if (motor_pack_resetid(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      _can_id =
          motor_rs_pack_canid_resetid(static_cast<uint8_t>(can_id & 0xFFu));
      break;
    case MotorType::MOTOR_TYPE_ENC:
      _can_id = 0x7FFU;
      _dlc = 6;
      if (motor_pack_resetid(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    case MotorType::MOTOR_TYPE_MR: {
      const uint8_t motor_id = static_cast<uint8_t>(can_id & 0xFFU);
      const uint32_t old_param_can_id = 0x600U + motor_id;

      std::array<uint8_t, 8> wr{};
      if (motor_pack_resetid(MotorType::MOTOR_TYPE_MR, motor_id, wr) != 0) {
        return false;
      }
      const bool ok_write =
          send_canframe(can_id, msg_id, old_param_can_id, can_port, _ide, 8, wr.data());

      std::array<uint8_t, 8> sv{};
      if (motor_pack_regs(MotorType::MOTOR_TYPE_MR, motor_id, 0, sv) != 0) {
        return false;
      }
      /* RESETID writes ID=1, so the save command uses the new ID. */
      const uint32_t new_param_can_id = 0x601U;
      const bool ok_save =
          send_canframe(1U, msg_id, new_param_can_id, can_port, _ide, 8, sv.data());
      return ok_write && ok_save;
    }
    case MotorType::MOTOR_TYPE_LS:
      _can_id = 0x7FFU;
      _dlc = 6;
      if (motor_pack_resetid(type, static_cast<uint16_t>(can_id), _data) != 0) {
        return false;
      }
      break;
    default:
      return false;
    }
    break;
  default:
    return false;
  }

  const bool broadcast = (type == MotorType::MOTOR_TYPE_LS || type == MotorType::MOTOR_TYPE_ENC) &&
      (command == MotorCommand::MOTOR_CMD_GETID || command == MotorCommand::MOTOR_CMD_RESETID);
  return send_canframe(broadcast ? kFrameCanIdUnused : can_id, msg_id, _can_id, can_port, _ide, _dlc, _data.data());
}

bool HpmFrameTransport::unpack_motorcmd(MotorType type, MotorCommand command,
                                        const Frame &frame, MotorRx &out,
                                        MotorModel model) {
  const auto protocols = std::atomic_load(&impl_->motor_protocols);
  if (!protocols) {
    return false;
  }

  out = {};
  out.command = command;
  out.can_port = frame.port;
  out.can_id = frame.can_id;
  const uint8_t *data = frame.data.data();
  uint8_t mid8 = 0U;
  bool ok = true;

  switch (command) {
  case MotorCommand::MOTOR_CMD_CTRL:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_mit(*protocols, MotorType::MOTOR_TYPE_DM, data, out.mit) == 0);
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_unpack_mit(*protocols, MotorType::MOTOR_TYPE_RS, data, out.mit) == 0);
      if (ok) {
        ok = (motor_rs_unpack_canid_mit(frame.can_id, out.mit) == 0);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_mit(*protocols, MotorType::MOTOR_TYPE_ENC, data, out.mit) == 0);
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = (motor_unpack_mit(*protocols, MotorType::MOTOR_TYPE_MR, data, out.mit, model) == 0);
      if (ok) {
        out.mit.motor_id = static_cast<uint8_t>(frame.can_id & 0xFFu);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_mit(*protocols, MotorType::MOTOR_TYPE_LS, data, out.mit) == 0);
      if (ok) {
        out.motor_id = static_cast<uint16_t>(frame.can_id);
        out.mit.motor_id = static_cast<uint8_t>(frame.can_id & 0xFFu);
      }
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_ENABLE:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_enable(*protocols, MotorType::MOTOR_TYPE_DM, data, out.motor_id,
                                out.mit, out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_unpack_enable(*protocols, MotorType::MOTOR_TYPE_RS, data, out.motor_id,
                                out.mit, out.ok) == 0);
      if (ok) {
        ok = (motor_rs_unpack_canid_mit(frame.can_id, out.mit) == 0);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_enable(*protocols, MotorType::MOTOR_TYPE_ENC, data, out.motor_id,
                                out.mit, out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = (motor_unpack_enable(*protocols, MotorType::MOTOR_TYPE_MR, data, out.motor_id,
                                out.mit, out.ok, model) == 0);
      if (ok) {
        out.mit.motor_id = static_cast<uint8_t>(frame.can_id & 0xFFu);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_enable(*protocols, MotorType::MOTOR_TYPE_LS, data, out.motor_id,
                                out.mit, out.ok) == 0);
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_DISABLE:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_disable(*protocols, MotorType::MOTOR_TYPE_DM, data, out.motor_id,
                                 out.mit, out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_unpack_disable(*protocols, MotorType::MOTOR_TYPE_RS, data, out.motor_id,
                                 out.mit, out.ok) == 0);
      if (ok) {
        ok = (motor_rs_unpack_canid_mit(frame.can_id, out.mit) == 0);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_disable(*protocols, MotorType::MOTOR_TYPE_ENC, data, out.motor_id,
                                 out.mit, out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = (motor_unpack_disable(*protocols, MotorType::MOTOR_TYPE_MR, data, out.motor_id,
                                 out.mit, out.ok, model) == 0);
      if (ok) {
        out.mit.motor_id = static_cast<uint8_t>(frame.can_id & 0xFFu);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_disable(*protocols, MotorType::MOTOR_TYPE_LS, data, out.motor_id,
                                 out.mit, out.ok) == 0);
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_RESET:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_reset(*protocols, MotorType::MOTOR_TYPE_DM, data, out.motor_id,
                               out.mit, out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_unpack_reset(*protocols, MotorType::MOTOR_TYPE_RS, data, out.motor_id,
                               out.mit, out.ok) == 0);
      if (ok) {
        ok = (motor_rs_unpack_canid_mit(frame.can_id, out.mit) == 0);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_reset(*protocols, MotorType::MOTOR_TYPE_ENC, data, out.motor_id,
                               out.mit, out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = (motor_unpack_reset(*protocols, MotorType::MOTOR_TYPE_MR, data, out.motor_id,
                               out.mit, out.ok, model) == 0);
      if (ok) {
        out.mit.motor_id = static_cast<uint8_t>(frame.can_id & 0xFFu);
        out.motor_id = out.mit.motor_id;
        out.ok = (out.mit.err == 0U);
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_reset(*protocols, MotorType::MOTOR_TYPE_LS, data, out.motor_id,
                               out.mit, out.ok) == 0);
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_SETZERO:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_setzero(*protocols, MotorType::MOTOR_TYPE_DM, data, out.motor_id,
                                 out.mit, out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_unpack_setzero(*protocols, MotorType::MOTOR_TYPE_RS, data, out.motor_id,
                                 out.mit, out.ok) == 0);
      if (ok) {
        ok = (motor_rs_unpack_canid_mit(frame.can_id, out.mit) == 0);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_setzero(*protocols, MotorType::MOTOR_TYPE_ENC, data, out.motor_id,
                                 out.mit, out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = (motor_unpack_setzero(*protocols, MotorType::MOTOR_TYPE_MR, data, out.motor_id,
                                 out.mit, out.ok, model) == 0);
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_setzero(*protocols, MotorType::MOTOR_TYPE_LS, data, out.motor_id,
                                 out.mit, out.ok) == 0);
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_GETID:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_getid(MotorType::MOTOR_TYPE_DM, data, out.motor_id,
                               out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_rs_unpack_canid_getid(frame.can_id, out.motor_id, out.ok) ==
            0);
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_getid(MotorType::MOTOR_TYPE_ENC, data, out.motor_id,
                               out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = mr_param_frame_valid(frame) &&
           (motor_unpack_getid(MotorType::MOTOR_TYPE_MR, data, out.motor_id,
                               out.ok) == 0);
      if (ok) {
        out.addr = 36U;
        out.value = out.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_getid(MotorType::MOTOR_TYPE_LS, data, out.motor_id,
                               out.ok) == 0);
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_SETID:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      if (data[2] == 0x55U && data[3] == 0x08U) {
        ok = (motor_unpack_regw(MotorType::MOTOR_TYPE_DM, data, mid8, out.addr,
                                out.value) == 0);
        out.motor_id = mid8;
        out.ok = ok;
      } else if (data[2] == 0xAAU && data[3] == 0x08U) {
        ok = (motor_unpack_regs(MotorType::MOTOR_TYPE_DM, data, mid8, out.addr,
                                out.value) == 0);
        out.motor_id = mid8;
        out.ok = ok;
      } else {
        ok = false;
        out.ok = false;
      }
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_rs_unpack_canid_setid(frame.can_id, out.motor_id, out.ok) ==
            0);
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_setid(MotorType::MOTOR_TYPE_ENC, data, out.motor_id,
                               out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_MR:
      /* 写 Motor ID(Index=36) 回包 / 存 Flash(Index=0) 回包 */
      if (!mr_param_frame_valid(frame)) {
        ok = false;
        out.ok = false;
      } else if (data[1] == 36U) {
        ok = (motor_unpack_regw(MotorType::MOTOR_TYPE_MR, data, mid8, out.addr,
                                out.value) == 0);
        out.motor_id = mid8;
        out.ok = ok;
      } else if (data[1] == 0x00U) {
        ok = (motor_unpack_regs(MotorType::MOTOR_TYPE_MR, data, mid8, out.addr,
                                out.value) == 0);
        out.motor_id = mid8;
        out.ok = ok;
      } else {
        ok = false;
        out.ok = false;
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_setid(MotorType::MOTOR_TYPE_LS, data, out.motor_id,
                               out.ok) == 0);
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_RESETID:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      /* 与 SETID 相同: 写 0x08=0x01 回包 / 保存回包 */
      if (data[2] == 0x55U && data[3] == 0x08U) {
        ok = (motor_unpack_regw(MotorType::MOTOR_TYPE_DM, data, mid8, out.addr,
                                out.value) == 0);
        out.motor_id = mid8;
        out.ok = ok;
      } else if (data[2] == 0xAAU && data[3] == 0x08U) {
        ok = (motor_unpack_regs(MotorType::MOTOR_TYPE_DM, data, mid8, out.addr,
                                out.value) == 0);
        out.motor_id = mid8;
        out.ok = ok;
      } else {
        ok = false;
        out.ok = false;
      }
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_rs_unpack_canid_resetid(frame.can_id, out.motor_id, out.ok) ==
            0);
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_resetid(MotorType::MOTOR_TYPE_ENC, data, out.motor_id,
                                 out.ok) == 0);
      break;
    case MotorType::MOTOR_TYPE_MR:
      if (!mr_param_frame_valid(frame)) {
        ok = false;
        out.ok = false;
      } else if (data[1] == 36U) {
        ok = (motor_unpack_regw(MotorType::MOTOR_TYPE_MR, data, mid8, out.addr,
                                out.value) == 0);
        ok = ok && (out.value == 1U);
        out.motor_id = mid8;
        out.ok = ok;
      } else if (data[1] == 0x00U) {
        ok = (motor_unpack_regs(MotorType::MOTOR_TYPE_MR, data, mid8, out.addr,
                                out.value) == 0);
        out.motor_id = mid8;
        out.ok = ok;
      } else {
        ok = false;
        out.ok = false;
      }
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_resetid(MotorType::MOTOR_TYPE_LS, data, out.motor_id,
                                 out.ok) == 0);
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_READREG:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_regr(MotorType::MOTOR_TYPE_DM, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_unpack_regr(MotorType::MOTOR_TYPE_RS, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      if (ok) {
        ok = (motor_rs_unpack_canid_regr(frame.can_id, mid8, out.ok) == 0);
        out.motor_id = mid8;
      }
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_regr(MotorType::MOTOR_TYPE_ENC, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = mr_param_frame_valid(frame) &&
           (motor_unpack_regr(MotorType::MOTOR_TYPE_MR, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      out.ok = ok;
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_regr(MotorType::MOTOR_TYPE_LS, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_WRITEREG:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_regw(MotorType::MOTOR_TYPE_DM, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_unpack_mit(*protocols, MotorType::MOTOR_TYPE_RS, data, out.mit) == 0);
      if (ok) {
        ok = (motor_rs_unpack_canid_mit(frame.can_id, out.mit) == 0);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_regw(MotorType::MOTOR_TYPE_ENC, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = mr_param_frame_valid(frame) &&
           (motor_unpack_regw(MotorType::MOTOR_TYPE_MR, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      out.ok = ok;
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_regw(MotorType::MOTOR_TYPE_LS, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    default:
      ok = false;
      break;
    }
    break;
  case MotorCommand::MOTOR_CMD_SAVEREG:
    switch (type) {
    case MotorType::MOTOR_TYPE_DM:
      ok = (motor_unpack_regs(MotorType::MOTOR_TYPE_DM, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    case MotorType::MOTOR_TYPE_RS:
      ok = (motor_unpack_mit(*protocols, MotorType::MOTOR_TYPE_RS, data, out.mit) == 0);
      if (ok) {
        ok = (motor_rs_unpack_canid_mit(frame.can_id, out.mit) == 0);
        out.motor_id = out.mit.motor_id;
      }
      break;
    case MotorType::MOTOR_TYPE_ENC:
      ok = (motor_unpack_regs(MotorType::MOTOR_TYPE_ENC, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    case MotorType::MOTOR_TYPE_MR:
      ok = mr_param_frame_valid(frame) &&
           (motor_unpack_regs(MotorType::MOTOR_TYPE_MR, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      out.ok = ok;
      break;
    case MotorType::MOTOR_TYPE_LS:
      ok = (motor_unpack_regs(MotorType::MOTOR_TYPE_LS, data, mid8, out.addr,
                              out.value) == 0);
      out.motor_id = mid8;
      break;
    default:
      ok = false;
      break;
    }
    break;
  default:
    out.command = MotorCommand::MOTOR_CMD_UNKNOWN;
    return false;
  }
  return ok;
}

HeartbeatStatus HpmFrameTransport::heartbeat_get() const {
  std::lock_guard<std::mutex> lock(impl_->hb_mutex);
  HeartbeatStatus s = impl_->hb_status;
  if (impl_->config.heartbeat_timeout_ms > 0 &&
      impl_->hb_status.state == HeartbeatState::Alive) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - impl_->hb_last_rx_time)
                       .count();
    if (elapsed > impl_->config.heartbeat_timeout_ms) {
      s.state = HeartbeatState::Stale;
    }
  }
  return s;
}

UdpTransportStats HpmFrameTransport::udp_stats_get() const {
  std::lock_guard<std::mutex> lock(impl_->udp_status_mutex);
  return impl_->udp_stats;
}

void HpmFrameTransport::set_rcvcallback(FrameCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->tcp_callback = std::move(callback);
}

void HpmFrameTransport::set_motor_snapshot_callback(MotorSnapshotCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->motor_snapshot_callback = std::move(callback);
}

void HpmFrameTransport::set_battery_snapshot_callback(BatterySnapshotCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->battery_snapshot_callback = std::move(callback);
}

void HpmFrameTransport::set_tcplink_callback(TcpLinkCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->tcp_link_callback = std::move(callback);
}

void HpmFrameTransport::set_udp_state_callback(UdpStateCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->udp_state_callback = std::move(callback);
}

void HpmFrameTransport::set_frame_gap_callback(FrameGapCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->frame_gap_callback = std::move(callback);
}

void HpmFrameTransport::set_rcvcallback_udp(FrameCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->tcp_callback = std::move(callback);
}

} // namespace frameTransport
