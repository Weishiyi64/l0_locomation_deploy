// Copyright (c) 2026 HPMicro
// SPDX-License-Identifier: BSD-3-Clause
#include "motor_can_log.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <iomanip>
#include <sstream>
#include <string_view>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cwchar>
#include <fcntl.h>
#include <locale.h>
#include <regex>
#include <sys/file.h>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace std::chrono;
namespace {
constexpr size_t queue_limit = 16384;
constexpr char hex[] = "0123456789ABCDEF";

std::string local_timestamp(microseconds since_epoch, bool for_filename) {
  const auto sec = floor<seconds>(since_epoch);
  const auto t = static_cast<time_t>(sec.count());
  tm local{};
  char date[32], fraction[8];
  if (!localtime_r(&t, &local) ||
      !strftime(date, sizeof(date), for_filename ? "%Y%m%d_%H%M%S" : "%Y-%m-%d %H:%M:%S", &local))
    throw std::runtime_error("cannot format local motor log time");
  snprintf(fraction, sizeof(fraction), "%c%06u", for_filename ? '_' : '.',
    static_cast<unsigned>((since_epoch - sec).count()));
  return std::string(date) + fraction;
}

void io_error(const std::string &action) {
  throw std::system_error(errno, std::generic_category(), action);
}

struct File {
  int fd{-1};
  File() = default;
  File(const File &) = delete;
  File &operator=(const File &) = delete;
  ~File() { close(); }
  void close() { if (fd >= 0) { ::close(fd); fd = -1; } }
};

// Serialize quota checks across recorders. A second lock on each live shard
// keeps it out of retention while its writer is between batches.
struct DirectoryLock : File {
  explicit DirectoryLock(const fs::path &dir) {
    fd = ::open(dir.c_str(), O_DIRECTORY | O_RDONLY | O_CLOEXEC);
    if (fd < 0 || flock(fd, LOCK_EX) != 0) io_error("lock " + dir.string());
  }
};

struct Shard {
  fs::path path;
  fs::file_time_type modified;
  uintmax_t bytes;
};

class LogFiles {
public:
  LogFiles(const fs::path &dir, size_t shard, size_t quota)
    : dir_(dir), shard_(shard), quota_(quota) {}

  void begin_batch() {
    static const std::regex filename(
      R"(^motor_can_[0-9]{8}_[0-9]{6}_[0-9]{6}_[0-9]{6,}\.log$)");
    total_ = 0;
    old_.clear();
    for (const auto &e : fs::directory_iterator(dir_)) {
      if (!fs::is_regular_file(e.symlink_status()) ||
          !std::regex_match(e.path().filename().string(), filename)) continue;
      auto bytes = e.file_size();
      total_ += bytes;
      if (e.path() != current_) old_.push_back({e.path(), e.last_write_time(), bytes});
    }
    std::sort(old_.begin(), old_.end(), [](const Shard &a, const Shard &b) {
      return std::tie(a.modified, a.path) > std::tie(b.modified, b.path);
    });
  }

  void open_shard() {
    const auto stamp = local_timestamp(duration_cast<microseconds>(
      system_clock::now().time_since_epoch()), true);
    for (;;) {
      char filename[96];
      snprintf(filename, sizeof(filename), "motor_can_%s_%06lu.log", stamp.c_str(), ++serial_);
      auto path = dir_ / filename;
      file_.fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0640);
      if (file_.fd >= 0) {
        if (flock(file_.fd, LOCK_EX) != 0) {
          const int err = errno;
          file_.close();
          throw std::system_error(err, std::generic_category(), "lock " + path.string());
        }
        current_ = path;
        size_ = 0;
        return;
      }
      if (errno != EEXIST) io_error("create " + path.string());
    }
  }

  void write_rows(const std::vector<std::string> &rows, size_t &written) {
    begin_batch();
    while (written < rows.size()) {
      if (rows[written].size() > shard_)
        throw std::runtime_error("motor log line exceeds shard size");
      if (file_.fd < 0 || size_ + rows[written].size() > shard_) {
        file_.close();
        current_.clear();
        begin_batch();  // the previous shard can now be pruned
        open_shard();
      }
      std::string data;
      size_t end = written;
      while (end < rows.size() && size_ + data.size() + rows[end].size() <= shard_)
        data += rows[end++];
      while (total_ + data.size() > quota_) {
        if (old_.empty()) throw std::runtime_error("motor log quota occupied by active shards");
        auto oldest = old_.back();
        old_.pop_back();
        File candidate;
        candidate.fd = ::open(oldest.path.c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
        if (candidate.fd < 0) io_error("open " + oldest.path.string());
        if (flock(candidate.fd, LOCK_EX | LOCK_NB) != 0) {
          if (errno == EWOULDBLOCK) continue;
          io_error("lock old motor log");
        }
        fs::remove(oldest.path);
        total_ -= oldest.bytes;
      }
      append(data);
      written = end;
    }
  }

private:
  void append(const std::string &data) {
    size_t offset = 0;
    while (offset < data.size()) {
      ssize_t n = ::pwrite(file_.fd, data.data() + offset, data.size() - offset, size_ + offset);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) {
        const int err = n < 0 ? errno : EIO;
        if (offset && ftruncate(file_.fd, static_cast<off_t>(size_)) < 0) {
          const int rollback_error = errno;
          file_.close();
          current_.clear();
          throw std::system_error(rollback_error, std::generic_category(), "rollback partial motor log write");
        }
        throw std::system_error(err, std::generic_category(), "write motor log");
      }
      offset += static_cast<size_t>(n);
    }
    size_ += data.size();
    total_ += data.size();
  }
  fs::path dir_, current_;
  size_t shard_, quota_, size_{0};
  uintmax_t total_{0};
  unsigned long serial_{0};
  File file_;
  std::vector<Shard> old_;
};

bool matches_rx(const hpmMotor::MotorDefinition &motor, const frameTransport::Frame &f) {
  using Type = frameTransport::MotorType;
  const auto id = motor.motor_id;
  if (motor.protocol == Type::MOTOR_TYPE_RS)
    return f.ide == frameTransport::kFrameIdeExtended && ((f.can_id >> 8) & 0xff) == id;
  if (f.ide != frameTransport::kFrameIdeStandard) return false;
  if ((motor.protocol == Type::MOTOR_TYPE_LS || motor.protocol == Type::MOTOR_TYPE_ENC) &&
      f.can_id == 0x7ff && (f.dlc == 4 || f.dlc == 5)) {
    if (f.dlc == 5 && f.data[0] == 0xff && f.data[1] == 0xff && f.data[2] == 1)
      return ((uint32_t(f.data[3]) << 8) | f.data[4]) == id;
    return ((uint32_t(f.data[0]) << 8) | f.data[1]) == id;
  }
  switch (motor.protocol) {
    case Type::MOTOR_TYPE_LS: return f.can_id == id;
    case Type::MOTOR_TYPE_ENC: return f.dlc == 8 && (f.data[0] >> 5) == id;
    case Type::MOTOR_TYPE_DM:
      if (f.can_id == 0x7ff && f.dlc == 8) return f.data[0] == id;
      return f.dlc == 8 && (f.data[0] & 15u) == id;
    case Type::MOTOR_TYPE_MR:
      return f.can_id == id || f.can_id == 0x300 + id || f.can_id == 0x600 + id;
    default: return false;
  }
}

std::string quoted_name(const std::string &name) {
  std::string escaped = "\"";
  for (unsigned char ch : name) {
    if (ch < 0x20 || ch == 0x7f) {
      escaped += "\\u00";
      escaped += hex[ch >> 4];
      escaped += hex[ch & 15];
    } else {
      if (ch == '"' || ch == '\\') escaped += '\\';
      escaped += static_cast<char>(ch);
    }
  }
  return escaped + '"';
}

// Count terminal columns once at startup. UTF-8 joint names may include wide or combining characters.
size_t display_width(const std::string &text) {
  static const locale_t utf8 = newlocale(LC_CTYPE_MASK, "C.UTF-8", nullptr);
  if (!utf8) return text.size();
  std::wstring wide(text.size(), L'\0');
  const locale_t previous = uselocale(utf8);
  const size_t count = std::mbstowcs(wide.data(), text.c_str(), wide.size());
  const int width = count == static_cast<size_t>(-1) ? -1 : wcswidth(wide.data(), count);
  uselocale(previous);
  return width < 0 ? text.size() : static_cast<size_t>(width);
}
}  // namespace

MotorCanLog::MotorCanLog(fs::path dir, size_t shard, size_t total,
                         std::vector<hpmMotor::MotorDefinition> motors, rclcpp::Logger logger)
: directory_(std::move(dir)), shard_bytes_(shard), total_bytes_(total),
  motors_(std::move(motors)), logger_(logger) {
  if (!shard || total < shard)
    throw std::invalid_argument("motor log requires 0 < shard bytes <= total bytes");
  name_column_.emplace("unknown", "\"unknown\"");
  name_column_.emplace("broadcast", "\"broadcast\"");
  for (const auto &motor : motors_) name_column_.emplace(motor.name, quoted_name(motor.name));
  size_t width = 0;
  for (const auto &entry : name_column_) width = std::max(width, display_width(entry.second));
  for (auto &entry : name_column_) entry.second.append(width - display_width(entry.second), ' ');
  pending_.reserve(queue_limit);
  thread_ = std::thread(&MotorCanLog::run, this);
}

MotorCanLog::~MotorCanLog() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_one();
  thread_.join();
}

void MotorCanLog::tx(const frameTransport::Frame &frame, uint32_t id) {
  enqueue(frame, true, id, -1);
}
void MotorCanLog::tx(const frameTransport::Frame &frame, uint32_t id, hpmMotor::TxTopic topic) {
  enqueue(frame, true, id, -1, std::move(topic));
}
void MotorCanLog::rx(const frameTransport::Frame &frame) {
  if (frame.msg_id >= 0x10 && frame.msg_id <= 0x1f) enqueue(frame, false, 0, -1);
}

void MotorCanLog::snapshot(const frameTransport::MotorSnapshot &snapshot, hpmMotor::HpmMotor *motor) {
  // Timestamp every raw slot on receipt. Only the file writer waits for the publication result.
  std::promise<Published> publication;
  const auto result = publication.get_future().share();
  for (size_t slot = 0; slot < snapshot.slots.size(); ++slot) {
    const auto &frame = snapshot.slots[slot];
    if ((snapshot.feedback_mask & (1U << slot)) && frame.msg_id >= 0x10 && frame.msg_id <= 0x1f) {
      enqueue(frame, false, 0, slot, result);
    }
  }
  Published published;
  if (motor) {
    published.topic = motor->state_topic();
    try {
      motor->handle_motor_snapshot(snapshot, &published.message);
    } catch (...) {
      publication.set_value(std::move(published));
      throw;
    }
  }
  publication.set_value(std::move(published));
}

void MotorCanLog::enqueue(const frameTransport::Frame &frame, bool tx, uint32_t id, int slot, Topic topic) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++sequence_;
  if (pending_.size() == queue_limit) { ++dropped_; return; }
  pending_.push_back({frame, tx, id, slot,
    duration_cast<microseconds>(system_clock::now().time_since_epoch()).count(),
    duration_cast<microseconds>(steady_clock::now() - start_).count(), sequence_, std::move(topic)});
  if (pending_.size() == 1024) wake_.notify_one();
}

std::string MotorCanLog::row(const Record &r) const {
  const auto &f = r.frame;
  const bool broadcast = r.tx && r.target_id == frameTransport::kFrameCanIdUnused;
  const hpmMotor::MotorDefinition *match = nullptr;
  for (const auto &motor : motors_) {
    if (f.port != motor.bus_index || (r.slot >= 0 && r.slot != motor.index)) continue;
    if (r.tx ? r.target_id == motor.motor_id : matches_rx(motor, f)) {
      if (match) { match = nullptr; break; }
      match = &motor;
    }
  }
  const auto &name = match ? name_column_.at(match->name) :
    name_column_.at(broadcast ? "broadcast" : "unknown");
  const std::string id = match ? std::to_string(match->motor_id) :
    r.tx && !broadcast ? std::to_string(r.target_id) : "-";
  char prefix[32], fields[96], tail[72];
  snprintf(prefix, sizeof(prefix), " %s bus=%-3u motor=", r.tx ? "TX" : "RX",
    static_cast<unsigned>(f.port));
  snprintf(fields, sizeof(fields), " motor_id=%-10s can_id=0x%-8X format=%s dlc=%u data=", id.c_str(), f.can_id,
    f.ide == frameTransport::kFrameIdeExtended ? "EXT" : "STD", static_cast<unsigned>(f.dlc));
  std::string out = local_timestamp(microseconds(r.unix_us), false) + prefix + name + fields;
  const size_t data_start = out.size();
  out += '[';
  for (size_t i = 0; i < f.dlc && i < f.data.size(); ++i) {
    if (i) out += ' ';
    out += hex[f.data[i] >> 4];
    out += hex[f.data[i] & 15];
  }
  out += ']';
  out.resize(data_start + 3 * f.data.size() + 1, ' ');
  snprintf(tail, sizeof(tail), " elapsed_us=%-19lld seq=%-20llu",
    static_cast<long long>(r.elapsed_us), static_cast<unsigned long long>(r.sequence));
  return out + tail + topic_fields(r) + '\n';
}

std::string MotorCanLog::topic_fields(const Record &r) const {
  const auto *tx = std::get_if<hpmMotor::TxTopic>(&r.topic);
  const Published *rx = nullptr;
  const luvbot_hardware_interfaces::msg::MotorState *state = nullptr;
  if (const auto *result = std::get_if<std::shared_future<Published>>(&r.topic)) {
    rx = &result->get();
    for (const auto &item : rx->message.states) {
      if (item.route.motor_type == r.slot) { state = &item; break; }
    }
  }
  if (!tx && !state) return " topic=-";

  const auto &stamp = tx ? tx->stamp : rx->message.stamp;
  char ros_stamp[32];
  snprintf(ros_stamp, sizeof(ros_stamp), "%d.%09u", stamp.sec, stamp.nanosec);
  std::ostringstream out;
  out << std::left << " topic=" << std::setw(23) << (tx ? tx->topic : rx->topic)
      << " sequence_id=" << std::setw(10) << (tx ? tx->sequence_id : rx->message.sequence_id)
      << " stamp=" << std::setw(21) << ros_stamp;
  auto number = [&](const char *key, double value) {
    char buf[32];
    auto end = std::to_chars(buf, buf + sizeof(buf), value).ptr;
    out << ' ' << key << '=' << std::setw(24) << std::string_view(buf, end - buf);
  };
  auto route = [&](const auto &v) {
    out << " motor_type=" << std::setw(3) << +v.motor_type
        << " bus_index=" << std::setw(3) << +v.bus_index << " motor_id=" << std::setw(10) << v.motor_id;
  };
  if (tx) {
    if (const auto *target = std::get_if<luvbot_hardware_interfaces::msg::MotorTarget>(&tx->item)) {
      out << " control_mode=" << std::setw(3) << +tx->control_mode;
      route(target->route);
      number("q", target->q); number("dq", target->dq); number("tau", target->tau);
      number("kp", target->kp); number("kd", target->kd);
    } else {
      out << " motor_type=" << std::setw(3) << +tx->motor_type
          << " enabled=" << std::boolalpha << std::get<bool>(tx->item);
    }
  } else {
    out << " feedback_mask=" << std::setw(10) << rx->message.feedback_mask;
    route(state->route);
    out << " valid=" << std::boolalpha << std::setw(5) << state->valid
        << " error_code=" << std::setw(5) << state->error_code
        << " operating_mode=" << std::setw(3) << +state->operating_mode
        << " enable_state=" << std::setw(3) << +state->enable_state;
    number("q", state->q); number("dq", state->dq); number("tau", state->tau);
    number("mos_temperature", state->mos_temperature);
    number("rotor_temperature", state->rotor_temperature);
  }
  auto text = out.str();
  text.resize(text.find_last_not_of(' ') + 1);
  return text;
}

void MotorCanLog::run() {
  LogFiles files(directory_, shard_bytes_, total_bytes_);
  bool started = false;
  std::vector<Record> batch;
  batch.reserve(queue_limit);
  uint64_t lost = 0, reported = 0;
  auto next_warning = steady_clock::time_point::min();
  for (;;) {
    size_t written = 0;
    std::string error;
    try {
      if (!started) {
        fs::create_directories(directory_);
        DirectoryLock lock(directory_);
        files.open_shard();
        started = true;
      }
      if (!batch.empty()) {
        std::vector<std::string> rows;
        rows.reserve(batch.size());
        for (const auto &record : batch) rows.push_back(row(record));
        DirectoryLock lock(directory_);
        files.write_rows(rows, written);
      }
    } catch (const std::exception &e) {
      error = e.what();
      lost += batch.size() - written;
    }
    batch.clear();
    std::unique_lock<std::mutex> lock(mutex_);
    const auto total_lost = lost + dropped_;
    const bool done = stopping_ && pending_.empty();
    if ((!error.empty() || total_lost != reported) &&
        (done || steady_clock::now() >= next_warning)) {
      lock.unlock();
      RCLCPP_WARN(logger_, "Motor CAN log %s: %s; dropped frames total=%llu",
        directory_.c_str(), error.empty() ? "queue overflow" : error.c_str(),
        static_cast<unsigned long long>(total_lost));
      next_warning = steady_clock::now() + seconds(2);
      reported = total_lost;
      lock.lock();
    }
    if (done) return;
    wake_.wait_for(lock, milliseconds(100), [this] { return stopping_ || pending_.size() >= 1024; });
    batch.swap(pending_);
  }
}
