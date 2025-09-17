// Copyright 2019 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fw/can_manager.h"

#include <optional>

#include "mbed.h"

#include "mjlib/base/tokenizer.h"
#include "mjlib/base/visitor.h"

#include "fw/fdcan.h"
#include "fw/fifo.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;

namespace fw {
namespace {

constexpr std::size_t kFilterSize = 16;
constexpr std::size_t kMaxEmittedLine = 256;
constexpr std::size_t kBufferSize = 2048;


struct Config {
  int32_t bitrate = 1000000;
  int32_t fd_bitrate = 5000000;
  float sample_point = 0.666f;
  float fd_sample_point = 0.666f;
  bool automatic_retransmission = true;
  bool fdcan_frame = true;
  bool bitrate_switch = true;
  bool restricted_mode = false;
  bool bus_monitor = false;
  bool termination = true;
  bool autostart = true;
  bool autorecover = false;

  // We by default start out set for a TCAN1057 with its longer loop delay:
  //  start - 13 / 85MHz ~= 152ns
  //  min   - 1 / 85Mhz ~= 12ns
  bool delay_compensation = true;
  int32_t tdc_offset = 13;
  int32_t tdc_filter = 1;

  struct Global {
    uint8_t std_action = 0;
    uint8_t ext_action = 0;
    uint8_t remote_std_action = 0;
    uint8_t remote_ext_action = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(std_action));
      a->Visit(MJ_NVP(ext_action));
      a->Visit(MJ_NVP(remote_std_action));
      a->Visit(MJ_NVP(remote_ext_action));
    }
  };

  Global global;

  struct Filter {
    uint32_t id1 = 0;
    uint32_t id2 = 0;
    uint8_t mode = 0;
    uint8_t type = 0;
    uint8_t action = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id1));
      a->Visit(MJ_NVP(id2));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(type));
      a->Visit(MJ_NVP(action));
    }
  };

  std::array<Filter, kFilterSize> filter = { {} };

  struct RateConfig {
    int32_t prescaler = -1;
    int32_t sync_jump_width = -1;
    int32_t time_seg1 = -1;
    int32_t time_seg2 = -1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(prescaler));
      a->Visit(MJ_NVP(sync_jump_width));
      a->Visit(MJ_NVP(time_seg1));
      a->Visit(MJ_NVP(time_seg2));
    }

    FDCan::Rate rate() const {
      FDCan::Rate result;
      result.prescaler = prescaler;
      result.sync_jump_width = sync_jump_width;
      result.time_seg1 = time_seg1;
      result.time_seg2 = time_seg2;
      return result;
    }
  };

  RateConfig rate;
  RateConfig fdrate;

  // If no progress has been made sending frames in the last N ms,
  // cancel the entire hardware CAN queue.  This can keep hardware
  // from getting stuck if a frame is sent that nothing will ever
  // acknowledge.  If zero, then never cancel frames.
  uint32_t cancel_all_ms = 50;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(bitrate));
    a->Visit(MJ_NVP(fd_bitrate));
    a->Visit(MJ_NVP(sample_point));
    a->Visit(MJ_NVP(fd_sample_point));
    a->Visit(MJ_NVP(automatic_retransmission));
    a->Visit(MJ_NVP(fdcan_frame));
    a->Visit(MJ_NVP(bitrate_switch));
    a->Visit(MJ_NVP(restricted_mode));
    a->Visit(MJ_NVP(bus_monitor));
    a->Visit(MJ_NVP(termination));
    a->Visit(MJ_NVP(autostart));
    a->Visit(MJ_NVP(autorecover));
    a->Visit(MJ_NVP(delay_compensation));
    a->Visit(MJ_NVP(tdc_offset));
    a->Visit(MJ_NVP(tdc_filter));
    a->Visit(MJ_NVP(global));
    a->Visit(MJ_NVP(filter));
    a->Visit(MJ_NVP(rate));
    a->Visit(MJ_NVP(fdrate));
    a->Visit(MJ_NVP(cancel_all_ms));
  }
};

int ParseHexNybble(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  return -1;
}

int ParseHexByte(const char* value) {
  int high = ParseHexNybble(value[0]);
  if (high < 0) { return high; }
  int low = ParseHexNybble(value[1]);
  if (low < 0) { return low; }
  return (high << 4) | low;
}
}

class CanManager::Impl {
 public:
  Impl(micro::PersistentConfig& persistent_config,
       micro::CommandManager& command_manager,
       micro::AsyncExclusive<micro::AsyncWriteStream>& stream,
       const Options& options)
      : stream_(stream),
        options_(options) {
    persistent_config.Register("can", &config_, [this]() {
        this->UpdateConfig();
      });
    command_manager.Register("can", [this](auto&& command, auto&& response) {
        this->Command(command, response);
      });
  }

  void Start() {
    if (!config_.autostart) { return; }

    StartCan();
  }

  void UpdateConfig() {
    can_term_.write(config_.termination ? 0 : 1);
  }

  void Command(const std::string_view& command,
               const micro::CommandManager::Response& response) {
    base::Tokenizer tokenizer(command, " ");

    auto cmd = tokenizer.next();
    if (cmd == "on") {
      Command_BusOn(response);
    } else if (cmd == "off") {
      Command_BusOff(response);
    } else if (cmd == "std" || cmd == "ext" || cmd == "send") {
      Command_Send(cmd, tokenizer.remaining(), response);
    } else if (cmd == "status") {
      Command_Status(response);
    } else if (cmd == "config") {
      Command_Config(response);
    } else {
      WriteMessage("ERR unknown subcommand\r\n", response);
    }
  }

  void Command_BusOn(const micro::CommandManager::Response& response) {
    if (can_) {
      WriteMessage("ERR already in BusOn\r\n", response);
      return;
    }

    StartCan();

    WriteOK(response);
  }

  void StartCan() {
    auto map_mode = [](auto value) {
      using FM = FDCan::FilterMode;
      switch (value) {
        case 0: return FM::kRange;
        case 1: return FM::kDual;
        case 2: return FM::kMask;
      }
      return FM::kMask;
    };

    auto map_action = [](auto value) {
      using FA = FDCan::FilterAction;
      switch (value) {
        case 0: return FA::kDisable;
        case 1: return FA::kAccept;
        case 2: return FA::kReject;
      }
      return FA::kDisable;
    };

    auto map_type = [](auto value) {
      using FT = FDCan::FilterType;
      switch (value) {
        case 0: return FT::kStandard;
        case 1: return FT::kExtended;
      }
      return FT::kStandard;
    };

    // Update our filters.
    for (size_t i = 0; i < config_.filter.size(); i++) {
      const auto& src = config_.filter[i];
      auto& dst = fdcan_filter_[i];

      dst.id1 = src.id1;
      dst.id2 = src.id2;
      dst.mode = map_mode(src.mode);
      dst.action = map_action(src.action);
      dst.type = map_type(src.type);
    }

    auto map_global_action = [](auto value) {
      switch (value) {
        case 0:
        case 1: {
          return FDCan::FilterAction::kAccept;
        }
        case 2: {
          return FDCan::FilterAction::kReject;
        }
      }
      return FDCan::FilterAction::kReject;
    };

    can_.emplace([&]() {
        FDCan::Options options;
        options.td = options_.td;
        options.rd = options_.rd;

        options.slow_bitrate = config_.bitrate;
        options.slow_sample_point = config_.sample_point;
        options.fast_bitrate = config_.fd_bitrate;
        options.fast_sample_point = config_.fd_sample_point;
        options.automatic_retransmission = config_.automatic_retransmission;
        options.fdcan_frame = config_.fdcan_frame;
        options.bitrate_switch = config_.bitrate_switch;
        options.restricted_mode = config_.restricted_mode;
        options.bus_monitor = config_.bus_monitor;
        options.delay_compensation = config_.delay_compensation;
        options.tdc_offset = config_.tdc_offset;
        options.tdc_filter = config_.tdc_filter;

        options.global_std_action =
            map_global_action(config_.global.std_action);
        options.global_ext_action =
            map_global_action(config_.global.ext_action);
        options.global_remote_std_action =
            map_global_action(config_.global.remote_std_action);
        options.global_remote_ext_action =
            map_global_action(config_.global.remote_ext_action);

        options.filter_begin = fdcan_filter_.begin();
        options.filter_end = fdcan_filter_.end();

        options.rate_override = config_.rate.rate();
        options.fdrate_override = config_.fdrate.rate();

        return options;
      }());
  }

  void Command_BusOff(const micro::CommandManager::Response& response) {
    if (!can_) {
      WriteMessage("ERR already in BusOff\r\n", response);
      return;
    }

    can_.reset();
    WriteOK(response);
  }

  void Command_Send(std::string_view command, std::string_view remainder,
                    const micro::CommandManager::Response& response) {
    if (!can_) {
      WriteMessage("ERR not in BusOn state\r\n", response);
      return;
    }

    base::Tokenizer tokenizer(remainder, " ");
    const auto hexid = tokenizer.next();
    const auto hexdata = tokenizer.next();
    if (hexdata.size() == 0) {
      WriteMessage("ERR insufficient options\r\n", response);
      return;
    }

    FDCan::SendOptions opts;
    using FO = FDCan::Override;

    for (char c : tokenizer.remaining()) {
      if (c == 'B') { opts.bitrate_switch = FO::kRequire; }
      else if (c == 'b') { opts.bitrate_switch = FO::kDisable; }
      else if (c == 'F') { opts.fdcan_frame = FO::kRequire; }
      else if (c == 'f') { opts.fdcan_frame = FO::kDisable; }
      else if (c == 'R') { opts.remote_frame = FO::kRequire; }
      else if (c == 'r') { opts.remote_frame = FO::kDisable; }
      else {
        WriteMessage("ERR unknown flag\r\n", response);
        return;
      }
    }

    const auto id = std::strtol(hexid.data(), nullptr, 16);
    if (id < 0 || id >= (1 << 29)) {
      WriteMessage("ERR bad id\r\n", response);
      return;
    }

    if ((hexdata.size() % 2) != 0) {
      WriteMessage("ERR data invalid length\r\n", response);
      return;
    }

    char data[64] = {};
    ssize_t bytes = 0;

    for (size_t i = 0; i < hexdata.size(); i += 2) {
      int value = ParseHexByte(&hexdata[i]);
      if (value < 0) {
        WriteMessage("ERR invalid data\r\n", response);
        return;
      }
      data[bytes++] = value;
    }

    if (command == "std") {
      opts.extended_id = FDCan::Override::kDisable;
    } else if (command == "ext") {
      opts.extended_id = FDCan::Override::kRequire;
    }

    led_tx_.write(1);

    // If we already have things queued, then don't even try to send
    // more.  That way we send things out in the order we were asked
    // to.
    const auto result =
        !send_queue_.empty() ?
        FDCan::kNoSpace :
        can_->Send(id, std::string_view(data, bytes), opts);
    switch (result) {
      case FDCan::kNoSpace: {
        // Attempt to stick this into one of our send queue entries.
        SendQueueItem item;

        std::memcpy(item.data, data, bytes);
        item.bytes = bytes;
        item.id = id;
        item.opts = opts;

        send_queue_.push(item);
        break;
      }
      case FDCan::kSuccess: {
        // All good here!

        // Clear out our cancel count.
        can_cancel_all_count_ = 0;
        break;
      }
    }

    WriteOK(response);
  }

  void Command_Status(const micro::CommandManager::Response& response) {
    if (!can_) {
      WriteMessage("ERR not in BusOn\r\n", response);
      return;
    }

    auto status = can_->status();
    snprintf(status_line_, sizeof(status_line_),
             "lec=%lu dlec=%lu err=%lu warn=%lu busoff=%lu "
             "pexc=%lu tdc=%lu\r\n",
             status.LastErrorCode,
             status.DataLastErrorCode,
             status.ErrorPassive,
             status.Warning,
             status.BusOff,
             status.ProtocolException,
             status.TDCvalue);
    WriteMessage(status_line_, response);
  }

  void Command_Config(const micro::CommandManager::Response& response) {
    if (!can_) {
      WriteMessage("ERR not in BusOn\r\n", response);
      return;
    }

    const auto config = can_->config();
    snprintf(status_line_, sizeof(status_line_),
             "clk=%d "
             "np=%d nsjw=%d nts1=%d nts2=%d "
             "dp=%d dsjw=%d dts1=%d dts2=%d\r\n",
             config.clock,
             config.nominal.prescaler, config.nominal.sync_jump_width,
             config.nominal.time_seg1, config.nominal.time_seg2,
             config.data.prescaler, config.data.sync_jump_width,
             config.data.time_seg1, config.data.time_seg2);
    WriteMessage(status_line_, response);
  }

  void WriteOK(const micro::CommandManager::Response& response) {
    WriteMessage("OK\r\n", response);
  }

  void WriteMessage(const std::string_view& message,
                    const micro::CommandManager::Response& response) {
    micro::AsyncWrite(*response.stream, message, response.callback);
  }

  void PollMillisecond() {
    can_cancel_all_count_++;

    if (can_ &&
        can_cancel_all_count_ > config_.cancel_all_ms &&
        config_.cancel_all_ms > 0) {
      can_->CancelAll();
      can_cancel_all_count_ = 0;
    }
  }

  void Poll10Ms() {
    led_rx_.write(0);
    led_tx_.write(0);

    if (config_.autorecover) {
      const auto status = can_->status();
      if (status.BusOff) {
        can_->RecoverBusOff();
      }
    }
  }

  void Poll() {
    if (!can_) { return; }

    // First, see if we can emit any more CAN frames onto the bus.
    if (!send_queue_.empty()) {
      // Check to see if there is space to emit anything.
      if (!can_->tx_queue_full()) {
        // We can send something!
        const auto item = send_queue_.top();

        const auto result = can_->Send(
            item.id, std::string_view(item.data, item.bytes), item.opts);

        switch (result) {
          case FDCan::kSuccess: {
            send_queue_.pop();
            break;
          }
          case FDCan::kNoSpace: {
            // Huh?
            break;
          }
        }
      }
    }

    // Then check if we can send anything over USB.
    if (!write_outstanding_ &&
        device_buffer_->pos > 0) {
      // We have data we need to emit.
      EmitData();
    }

    // We cannot accept more frames over USB if we are currently out
    // of room to save the results.
    if ((sizeof(device_buffer_->buf) - device_buffer_->pos) <
        kMaxEmittedLine) {
      // We do not have enough room remaining in our device buffer to
      // emit another line.
      return;
    }

    // Finally, try to look for a new inbound CAN frame.
    const bool frame_found = can_->Poll(&rx_header_, rx_data_);
    if (!frame_found) {
      return;
    }

    led_rx_.write(1);

    auto pos = &device_buffer_->pos;
    auto buf = &device_buffer_->buf[0];

    auto write_char = [&](uint8_t c) {
      if ((kBufferSize - *pos) < 2) {
        return;
      }

      buf[*pos] = c;
      (*pos)++;
    };

    auto write_str = [&](auto str, size_t length) {
      if ((kBufferSize - *pos) < length) {
        return;
      }

      // This is intended to only be used with string constants, whose
      // size includes the null terminator.  Thus we have to remove
      // it.
      std::memcpy(&buf[*pos], &str[0], length);
      *pos += length;
    };

    auto write_hex = [&](uint8_t byte) {
      const auto high_nyb = byte >> 4;
      const auto low_nyb = byte & 0x0f;
      constexpr const char* tohex = "0123456789ABCDEF";

      write_char(tohex[high_nyb]);
      write_char(tohex[low_nyb]);
    };

    auto write_hex32 = [&](uint32_t word) {
      if (word > 0xffffff) { write_hex((word >> 24) & 0xff); }
      if (word > 0xffff) { write_hex((word >> 16) & 0xff); }
      if (word > 0xff) { write_hex((word >> 8) & 0xff); }
      write_hex(word & 0xff);
    };


    write_str("rcv ", 4);

    write_hex32(rx_header_.Identifier);
    write_char(' ');

    const int dlc = [&]() {
      auto result = FDCan::ParseDlc(rx_header_.DataLength);
      if (rx_header_.FDFormat != FDCAN_FD_CAN) {
        result = std::min(8, result);
      }
      return result;
    }();

    for (int i = 0; i < dlc; i++) {
      write_hex(rx_data_[i]);
    }

    write_char(' ');
    write_char((rx_header_.IdType == FDCAN_EXTENDED_ID) ? 'E' : 'e');
    write_char(' ');
    write_char((rx_header_.BitRateSwitch == FDCAN_BRS_ON) ? 'B' : 'b');
    write_char(' ');
    write_char((rx_header_.FDFormat == FDCAN_FD_CAN) ? 'F' : 'f');
    write_char(' ');
    write_char((rx_header_.RxFrameType == FDCAN_REMOTE_FRAME) ? 'R' : 'r');
    write_char(' ');
    write_char('f');
    if (!rx_header_.IsFilterMatchingFrame) {
      write_hex(rx_header_.FilterIndex);
    } else {
      write_str("-1", 2);
    }

    write_str("\r\n", 2);

    buf[(*pos)] = 0;

    // See if we can start our new emission.
    if (write_outstanding_) { return; }

    EmitData();
  }

  void EmitData() {
    // We can swap buffers and start sending one.
    std::swap(device_buffer_, host_buffer_);
    device_buffer_->pos = 0;

    write_outstanding_ = true;

    stream_.AsyncStart(
        [this](micro::AsyncWriteStream* write_stream,
               micro::VoidCallback done_callback) {

          done_callback_ = done_callback;
          micro::AsyncWrite(
              *write_stream,
              std::string_view(host_buffer_->buf, host_buffer_->pos),
              [this](auto ec) {
                auto done = this->done_callback_;
                this->done_callback_ = {};
                this->write_outstanding_ = false;
                done();
              });
        });
  }

  micro::AsyncExclusive<micro::AsyncWriteStream>& stream_;
  const Options options_;
  Config config_;
  std::optional<FDCan> can_;
  std::array<FDCan::Filter, kFilterSize> fdcan_filter_ = { {} };

  DigitalOut can_stb_{PB_11, 0};
  DigitalOut can_shdn_{PB_15, 0};
  DigitalOut can_term_{PB_14, 0};

  DigitalOut led_rx_{PB_4, 0};
  DigitalOut led_tx_{PB_3, 0};

  FDCAN_RxHeaderTypeDef rx_header_ = {};
  char rx_data_[64] = {};
  bool write_outstanding_ = false;
  char status_line_[256] = {};

  struct OutputBuffer {
    size_t pos = 0;
    char buf[kBufferSize] = {};
  };

  OutputBuffer emit_buffer1_, emit_buffer2_;

  OutputBuffer* device_buffer_ = &emit_buffer1_;
  OutputBuffer* host_buffer_ = &emit_buffer2_;

  struct SendQueueItem {
    char data[64] = {};
    uint8_t bytes = 0;
    uint32_t id = 0;
    FDCan::SendOptions opts;
  };

  Fifo<SendQueueItem, 32> send_queue_;

  micro::VoidCallback done_callback_;

  uint32_t can_cancel_all_count_ = 0;
};

CanManager::CanManager(micro::Pool& pool,
                       micro::PersistentConfig& persistent_config,
                       micro::CommandManager& command_manager,
                       micro::AsyncExclusive<micro::AsyncWriteStream>& stream,
                       const Options& options)
    : impl_(&pool, persistent_config, command_manager, stream, options) {}

CanManager::~CanManager() {}

void CanManager::Poll() {
  impl_->Poll();
}

void CanManager::PollMillisecond() {
  impl_->PollMillisecond();
}

void CanManager::Poll10Ms() {
  impl_->Poll10Ms();
}

void CanManager::Start() {
  impl_->Start();
}

}
