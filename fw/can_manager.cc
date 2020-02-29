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

namespace base = mjlib::base;
namespace micro = mjlib::micro;

namespace fw {
namespace {

constexpr std::size_t kFilterSize = 16;

struct Config {
  int32_t bitrate = 1000000;
  int32_t fd_bitrate = 5000000;
  bool automatic_retransmission = false;
  bool fdcan_frame = true;
  bool bitrate_switch = true;
  bool restricted_mode = false;
  bool bus_monitor = false;
  bool termination = true;
  bool autostart = true;

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

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(bitrate));
    a->Visit(MJ_NVP(fd_bitrate));
    a->Visit(MJ_NVP(automatic_retransmission));
    a->Visit(MJ_NVP(fdcan_frame));
    a->Visit(MJ_NVP(bitrate_switch));
    a->Visit(MJ_NVP(restricted_mode));
    a->Visit(MJ_NVP(bus_monitor));
    a->Visit(MJ_NVP(termination));
    a->Visit(MJ_NVP(autostart));
    a->Visit(MJ_NVP(global));
    a->Visit(MJ_NVP(filter));
    a->Visit(MJ_NVP(rate));
    a->Visit(MJ_NVP(fdrate));
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
        options.fast_bitrate = config_.fd_bitrate;
        options.automatic_retransmission = config_.automatic_retransmission;
        options.fdcan_frame = config_.fdcan_frame;
        options.bitrate_switch = config_.bitrate_switch;
        options.restricted_mode = config_.restricted_mode;
        options.bus_monitor = config_.bus_monitor;

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

    can_->Send(id, std::string_view(data, bytes), opts);

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

  void Poll10Ms() {
    led_rx_.write(0);
    led_tx_.write(0);
  }

  void Poll() {
    if (!can_) { return; }

    const bool frame_found = can_->Poll(&rx_header_, rx_data_);
    if (!frame_found) { return; }

    if (write_outstanding_) { return; }

    led_rx_.write(1);

    write_outstanding_ = true;
    ssize_t pos = 0;
    auto fmt = [&](auto ...args) {
      pos += snprintf(&emit_line_[pos], sizeof(emit_line_) - pos, args...);
    };

    fmt("rcv %X ", rx_header_.Identifier);
    const int dlc = [&]() {
      auto result = FDCan::ParseDlc(rx_header_.DataLength);
      if (rx_header_.FDFormat != FDCAN_FD_CAN) {
        result = std::min(8, result);
      }
      return result;
    }();
    for (int i = 0; i < dlc; i++) {
      fmt("%02X", rx_data_[i]);
    }

    fmt(" %c %c %c %c f%d\r\n",
        (rx_header_.IdType == FDCAN_EXTENDED_ID) ? 'E' : 'e',
        (rx_header_.BitRateSwitch == FDCAN_BRS_ON) ? 'B' : 'b',
        (rx_header_.FDFormat == FDCAN_FD_CAN) ? 'F' : 'f',
        (rx_header_.RxFrameType == FDCAN_REMOTE_FRAME) ? 'R' : 'r',
        !rx_header_.IsFilterMatchingFrame ? rx_header_.FilterIndex : -1);

    stream_.AsyncStart(
        [this](micro::AsyncWriteStream* write_stream,
               micro::VoidCallback done_callback) {

          done_callback_ = done_callback;
          micro::AsyncWrite(*write_stream, emit_line_, [this](auto ec) {
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
  char emit_line_[256] = {};
  char status_line_[256] = {};

  micro::VoidCallback done_callback_;
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

void CanManager::Poll10Ms() {
  impl_->Poll10Ms();
}

void CanManager::Start() {
  impl_->Start();
}

}
