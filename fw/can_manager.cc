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
  int32_t bitrate = 250000;
  int32_t fd_bitrate = 250000;
  bool automatic_retransmission = false;
  bool fdcan_frame = true;
  bool bitrate_switch = true;
  bool restricted_mode = false;
  bool bus_monitor = false;
  bool termination = true;

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
    a->Visit(MJ_NVP(global));
    a->Visit(MJ_NVP(filter));
  }
};

int ParseDlc(uint32_t dlc_code) {
  if (dlc_code == FDCAN_DLC_BYTES_0) { return 0; }
  if (dlc_code == FDCAN_DLC_BYTES_1) { return 1; }
  if (dlc_code == FDCAN_DLC_BYTES_2) { return 2; }
  if (dlc_code == FDCAN_DLC_BYTES_3) { return 3; }
  if (dlc_code == FDCAN_DLC_BYTES_4) { return 4; }
  if (dlc_code == FDCAN_DLC_BYTES_5) { return 5; }
  if (dlc_code == FDCAN_DLC_BYTES_6) { return 6; }
  if (dlc_code == FDCAN_DLC_BYTES_7) { return 7; }
  if (dlc_code == FDCAN_DLC_BYTES_8) { return 8; }
  if (dlc_code == FDCAN_DLC_BYTES_12) { return 12; }
  if (dlc_code == FDCAN_DLC_BYTES_16) { return 16; }
  if (dlc_code == FDCAN_DLC_BYTES_20) { return 20; }
  if (dlc_code == FDCAN_DLC_BYTES_24) { return 24; }
  if (dlc_code == FDCAN_DLC_BYTES_32) { return 32; }
  if (dlc_code == FDCAN_DLC_BYTES_48) { return 48; }
  if (dlc_code == FDCAN_DLC_BYTES_64) { return 64; }
  mbed_die();
  return 0;
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
    persistent_config.Register("can", &config_, [this]() {});
    command_manager.Register("can", [this](auto&& command, auto&& response) {
        this->Command(command, response);
      });
  }

  void Command(const std::string_view& command,
               const micro::CommandManager::Response& response) {
    base::Tokenizer tokenizer(command, " ");

    auto cmd = tokenizer.next();
    if (cmd == "on") {
      Command_BusOn(response);
    } else if (cmd == "off") {
      Command_BusOff(response);
    } else if (cmd == "std" || cmd == "ext") {
      Command_Send(cmd, tokenizer.remaining(), response);
    } else if (cmd == "status") {
      Command_Status(response);
    } else {
      WriteMessage("ERR unknown subcommand\r\n", response);
    }
  }

  void Command_BusOn(const micro::CommandManager::Response& response) {
    if (can_) {
      WriteMessage("ERR already in BusOn\r\n", response);
      return;
    }

    auto map_mode = [](auto value) {
      switch (value) {
        case 0: return FDCan::kRange;
        case 1: return FDCan::kDual;
        case 2: return FDCan::kMask;
      }
      return FDCan::kMask;
    };

    auto map_action = [](auto value) {
      switch (value) {
        case 0: return FDCan::kDisable;
        case 1: return FDCan::kAccept;
        case 2: return FDCan::kReject;
      }
      return FDCan::kDisable;
    };

    auto map_type = [](auto value) {
      switch (value) {
        case 0: return FDCan::kStandard;
        case 1: return FDCan::kExtended;
      }
      return FDCan::kStandard;
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
          return FDCan::kAccept;
        }
        case 2: {
          return FDCan::kReject;
        }
      }
      return FDCan::kReject;
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

        return options;
      }());

    WriteOK(response);
  }

  void Command_BusOff(const micro::CommandManager::Response& response) {
    if (!can_) {
      WriteMessage("ERR already in BusOff\r\n", response);
      return;
    }

    can_.reset();
    WriteOK(response);
  }

  void Command_Send(std::string_view command, std::string_view data,
                    const micro::CommandManager::Response& response) {
    WriteMessage("ERR not implemented\r\n", response);
  }

  void Command_Status(const micro::CommandManager::Response& response) {
    WriteMessage("ERR not implemented\r\n", response);
  }

  void WriteOK(const micro::CommandManager::Response& response) {
    WriteMessage("OK\r\n", response);
  }

  void WriteMessage(const std::string_view& message,
                    const micro::CommandManager::Response& response) {
    micro::AsyncWrite(*response.stream, message, response.callback);
  }

  void Poll() {
    if (!can_) { return; }

    const bool frame_found = can_->Poll(&rx_header_, rx_data_);
    if (!frame_found) { return; }

    if (write_outstanding_) { return; }

    write_outstanding_ = true;
    ssize_t pos = 0;
    auto fmt = [&](auto ...args) {
      pos += snprintf(&emit_line_[pos], sizeof(emit_line_) - pos, args...);
    };

    fmt("rcv %X ", rx_header_.Identifier);
    const int dlc = ParseDlc(rx_header_.DataLength);
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

  FDCAN_RxHeaderTypeDef rx_header_ = {};
  char rx_data_[64] = {};
  bool write_outstanding_ = false;
  char emit_line_[256] = {};

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

}