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

struct Config {
  int32_t bitrate = 250000;
  int32_t fd_bitrate = 250000;
  bool automatic_retransmission = false;
  bool fdcan_frame = true;
  bool bitrate_switch = true;
  bool restricted_mode = false;
  bool bus_monitor = false;

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

  std::array<Filter, 16> filter = { {} };

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(bitrate));
    a->Visit(MJ_NVP(fd_bitrate));
    a->Visit(MJ_NVP(automatic_retransmission));
    a->Visit(MJ_NVP(fdcan_frame));
    a->Visit(MJ_NVP(bitrate_switch));
    a->Visit(MJ_NVP(restricted_mode));
    a->Visit(MJ_NVP(bus_monitor));
    a->Visit(MJ_NVP(filter));
  }
};

class CanManager::Impl {
 public:
  Impl(micro::PersistentConfig& persistent_config,
       micro::CommandManager& command_manager,
       micro::AsyncExclusive<micro::AsyncWriteStream>& stream)
      : stream_(stream) {
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

    can_.emplace([&]() {
        FDCan::Options options;
        options.td = PA_12;
        options.rd = PA_11;

        options.slow_bitrate = config_.bitrate;
        options.fast_bitrate = config_.fd_bitrate;
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
  }

  micro::AsyncExclusive<micro::AsyncWriteStream>& stream_;
  Config config_;
  std::optional<FDCan> can_;
};

CanManager::CanManager(micro::Pool& pool,
                       micro::PersistentConfig& persistent_config,
                       micro::CommandManager& command_manager,
                       micro::AsyncExclusive<micro::AsyncWriteStream>& stream)
    : impl_(&pool, persistent_config, command_manager, stream) {}

CanManager::~CanManager() {}

void CanManager::Poll() {
  impl_->Poll();
}

}
