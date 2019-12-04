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

#pragma once

#include <string_view>

#include "mbed.h"

#include "mjlib/base/string_span.h"

namespace fw {

class FDCan {
 public:
  struct Options {
    PinName td = NC;
    PinName rd = NC;
    int slow_bitrate = 1000000;
    int fast_bitrate = 5000000;

    bool automatic_retransmission = false;
    bool fdcan_frame = false;
    bool bitrate_switch = false;
    bool restricted_mode = false;
    bool bus_monitor = false;

    Options() {}
  };

  FDCan(const Options& options = Options());

  void Send(uint16_t dest_id,
            std::string_view data);

  /// @return true if a packet was available.
  bool Poll(FDCAN_RxHeaderTypeDef* header, mjlib::base::string_span);

 private:
  FDCAN_GlobalTypeDef* can_ = nullptr;
  FDCAN_HandleTypeDef hfdcan1_;
  uint32_t last_tx_request_ = 0;
};

}
