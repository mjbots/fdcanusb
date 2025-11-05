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

#include "mbed.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"

namespace fw {

class Stm32G4AsyncUsbCdc;

class CanManager {
 public:
  struct Options {
    PinName td = NC;
    PinName rd = NC;
    Stm32G4AsyncUsbCdc* cdc = nullptr;
  };
  CanManager(mjlib::micro::Pool&,
             mjlib::micro::PersistentConfig&,
             mjlib::micro::CommandManager&,
             mjlib::micro::AsyncExclusive<
             mjlib::micro::AsyncWriteStream>& stream,
             const Options&);
  ~CanManager();

  void Poll();
  void PollMillisecond();
  void Poll10Ms();
  void Start();

  struct CanFrame {
    uint32_t id = 0;
    uint8_t size = 0;
    uint8_t data[64] = {};
    bool extended = false;
    bool remote = false;
    bool fd_frame = false;
    bool bitrate_switch = false;
  };

  using FrameCallback = mjlib::base::inplace_function<void(const CanFrame&)>;
  using TxCompleteCallback = mjlib::base::inplace_function<void(uint32_t context, bool success)>;

  bool bus_active() const;
  void SetBusActive(bool);

  // Send a CAN frame.
  //
  // @param context will be passed to the TX complete callback
  //
  // Returns true if queued successfully.
  bool SendFrame(const CanFrame&, uint32_t context);

  // This callback will be invoked for received frames.
  void RegisterFrameCallback(FrameCallback callback);

  // This callback will be invoked when the frame is accepted by the
  // hardware.
  //
  // Note: It does not indicate that the frame was successfully
  // transmitted over the bus.
  void RegisterTxCompleteCallback(TxCompleteCallback callback);

  struct BitTiming {
    int prescaler = 0;
    int sync_jump_width = 0;
    int time_seg1 = 0;
    int time_seg2 = 0;
  };

  // Update the current timings for nominal or data.  Note, these will
  // not take effect until SetBusActive is used to first disable the
  // bus and then enable it.
  void SetNominalTiming(const BitTiming&);
  void SetDataTiming(const BitTiming&);

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
