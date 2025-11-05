// Copyright 2025 Josh Pieper, jjp@pobox.com.
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

#include <cstdint>

#include "usbd_core.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/micro/pool_ptr.h"

#include "fw/can_manager.h"
#include "fw/millisecond_timer.h"

namespace fw {

// Implements the gs_usb vendor-specific USB protocol for CAN adapters.
// This provides Linux SocketCAN compatibility via the mainline gs_usb kernel driver.
//
// The gs_usb protocol uses:
// - USB vendor control requests for configuration (bit timing, mode, etc.)
// - Bulk endpoints for CAN frame transfer (EP2 OUT, EP1 IN - standard gs_usb endpoints)
//
// This class integrates with CanManager for actual CAN bus operations.
class Stm32G4GsUsb {
 public:
  struct Options {
    // CAN peripheral clock frequency (for bit timing calculations)
    // Should match the actual FDCAN clock configuration
    mjlib::base::inplace_function<int32_t()> get_can_clock_hz;

    // Power LED for identification (optional, can be nullptr)
    DigitalOut* power_led = nullptr;

    // Microsecond timer for hardware timestamps (optional, can be nullptr)
    MillisecondTimer* timer = nullptr;
  };

  Stm32G4GsUsb(mjlib::micro::Pool&, CanManager&, const Options&);
  ~Stm32G4GsUsb();

  // USB device callbacks
  // These are called by the USB stack and should be registered with usbd_device

  // Handle vendor control requests on EP0 (interface 0)
  // Returns usbd_ack if handled, usbd_fail otherwise
  usbd_respond HandleControl(usbd_device* dev, usbd_ctlreq* req, usbd_rqc_callback* callback);

  // Handle bulk endpoint RX (EP2 OUT - host sending CAN frames to device)
  void HandleRxEndpoint(usbd_device* dev, uint8_t event, uint8_t ep);

  // Handle bulk endpoint TX (EP1 IN - device sending CAN frames to host)
  void HandleTxEndpoint(usbd_device* dev, uint8_t event, uint8_t ep);

  // CAN callbacks
  // These are called by CanManager when CAN events occur

  // Called when a CAN frame is received from the bus
  void OnCanFrameReceived(const CanManager::CanFrame& frame);

  // Called when a CAN frame transmission is complete
  // echo_id identifies which frame completed (for TX confirmations)
  void OnCanFrameTxComplete(uint32_t echo_id, bool success);

  // Polling
  void Poll();
  void PollMillisecond();  // Called every millisecond for LED blinking, etc.

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}  // namespace fw
