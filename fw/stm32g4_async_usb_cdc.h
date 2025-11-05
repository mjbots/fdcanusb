// Copyright 2018-2025 Josh Pieper, jjp@pobox.com.
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

#include "mbed.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/string_span.h"

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/pool_ptr.h"

#include "usbd_core.h"

namespace fw {

class MillisecondTimer;

class Stm32G4AsyncUsbCdc : public mjlib::micro::AsyncStream {
 public:
  struct Options {
    MillisecondTimer* timer = nullptr;
  };
  Stm32G4AsyncUsbCdc(mjlib::micro::Pool*, const Options&);
  ~Stm32G4AsyncUsbCdc() override;

  void AsyncReadSome(const mjlib::base::string_span&,
                     const mjlib::micro::SizeCallback&) override;
  void AsyncWriteSome(const std::string_view&,
                      const mjlib::micro::SizeCallback&) override;

  void Poll();
  void Poll10Ms();

  // Is the host actively reading data from the CDC interface?
  bool IsCdcActive() const;

  using VendorControlHandler =
      mjlib::base::inplace_function<usbd_respond(
      usbd_device*, usbd_ctlreq*, usbd_rqc_callback*)>;
  void RegisterGsUsbHandler(VendorControlHandler handler);

  using EndpointHandler =
      mjlib::base::inplace_function<void(
          usbd_device* dev, uint8_t event, uint8_t ep)>;
  void RegisterGsUsbEndpointHandlers(EndpointHandler rx_handler,
                                     EndpointHandler tx_handler);

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
