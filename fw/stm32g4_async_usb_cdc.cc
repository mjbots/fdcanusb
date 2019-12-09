// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#include "fw/stm32g4_async_usb_cdc.h"

namespace micro = mjlib::micro;

namespace fw {

class Stm32G4AsyncUsbCdc::Impl {
 public:
  Impl(const Options&) {}

  void AsyncReadSome(const mjlib::base::string_span&,
                     const micro::SizeCallback&) {
  }

  void AsyncWriteSome(const std::string_view&,
                      const micro::SizeCallback&) {
  }
};

Stm32G4AsyncUsbCdc::Stm32G4AsyncUsbCdc(mjlib::micro::Pool* pool,
                                       const Options& options)
    : impl_(pool, options) {}

Stm32G4AsyncUsbCdc::~Stm32G4AsyncUsbCdc() {}

void Stm32G4AsyncUsbCdc::AsyncReadSome(const mjlib::base::string_span& data,
                                       const micro::SizeCallback& callback) {
  impl_->AsyncReadSome(data, callback);
}

void Stm32G4AsyncUsbCdc::AsyncWriteSome(const std::string_view& data,
                                        const micro::SizeCallback& callback) {
  impl_->AsyncWriteSome(data, callback);
}

void Stm32G4AsyncUsbCdc::Poll() {
}

}
