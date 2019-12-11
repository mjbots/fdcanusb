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

#pragma once

#include "mbed.h"

#include "mjlib/base/string_span.h"

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/pool_ptr.h"

namespace fw {

class Stm32G4AsyncUsbCdc : public mjlib::micro::AsyncStream {
 public:
  struct Options {
  };
  Stm32G4AsyncUsbCdc(mjlib::micro::Pool*, const Options&);
  ~Stm32G4AsyncUsbCdc() override;

  void AsyncReadSome(const mjlib::base::string_span&,
                     const mjlib::micro::SizeCallback&) override;
  void AsyncWriteSome(const std::string_view&,
                      const mjlib::micro::SizeCallback&) override;

  void Poll();
  void Poll10Ms();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
