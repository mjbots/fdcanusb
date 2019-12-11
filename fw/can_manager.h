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

#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"

namespace fw {

class CanManager {
 public:
  struct Options {
    PinName td = NC;
    PinName rd = NC;
  };
  CanManager(mjlib::micro::Pool&,
             mjlib::micro::PersistentConfig&,
             mjlib::micro::CommandManager&,
             mjlib::micro::AsyncExclusive<
             mjlib::micro::AsyncWriteStream>& stream,
             const Options&);
  ~CanManager();

  void Poll();
  void Poll10Ms();
  void Start();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
