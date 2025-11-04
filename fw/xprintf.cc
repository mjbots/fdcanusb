// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include "fw/xprintf.h"

#include <stdarg.h>
#include <stdio.h>

#include <optional>

#include "fw/stm32g4_dma_uart.h"

namespace fw {
struct DebugPrint {

  fw::Stm32G4DmaUart uart{
      []() {
        fw::Stm32G4DmaUart::Options options;

        options.tx = PB_3;
        options.rx = PB_4;
        options.baud_rate = 1000000;

        return options;
      }()};

  char buffer[512] = {};
  size_t buffer_size = 0;

  size_t write_outstanding = 0;

  void MaybeStartWrite() {
    // If we don't currently have a DMA write outstanding, make one.
    if (write_outstanding == 0 && buffer_size != 0) {
      write_outstanding = buffer_size;
      uart.start_dma_write({buffer, buffer_size});
    }
  }
};

std::optional<DebugPrint> g_debug_print;

void debug_print_init() {
  g_debug_print.emplace();
}

void debug_print_poll() {
  if (!g_debug_print) { return; }

  auto& debug_print = *g_debug_print;

  if (debug_print.write_outstanding &&
      debug_print.uart.is_dma_write_finished()) {
    debug_print.uart.finish_dma_write();

    const size_t to_move = debug_print.buffer_size - debug_print.write_outstanding;
    std::memmove(&debug_print.buffer[0], &debug_print.buffer[debug_print.write_outstanding], to_move);
    debug_print.buffer_size = to_move;
    debug_print.write_outstanding = 0;

    // Maybe write again.
    debug_print.MaybeStartWrite();
  }
}
}

void xprintf_impl(const char* format, ...) {
  if (!fw::g_debug_print) {}

  auto& debug_print = *fw::g_debug_print;

  va_list args;
  va_start(args, format);

  const size_t largest_write_possible =
      sizeof(debug_print.buffer) - debug_print.buffer_size;
  const int written = vsnprintf(
      &debug_print.buffer[debug_print.buffer_size],
      largest_write_possible, format, args);
  va_end(args);

  if (written <= 0) {
    // Ignore errors for now.
    return;
  }

  debug_print.buffer_size += written;

  debug_print.MaybeStartWrite();
}
