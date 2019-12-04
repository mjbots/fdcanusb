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

#include "mbed.h"
#include "PeripheralPins.h"

#include "mjlib/base/string_span.h"

#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"

#include "fw/fdcan.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_async_uart.h"

namespace {
namespace base = mjlib::base;
namespace micro = mjlib::micro;

class NullFlash : public micro::FlashInterface {
 public:
  Info GetInfo() override {
    return {};
  }

  void Erase() override {
  }

  void Unlock() override {
  }

  void Lock() override {
  }

  void ProgramByte(char*, uint8_t) override {
  }
};

void SetupClock() {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 170 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  //  85 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  //  85 MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    mbed_die();
  }

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_USART2;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      mbed_die();
    }
  }
}
}

DigitalOut led1(PA_5);
bool g_led_value = false;

int main(void) {
  SetupClock();

  fw::FDCan can([]() {
      fw::FDCan::Options options;
      options.td = PA_12;
      options.rd = PA_11;
      return options;
    }());

  fw::MillisecondTimer timer;

  micro::SizedPool<12288> pool;
  fw::Stm32G4AsyncUart uart(
      &pool,
      &timer,
      []() {
        fw::Stm32G4AsyncUart::Options options;

        options.tx = PA_2;
        options.rx = PA_3;
        options.baud_rate = 115200;

        return options;
      }());

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(&uart);
  micro::CommandManager command_manager(&pool, &uart, &write_stream);
  NullFlash flash_interface;
  micro::PersistentConfig persistent_config(
      pool, command_manager, flash_interface);

  command_manager.AsyncStart();

  char tx_data[16] = {0, 3, 7, 12, 18, 25, 33, 42,
                      1, 4, 8, 13, 19, 26, 34, 43};

  FDCAN_RxHeaderTypeDef rx_header = {};
  char rx_data[8] = {};

  while (true) {
    const uint32_t start = timer.read_ms();
    while (true) {
      const uint32_t now = timer.read_ms();
      if (now - start > 1000) { break; }

      uart.Poll();

      if (can.Poll(&rx_header, base::string_span(rx_data))) {
        g_led_value = !g_led_value;
        led1.write(g_led_value);
      }
    }

    can.Send(0x321, tx_data);
  }
}

extern "C" {
void SysTick_Handler(void) {
  HAL_IncTick();
}

void abort() {
  mbed_die();
}
}
