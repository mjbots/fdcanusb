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

#include <inttypes.h>

#include "mbed.h"
#include "PeripheralPins.h"

#include "mjlib/base/string_span.h"

#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/can_manager.h"
#include "fw/firmware_info.h"
#include "fw/git_info.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_async_uart.h"
#include "fw/stm32g4_flash.h"
#include "fw/stm32g4_async_usb_cdc.h"
#include "fw/uuid.h"

namespace {
namespace base = mjlib::base;
namespace micro = mjlib::micro;

void SetupClock(int clock_rate_hz) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  // Temporarily stop running off the PLL so we can change it.
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    mbed_die();
  }

  RCC_OscInitTypeDef RCC_OscInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = (clock_rate_hz / 1000000) * 24 / 48;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    mbed_die();
  }

  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    mbed_die();
  }

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      mbed_die();
    }
  }
}

class ClockManager {
 public:
  ClockManager(fw::MillisecondTimer* timer,
               micro::PersistentConfig& persistent_config,
               micro::CommandManager& command_manager)
      : timer_(timer) {
    persistent_config.Register("clock", &clock_, [this]() {
        this->UpdateConfig();
      });
    command_manager.Register("clock", [this](auto&& command, auto&& response) {
        this->Command(command, response);
      });
  }

  void UpdateConfig() {
    const int can_clock_hz = [&]() {
      if (clock_.can_hz >= 85000000) { return 85000000; }
      if (clock_.can_hz >= 80000000) { return 80000000; }
      if (clock_.can_hz >= 60000000) { return 60000000; }
      return 85000000;
    }();

    SetupClock(can_clock_hz * 2);

    const int32_t trim = std::max<int32_t>(0, std::min<int32_t>(127, clock_.hsitrim));
    RCC->ICSCR = (RCC->ICSCR & ~0xff000000) | (trim << 24);
  }

  void Command(const std::string_view& command,
               const micro::CommandManager::Response& response) {
    if (command == "us") {
      snprintf(output_, sizeof(output_), "%" PRIu32 "\r\n", timer_->read_us());
      WriteMessage(output_, response);
    } else {
      WriteMessage("ERR unknown clock\r\n", response);
    }
  }

  void WriteMessage(const std::string_view& message,
                    const micro::CommandManager::Response& response) {
    micro::AsyncWrite(*response.stream, message, response.callback);
  }

 private:
  struct Config {
    int32_t can_hz = 85000000;
    int32_t hsitrim = 64;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(can_hz));
      a->Visit(MJ_NVP(hsitrim));
    }
  };

  fw::MillisecondTimer* const timer_;
  Config clock_;
  char output_[16] = {};
};
}

int main(void) {
  SetupClock(170000000);

  DigitalOut power_led{PB_5, 1};

  fw::MillisecondTimer timer;

  micro::SizedPool<12288> pool;

  fw::Stm32G4AsyncUsbCdc usb(&pool, {});

  fw::Stm32G4AsyncUart uart(
      &pool,
      &timer,
      []() {
        fw::Stm32G4AsyncUart::Options options;

        options.tx = PA_2;
        options.rx = PA_3;
        options.baud_rate = 115200;
        options.rx_buffer_size = 300;

        return options;
      }());

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(&usb);
  micro::CommandManager command_manager(
      &pool, &usb, &write_stream,
      []() {
        micro::CommandManager::Options options;
        options.max_line_length = 300;
        return options;
      }());

  char micro_output_buffer[2048] = {};
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream, micro_output_buffer);

  fw::Stm32G4Flash flash_interface;
  micro::PersistentConfig persistent_config(
      pool, command_manager, flash_interface, micro_output_buffer);

  fw::Uuid uuid(persistent_config);
  ClockManager clock(&timer, persistent_config, command_manager);

  fw::CanManager can_manager(
      pool, persistent_config, command_manager, write_stream,
      []() {
        fw::CanManager::Options options;
        options.td = PB_13;
        options.rd = PB_12;
        return options;
      }());

  fw::FirmwareInfo firmware_info(pool, telemetry_manager);

  fw::GitInfo git_info;
  telemetry_manager.Register("git", &git_info);

  persistent_config.Load();

  command_manager.AsyncStart();
  can_manager.Start();

  while (true) {
    const uint32_t start = timer.read_ms();
    while (true) {
      const uint32_t now = timer.read_ms();
      if (now - start > 10) { break; }

      uart.Poll();
      can_manager.Poll();
      usb.Poll();
    }

    can_manager.Poll10Ms();
    usb.Poll10Ms();
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
