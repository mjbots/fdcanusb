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

#include "fw/millisecond_timer.h"

extern const PinMap PinMap_CAN_TD[];
extern const PinMap PinMap_CAN_RD[];

namespace {
namespace base = mjlib::base;
namespace micro = mjlib::micro;

class NullStream : public micro::AsyncStream {
 public:
  void AsyncReadSome(const base::string_span&,
                     const micro::SizeCallback&) override {
  }

  void AsyncWriteSome(const std::string_view&,
                      const micro::SizeCallback&) override {
  }
};

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

constexpr uint32_t RoundUpDlc(size_t size) {
  if (size == 0) { return FDCAN_DLC_BYTES_0; }
  if (size == 1) { return FDCAN_DLC_BYTES_1; }
  if (size == 2) { return FDCAN_DLC_BYTES_2; }
  if (size == 3) { return FDCAN_DLC_BYTES_3; }
  if (size == 4) { return FDCAN_DLC_BYTES_4; }
  if (size == 5) { return FDCAN_DLC_BYTES_5; }
  if (size == 6) { return FDCAN_DLC_BYTES_6; }
  if (size == 7) { return FDCAN_DLC_BYTES_7; }
  if (size == 8) { return FDCAN_DLC_BYTES_8; }
  if (size <= 12) { return FDCAN_DLC_BYTES_12; }
  if (size <= 16) { return FDCAN_DLC_BYTES_16; }
  if (size <= 20) { return FDCAN_DLC_BYTES_20; }
  if (size <= 24) { return FDCAN_DLC_BYTES_24; }
  if (size <= 32) { return FDCAN_DLC_BYTES_32; }
  if (size <= 48) { return FDCAN_DLC_BYTES_48; }
  if (size <= 64) { return FDCAN_DLC_BYTES_64; }
  return 0;
}

class FDCan {
 public:
  struct Options {
    PinName td = NC;
    PinName rd = NC;
    int slow_bitrate = 1000000;
    int fast_bitrate = 5000000;

    Options() {}
  };

  FDCan(const Options& options = Options()) {
    __HAL_RCC_FDCAN_CLK_ENABLE();

    {
      const auto can_td = pinmap_peripheral(options.td, PinMap_CAN_TD);
      const auto can_rd = pinmap_peripheral(options.rd, PinMap_CAN_RD);
      can_ = reinterpret_cast<FDCAN_GlobalTypeDef*>(
          pinmap_merge(can_td, can_rd));
    }

    pinmap_pinout(options.td, PinMap_CAN_TD);
    pinmap_pinout(options.rd, PinMap_CAN_RD);

    auto& can = hfdcan1_;

    can.Instance = FDCAN1;
    can.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    can.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    can.Init.Mode = FDCAN_MODE_NORMAL;
    can.Init.AutoRetransmission = DISABLE;
    can.Init.TransmitPause = ENABLE;
    can.Init.ProtocolException = DISABLE;
    can.Init.NominalPrescaler = 2;
    can.Init.NominalSyncJumpWidth = 16;
    can.Init.NominalTimeSeg1 = 63;
    can.Init.NominalTimeSeg2 = 16;
    can.Init.DataPrescaler = 4;
    can.Init.DataSyncJumpWidth = 2;
    can.Init.DataTimeSeg1 = 3;
    can.Init.DataTimeSeg2 = 2;
    can.Init.StdFiltersNbr = 1;
    can.Init.ExtFiltersNbr = 0;
    can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&can) != HAL_OK) {
      mbed_die();
    }

    /* Configure Rx filter */
    {
      FDCAN_FilterTypeDef sFilterConfig;
      sFilterConfig.IdType = FDCAN_STANDARD_ID;
      sFilterConfig.FilterIndex = 0;
      sFilterConfig.FilterType = FDCAN_FILTER_MASK;
      sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
      sFilterConfig.FilterID1 = 0x321;
      sFilterConfig.FilterID2 = 0x7FF;
      if (HAL_FDCAN_ConfigFilter(&can, &sFilterConfig) != HAL_OK)
      {
        mbed_die();
      }
    }


    /* Configure global filter:
       Filter all remote frames with STD and EXT ID
       Reject non matching frames with STD ID and EXT ID */
    if (HAL_FDCAN_ConfigGlobalFilter(
            &can, FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
      mbed_die();
    }

    if (HAL_FDCAN_Start(&can) != HAL_OK) {
      mbed_die();
    }

    if (HAL_FDCAN_ActivateNotification(
            &can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
      mbed_die();
    }
  }

  void Send(uint16_t dest_id,
            uint8_t* data,
            size_t size) {
    // Abort anything we have started that hasn't finished.
    if (last_tx_request_) {
      HAL_FDCAN_AbortTxRequest(&hfdcan1_, last_tx_request_);
    }

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = dest_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = RoundUpDlc(size);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1_, &tx_header, data) != HAL_OK) {
      mbed_die();
    }
    last_tx_request_ = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1_);
  }

  /// @return true if a packet was available.
  bool Poll(FDCAN_RxHeaderTypeDef* header, uint8_t* data) {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1_, FDCAN_RX_FIFO0, header, data) != HAL_OK) {
      return false;
    }

    return true;
  }

  FDCAN_GlobalTypeDef* can_;
  FDCAN_HandleTypeDef hfdcan1_;
  uint32_t last_tx_request_ = 0;
};

void SetupClock() {
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

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
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

  FDCan can([]() {
      FDCan::Options options;
      options.td = PA_12;
      options.rd = PA_11;
      return options;
    }());

  fw::MillisecondTimer timer;

  micro::SizedPool<12288> pool;
  NullStream command_stream;
  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(&command_stream);
  micro::CommandManager command_manager(&pool, &command_stream, &write_stream);
  NullFlash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  // auto* const can1 = FDCAN1;
  uint8_t tx_data[16] = {0, 3, 7, 12, 18, 25, 33, 42,
                         1, 4, 8, 13, 19, 26, 34, 43};

  FDCAN_RxHeaderTypeDef rx_header = {};
  uint8_t rx_data[8] = {};

  while (true) {
    const uint32_t start = timer.read_ms();
    while (true) {
      uint32_t now = timer.read_ms();
      if (now - start > 1000) { break; }

      if (can.Poll(&rx_header, rx_data)) {
        g_led_value = !g_led_value;
        led1.write(g_led_value);
      }
    }

    can.Send(0x321, tx_data, sizeof(tx_data));
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
