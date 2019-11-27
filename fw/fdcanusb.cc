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

extern const PinMap PinMap_CAN_TD[];
extern const PinMap PinMap_CAN_RD[];

namespace {
class FDCan {
  struct Options {
    PinName td = NC;
    PinName rd = NC;
    int slow_bitrate = 1000000;
    int fast_bitrate = 5000000;

    Options() {}
  };
  FDCan(const Options& options = Options()) {
    {
      const auto can_td = pinmap_peripheral(options.td, PinMap_CAN_TD);
      const auto can_rd = pinmap_peripheral(options.rd, PinMap_CAN_RD);
      can_ = reinterpret_cast<FDCAN_GlobalTypeDef*>(
          pinmap_merge(can_td, can_rd));
    }

    // Enter the initialization mode
    can_->CCCR |= FDCAN_CCCR_INIT;

    // Zero everything and enable register access.
    can_->CCCR |= FDCAN_CCCR_CCE;

    // Enable CAN-FD.
    can_->CCCR |= FDCAN_CCCR_FDOE;

    // TODO: query the pclk rate.
    // const int pclk_rate = 42500000;

    // CKDIV is used to dived the ABP clock

    // Set standard bit timing in BTP.
    can_->DBTP = 0;

    // What is NBTP used for?

    // Set fast bit timing in FBTP.

    // Set Tx FIFO instead of Tx Queue.
    // TXBC[TFQM] = 0


    // Probably want tmitter delay compensation?  DBTP.TDC

    // Need to check for restricted operation.  CCCR.ASM
    // Need to check protocol status register PSR
  }

  FDCAN_GlobalTypeDef* can_;
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
}
}

FDCAN_HandleTypeDef hfdcan1;
DigitalOut led1(PA_6);
bool g_led_value = false;

int main(void) {
  SetupClock();

  // auto* const can1 = FDCAN1;
  uint8_t tx_data[16] = {0, 3, 7, 12, 18, 25, 33, 42,
                         1, 4, 8, 13, 19, 26, 34, 43};
  uint32_t last_tx_request = 0;

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_FDCAN_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      mbed_die();
    }
  }

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  pinmap_pinout(PA_12, PinMap_CAN_TD);
  pinmap_pinout(PA_11, PinMap_CAN_RD);


  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 63;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 4;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 3;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
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
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
      mbed_die();
    }
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(
          &hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
          FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    mbed_die();
  }



  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    mbed_die();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    mbed_die();
  }


  FDCAN_TxHeaderTypeDef tx_header;
  tx_header.Identifier = 0x321;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_16;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_ON;
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  while (true) {
    wait(1.0);
    // Abort anything we've already started.
    if (last_tx_request) {
      HAL_FDCAN_AbortTxRequest(&hfdcan1, last_tx_request);
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK) {
      mbed_die();
    }
    last_tx_request = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
    for (auto& data : tx_data) { data++; }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef RxHeader = {};
  uint8_t RxData[8] = {};

  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
      mbed_die();
    }

    g_led_value = !g_led_value;
    led1.write(g_led_value);

    // if ((RxHeader.Identifier == 0x321) &&
    //     (RxHeader.IdType == FDCAN_STANDARD_ID) &&
    //     (RxHeader.DataLength == FDCAN_DLC_BYTES_2)) {
    //   LED_Display(RxData[0]);
    //   ubKeyNumber = RxData[0];
    // }
  }
}

extern "C" {
void FDCAN1_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

void SysTick_Handler(void) {
  HAL_IncTick();
}
}
