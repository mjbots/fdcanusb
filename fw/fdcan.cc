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

#include "fw/fdcan.h"

#include "PeripheralPins.h"

extern const PinMap PinMap_CAN_TD[];
extern const PinMap PinMap_CAN_RD[];

namespace fw {
namespace {
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
}

FDCan::FDCan(const Options& options) {
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
  can.Init.FrameFormat = [&]() {
    if (options.fdcan_frame && options.bitrate_switch) {
      return FDCAN_FRAME_FD_BRS;
    } else if (options.fdcan_frame) {
      return FDCAN_FRAME_FD_NO_BRS;
    }
    return FDCAN_FRAME_CLASSIC;
  }();
  can.Init.Mode = [&]() {
    if (options.bus_monitor) {
      return FDCAN_MODE_BUS_MONITORING;
    } else if (options.restricted_mode) {
      return FDCAN_MODE_RESTRICTED_OPERATION;
    }
    return FDCAN_MODE_NORMAL;
  }();
  can.Init.AutoRetransmission =
      options.automatic_retransmission ? ENABLE : DISABLE;
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
  can.Init.StdFiltersNbr =
      std::count_if(
          options.filter_begin, options.filter_end,
          [](const auto& filter) {
            return filter.action != kDisable && filter.type == kStandard;
          });
  can.Init.ExtFiltersNbr =
      std::count_if(
          options.filter_begin, options.filter_end,
          [](const auto& filter) {
            return filter.action != kDisable && filter.type == kExtended;
          });
  can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&can) != HAL_OK) {
    mbed_die();
  }

  int standard_index = 0;
  int extended_index = 0;
  std::for_each(
      options.filter_begin, options.filter_end,
      [&](const auto& filter) {
        if (filter.action == kDisable) {
          return;
        }

        FDCAN_FilterTypeDef sFilterConfig;
        sFilterConfig.IdType = [&]() {
          switch (filter.type) {
            case kStandard: return FDCAN_STANDARD_ID;
            case kExtended: return FDCAN_EXTENDED_ID;
          }
          mbed_die();
        }();
        sFilterConfig.FilterIndex = [&]() {
          switch (filter.type) {
            case kStandard: {
              return standard_index++;
            }
            case kExtended: {
              return extended_index++;
            }
          }
          mbed_die();
        }();
        sFilterConfig.FilterType = [&]() {
          switch (filter.mode) {
            case kRange: return FDCAN_FILTER_RANGE;
            case kDual: return FDCAN_FILTER_DUAL;
            case kMask: return FDCAN_FILTER_MASK;
          }
          mbed_die();
        }();

        sFilterConfig.FilterConfig = [&]() {
          switch (filter.action) {
            case kDisable:
            case kReject: return FDCAN_FILTER_REJECT;
            case kAccept: return FDCAN_FILTER_TO_RXFIFO0;
          }
          mbed_die();
        }();
        sFilterConfig.FilterID1 = filter.id1;
        sFilterConfig.FilterID2 = filter.id2;

        if (HAL_FDCAN_ConfigFilter(&can, &sFilterConfig) != HAL_OK)
        {
          mbed_die();
        }

      });

  auto map_filter_action = [](auto value) {
    switch (value) {
      case kDisable:
      case kAccept: {
        return FDCAN_ACCEPT_IN_RX_FIFO0;
      }
      case kReject: {
        return FDCAN_REJECT;
      }
    }
    mbed_die();
  };

  auto map_remote_action = [](auto value) {
    switch (value) {
      case kDisable:
      case kAccept: {
        return FDCAN_FILTER_REMOTE;
      }
      case kReject: {
        return FDCAN_REJECT_REMOTE;
      }
    }
    mbed_die();
  };

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(
          &can,
          map_filter_action(options.global_std_action),
          map_filter_action(options.global_ext_action),
          map_remote_action(options.global_remote_std_action),
          map_remote_action(options.global_remote_ext_action)) != HAL_OK) {
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

void FDCan::Send(uint16_t dest_id,
                 std::string_view data) {

  // Abort anything we have started that hasn't finished.
  if (last_tx_request_) {
    HAL_FDCAN_AbortTxRequest(&hfdcan1_, last_tx_request_);
  }

  FDCAN_TxHeaderTypeDef tx_header;
  tx_header.Identifier = dest_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = RoundUpDlc(data.size());
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_ON;
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(
          &hfdcan1_, &tx_header,
          const_cast<uint8_t*>(
              reinterpret_cast<const uint8_t*>(data.data()))) != HAL_OK) {
    mbed_die();
  }
  last_tx_request_ = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1_);
}

bool FDCan::Poll(FDCAN_RxHeaderTypeDef* header,
                 mjlib::base::string_span data) {
  if (HAL_FDCAN_GetRxMessage(
          &hfdcan1_, FDCAN_RX_FIFO0, header,
          reinterpret_cast<uint8_t*>(data.data())) != HAL_OK) {
    return false;
  }

  return true;
}

}
