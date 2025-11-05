// Copyright 2025 Josh Pieper, jjp@pobox.com.
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

#include "fw/stm32g4_gs_usb.h"

#include <cstring>
#include <algorithm>

#include "mbed.h"

#include "fw/can_manager.h"
#include "fw/fdcan.h"
#include "fw/fifo.h"
#include "fw/gs_usb.h"
#include "fw/xprintf.h"

namespace fw {

namespace {

// gs_usb USB endpoint addresses (must match stm32g4_async_usb_cdc.cc configuration)
// Standard gs_usb endpoints for python-can compatibility
constexpr uint8_t GSUSB_RXD_EP = 0x02;  // Endpoint 2 OUT (Host→Device)
constexpr uint8_t GSUSB_TXD_EP = 0x81;  // Endpoint 1 IN (Device→Host)

// STM32G4 FDCAN bit timing constraints (from reference manual)
// These values are for the nominal bit rate
constexpr uint32_t FDCAN_TSEG1_MIN = 1;
constexpr uint32_t FDCAN_TSEG1_MAX = 256;
constexpr uint32_t FDCAN_TSEG2_MIN = 1;
constexpr uint32_t FDCAN_TSEG2_MAX = 128;
constexpr uint32_t FDCAN_SJW_MAX = 128;
constexpr uint32_t FDCAN_BRP_MIN = 1;
constexpr uint32_t FDCAN_BRP_MAX = 512;

// CAN-FD data phase bit timing constraints
constexpr uint32_t FDCAN_FD_TSEG1_MIN = 1;
constexpr uint32_t FDCAN_FD_TSEG1_MAX = 32;
constexpr uint32_t FDCAN_FD_TSEG2_MIN = 1;
constexpr uint32_t FDCAN_FD_TSEG2_MAX = 16;
constexpr uint32_t FDCAN_FD_SJW_MAX = 16;
constexpr uint32_t FDCAN_FD_BRP_MIN = 1;
constexpr uint32_t FDCAN_FD_BRP_MAX = 32;

// Software/hardware version constants
constexpr uint32_t GS_USB_SW_VERSION = 2;
constexpr uint32_t GS_USB_HW_VERSION = 1;

// Feature flags supported by this implementation
constexpr uint32_t SUPPORTED_FEATURES =
    GS_CAN_FEATURE_LISTEN_ONLY |
    GS_CAN_FEATURE_LOOP_BACK |
    GS_CAN_FEATURE_ONE_SHOT |
    GS_CAN_FEATURE_HW_TIMESTAMP |
    GS_CAN_FEATURE_FD |
    GS_CAN_FEATURE_BT_CONST_EXT |
    GS_CAN_FEATURE_IDENTIFY |
    GS_CAN_FEATURE_TERMINATION |
    0;

}  // namespace

class Stm32G4GsUsb::Impl {
 public:
  Impl(CanManager& can_manager, const Options& options)
      : can_manager_(can_manager),
        options_(options) {
    // Initialize frame buffers to prevent garbage data
    rx_frame_buffer_ = {};
    tx_frame_ = {};

    // CRITICAL: Initialize echo_id to 0xFFFFFFFF to prevent "Unexpected unused echo id 0" error
    // If a spurious frame is sent during USB initialization, it will look like an RX frame
    tx_frame_.echo_id = 0xFFFFFFFF;

    // Register callback to receive CAN frames
    can_manager_.RegisterFrameCallback([this](const CanManager::CanFrame& frame) {
      this->OnCanFrameReceived(frame);
    });

    // Register callback for TX completion
    can_manager_.RegisterTxCompleteCallback([this](uint32_t context, bool success) {
      this->OnCanFrameTxComplete(context, success);
    });
  }

  usbd_respond HandleControl(usbd_device* dev, usbd_ctlreq* req, usbd_rqc_callback* callback) {
    // Store device pointer for use in OnCanFrameReceived
    usb_device_ = dev;

    const auto breq = static_cast<gs_usb_breq>(req->bRequest);

    switch (breq) {
      case GS_USB_BREQ_HOST_FORMAT:
        return HandleHostFormat(dev, req);

      case GS_USB_BREQ_DEVICE_CONFIG:
        return HandleDeviceConfig(dev, req);

      case GS_USB_BREQ_BT_CONST:
        return HandleBtConst(dev, req);

      case GS_USB_BREQ_BT_CONST_EXT:
        return HandleBtConstExt(dev, req);

      case GS_USB_BREQ_BITTIMING:
        return HandleBittiming(dev, req);

      case GS_USB_BREQ_DATA_BITTIMING:
        return HandleDataBittiming(dev, req);

      case GS_USB_BREQ_MODE:
        return HandleMode(dev, req);

      case GS_USB_BREQ_TIMESTAMP:
        return HandleTimestamp(dev, req);

      case GS_USB_BREQ_IDENTIFY:
        return HandleIdentify(dev, req);

      case GS_USB_BREQ_SET_TERMINATION:
        return HandleSetTermination(dev, req);

      case GS_USB_BREQ_GET_TERMINATION:
        return HandleGetTermination(dev, req);

      case GS_USB_BREQ_GET_STATE:
        return HandleGetState(dev, req);

      default:
        return usbd_fail;
    }

    // Should never reach here, but make compiler happy
    return usbd_fail;
  }

  void HandleRxEndpoint(usbd_device* dev, uint8_t event, uint8_t ep) {
    // Bulk RX: Host sending CAN frames to device for transmission
    // For frames > 64 bytes, this callback is invoked multiple times (once per 64-byte packet)
    if (!dev || !bus_active_) {
      return;
    }

    // Read next chunk from endpoint into buffer at current offset
    uint8_t* write_ptr = reinterpret_cast<uint8_t*>(&rx_frame_buffer_) + rx_frame_bytes_received_;
    const int max_bytes = sizeof(rx_frame_buffer_) - rx_frame_bytes_received_;
    int bytes_read = usbd_ep_read(dev, ep, write_ptr, max_bytes);

    if (bytes_read <= 0) {
      rx_frame_bytes_received_ = 0;
      return;
    }

    rx_frame_bytes_received_ += bytes_read;

    // Check if transfer is complete:
    // - If bytes_read < 64, this is the last packet (short packet indicates end)
    // - If bytes_read == 64, more packets may be coming
    const bool transfer_complete = (bytes_read < 64);

    if (!transfer_complete) {
      return;  // Wait for next packet
    }

    // Transfer complete - validate and process the frame
    // Validate that we received at least the minimum frame size (12 byte header)
    constexpr int MIN_FRAME_SIZE = 12;
    if (rx_frame_bytes_received_ < MIN_FRAME_SIZE) {
      rx_frame_bytes_received_ = 0;
      return;
    }

    // Additional validation: Reject frames with both zero CAN ID and zero DLC
    if (rx_frame_buffer_.can_id == 0 && rx_frame_buffer_.can_dlc == 0) {
      rx_frame_bytes_received_ = 0;
      return;
    }

    // Store the frame for echo - must be done BEFORE sending
    __disable_irq();
    const auto frame_copy = rx_frame_buffer_;
    if (pending_tx_count_ < MAX_PENDING_TX) {
      pending_tx_frames_[pending_tx_count_].echo_id = frame_copy.echo_id;
      pending_tx_frames_[pending_tx_count_].frame = frame_copy;
      pending_tx_count_++;
    }
    __enable_irq();

    // Convert gs_host_frame to CanManager::CanFrame
    const auto& frame = frame_copy;
    CanManager::CanFrame can_frame;
    can_frame.id = frame.can_id & 0x1FFFFFFF;
    can_frame.size = FDCan::DlcToSize(frame.can_dlc);
    std::memcpy(can_frame.data, frame.data, std::min<size_t>(can_frame.size, 64));
    can_frame.extended = (frame.can_id & GS_CAN_EFF_FLAG) != 0;
    can_frame.remote = (frame.can_id & GS_CAN_RTR_FLAG) != 0;
    can_frame.fd_frame = (frame.flags & GS_CAN_FLAG_FD) != 0;
    can_frame.bitrate_switch = (frame.flags & GS_CAN_FLAG_BRS) != 0;

    // Reset RX state for next frame
    rx_frame_bytes_received_ = 0;
    rx_frame_buffer_ = {};  // Clear buffer for next frame

    // Send to CAN bus
    can_manager_.SendFrame(can_frame, frame.echo_id);
  }

  void HandleTxEndpoint(usbd_device* dev, uint8_t event, uint8_t ep) {
    // Bulk TX: Transfer complete callback
    (void)event;
    (void)ep;

    if (!tx_transfer_initiated_) {
      return;
    }

    tx_transfer_initiated_ = false;

    // Check if we're in the middle of a multi-packet transfer
    if (tx_frame_bytes_sent_ < tx_frame_total_size_) {
      // More bytes to send - continue with next packet
      const uint16_t bytes_remaining = tx_frame_total_size_ - tx_frame_bytes_sent_;
      const uint16_t chunk_size = (bytes_remaining > 64) ? 64 : bytes_remaining;

      // Send next chunk from offset tx_frame_bytes_sent_
      tx_transfer_initiated_ = true;
      int result = usbd_ep_write(dev, GSUSB_TXD_EP,
                                 reinterpret_cast<uint8_t*>(&tx_frame_) + tx_frame_bytes_sent_,
                                 chunk_size);

      if (result > 0) {
        tx_frame_bytes_sent_ += chunk_size;
      } else {
        tx_transfer_initiated_ = false;
        tx_endpoint_busy_ = false;  // Give up on this frame
        tx_frame_bytes_sent_ = 0;
        tx_frame_total_size_ = 0;
      }
    } else {
      // All data sent, but check if we need a Zero-Length Packet (ZLP)
      // USB spec: If transfer size is a multiple of wMaxPacketSize (64), send ZLP to indicate completion
      const bool need_zlp = (tx_frame_total_size_ % 64) == 0;

      if (need_zlp && tx_frame_bytes_sent_ == tx_frame_total_size_) {
        // Send ZLP to signal end of transfer
        tx_transfer_initiated_ = true;
        // Set tx_frame_bytes_sent_ > total to indicate ZLP has been sent
        tx_frame_bytes_sent_ = tx_frame_total_size_ + 1;
        int result = usbd_ep_write(dev, GSUSB_TXD_EP, nullptr, 0);

        if (result < 0) {
          tx_transfer_initiated_ = false;
          tx_endpoint_busy_ = false;
          tx_frame_bytes_sent_ = 0;
          tx_frame_total_size_ = 0;
        }
      } else {
        // Transfer complete - mark endpoint as ready for next frame
        tx_endpoint_busy_ = false;
        tx_frame_bytes_sent_ = 0;
        tx_frame_total_size_ = 0;
      }
    }
  }

  void OnCanFrameReceived(const CanManager::CanFrame& frame) {
    if (!usb_device_ || !bus_active_) {
      return;
    }

    // Convert CanManager::CanFrame to gs_host_frame
    gs_host_frame host_frame = {};
    host_frame.echo_id = 0xFFFFFFFF;  // RX frames use 0xFFFFFFFF
    host_frame.can_id = frame.id;
    if (frame.extended) {
      host_frame.can_id |= GS_CAN_EFF_FLAG;
    }
    if (frame.remote) {
      host_frame.can_id |= GS_CAN_RTR_FLAG;
    }
    host_frame.can_dlc = FDCan::SizeToDlc(frame.size);
    host_frame.channel = 0;
    host_frame.flags = 0;
    if (frame.fd_frame) {
      host_frame.flags |= GS_CAN_FLAG_FD;
    }
    if (frame.bitrate_switch) {
      host_frame.flags |= GS_CAN_FLAG_BRS;
    }
    host_frame.reserved = 0;
    std::memcpy(host_frame.data, frame.data, std::min<size_t>(frame.size, 64));

    // Append timestamp after data (if timer available)
    // Timestamp is written to data[GetDataLength()], which is safe because
    // data[] is 68 bytes (64 for CAN data + 4 for timestamp)
    if (options_.timer) {
      const uint32_t timestamp_us = options_.timer->read_us();
      const uint8_t data_len = FDCan::DlcToSize(host_frame.can_dlc);
      // Write timestamp as little-endian uint32_t after data
      std::memcpy(&host_frame.data[data_len], &timestamp_us, sizeof(timestamp_us));
    }

    // Queue the frame for Poll() to send
    __disable_irq();
    if (!tx_queue_.full()) {
      tx_queue_.push(host_frame);
    }
    __enable_irq();
  }

  void OnCanFrameTxComplete(uint32_t echo_id, bool success) {
    // Send echo frame back to host to confirm transmission
    if (!usb_device_ || !bus_active_) {
      return;
    }

    // Find the stored TX frame by echo_id
    gs_host_frame echo_frame = {};
    bool found = false;

    __disable_irq();
    for (uint8_t i = 0; i < pending_tx_count_; i++) {
      if (pending_tx_frames_[i].echo_id == echo_id) {
        echo_frame = pending_tx_frames_[i].frame;
        echo_frame.echo_id = echo_id;  // Ensure echo_id is set
        found = true;

        // Remove from array by shifting remaining elements
        for (uint8_t j = i; j < pending_tx_count_ - 1; j++) {
          pending_tx_frames_[j] = pending_tx_frames_[j + 1];
        }
        pending_tx_count_--;
        break;
      }
    }
    __enable_irq();

    // If frame not found, something went wrong - don't send echo
    if (!found) {
      return;
    }

    // Queue the echo frame for transmission via EP5
    __disable_irq();
    if (!tx_queue_.full()) {
      tx_queue_.push(echo_frame);
    }
    __enable_irq();
  }

  void Poll() {
    if (!usb_device_ || !bus_active_ || tx_endpoint_busy_) {
      return;
    }

    // Get next frame from queue
    __disable_irq();
    const bool queue_empty = tx_queue_.empty();
    gs_host_frame frame = {};
    if (!queue_empty) {
      frame = tx_queue_.pop();
    }
    __enable_irq();

    if (queue_empty) {
      return;
    }

    // Copy to persistent buffer (USB DMA may access asynchronously)
    tx_frame_ = frame;

    // Calculate actual frame size (header + data bytes + optional timestamp)
    // Header: 12 bytes (echo_id(4) + can_id(4) + can_dlc(1) + channel(1) + flags(1) + reserved(1))
    // Data: GetDataLength() bytes
    // Timestamp: 4 bytes (if HW_TIMESTAMP feature enabled and timer available)
    tx_frame_total_size_ = 12 + FDCan::DlcToSize(tx_frame_.can_dlc);
    if (options_.timer) {
      tx_frame_total_size_ += 4;  // Add timestamp size
    }

    // For frames > 64 bytes, we need multi-packet transfer
    // Send first chunk (up to 64 bytes), HandleTxEndpoint will continue
    const uint16_t first_chunk_size = (tx_frame_total_size_ > 64) ? 64 : tx_frame_total_size_;

    // Write first chunk to endpoint
    // CRITICAL: Set tx_frame_bytes_sent_ BEFORE usbd_ep_write() to prevent race condition
    // If HandleTxEndpoint() fires before we update this variable, it will see incorrect state
    tx_transfer_initiated_ = true;
    tx_frame_bytes_sent_ = first_chunk_size;  // Set to first chunk size BEFORE write
    int result = usbd_ep_write(usb_device_, GSUSB_TXD_EP, &tx_frame_, first_chunk_size);

    if (result > 0) {
      tx_endpoint_busy_ = true;
      // tx_frame_bytes_sent_ already set above
    } else {
      tx_transfer_initiated_ = false;
      tx_frame_bytes_sent_ = 0;
      tx_frame_total_size_ = 0;
    }
  }

  void PollMillisecond() {
    // Handle LED identification blinking
    if (identify_mode_ && options_.power_led) {
      identify_blink_counter_++;
      if (identify_blink_counter_ >= 100) {  // Blink every 100ms (~5Hz)
        identify_blink_counter_ = 0;
        // Toggle LED
        options_.power_led->write(options_.power_led->read() ? 0 : 1);
      }
    } else if (!identify_mode_ && options_.power_led) {
      // Ensure LED is on when not in identify mode
      options_.power_led->write(1);
    }
  }

 private:
  // Vendor command handlers

  usbd_respond HandleHostFormat(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_HOST_FORMAT - return host data format
    // Always return 0 (little-endian)
    response_buffer_.host_format.format = 0;

    dev->status.data_ptr = &response_buffer_.host_format;
    dev->status.data_count = sizeof(response_buffer_.host_format);
    return usbd_ack;
  }

  usbd_respond HandleDeviceConfig(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_DEVICE_CONFIG - return device capabilities
    response_buffer_.device_config.reserved1 = 0;
    response_buffer_.device_config.reserved2 = 0;
    response_buffer_.device_config.reserved3 = 0;
    response_buffer_.device_config.icount = 0;  // Single channel (0 means 1 channel)
    response_buffer_.device_config.sw_version = GS_USB_SW_VERSION;
    response_buffer_.device_config.hw_version = GS_USB_HW_VERSION;

    dev->status.data_ptr = &response_buffer_.device_config;
    dev->status.data_count = sizeof(response_buffer_.device_config);
    return usbd_ack;
  }

  usbd_respond HandleBtConst(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_BT_CONST - return nominal bit timing constants
    response_buffer_.bt_const.feature = SUPPORTED_FEATURES;
    response_buffer_.bt_const.fclk_can = options_.get_can_clock_hz();
    response_buffer_.bt_const.btc.tseg1_min = FDCAN_TSEG1_MIN;
    response_buffer_.bt_const.btc.tseg1_max = FDCAN_TSEG1_MAX;
    response_buffer_.bt_const.btc.tseg2_min = FDCAN_TSEG2_MIN;
    response_buffer_.bt_const.btc.tseg2_max = FDCAN_TSEG2_MAX;
    response_buffer_.bt_const.btc.sjw_max = FDCAN_SJW_MAX;
    response_buffer_.bt_const.btc.brp_min = FDCAN_BRP_MIN;
    response_buffer_.bt_const.btc.brp_max = FDCAN_BRP_MAX;
    response_buffer_.bt_const.btc.brp_inc = 1;

    dev->status.data_ptr = &response_buffer_.bt_const;
    dev->status.data_count = sizeof(response_buffer_.bt_const);
    return usbd_ack;
  }

  usbd_respond HandleBtConstExt(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_BT_CONST_EXT - return extended bit timing constants (CAN-FD)
    // This returns both nominal AND data phase bit timing constants
    response_buffer_.bt_const_ext.feature = SUPPORTED_FEATURES;
    response_buffer_.bt_const_ext.fclk_can = options_.get_can_clock_hz();

    // Nominal bit timing constants
    response_buffer_.bt_const_ext.btc.tseg1_min = FDCAN_TSEG1_MIN;
    response_buffer_.bt_const_ext.btc.tseg1_max = FDCAN_TSEG1_MAX;
    response_buffer_.bt_const_ext.btc.tseg2_min = FDCAN_TSEG2_MIN;
    response_buffer_.bt_const_ext.btc.tseg2_max = FDCAN_TSEG2_MAX;
    response_buffer_.bt_const_ext.btc.sjw_max = FDCAN_SJW_MAX;
    response_buffer_.bt_const_ext.btc.brp_min = FDCAN_BRP_MIN;
    response_buffer_.bt_const_ext.btc.brp_max = FDCAN_BRP_MAX;
    response_buffer_.bt_const_ext.btc.brp_inc = 1;

    // Data phase bit timing constants (CAN-FD)
    response_buffer_.bt_const_ext.dbtc.tseg1_min = FDCAN_FD_TSEG1_MIN;
    response_buffer_.bt_const_ext.dbtc.tseg1_max = FDCAN_FD_TSEG1_MAX;
    response_buffer_.bt_const_ext.dbtc.tseg2_min = FDCAN_FD_TSEG2_MIN;
    response_buffer_.bt_const_ext.dbtc.tseg2_max = FDCAN_FD_TSEG2_MAX;
    response_buffer_.bt_const_ext.dbtc.sjw_max = FDCAN_FD_SJW_MAX;
    response_buffer_.bt_const_ext.dbtc.brp_min = FDCAN_FD_BRP_MIN;
    response_buffer_.bt_const_ext.dbtc.brp_max = FDCAN_FD_BRP_MAX;
    response_buffer_.bt_const_ext.dbtc.brp_inc = 1;

    dev->status.data_ptr = &response_buffer_.bt_const_ext;
    dev->status.data_count = sizeof(response_buffer_.bt_const_ext);
    return usbd_ack;
  }

  usbd_respond HandleBittiming(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_BITTIMING - set nominal bit timing
    if (req->wLength < sizeof(gs_device_bittiming)) {
      return usbd_fail;
    }

    gs_device_bittiming timing;
    std::memcpy(&timing, req->data, sizeof(timing));

    // Don't change bit timing while bus is active
    if (bus_active_) {
      return usbd_ack;
    }

    // Convert gs_device_bittiming to CanManager::BitTiming format
    // gs_usb uses separate prop_seg and phase_seg1, but STM32 FDCAN
    // combines them into time_seg1
    CanManager::BitTiming bit_timing;
    bit_timing.prescaler = timing.brp;
    bit_timing.sync_jump_width = timing.sjw;
    bit_timing.time_seg1 = timing.prop_seg + timing.phase_seg1;
    bit_timing.time_seg2 = timing.phase_seg2;

    // Apply nominal bit timing to CanManager
    can_manager_.SetNominalTiming(bit_timing);

    return usbd_ack;
  }

  usbd_respond HandleDataBittiming(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_DATA_BITTIMING - set data phase bit timing (CAN-FD)
    if (req->wLength < sizeof(gs_device_bittiming)) {
      return usbd_fail;
    }

    gs_device_bittiming timing;
    std::memcpy(&timing, req->data, sizeof(timing));

    // Don't change bit timing while bus is active
    if (bus_active_) {
      return usbd_ack;
    }

    // Convert gs_device_bittiming to CanManager::BitTiming format
    // gs_usb uses separate prop_seg and phase_seg1, but STM32 FDCAN
    // combines them into time_seg1
    CanManager::BitTiming bit_timing;
    bit_timing.prescaler = timing.brp;
    bit_timing.sync_jump_width = timing.sjw;
    bit_timing.time_seg1 = timing.prop_seg + timing.phase_seg1;
    bit_timing.time_seg2 = timing.phase_seg2;

    // Apply data phase bit timing to CanManager
    can_manager_.SetDataTiming(bit_timing);

    return usbd_ack;
  }

  usbd_respond HandleMode(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_MODE - set operating mode
    if (req->wLength < sizeof(gs_device_mode)) {
      return usbd_fail;
    }

    gs_device_mode mode;
    std::memcpy(&mode, req->data, sizeof(mode));

    // Check if one-shot mode changed
    const bool old_one_shot = (mode_flags_ & GS_CAN_MODE_ONE_SHOT) != 0;
    const bool new_one_shot = (mode.flags & GS_CAN_MODE_ONE_SHOT) != 0;
    const bool one_shot_changed = (old_one_shot != new_one_shot);

    // Check if listen-only (bus monitor) mode changed
    const bool old_listen_only = (mode_flags_ & GS_CAN_MODE_LISTEN_ONLY) != 0;
    const bool new_listen_only = (mode.flags & GS_CAN_MODE_LISTEN_ONLY) != 0;
    const bool listen_only_changed = (old_listen_only != new_listen_only);

    // Check if loopback mode changed
    const bool old_loopback = (mode_flags_ & GS_CAN_MODE_LOOP_BACK) != 0;
    const bool new_loopback = (mode.flags & GS_CAN_MODE_LOOP_BACK) != 0;
    const bool loopback_changed = (old_loopback != new_loopback);

    mode_flags_ = mode.flags;

    // TODO: Actually handle mode flag changes.

    // Control bus state based on MODE_START flag
    const bool should_start = (mode.mode & GS_CAN_MODE_START) != 0;

    // If one-shot, listen-only, or loopback mode changed while bus is active, restart to apply changes
    if ((one_shot_changed || listen_only_changed || loopback_changed) && bus_active_ && should_start) {
      can_manager_.SetBusActive(false);  // Stop
      bus_active_ = false;
    }

    can_manager_.SetBusActive(should_start);
    bus_active_ = should_start;

    if (should_start) {
      tx_endpoint_busy_ = false;
    } else {
      rx_endpoint_primed_ = false;
      // Clear pending TX frames when bus is stopped
      pending_tx_count_ = 0;
    }

    return usbd_ack;
  }

  usbd_respond HandleTimestamp(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_TIMESTAMP - get timestamp
    // Return current timestamp in microseconds
    response_buffer_.timestamp = options_.timer ? options_.timer->read_us() : 0;

    dev->status.data_ptr = &response_buffer_.timestamp;
    dev->status.data_count = sizeof(response_buffer_.timestamp);
    return usbd_ack;
  }

  usbd_respond HandleIdentify(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_IDENTIFY - flash LED for identification
    if (req->wLength < sizeof(gs_identify_mode)) {
      return usbd_fail;
    }

    gs_identify_mode identify;
    std::memcpy(&identify, req->data, sizeof(identify));

    identify_mode_ = (identify.mode == GS_CAN_IDENTIFY_ON);
    identify_blink_counter_ = 0;

    // If we have a power LED, set initial state
    if (options_.power_led) {
      options_.power_led->write(identify_mode_ ? 0 : 1);  // Start with LED off if identifying
    }

    return usbd_ack;
  }

  usbd_respond HandleSetTermination(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_SET_TERMINATION - set termination resistor
    if (req->wLength < sizeof(gs_device_termination_state)) {
      return usbd_fail;
    }

    gs_device_termination_state term_state;
    std::memcpy(&term_state, req->data, sizeof(term_state));

    // TODO: Actually handle termination changes.
    return usbd_ack;
  }

  usbd_respond HandleGetTermination(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_GET_TERMINATION - get termination state

    // TODO: Actually handle retrieving the current termination state.
    response_buffer_.termination_state.state = true;

    dev->status.data_ptr = &response_buffer_.termination_state;
    dev->status.data_count = sizeof(response_buffer_.termination_state);
    return usbd_ack;
  }

  usbd_respond HandleGetState(usbd_device* dev, usbd_ctlreq* req) {
    // GS_USB_BREQ_GET_STATE - get current bus state
    // Return bus state based on whether the bus is active
    if (bus_active_) {
      response_buffer_.device_state.state = GS_CAN_STATE_ERROR_ACTIVE;
    } else {
      response_buffer_.device_state.state = GS_CAN_STATE_STOPPED;
    }

    // TODO: Get actual error counters from CAN controller

    dev->status.data_ptr = &response_buffer_.device_state;
    dev->status.data_count = sizeof(response_buffer_.device_state);
    return usbd_ack;
  }

  CanManager& can_manager_;
  Options options_;

  // State
  usbd_device* usb_device_ = nullptr;
  bool bus_active_ = false;
  uint32_t mode_flags_ = 0;
  uint32_t requested_bitrate_ = 1000000;  // Default 1 Mbit/s
  double requested_sample_point_ = 0.666;  // Default sample point
  bool identify_mode_ = false;  // LED identification mode
  uint8_t identify_blink_counter_ = 0;  // Counter for LED blink timing

  // RX/TX frame state (persistent buffers for async USB transfers)
  gs_host_frame rx_frame_buffer_ = {};  // RX endpoint buffer
  gs_host_frame tx_frame_ = {};         // TX endpoint buffer (active transfer)
  Fifo<gs_host_frame, 32> tx_queue_;    // Queue of pending TX frames
  bool tx_endpoint_busy_ = true;        // Start busy to prevent spurious sends during init
  bool tx_transfer_initiated_ = false;  // True when we've called usbd_ep_write
  bool rx_endpoint_primed_ = false;     // True when RX endpoint has been primed

  // Multi-packet transfer state (for frames > 64 bytes)
  uint16_t tx_frame_total_size_ = 0;    // Total size of frame being sent (TX)
  uint16_t tx_frame_bytes_sent_ = 0;    // Bytes sent so far (TX)
  uint16_t rx_frame_bytes_received_ = 0; // Bytes received so far (RX)

  // Pending TX frames for echo generation
  // We need to store the original TX frame data because rx_frame_buffer_ gets
  // overwritten by new incoming frames before the TX complete callback fires
  struct PendingTx {
    uint32_t echo_id;
    gs_host_frame frame;
  };
  static constexpr uint8_t MAX_PENDING_TX = 8;  // Support up to 8 in-flight frames
  PendingTx pending_tx_frames_[MAX_PENDING_TX];
  uint8_t pending_tx_count_ = 0;

  // Response buffers for vendor commands (must persist during USB transfer)
  union {
    gs_host_format host_format;
    gs_device_config device_config;
    gs_device_bt_const bt_const;
    gs_device_bt_const_extended bt_const_ext;
    gs_device_termination_state termination_state;
    gs_device_state device_state;
    uint32_t timestamp;
  } response_buffer_;
};

// Public interface implementation

Stm32G4GsUsb::Stm32G4GsUsb(mjlib::micro::Pool& pool,
                           CanManager& can_manager,
                           const Options& options)
    : impl_(&pool, can_manager, options) {}

Stm32G4GsUsb::~Stm32G4GsUsb() {}

usbd_respond Stm32G4GsUsb::HandleControl(usbd_device* dev, usbd_ctlreq* req, usbd_rqc_callback* callback) {
  return impl_->HandleControl(dev, req, callback);
}

void Stm32G4GsUsb::HandleRxEndpoint(usbd_device* dev, uint8_t event, uint8_t ep) {
  impl_->HandleRxEndpoint(dev, event, ep);
}

void Stm32G4GsUsb::HandleTxEndpoint(usbd_device* dev, uint8_t event, uint8_t ep) {
  impl_->HandleTxEndpoint(dev, event, ep);
}

void Stm32G4GsUsb::OnCanFrameReceived(const CanManager::CanFrame& frame) {
  impl_->OnCanFrameReceived(frame);
}

void Stm32G4GsUsb::OnCanFrameTxComplete(uint32_t echo_id, bool success) {
  impl_->OnCanFrameTxComplete(echo_id, success);
}

void Stm32G4GsUsb::Poll() {
  impl_->Poll();
}

void Stm32G4GsUsb::PollMillisecond() {
  impl_->PollMillisecond();
}

}  // namespace fw
