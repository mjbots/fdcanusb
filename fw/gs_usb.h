// The MIT License (MIT)

// Copyright (c) 2016 Hubert Denkmair
// Copyright 2025 Josh Pieper, jjp@pobox.com.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <stdint.h>

namespace fw {

enum gs_usb_breq {
  GS_USB_BREQ_HOST_FORMAT = 0,
  GS_USB_BREQ_BITTIMING,
  GS_USB_BREQ_MODE,
  GS_USB_BREQ_BERR,
  GS_USB_BREQ_BT_CONST,
  GS_USB_BREQ_DEVICE_CONFIG,
  GS_USB_BREQ_TIMESTAMP,
  GS_USB_BREQ_IDENTIFY,
  GS_USB_BREQ_GET_USER_ID,    //not implemented
  GS_USB_BREQ_SET_USER_ID,    //not implemented
  GS_USB_BREQ_DATA_BITTIMING,
  GS_USB_BREQ_BT_CONST_EXT,
  GS_USB_BREQ_SET_TERMINATION,
  GS_USB_BREQ_GET_TERMINATION,
  GS_USB_BREQ_GET_STATE,
};

enum gs_can_feature {
  GS_CAN_FEATURE_LISTEN_ONLY = (1<<0),
  GS_CAN_FEATURE_LOOP_BACK = (1<<1),
  GS_CAN_FEATURE_TRIPLE_SAMPLE = (1<<2),
  GS_CAN_FEATURE_ONE_SHOT = (1<<3),
  GS_CAN_FEATURE_HW_TIMESTAMP = (1<<4),
  GS_CAN_FEATURE_IDENTIFY = (1<<5),
  GS_CAN_FEATURE_USER_ID = (1<<6),
  GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE = (1<<7),
  GS_CAN_FEATURE_FD = (1<<8), /* device supports CAN-FD */
  /* request workaround for LPC546XX erratum USB.15:
   * let host driver add a padding byte to each USB frame
   */
  GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX = (1<<9),
  /* device supports separate bit timing constants for CAN-FD
   * arbitration and data phase, see:
   * GS_USB_BREQ_BT_CONST_EXT and struct gs_device_bt_const_extended
   */
  GS_CAN_FEATURE_BT_CONST_EXT = (1<<10),
  /* device supports switchable termination, see:
   * - GS_USB_BREQ_SET_TERMINATION
   * - GS_USB_BREQ_GET_TERMINATION
   * - struct gs_device_termination_state
   */
  GS_CAN_FEATURE_TERMINATION = (1<<11),
  GS_CAN_FEATURE_BERR_REPORTING = (1<<12),
  GS_CAN_FEATURE_GET_STATE = (1<<13),
};

enum gs_can_mode {
  GS_CAN_MODE_RESET = 0,
  GS_CAN_MODE_START,
};

enum gs_can_mode_flags {
  GS_CAN_MODE_NORMAL = 0,

  GS_CAN_MODE_LISTEN_ONLY = (1<<0),
  GS_CAN_MODE_LOOP_BACK = (1<<1),
  GS_CAN_MODE_TRIPLE_SAMPLE = (1<<2),
  GS_CAN_MODE_ONE_SHOT = (1<<3),
  GS_CAN_MODE_HW_TIMESTAMP = (1<<4),
  GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE = (1<<7),
  GS_CAN_MODE_FD = (1<<8),    /* switch device to CAN-FD mode */
  GS_CAN_MODE_BERR_REPORTING = (1<<12),
};

enum gs_can_state {
  GS_CAN_STATE_ERROR_ACTIVE = 0,
  GS_CAN_STATE_ERROR_WARNING,
  GS_CAN_STATE_ERROR_PASSIVE,
  GS_CAN_STATE_BUS_OFF,
  GS_CAN_STATE_STOPPED,
  GS_CAN_STATE_SLEEPING,
};

enum gs_can_identify_mode {
  GS_CAN_IDENTIFY_OFF = 0,
  GS_CAN_IDENTIFY_ON = 1,
};

static constexpr uint8_t GS_CAN_FLAG_OVERFLOW = (1<<0);
static constexpr uint8_t GS_CAN_FLAG_FD = (1<<1);  /* is a CAN-FD frame */
static constexpr uint8_t GS_CAN_FLAG_BRS = (1<<2); /* bit rate switch (for CAN-FD frames) */
static constexpr uint8_t GS_CAN_FLAG_ESI = (1<<3); /* error state indicator (for CAN-FD frames) */

static constexpr uint32_t GS_CAN_EFF_FLAG = 0x80000000U; /* EFF/SFF is set in the MSB */
static constexpr uint32_t GS_CAN_RTR_FLAG = 0x40000000U; /* remote transmission request */
static constexpr uint32_t GS_CAN_ERR_FLAG = 0x20000000U; /* error message frame */

static constexpr uint32_t GS_USB_ECHO_ID_RX_FRAME = 0xffffffffu;

static constexpr uint8_t GS_CAN_DLC_MAX_CLASSIC = 8;
static constexpr uint8_t GS_CAN_DLC_MAX_FD = 15;
static constexpr uint8_t GS_CAN_DATA_MAX_FD = 64;

/* The firmware on the original USB2CAN by Geschwister Schneider
 * Technologie Entwicklungs- und Vertriebs UG exchanges all data
 * between the host and the device in host byte order. This is done
 * with the struct gs_host_config::byte_order member, which is sent
 * first to indicate the desired byte order.
 *
 * The widely used open source firmware candleLight doesn't support
 * this feature and exchanges the data in little endian byte order.
 */
struct __attribute__((packed)) gs_device_config {
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t icount;
	uint32_t sw_version;
	uint32_t hw_version;
};

static_assert(sizeof(gs_device_config) == 12, "gs_device_config size");

struct __attribute__((packed)) can_bittiming_const {
	uint32_t tseg1_min;
	uint32_t tseg1_max;
	uint32_t tseg2_min;
	uint32_t tseg2_max;
	uint32_t sjw_max;
	uint32_t brp_min;
	uint32_t brp_max;
	uint32_t brp_inc;
};

struct __attribute__((packed)) gs_device_bt_const {
	uint32_t feature;
	uint32_t fclk_can;
	struct can_bittiming_const btc;
};

static_assert(sizeof(gs_device_bt_const) == 40, "gs_device_bt_const size");

struct __attribute__((packed)) gs_device_bt_const_extended {
	uint32_t feature;
	uint32_t fclk_can;
	struct can_bittiming_const btc;
	struct can_bittiming_const dbtc;
};

static_assert(sizeof(gs_device_bt_const) == 40, "gs_device_bt_const size");

struct __attribute__((packed)) gs_device_bittiming {
	uint32_t prop_seg;
	uint32_t phase_seg1;
	uint32_t phase_seg2;
	uint32_t sjw;
	uint32_t brp;
};

static_assert(sizeof(gs_device_bittiming) == 20, "gs_device_bittiming size");

struct __attribute__((packed)) gs_device_mode {
	uint32_t mode;
	uint32_t flags;
};

static_assert(sizeof(gs_device_mode) == 8, "gs_device_mode size");

struct __attribute__((packed)) gs_device_state {
	uint32_t state;
	uint32_t rxerr;
	uint32_t txerr;
};

static_assert(sizeof(gs_device_state) == 12, "gs_device_state size");

struct __attribute__((packed)) gs_host_frame {
  uint32_t echo_id;
  uint32_t can_id;

  uint8_t can_dlc;
  uint8_t channel;
  uint8_t flags;
  uint8_t reserved;
  uint8_t data[64];
  uint32_t timestamp_us;
};

static_assert(sizeof(gs_host_frame) == 80, "gs_host_frame size");

struct __attribute__((packed)) gs_identify_mode {
	uint32_t mode;
};

static_assert(sizeof(gs_identify_mode) == 4, "gs_identify_mode size");

struct __attribute__((packed)) gs_device_termination_state {
	uint32_t state;
};

static_assert(sizeof(gs_device_termination_state) == 4, "gs_device_termination_state size");

struct __attribute__((packed)) gs_host_format {
  uint32_t format;
};

static_assert(sizeof(gs_host_format) == 4, "gs_host_format size");

}
