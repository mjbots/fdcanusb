// Copyright 2018-2025 Josh Pieper, jjp@pobox.com.
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

#include "fw/stm32g4_async_usb_cdc.h"

#include "usb.h"
#include "usb_cdc.h"

#include "fw/millisecond_timer.h"

// CDC endpoints

#define CDC_EP0_SIZE    0x08
#define CDC_RXD_EP      0x03
#define CDC_TXD_EP      0x84
#define CDC_DATA_SZ     0x40
#define CDC_NTF_EP      0x85
#define CDC_NTF_SZ      0x08

// gs_usb endpoints

#define GSUSB_RXD_EP    0x02
#define GSUSB_TXD_EP    0x81
#define GSUSB_DATA_SZ   0x40

// Buffer size for frame data (80 bytes: 12 header + 64 data + 4
// timestamp)
#define GSUSB_BUFFER_SZ 0x50

#define CDC_PROTOCOL USB_PROTO_NONE

namespace {
struct composite_config {
    struct usb_config_descriptor        config;

    // The gs_usb must be on interface 0 for the Linux gs_usb driver to
    // use it.
    struct usb_iad_descriptor           gs_usb_iad;
    struct usb_interface_descriptor     gs_usb_intf;
    struct usb_endpoint_descriptor      gs_usb_eprx;
    struct usb_endpoint_descriptor      gs_usb_eptx;

    // The CDC can be on any interface it seems, we'll use 1 and 2.
    struct usb_iad_descriptor           comm_iad;
    struct usb_interface_descriptor     comm;
    struct usb_cdc_header_desc          cdc_hdr;
    struct usb_cdc_call_mgmt_desc       cdc_mgmt;
    struct usb_cdc_acm_desc             cdc_acm;
    struct usb_cdc_union_desc           cdc_union;
    struct usb_endpoint_descriptor      comm_ep;
    struct usb_interface_descriptor     data;
    struct usb_endpoint_descriptor      data_eprx;
    struct usb_endpoint_descriptor      data_eptx;
} __attribute__((packed));

/* Device descriptor */
const struct usb_device_descriptor device_desc = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DTYPE_DEVICE,
    .bcdUSB             = VERSION_BCD(2,1,0),
    .bDeviceClass       = USB_CLASS_IAD,
    .bDeviceSubClass    = USB_SUBCLASS_IAD,
    .bDeviceProtocol    = USB_PROTO_IAD,
    .bMaxPacketSize0    = CDC_EP0_SIZE,
    // We use VID=0x1209 / PID=0x2323 as that is what the linux driver
    // requires to automatically load.
    .idVendor           = 0x1209,
    .idProduct          = 0x2323,
    .bcdDevice          = VERSION_BCD(2,0,0),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = INTSERIALNO_DESCRIPTOR,
    .bNumConfigurations = 1,
};

static const struct composite_config config_desc = {
    .config = {
        .bLength                = sizeof(struct usb_config_descriptor),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(struct composite_config),
        .bNumInterfaces         = 3,
        .bConfigurationValue    = 1,
        .iConfiguration         = NO_DESCRIPTOR,
        .bmAttributes           = USB_CFG_ATTR_RESERVED | USB_CFG_ATTR_SELFPOWERED,
        .bMaxPower              = USB_CFG_POWER_MA(100),
    },
    .gs_usb_iad = {
        .bLength = sizeof(struct usb_iad_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFASEASSOC,
        .bFirstInterface        = 0,
        .bInterfaceCount        = 1,
        .bFunctionClass         = USB_CLASS_VENDOR,
        .bFunctionSubClass      = USB_SUBCLASS_NONE,
        .bFunctionProtocol      = USB_PROTO_NONE,
        .iFunction              = NO_DESCRIPTOR,
    },
    .gs_usb_intf = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_VENDOR,
        .bInterfaceSubClass     = USB_SUBCLASS_NONE,
        .bInterfaceProtocol     = USB_PROTO_NONE,
        .iInterface             = NO_DESCRIPTOR,
    },
    .gs_usb_eprx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = GSUSB_RXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = GSUSB_DATA_SZ,
        .bInterval              = 0x00,
    },
    .gs_usb_eptx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = GSUSB_TXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = GSUSB_DATA_SZ,
        .bInterval              = 0x00,
    },
    .comm_iad = {
        .bLength = sizeof(struct usb_iad_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFASEASSOC,
        .bFirstInterface        = 1,
        .bInterfaceCount        = 2,
        .bFunctionClass         = USB_CLASS_CDC,
        .bFunctionSubClass      = USB_CDC_SUBCLASS_ACM,
        .bFunctionProtocol      = CDC_PROTOCOL,
        .iFunction              = NO_DESCRIPTOR,
    },
    .comm = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_CDC,
        .bInterfaceSubClass     = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol     = CDC_PROTOCOL,
        .iInterface             = NO_DESCRIPTOR,
    },
    .cdc_hdr = {
        .bFunctionLength        = sizeof(struct usb_cdc_header_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_HEADER,
        .bcdCDC                 = VERSION_BCD(1,1,0),
    },
    .cdc_mgmt = {
        .bFunctionLength        = sizeof(struct usb_cdc_call_mgmt_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_CALL_MANAGEMENT,
        .bmCapabilities         = 0,
        .bDataInterface         = 2,

    },
    .cdc_acm = {
        .bFunctionLength        = sizeof(struct usb_cdc_acm_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_ACM,
        .bmCapabilities         = 0,
    },
    .cdc_union = {
        .bFunctionLength        = sizeof(struct usb_cdc_union_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_UNION,
        .bMasterInterface0      = 1,
        .bSlaveInterface0       = 2,
    },
    .comm_ep = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_NTF_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = CDC_NTF_SZ,
        .bInterval              = 0xFF,
    },
    .data = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 2,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_CDC_DATA,
        .bInterfaceSubClass     = USB_SUBCLASS_NONE,
        .bInterfaceProtocol     = USB_PROTO_NONE,
        .iInterface             = NO_DESCRIPTOR,
    },
    .data_eprx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_RXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_SZ,
        .bInterval              = 0x01,
    },
    .data_eptx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_TXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_SZ,
        .bInterval              = 0x01,
    },
};

const struct usb_string_descriptor lang_desc     = USB_ARRAY_DESC(USB_LANGID_ENG_US);
const struct usb_string_descriptor manuf_desc_en{
  .bLength = 14,
      .bDescriptorType = USB_DTYPE_STRING,
      .wString = {0x6d, 0x6a, 0x62, 0x6f, 0x74, 0x73, 0x00},
};
const struct usb_string_descriptor prod_desc_en{
  .bLength = 18,
      .bDescriptorType = USB_DTYPE_STRING,
      .wString = {0x66, 0x64, 0x63, 0x61, 0x6e, 0x75, 0x73, 0x62, 0x00},
};
const struct usb_string_descriptor *const dtable[] = {
    &lang_desc,
    &manuf_desc_en,
    &prod_desc_en,
};

struct __attribute__((packed)) bos_descriptor {
    // BOS descriptor header
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumDeviceCaps;

    // MS OS 2.0 Platform Capability descriptor
    uint8_t  bCapLength;
    uint8_t  bCapDescriptorType;
    uint8_t  bCapabilityType;
    uint8_t  bReserved;
    uint8_t  platformUUID[16];
    uint32_t dwWindowsVersion;
    uint16_t wMSOSDescriptorSetTotalLength;
    uint8_t  bMS_VendorCode;
    uint8_t  bAltEnumCode;
};

// Vendor request code for MS OS 2.0 Descriptor Set
static constexpr uint8_t MS_OS_20_VENDOR_CODE = 0x20;

// 10 (header) +
//   8 (config subset) +
//   8 (function subset) +
//   128 (registry property) +
//   20 (compatible ID) = 174 bytes
static constexpr uint16_t MS_OS_20_DESC_SET_LENGTH = 174;

static constexpr struct bos_descriptor bos_desc = {
    .bLength = 5,
    .bDescriptorType = 0x0F,  // BOS
    .wTotalLength = sizeof(struct bos_descriptor),
    .bNumDeviceCaps = 1,

    .bCapLength = 28,
    .bCapDescriptorType = 0x10,  // Device Capability
    .bCapabilityType = 0x05,     // Platform
    .bReserved = 0,
    // Microsoft OS 2.0 Platform Capability UUID:
    // {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
    .platformUUID = {
      0xDF, 0x60, 0xDD, 0xD8,
      0x89, 0x45,
      0xC7, 0x4C,
      0x9C, 0xD2,
      0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F
    },
    .dwWindowsVersion = 0x06030000,  // Windows 8.1 or later
    .wMSOSDescriptorSetTotalLength = MS_OS_20_DESC_SET_LENGTH,
    .bMS_VendorCode = MS_OS_20_VENDOR_CODE,
    .bAltEnumCode = 0,
};

struct __attribute__((packed)) ms_os_20_descriptor_set {
    // Descriptor Set Header
    uint16_t wLength;
    uint16_t wDescriptorType;
    uint32_t dwWindowsVersion;
    uint16_t wTotalLength;

    // Configuration Subset Header (REQUIRED for composite devices)
    uint16_t wConfigLength;
    uint16_t wConfigType;
    uint8_t  bConfigurationValue;
    uint8_t  bReserved1;
    uint16_t wConfigSubsetLength;

    // Function Subset Header (for interface 0)
    uint16_t wFunctionLength;
    uint16_t wFunctionType;
    uint8_t  bFirstInterface;
    uint8_t  bReserved2;
    uint16_t wSubsetLength;


    // Registry Property Feature Descriptor (Device Interface GUID)
    uint16_t wPropertyLength;
    uint16_t wPropertyType;
    uint16_t wPropertyDataType;
    uint16_t wPropertyNameLength;
    uint16_t propertyName[20];
    uint16_t wPropertyDataLength;
    uint16_t propertyData[39];

    // Compatible ID Feature Descriptor
    uint16_t wCompatIDLength;
    uint16_t wCompatIDType;
    uint8_t  compatibleID[8];
    uint8_t  subCompatibleID[8];
};

static constexpr struct ms_os_20_descriptor_set ms_os_20_desc_set = {
      // Descriptor Set Header
    .wLength = 10,
    .wDescriptorType = 0x0000,
    .dwWindowsVersion = 0x06030000,
    .wTotalLength = MS_OS_20_DESC_SET_LENGTH,

    // Configuration Subset Header
    .wConfigLength = 8,
    .wConfigType = 0x0001,
    .bConfigurationValue = 0,
    .bReserved1 = 0,
    // 8 (Config header) + 8 (Function header) + 128 (Property) + 20 (CompatID)
    .wConfigSubsetLength = 164,

    // Function Subset Header
    .wFunctionLength = 8,
    .wFunctionType = 0x0002,
    .bFirstInterface = 0,  // gs_usb interface
    .bReserved2 = 0,
    // 8 (Subset header) + 128 (Property) + 20 (CompatID)
    .wSubsetLength = 156,

    // Device Interface GUID Registry Property
    .wPropertyLength = 128,
    .wPropertyType = 0x0004,
    .wPropertyDataType = 0x0001,  // REG_SZ (single null-terminated string)
    .wPropertyNameLength = 40,
    // "DeviceInterfaceGUID" in UTF-16LE (19 chars + null terminator)
    .propertyName = {
        'D','e','v','i','c','e','I','n','t','e','r','f','a','c','e',
        'G','U','I','D',
        0
    },
    .wPropertyDataLength = 78,

    // "{5805f81d-5c31-4533-bc20-ab32e3c3e72e}" - CUSTOM fdcanusb GUID
    //
    // It seems that lowercase is *required* for this to function
    // properly in Windows.
    .propertyData = {
        '{','5','8','0','5','f','8','1','d','-',
        '5','c','3','1','-',
        '4','5','3','3','-',
        'b','c','2','0','-',
        'a','b','3','2','e','3','c','3','e','7','2','e',
        '}',
        0
    },

    // Compatible ID
    .wCompatIDLength = 20,
    .wCompatIDType = 0x0003,
    .compatibleID = {'W', 'I', 'N', 'U', 'S', 'B', 0, 0},
    .subCompatibleID = {0, 0, 0, 0, 0, 0, 0, 0},
};

}

namespace micro = mjlib::micro;

namespace fw {

class Stm32G4AsyncUsbCdc::Impl {
 public:
  static Impl* g_impl;

  Impl(const Options& options) : options_(options) {
    g_impl = this;

    usbd_init(&udev_, &usbd_hw, CDC_EP0_SIZE, ubuf_, sizeof(ubuf_));
    usbd_reg_config(&udev_, &Impl::g_cdc_setconf);
    usbd_reg_control(&udev_, &Impl::g_cdc_control);
    usbd_reg_descr(&udev_, &Impl::g_cdc_getdesc);

    usbd_enable(&udev_, true);
    usbd_connect(&udev_, true);
  }

  void Poll() {
    ProcessWrite();
    usbd_poll(&udev_);
  }

  void Poll10Ms() {
    led_com_.write(0);
  }

  bool IsCdcActive() const {
    const uint32_t now_us = options_.timer->read_us();
    const uint32_t elapsed_us = now_us - last_cdc_activity_us_;
    constexpr uint32_t kActivityTimeoutUs = 1000000;
    return elapsed_us < kActivityTimeoutUs;
  }

  void AsyncReadSome(const mjlib::base::string_span& data,
                     const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_read_callback_);

    current_read_callback_ = callback;
    current_read_data_ = data;

    ProcessRead();
  }

  void AsyncWriteSome(const std::string_view& data,
                      const micro::SizeCallback& callback) {
    MJ_ASSERT(!current_write_callback_);

    current_write_callback_ = callback;
    current_write_data_ = data;
    current_write_size_ = data.size();
  }

  void ProcessWrite() {
    if (!current_write_callback_) {
      return;
    }

    // See if we can fill this now.
    if (buffer_.size < CDC_DATA_SZ) {
      const auto to_write = std::min(
          current_write_data_.size(),
          (CDC_DATA_SZ - buffer_.size));
      std::memcpy(&buffer_.buf[buffer_.size],
                  current_write_data_.data(),
                  to_write);
      buffer_.size += to_write;

      auto copy = current_write_callback_;
      current_write_callback_ = {};
      current_write_data_ = {};

      copy(micro::error_code(), to_write);
    }
  }

  usbd_respond cdc_setconf(uint8_t cfg) {
    switch (cfg) {
    case 0:
        /* deconfiguring device */
        usbd_ep_deconfig(&udev_, CDC_NTF_EP);
        usbd_ep_deconfig(&udev_, CDC_TXD_EP);
        usbd_ep_deconfig(&udev_, CDC_RXD_EP);
        usbd_reg_endpoint(&udev_, CDC_RXD_EP, 0);
        usbd_reg_endpoint(&udev_, CDC_TXD_EP, 0);

        usbd_ep_deconfig(&udev_, GSUSB_RXD_EP);
        usbd_ep_deconfig(&udev_, GSUSB_TXD_EP);
        usbd_reg_endpoint(&udev_, GSUSB_RXD_EP, 0);
        usbd_reg_endpoint(&udev_, GSUSB_TXD_EP, 0);
        return usbd_ack;
    case 1:
        /* configuring device */
        usbd_ep_config(&udev_, CDC_RXD_EP, USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/, CDC_DATA_SZ);
        usbd_ep_config(&udev_, CDC_TXD_EP, USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/, CDC_DATA_SZ);
        usbd_ep_config(&udev_, CDC_NTF_EP, USB_EPTYPE_INTERRUPT, CDC_NTF_SZ);
        usbd_reg_endpoint(&udev_, CDC_RXD_EP, g_cdc_rxonly);
        usbd_reg_endpoint(&udev_, CDC_TXD_EP, g_cdc_txonly);
        usbd_ep_write(&udev_, CDC_TXD_EP, 0, 0);

        // configure gs_usb endpoints
        usbd_ep_config(&udev_, GSUSB_RXD_EP, USB_EPTYPE_BULK, GSUSB_DATA_SZ);
        usbd_ep_config(&udev_, GSUSB_TXD_EP, USB_EPTYPE_BULK, GSUSB_BUFFER_SZ);
        usbd_reg_endpoint(&udev_, GSUSB_RXD_EP, g_gsusb_rxonly);
        usbd_reg_endpoint(&udev_, GSUSB_TXD_EP, g_gsusb_txonly);

        return usbd_ack;
    default:
        return usbd_fail;
    }
  }

  usbd_respond cdc_control(usbd_ctlreq* req, usbd_rqc_callback* callback) {
    // gs_usb vendor requests
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) ==
        (USB_REQ_INTERFACE | USB_REQ_VENDOR) &&
        req->wIndex == 0) {
      if (gs_usb_vendor_handler_) {
        return gs_usb_vendor_handler_(&udev_, req, callback);
      }
    }

    // CDC requests
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) ==
        (USB_REQ_INTERFACE | USB_REQ_CLASS) &&
        req->wIndex == 1 ) {
      switch (req->bRequest) {
        case USB_CDC_SET_CONTROL_LINE_STATE:
          return usbd_ack;
        case USB_CDC_SET_LINE_CODING:
          memcpy( req->data, &cdc_line_, sizeof(cdc_line_));
          return usbd_ack;
        case USB_CDC_GET_LINE_CODING:
          udev_.status.data_ptr = &cdc_line_;
          udev_.status.data_count = sizeof(cdc_line_);
          return usbd_ack;
        default:
          return usbd_fail;
      }
    }

    // MS OS 2.0 Descriptor Set request
    if ((req->bmRequestType == 0xC0 || req->bmRequestType == 0xC1) &&
        req->bRequest == MS_OS_20_VENDOR_CODE &&
        req->wIndex == 7) {
      udev_.status.data_ptr = (void*)(&ms_os_20_desc_set);
      udev_.status.data_count = sizeof(ms_os_20_desc_set);
      return usbd_ack;
    }

    return usbd_fail;
  }

  void RegisterGsUsbHandler(Stm32G4AsyncUsbCdc::VendorControlHandler handler) {
    gs_usb_vendor_handler_ = handler;
  }

  void RegisterGsUsbEndpointHandlers(Stm32G4AsyncUsbCdc::EndpointHandler rx_handler,
                                     Stm32G4AsyncUsbCdc::EndpointHandler tx_handler) {
    gs_usb_rx_handler_ = rx_handler;
    gs_usb_tx_handler_ = tx_handler;
  }

  usbd_respond cdc_getdesc(usbd_ctlreq* req, void** address, uint16_t* length) {
    const uint8_t dtype = req->wValue >> 8;
    const uint8_t dnumber = req->wValue & 0xFF;
    const void* desc = {};
    uint16_t len = 0;
    switch (dtype) {
      case USB_DTYPE_DEVICE:
        desc = &device_desc;
        break;
      case USB_DTYPE_CONFIGURATION:
        desc = &config_desc;
        len = sizeof(config_desc);
        break;
      case USB_DTYPE_STRING:
        if (dnumber < 3) {
          desc = dtable[dnumber];
        } else {
          return usbd_fail;
        }
        break;
      case 0x0F:  // BOS descriptor
        desc = &bos_desc;
        len = sizeof(bos_desc);
        break;
      default:
        return usbd_fail;
    }
    if (len == 0) {
      len = ((struct usb_header_descriptor*)desc)->bLength;
    }
    *address = (void*)desc;
    *length = len;
    return usbd_ack;
  }

  void cdc_rxonly(uint8_t event, uint8_t ep) {
    // We always want to read the full amount from the endpoint.
    const auto bytes_to_read = std::min<int>(CDC_DATA_SZ, sizeof(fifo_) - fpos_);
    const auto actual = usbd_ep_read(&udev_, ep, &fifo_[fpos_], bytes_to_read);
    fpos_ += actual;

    led_com_.write(1);

    ProcessRead();
  }

  void ProcessRead() {
    if (fpos_ == 0) { return; }
    if (!current_read_callback_) { return; }

    auto copy = current_read_callback_;
    const auto bytes_to_read = std::min<int>(fpos_, current_read_data_.size());
    std::memcpy(current_read_data_.data(), fifo_, bytes_to_read);
    std::memmove(&fifo_[0], &fifo_[bytes_to_read], fpos_ - bytes_to_read);
    fpos_ -= bytes_to_read;

    current_read_callback_ = {};
    current_read_data_ = {};

    copy(micro::error_code(), bytes_to_read);
  }

  void cdc_txonly(uint8_t event, uint8_t ep) {
    last_cdc_activity_us_ = options_.timer->read_us();

    if (buffer_.size == 0) {
      // Write nothing.
      usbd_ep_write(&udev_, ep, fifo_, 0);
      return;
    }

    led_com_.write(1);

    usbd_ep_write(&udev_, ep, &buffer_.buf[0], buffer_.size);
    buffer_.size = 0;
  }

  static usbd_respond g_cdc_setconf (usbd_device *dev, uint8_t cfg) {
    return g_impl->cdc_setconf(cfg);
  }

  static usbd_respond g_cdc_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
    return g_impl->cdc_control(req, callback);
  }

  static usbd_respond g_cdc_getdesc (usbd_ctlreq *req, void **address, uint16_t *length) {
    return g_impl->cdc_getdesc(req, address, length);
  }

  static void g_cdc_rxonly(usbd_device* dev, uint8_t event, uint8_t ep) {
    g_impl->cdc_rxonly(event, ep);
  }

  static void g_cdc_txonly(usbd_device* dev, uint8_t event, uint8_t ep) {
    g_impl->cdc_txonly(event, ep);
  }

  static void g_gsusb_rxonly(usbd_device* dev, uint8_t event, uint8_t ep) {
    if (g_impl->gs_usb_rx_handler_) {
      g_impl->gs_usb_rx_handler_(dev, event, ep);
    }
  }

  static void g_gsusb_txonly(usbd_device* dev, uint8_t event, uint8_t ep) {
    if (g_impl->gs_usb_tx_handler_) {
      g_impl->gs_usb_tx_handler_(dev, event, ep);
    }
  }

private:
  usbd_device udev_ = {};
  uint32_t ubuf_[0x20] = {};
  uint8_t fifo_[0x200] = {};
  uint32_t fpos_ = 0;

  struct usb_cdc_line_coding cdc_line_ = {
    .dwDTERate          = 115200,
    .bCharFormat        = USB_CDC_1_STOP_BITS,
    .bParityType        = USB_CDC_NO_PARITY,
    .bDataBits          = 8,
  };

  micro::SizeCallback current_write_callback_;
  std::string_view current_write_data_;
  ssize_t current_write_size_ = {};

  struct Buffer {
    char buf[CDC_DATA_SZ] = {};
    size_t size = 0;
  };

  Buffer buffer_;

  micro::SizeCallback current_read_callback_;
  mjlib::base::string_span current_read_data_;

  DigitalOut led_com_{PB_6};

  Stm32G4AsyncUsbCdc::VendorControlHandler gs_usb_vendor_handler_;
  Stm32G4AsyncUsbCdc::EndpointHandler gs_usb_rx_handler_;
  Stm32G4AsyncUsbCdc::EndpointHandler gs_usb_tx_handler_;

  Options options_;
  uint32_t last_cdc_activity_us_ = 0;
};

Stm32G4AsyncUsbCdc::Impl* Stm32G4AsyncUsbCdc::Impl::g_impl = nullptr;

Stm32G4AsyncUsbCdc::Stm32G4AsyncUsbCdc(mjlib::micro::Pool* pool,
                                       const Options& options)
    : impl_(pool, options) {}

Stm32G4AsyncUsbCdc::~Stm32G4AsyncUsbCdc() {}

void Stm32G4AsyncUsbCdc::AsyncReadSome(const mjlib::base::string_span& data,
                                       const micro::SizeCallback& callback) {
  impl_->AsyncReadSome(data, callback);
}

void Stm32G4AsyncUsbCdc::AsyncWriteSome(const std::string_view& data,
                                        const micro::SizeCallback& callback) {
  impl_->AsyncWriteSome(data, callback);
}

void Stm32G4AsyncUsbCdc::Poll() {
  impl_->Poll();
}

void Stm32G4AsyncUsbCdc::Poll10Ms() {
  impl_->Poll10Ms();
}

bool Stm32G4AsyncUsbCdc::IsCdcActive() const {
  return impl_->IsCdcActive();
}

void Stm32G4AsyncUsbCdc::RegisterGsUsbHandler(VendorControlHandler handler) {
  impl_->RegisterGsUsbHandler(handler);
}

void Stm32G4AsyncUsbCdc::RegisterGsUsbEndpointHandlers(
    EndpointHandler rx_handler,
    EndpointHandler tx_handler) {
  impl_->RegisterGsUsbEndpointHandlers(rx_handler, tx_handler);
}

}
