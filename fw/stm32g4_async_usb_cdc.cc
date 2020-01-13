// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#define CDC_EP0_SIZE    0x08
#define CDC_RXD_EP      0x01
#define CDC_TXD_EP      0x82
#define CDC_DATA_SZ     0x40
#define CDC_NTF_EP      0x83
#define CDC_NTF_SZ      0x08
#define HID_RIN_EP      0x84
#define HID_RIN_SZ      0x10

#define CDC_PROTOCOL USB_PROTO_NONE

namespace {
struct cdc_config {
    struct usb_config_descriptor        config;
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
    .bcdUSB             = VERSION_BCD(2,0,0),
    .bDeviceClass       = USB_CLASS_IAD,
    .bDeviceSubClass    = USB_SUBCLASS_IAD,
    .bDeviceProtocol    = USB_PROTO_IAD,
    .bMaxPacketSize0    = CDC_EP0_SIZE,
    .idVendor           = 0x0483,
    .idProduct          = 0x5740,
    .bcdDevice          = VERSION_BCD(1,0,0),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = INTSERIALNO_DESCRIPTOR,
    .bNumConfigurations = 1,
};

static const struct cdc_config config_desc = {
    .config = {
        .bLength                = sizeof(struct usb_config_descriptor),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(struct cdc_config),
        .bNumInterfaces         = 2,
        .bConfigurationValue    = 1,
        .iConfiguration         = NO_DESCRIPTOR,
        .bmAttributes           = USB_CFG_ATTR_RESERVED | USB_CFG_ATTR_SELFPOWERED,
        .bMaxPower              = USB_CFG_POWER_MA(100),
    },
    .comm_iad = {
        .bLength = sizeof(struct usb_iad_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFASEASSOC,
        .bFirstInterface        = 0,
        .bInterfaceCount        = 2,
        .bFunctionClass         = USB_CLASS_CDC,
        .bFunctionSubClass      = USB_CDC_SUBCLASS_ACM,
        .bFunctionProtocol      = CDC_PROTOCOL,
        .iFunction              = NO_DESCRIPTOR,
    },
    .comm = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
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
        .bDataInterface         = 1,

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
        .bMasterInterface0      = 0,
        .bSlaveInterface0       = 1,
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
        .bInterfaceNumber       = 1,
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

}

namespace micro = mjlib::micro;

namespace fw {

class Stm32G4AsyncUsbCdc::Impl {
 public:
  static Impl* g_impl;

  Impl(const Options&) {
    g_impl = this;

    usbd_init(&udev_, &usbd_hw, CDC_EP0_SIZE, ubuf_, sizeof(ubuf_));
    usbd_reg_config(&udev_, &Impl::g_cdc_setconf);
    usbd_reg_control(&udev_, &Impl::g_cdc_control);
    usbd_reg_descr(&udev_, &Impl::g_cdc_getdesc);

    usbd_enable(&udev_, true);
    usbd_connect(&udev_, true);
  }

  void Poll() {
    usbd_poll(&udev_);
  }

  void Poll10Ms() {
    led_com_.write(0);
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

  usbd_respond cdc_setconf(uint8_t cfg) {
    switch (cfg) {
    case 0:
        /* deconfiguring device */
        usbd_ep_deconfig(&udev_, CDC_NTF_EP);
        usbd_ep_deconfig(&udev_, CDC_TXD_EP);
        usbd_ep_deconfig(&udev_, CDC_RXD_EP);
        usbd_reg_endpoint(&udev_, CDC_RXD_EP, 0);
        usbd_reg_endpoint(&udev_, CDC_TXD_EP, 0);
        return usbd_ack;
    case 1:
        /* configuring device */
        usbd_ep_config(&udev_, CDC_RXD_EP, USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/, CDC_DATA_SZ);
        usbd_ep_config(&udev_, CDC_TXD_EP, USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/, CDC_DATA_SZ);
        usbd_ep_config(&udev_, CDC_NTF_EP, USB_EPTYPE_INTERRUPT, CDC_NTF_SZ);
        usbd_reg_endpoint(&udev_, CDC_RXD_EP, g_cdc_rxonly);
        usbd_reg_endpoint(&udev_, CDC_TXD_EP, g_cdc_txonly);
        usbd_ep_write(&udev_, CDC_TXD_EP, 0, 0);
        return usbd_ack;
    default:
        return usbd_fail;
    }
  }

  usbd_respond cdc_control(usbd_ctlreq* req, usbd_rqc_callback* callback) {
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) ==
        (USB_REQ_INTERFACE | USB_REQ_CLASS) &&
        req->wIndex == 0 ) {
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
    return usbd_fail;
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
    if (!current_write_callback_) {
      // Write nothing.
      usbd_ep_write(&udev_, ep, fifo_, 0);
      return;
    }

    led_com_.write(1);

    const auto to_write = std::min<int>(current_write_data_.size(), CDC_DATA_SZ);

    usbd_ep_write(&udev_, ep, const_cast<void*>(reinterpret_cast<const void*>(
                                                    current_write_data_.data())),
                  to_write);
    current_write_data_ = std::string_view(
        current_write_data_.data() + to_write,
        current_write_data_.size() - to_write);
    if (current_write_data_.size() == 0) {
      auto copy = current_write_callback_;
      current_write_callback_ = {};
      current_write_data_ = {};
      copy(micro::error_code(), current_write_size_);
    }
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

  micro::SizeCallback current_read_callback_;
  mjlib::base::string_span current_read_data_;

  DigitalOut led_com_{PB_6};
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

}
