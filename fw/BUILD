# -*- python -*-

# Copyright 2019 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@com_github_ARMmbed_mbed-os//:rules.bzl", "mbed_binary")

cc_library(
    name = "git_info",
    hdrs = ["git_info.h"],
    srcs = ["git_info.cc"],
    deps = [
        "@mjlib//mjlib/base:visitor",
    ],
    linkstamp = "git_info_linkstamp.cc",
)

mbed_binary(
    name = "fdcanusb",
    srcs = [
        "can_manager.h",
        "can_manager.cc",
        "error.h",
        "error.cc",
        "fdcan.h",
        "fdcan.cc",
        "fdcanusb.cc",
        "firmware_info.h",
        "firmware_info.cc",
        "millisecond_timer.h",
        "stm32_serial.h",
        "stm32_serial.cc",
        "stm32g4_async_uart.h",
        "stm32g4_async_uart.cc",
        "stm32g4_async_usb_cdc.h",
        "stm32g4_async_usb_cdc.cc",
        "stm32g4_flash.h",
        "uuid.h",
        "usbd_stm32g474_devfs.c",
        "libusb_stm32/inc/usb.h",
        "libusb_stm32/inc/usb_cdc.h",
        "libusb_stm32/inc/usbd_core.h",
        "libusb_stm32/inc/usb_std.h",
        "libusb_stm32/src/usbd_core.c",
    ],
    deps = [
        ":git_info",
        "@mjlib//mjlib/micro:atomic_event_queue",
        "@mjlib//mjlib/micro:callback_table",
        "@mjlib//mjlib/micro:command_manager",
        "@mjlib//mjlib/micro:persistent_config",
        "@mjlib//mjlib/micro:telemetry_manager",
    ],
    includes = [
        "libusb_stm32/inc",
    ],
)

OCD = (
    "openocd -f interface/stlink.cfg -f target/stm32g4x.cfg "
)

genrule(
    name = "flash",
    tags = ["manual"],
    srcs = ["fdcanusb.bin"],
    outs = ["flash.stamp"],
    cmd = (OCD + "-c init -c \"reset_config none separate; program $(location fdcanusb.bin) verify 0x8000000 reset exit 0x8000000\" && touch $@"),
)
