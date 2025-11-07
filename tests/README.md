# fdcanusb Hardware-in-the-Loop Test Suite

Some tests can be performed using a physical test fixture.

## Hardware Overview

The test fixture consists of:

* A Legacy CDC-based fdcanusb: serial B79E5874
* The Device Under Test (DUT): serial 79678ECE
* A debug UART: /dev/serial/by-id/usb-FTDI_UMFT230XB_FT6VJM97-if00-port0`

The two fdcanusb devices are connected onto a common CAN bus, and the
debug UART is connected to the TX LED of the DUT under test.

## Prerequisites

```bash
sudo apt install can-utils
```

## Usage

Several commands either need to be configured for passwordless `sudo`,
or the tests must be run as root.
