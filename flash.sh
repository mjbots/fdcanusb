#!/bin/bash

openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
        -c init -c "reset_config none separate; program ./bazel-out/stm32g4-opt/bin/fw/fdcanusb.bin verify 0x8000000 reset exit 0x8000000"
