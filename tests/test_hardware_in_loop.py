#!/usr/bin/env python3

# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

"""
Hardware-in-the-loop tests for fdcanusb and mjcanfd-usb-1x.
"""

import unittest
import time
import subprocess
import os
from typing import Optional

from fdcanusb_test_utils import (
    FdcanUsbDevice,
    CanInterface,
    find_device_by_serial,
    wait_for_device,
    find_can_interface_by_serial
)

# These are the serials in the mjbots' physical fixture.
LEGACY_CDC_SERIAL = "F34E5D64"
DUT_CDC_SERIAL = "79678ECE"

# Dynamically discover the socketcan interface for the DUT.
DUT_CAN_INTERFACE = find_can_interface_by_serial(DUT_CDC_SERIAL)
if DUT_CAN_INTERFACE is None:
    raise RuntimeError(
        f"Could not find SocketCAN interface for DUT {DUT_CDC_SERIAL}")

# These are the default timing parameters used in the legacy device.
CAN_BITRATE = 1000000
CAN_DBITRATE = 5000000
CAN_SJW = 10
CAN_DSJW = 5
CAN_SAMPLE_POINT = 0.666
CAN_DSAMPLE_POINT = 0.666
CAN_RESTART_MS = 1000


class TestHardwareEnumeration(unittest.TestCase):
    """Test that all hardware is present and accessible."""

    def test_legacy_cdc_present(self):
        """Verify legacy CDC device is accessible."""
        device_path = find_device_by_serial(LEGACY_CDC_SERIAL)
        self.assertIsNotNone(
            device_path,
            f"Legacy CDC device with serial {LEGACY_CDC_SERIAL} not found"
        )

    def test_dut_cdc_present(self):
        """Verify DUT CDC interface is accessible."""
        device_path = find_device_by_serial(DUT_CDC_SERIAL)
        self.assertIsNotNone(
            device_path,
            f"DUT CDC device with serial {DUT_CDC_SERIAL} not found"
        )

    def test_dut_gs_usb_present(self):
        """Verify DUT gs_usb interface is available."""
        can_if = CanInterface(DUT_CAN_INTERFACE)
        self.assertTrue(
            can_if.is_available(),
            f"DUT gs_usb interface {DUT_CAN_INTERFACE} not available"
        )


class TestBasicCommunication(unittest.TestCase):
    """Test basic CAN frame transmission between devices."""

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures - configure CAN interface."""
        cls.can_if = CanInterface(DUT_CAN_INTERFACE)

        # Bring down if already up
        if cls.can_if.is_up():
            cls.can_if.bring_down()

        # Configure with legacy device parameters
        cls.can_if.configure(
            bitrate=CAN_BITRATE,
            dbitrate=CAN_DBITRATE,
            sjw=CAN_SJW,
            dsjw=CAN_DSJW,
            sample_point=CAN_SAMPLE_POINT,
            dsample_point=CAN_DSAMPLE_POINT,
            restart_ms=CAN_RESTART_MS,
            fd=True
        )

    @classmethod
    def tearDownClass(cls):
        """Clean up - optionally bring down CAN interface."""
        # Leave interface up for inspection unless explicitly torn down
        pass

    def setUp(self):
        """Set up for each test."""
        # Find device paths
        self.legacy_cdc_path = find_device_by_serial(LEGACY_CDC_SERIAL)
        self.dut_cdc_path = find_device_by_serial(DUT_CDC_SERIAL)

        self.assertIsNotNone(self.legacy_cdc_path, "Legacy CDC device not found")
        self.assertIsNotNone(self.dut_cdc_path, "DUT CDC device not found")

        self.legacy_manager = FdcanUsbDevice(self.legacy_cdc_path)
        self.legacy = self.legacy_manager.__enter__()

        self.legacy.send_command("conf set can.bitrate 1000000")
        self.legacy.send_command("conf set can.fd_bitrate 5000000")

        self.legacy.ensure_bus_off()

        # Ensure DUT bus is ON to keep can0 interface up for gs_usb tests
        with FdcanUsbDevice(self.dut_cdc_path) as dut:
            dut.can_on()

        # Ensure can0 interface is actually UP (not just bus on)
        if not self.can_if.is_up():
            self.can_if.configure(
                bitrate=CAN_BITRATE,
                dbitrate=CAN_DBITRATE,
                sjw=CAN_SJW,
                dsjw=CAN_DSJW,
                sample_point=CAN_SAMPLE_POINT,
                dsample_point=CAN_DSAMPLE_POINT,
                restart_ms=CAN_RESTART_MS,
                fd=True
            )
        time.sleep(0.1)  # Let interface settle

    def tearDown(self):
        """Clean up after each test."""
        # Ensure legacy device ends with bus off
        # Note: We don't turn off DUT bus here to keep can0 interface up
        self.legacy.ensure_bus_off()

        try:
            self.legacy_manager.__exit__(None, None, None)
        except:
            # Ignore errors during cleanup
            pass
        self.legacy_manager = None
        self.legacy = None

    def test_legacy_cdc_to_dut_gs_usb(self):
        """Test: Legacy CDC sends frame → DUT gs_usb receives it."""
        test_id = 0x123
        test_data = bytes.fromhex('DEADBEEF')

        # Start capture FIRST
        capture_proc = self.can_if.start_capture()
        time.sleep(0.3)  # Let candump fully start

        # Enable CAN on legacy device
        self.legacy.can_on()
        time.sleep(0.1)  # Let bus stabilize

        # Send frame from legacy CDC
        self.legacy.can_send(test_id, test_data)

        # Now wait for the frame
        frame = self.can_if.wait_for_frame(capture_proc, timeout=2.0)

        self.assertIsNotNone(frame, "No frame received on DUT gs_usb")
        self.assertEqual(frame['can_id'], test_id, "CAN ID mismatch")
        self.assertEqual(frame['data'], test_data, "Data payload mismatch")

    def test_dut_gs_usb_to_legacy_cdc(self):
        """Test: DUT gs_usb sends frame → Legacy CDC receives it."""
        test_id = 0x456
        test_data = b'\xCA\xFE\xBA\xBE'

        # Enable CAN on legacy device
        self.legacy.can_on()
        time.sleep(0.1)

        # Send frame from DUT gs_usb
        self.can_if.send_frame(test_id, test_data)

        # Receive on legacy CDC - give it time to arrive
        time.sleep(0.1)
        frame = self.legacy.wait_for_frame(timeout=0.5)

        self.assertIsNotNone(frame, "No frame received on legacy CDC")
        self.assertEqual(frame['can_id'], test_id, "CAN ID mismatch")
        self.assertEqual(frame['data'], test_data, "Data payload mismatch")

    def test_legacy_cdc_to_dut_cdc(self):
        """Test: Legacy CDC sends frame → DUT CDC receives it."""
        test_id = 0x124
        test_data = b'\x11\x22\x33\x44'

        with FdcanUsbDevice(self.dut_cdc_path) as dut:
            # Enable CAN on both devices
            self.legacy.can_on()
            dut.can_on()
            time.sleep(0.2)

            # Flush DUT input
            dut.serial.reset_input_buffer()

            # Send frame from legacy CDC
            self.legacy.can_send(test_id, test_data)

            # Receive on DUT CDC
            time.sleep(0.1)
            frame = dut.wait_for_frame(timeout=1.0)

        self.assertIsNotNone(frame, "No frame received on DUT CDC")
        self.assertEqual(frame['can_id'], test_id, "CAN ID mismatch")
        self.assertEqual(frame['data'], test_data, "Data payload mismatch")

    def test_dut_cdc_to_legacy_cdc(self):
        """Test: DUT CDC sends frame → Legacy CDC receives it."""
        test_id = 0x457
        test_data = b'\x55\x66\x77\x88'

        with FdcanUsbDevice(self.dut_cdc_path) as dut:
            # Enable CAN on both devices
            self.legacy.can_on()
            dut.can_on()
            time.sleep(0.2)

            # Flush legacy input
            self.legacy.serial.reset_input_buffer()

            # Send frame from DUT CDC
            dut.can_send(test_id, test_data)

            # Receive on legacy CDC
            time.sleep(0.1)
            frame = self.legacy.wait_for_frame(timeout=1.0)

        self.assertIsNotNone(frame, "No frame received on legacy CDC")
        self.assertEqual(frame['can_id'], test_id, "CAN ID mismatch")
        self.assertEqual(frame['data'], test_data, "Data payload mismatch")

    def test_bidirectional_simultaneous(self):
        """Test: Frames sent from both devices are received by both."""
        legacy_id = 0x100
        legacy_data = b'\x01\x02\x03\x04'
        dut_id = 0x200
        dut_data = b'\x05\x06\x07\x08'

        with FdcanUsbDevice(self.dut_cdc_path) as dut:
            # Enable CAN on both
            self.legacy.can_on()
            dut.can_on()
            time.sleep(0.2)

            # Send from legacy
            self.legacy.can_send(legacy_id, legacy_data)

            # Wait for DUT to receive it
            time.sleep(0.1)
            frame = dut.wait_for_frame(timeout=0.5)
            self.assertIsNotNone(frame, "DUT did not receive legacy's frame")
            self.assertEqual(frame['can_id'], legacy_id)
            self.assertEqual(frame['data'], legacy_data)

            # Send from DUT
            dut.can_send(dut_id, dut_data)

            # Wait for legacy to receive it
            time.sleep(0.1)
            frame = self.legacy.wait_for_frame(timeout=0.5)
            self.assertIsNotNone(frame, "Legacy did not receive DUT's frame")
            self.assertEqual(frame['can_id'], dut_id)
            self.assertEqual(frame['data'], dut_data)

            # Note: Don't call dut.can_off() as it would bring down can0


class TestConfiguration(unittest.TestCase):
    """Test configuration commands on DUT."""

    def setUp(self):
        """Set up for each test."""
        self.dut_cdc_path = find_device_by_serial(DUT_CDC_SERIAL)
        self.assertIsNotNone(self.dut_cdc_path, "DUT CDC device not found")

    def test_conf_get(self):
        """Test reading configuration values."""
        with FdcanUsbDevice(self.dut_cdc_path) as dut:
            bitrate = dut.conf_get("can.bitrate")
            self.assertIsNotNone(bitrate, "Failed to read can.bitrate")

            sample_point = dut.conf_get("can.sample_point")
            self.assertIsNotNone(sample_point, "Failed to read can.sample_point")

    def test_can_status(self):
        """Test can status command."""
        with FdcanUsbDevice(self.dut_cdc_path) as dut:

            # Bus is likely already on (from can0 being up), so just
            # check status works
            dut.can_on()

            response = dut.send_command("can status", expect_ok=False)
            self.assertTrue(len(response) > 0, "No status response")

            # Check for status indicators in response
            status_str = ' '.join(response)
            self.assertTrue("lec=" in status_str or "BUSON" in status_str,
                            "No valid status found")

            # Note: Don't call dut.can_off() as it would bring down
            # can0 for other tests


class TestAdvancedGsUsbFeatures(unittest.TestCase):
    """Test advanced gs_usb protocol features."""

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        cls.can_if = CanInterface(DUT_CAN_INTERFACE)

    def setUp(self):
        """Set up for each test."""
        # Find device paths
        self.legacy_cdc_path = find_device_by_serial(LEGACY_CDC_SERIAL)
        self.dut_cdc_path = find_device_by_serial(DUT_CDC_SERIAL)

        self.assertIsNotNone(self.legacy_cdc_path, "Legacy CDC device not found")
        self.assertIsNotNone(self.dut_cdc_path, "DUT CDC device not found")

        # Ensure legacy device starts with bus off
        self.legacy_manager = FdcanUsbDevice(self.legacy_cdc_path)
        self.legacy = self.legacy_manager.__enter__()

        self.legacy.send_command("conf set can.bitrate 1000000")
        self.legacy.send_command("conf set can.fd_bitrate 5000000")

        self.legacy.ensure_bus_off()

        # Ensure DUT bus is ON
        with FdcanUsbDevice(self.dut_cdc_path) as dut:
            dut.can_on()

        # Ensure can0 interface is UP
        if not self.can_if.is_up():
            self.can_if.configure(
                bitrate=CAN_BITRATE,
                dbitrate=CAN_DBITRATE,
                sjw=CAN_SJW,
                dsjw=CAN_DSJW,
                sample_point=CAN_SAMPLE_POINT,
                dsample_point=CAN_DSAMPLE_POINT,
                restart_ms=CAN_RESTART_MS,
                fd=True
            )
        time.sleep(0.1)

    def tearDown(self):
        """Clean up after each test."""
        # Ensure legacy device ends with bus off and normal mode
        try:
            self.legacy.ensure_bus_off()
            # Reset any special modes
            self.legacy.send_command("conf set can.bus_monitor 0")
            self.legacy.send_command("conf set can.restricted_mode 0")
            self.legacy.send_command("conf set can.automatic_retransmission 1")
            self.legacy_manager.__exit__(None, None, None)
        except:
            pass

        self.legacy_manager = None
        self.legacy = None

    def test_canfd_frame_transmission(self):
        """Test CAN-FD frame transmission with large payload.

        This test just uses 16 byte frames, we'll test larger ones
        later.
        """

        test_id = 0x234
        test_data = bytes(range(16))

        # Start capture
        capture_proc = self.can_if.start_capture()
        time.sleep(0.3)

        self.legacy.can_on()
        time.sleep(0.1)

        # Send CAN-FD frame
        self.legacy.can_send(test_id, test_data)

        frame = self.can_if.wait_for_frame(capture_proc, timeout=2.0)

        self.assertIsNotNone(frame, "No CAN-FD frame received")
        self.assertEqual(frame['can_id'], test_id)
        self.assertEqual(frame['data'], test_data)

    def test_brs_frame_transmission(self):
        """Test CAN-FD frame with bitrate switching (BRS).
        """
        test_data = b'\x01\x02\x03\x04\x05\x06\x07\x08'

        # Test with all combinations of FD and BRS.
        for is_fd in [False, True]:
            for is_brs in [False, True]:
                if is_fd == False and is_brs == True:
                    # This combination isn't possible
                    continue

                # Start capture
                capture_proc = self.can_if.start_capture()
                time.sleep(0.3)

                self.legacy.can_on()
                time.sleep(0.1)

                test_id = 0x340 + (1 if is_fd else 0) + (2 if is_brs else 0)
                flags = ("B" if is_brs else "b") + ("F" if is_fd else "f")

                # Send CAN-FD frame with BRS
                self.legacy.can_send(test_id, test_data, flags=flags)

                frame = self.can_if.wait_for_frame(capture_proc, timeout=2.0)

                self.assertIsNotNone(frame, "No BRS frame received")
                self.assertEqual(frame['can_id'], test_id)
                self.assertEqual(frame['data'], test_data)
                self.assertEqual(frame['brs'], is_brs)
                self.assertEqual(frame['fd'], is_fd)

                self.legacy.can_off()
                time.sleep(0.1)


    def test_bitrate_configuration(self):
        """Test configuring different bitrates via gs_usb."""
        # Test changing to a different bitrate
        test_bitrate = 500000
        test_dbitrate = 2000000

        # Reconfigure can0 interface with new bitrates
        self.can_if.configure(
            bitrate=test_bitrate,
            dbitrate=test_dbitrate,
            sjw=CAN_SJW,
            dsjw=CAN_DSJW,
            sample_point=CAN_SAMPLE_POINT,
            dsample_point=CAN_DSAMPLE_POINT,
            restart_ms=CAN_RESTART_MS,
            fd=True
        )

        # Verify interface is up with new settings
        self.assertTrue(self.can_if.is_up(), "Interface should be up")

        # Configure legacy device to match
        self.legacy.send_command(f"conf set can.bitrate {test_bitrate}")
        self.legacy.send_command(f"conf set can.fd_bitrate {test_dbitrate}")
        self.legacy.can_on()
        time.sleep(0.1)

        # Send test frame
        test_id = 0x9CD
        test_data = b'\x11\x22\x33\x44'

        # Start capture
        capture_proc = self.can_if.start_capture()
        time.sleep(0.3)

        self.legacy.can_send(test_id, test_data)

        frame = self.can_if.wait_for_frame(capture_proc, timeout=2.0)
        self.legacy.can_off()

        # Restore original bitrates on legacy
        self.legacy.send_command(f"conf set can.bitrate {CAN_BITRATE}")
        self.legacy.send_command(f"conf set can.fd_bitrate {CAN_DBITRATE}")

        self.assertIsNotNone(frame, "No frame received at new bitrate")
        self.assertEqual(frame['can_id'], test_id)

        # Restore original bitrates on can0
        self.can_if.configure(
            bitrate=CAN_BITRATE,
            dbitrate=CAN_DBITRATE,
            sjw=CAN_SJW,
            dsjw=CAN_DSJW,
            sample_point=CAN_SAMPLE_POINT,
            dsample_point=CAN_DSAMPLE_POINT,
            restart_ms=CAN_RESTART_MS,
            fd=True
        )


class TestLargeFrames(unittest.TestCase):
    """Test transmission of large CAN-FD frames (48 and 64 bytes).
    """

    @classmethod
    def setUpClass(cls):
        """One-time setup for the test class."""
        cls.legacy_cdc_path = find_device_by_serial(LEGACY_CDC_SERIAL)
        cls.dut_cdc_path = find_device_by_serial(DUT_CDC_SERIAL)
        cls.can_if = CanInterface(DUT_CAN_INTERFACE)

    def setUp(self):
        """Per-test setup."""
        # Ensure DUT bus is ON to keep can0 up
        with FdcanUsbDevice(self.dut_cdc_path) as dut:
            dut.can_on()

        # Ensure can0 interface is UP
        if not self.can_if.is_up():
            self.can_if.configure(
                bitrate=CAN_BITRATE,
                dbitrate=CAN_DBITRATE,
                sjw=CAN_SJW,
                dsjw=CAN_DSJW,
                sample_point=CAN_SAMPLE_POINT,
                dsample_point=CAN_DSAMPLE_POINT,
                restart_ms=CAN_RESTART_MS,
                fd=True
            )
        time.sleep(0.1)

        self.legacy_manager = FdcanUsbDevice(self.legacy_cdc_path)
        self.legacy = self.legacy_manager.__enter__()
        self.legacy.send_command("conf set can.bitrate 1000000")
        self.legacy.send_command("conf set can.fd_bitrate 5000000")
        self.legacy.ensure_bus_off()


    def test_large_frames(self):
        """Verify 48 and 64 byte frames can be sent between all interfaces"""
        for frame_size in [48, 64]:
            for brs in [False, True]:
                for src in ['legacy_cdc', 'dut_cdc', 'dut_gsusb']:
                    for dst in ['legacy_cdc', 'dut_cdc', 'dut_gsusb']:
                        with self.subTest(frame_size=frame_size,
                                          brs=brs,
                                          src=src,
                                          dst=dst):
                            # We won't check sending to and from the same device.
                            if src.split('_')[0] == dst.split('_')[0]:
                                continue

                            self.do_large_frame_test(frame_size, src, dst, brs=brs)

    def do_large_frame_test(self, frame_size, src, dst, brs=None):
        dut_cdc_manager = None
        dut_cdc = None

        if src == 'dut_cdc' or dst == 'dut_cdc':
            dut_cdc_manager = FdcanUsbDevice(self.dut_cdc_path, timeout=2.0)
            dut_cdc = dut_cdc_manager.__enter__()

        self.legacy.can_off()
        time.sleep(0.1)
        self.legacy.can_on()
        time.sleep(0.1)

        if dut_cdc:
            dut_cdc.can_off()
            time.sleep(0.1)
            dut_cdc.can_on()
            time.sleep(0.1)

        try:
            test_id = 0x123
            test_data = bytes(range(frame_size))

            capture_proc = None

            if dst == 'dut_gsusb':
                capture_proc = self.can_if.start_capture()
            elif dst == 'dut_cdc':
                pass
            elif dst == 'legacy_cdc':
                pass

            if src == 'legacy_cdc':
                self.legacy.can_send(test_id, test_data, flags="B" if brs else "b")
            elif src == 'dut_cdc':
                dut_cdc.can_send(test_id, test_data, flags="B" if brs else "b")
            elif src == 'dut_gsusb':
                self.can_if.send_frame(test_id, test_data, fd=True, brs=brs)

            time.sleep(0.3)

            # Try to receive the result.

            if dst == 'dut_gsusb':
                frame = self.can_if.wait_for_frame(capture_proc, timeout=1.0)
            elif dst == 'dut_cdc':
                frame = dut_cdc.wait_for_frame(timeout=1.0)
            elif dst == 'legacy_cdc':
                frame = self.legacy.wait_for_frame(timeout=1.0)

            self.assertIsNotNone(frame, f"{dst} did not receive {frame_size} byte frame")
            if frame:
                self.assertEqual(len(frame['data']), frame_size)
                self.assertEqual(frame['brs'], brs)

        finally:
            if dut_cdc_manager:
                dut_cdc_manager.__exit__(None, None, None)
                dut_cdc_manager = None
                dut_cdc = None


if __name__ == '__main__':
    unittest.main()
