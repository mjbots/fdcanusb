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
Utility functions for fdcanusb hardware-in-the-loop testing.
"""

import os
import re
import select
import serial
import subprocess
import time
from typing import Optional, List, Tuple, Dict


class FdcanUsbDevice:
    """Manages communication with an fdcanusb device via CDC ACM interface."""

    def __init__(self, device_path: str, timeout: float = 1.0):
        """
        Initialize connection to fdcanusb device.

        Args:
            device_path: Path to serial device (e.g., /dev/ttyACM0)
            timeout: Read timeout in seconds
        """
        self.device_path = device_path
        self.timeout = timeout
        self.serial = None

    def __enter__(self):
        """Context manager entry - open serial port."""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - close serial port."""
        self.close()

    def open(self):
        """Open the serial port connection."""
        self.serial = serial.Serial(
            self.device_path,
            # The baudrate doesn't matter for fdcanusb CDC
            baudrate=115200,
            timeout=self.timeout
        )
        # Flush any pending data
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self):
        """Close the serial port connection."""
        if self.serial:
            self.serial.close()
            self.serial = None

    def send_command(self,
                     command: str,
                     ignore_rcv: bool = True,
                     expect_ok: bool = True) -> List[str]:
        """
        Send a command and wait for response.

        Args:
            command: Command string to send (newline will be added)
            ignore_rcv: If True, skip 'rcv' lines in response (default True)
            expect_ok: If True, expect OK/ERR termination (default True).
                      If False, return after first non-empty response line(s)

        Returns:
            List of response lines (not including final OK/ERR)

        Raises:
            RuntimeError: If command returns ERR or times out
        """
        if not self.serial:
            raise RuntimeError("Serial port not open")

        # Flush any pending input
        self.serial.reset_input_buffer()

        # Send command
        cmd_bytes = (command + '\n').encode('ascii')
        self.serial.write(cmd_bytes)
        self.serial.flush()

        # Read response lines
        response_lines = []
        max_lines = 100  # Safety limit to prevent infinite loops
        lines_read = 0

        while lines_read < max_lines:
            line = self.serial.readline().decode('ascii', errors='ignore').strip()
            lines_read += 1

            if not line:
                if not expect_ok and response_lines:
                    # For commands that don't send OK, return what we have
                    return response_lines
                raise RuntimeError(f"Timeout waiting for response to: {command}")

            # Skip rcv frames if requested (they're spontaneous emissions)
            if ignore_rcv and line.startswith("rcv "):
                continue

            if line == "OK":
                return response_lines
            elif line.startswith("ERR"):
                raise RuntimeError(f"Command failed: {command} -> {line}")
            else:
                response_lines.append(line)
                if not expect_ok:
                    # For commands that don't send OK, return after first response
                    return response_lines

        raise RuntimeError(f"Too many lines reading response to: {command}")

    def readline(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read a single line from the device.

        Args:
            timeout: Optional timeout override

        Returns:
            Line string or None if timeout
        """
        if timeout is not None:
            old_timeout = self.serial.timeout
            self.serial.timeout = timeout

        try:
            line = self.serial.readline().decode('ascii', errors='ignore').strip()
            return line if line else None
        finally:
            if timeout is not None:
                self.serial.timeout = old_timeout

    def can_on(self, force: bool = False):
        """
        Enable CAN bus (enter Bus On state).

        Args:
            force: If True, turn off first if already on
        """
        try:
            self.send_command("can on")
        except RuntimeError as e:
            if "already in BusOn" in str(e):
                if force:
                    self.can_off()
                    self.send_command("can on")
                # else: already on, that's fine
            else:
                raise

    def can_off(self, force: bool = False):
        """
        Disable CAN bus (enter Bus Off state).

        Args:
            force: If True, ignore if already off
        """
        try:
            self.send_command("can off")
        except RuntimeError as e:
            if "already in BusOff" in str(e):
                if not force:
                    pass  # Already off, that's fine
            else:
                raise

    def ensure_bus_off(self):
        """Ensure the bus is in Bus Off state, turning it off if needed."""
        try:
            self.can_off()
        except RuntimeError as e:
            # Ignore errors about already being off
            if "BusOff" not in str(e):
                raise

    def can_send(self, can_id: int, data: bytes, flags: str = ""):
        """
        Send a CAN frame.

        Args:
            can_id: CAN identifier (11-bit or 29-bit)
            data: Payload bytes
            flags: Optional flags string (e.g., "BFR")
        """
        data_hex = data.hex().upper()
        cmd = f"can send {can_id:X} {data_hex}"
        if flags:
            cmd += f" {flags}"

        self.send_command(cmd)

    def wait_for_frame(self, timeout: float = 1.0) -> Optional[Dict]:
        """
        Wait for a received CAN frame.

        Args:
            timeout: Maximum time to wait

        Returns:
            Dictionary with frame data or None if timeout:
            {
                'can_id': int,
                'data': bytes,
                'extended': bool,
                'fd': bool,
                'brs': bool,
                'rtr': bool,
                'timestamp': Optional[int],
                'filter_id': Optional[int]
            }
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = deadline - time.time()
            line = self.readline(timeout=remaining)
            if line and line.startswith("rcv "):
                return parse_rcv_frame(line)
        return None

    def conf_get(self, key: str) -> str:
        """
        Get a configuration value.
        """
        if not self.serial:
            raise RuntimeError("Serial port not open")

        # Flush any pending input
        self.serial.reset_input_buffer()

        # Send command
        cmd = f"conf get {key}\n"
        self.serial.write(cmd.encode('ascii'))
        self.serial.flush()

        # Read the value line
        line = self.serial.readline().decode('ascii', errors='ignore').strip()
        if not line:
            raise RuntimeError(f"Timeout waiting for response to: conf get {key}")
        if line.startswith("ERR"):
            raise RuntimeError(f"Command failed: conf get {key} -> {line}")

        return line

    def conf_set(self, key: str, value: str):
        """Set a configuration value."""
        self.send_command(f"conf set {key} {value}")


def parse_rcv_frame(line: str) -> Dict:
    """
    Parse a 'rcv' line into frame data.

    Args:
        line: Raw 'rcv' line from device

    Returns:
        Dictionary with parsed frame data
    """
    # Format: rcv <HEXID> <HEXDATA> <flags>
    # Flags: E/e (extended), B/b (BRS), F/f (FD), R/r (RTR), xNN (filter)

    parts = line.split()
    if len(parts) < 3 or parts[0] != "rcv":
        raise ValueError(f"Invalid rcv line: {line}")

    frame = {
        'can_id': int(parts[1], 16),
        'data': bytes.fromhex(parts[2]),
        'extended': False,
        'fd': False,
        'brs': False,
        'rtr': False,
        'timestamp': None,
        'filter_id': None
    }

    # Parse flags
    for flag in parts[3:]:
        if flag == 'E':
            frame['extended'] = True
        elif flag == 'e':
            frame['extended'] = False
        elif flag == 'B':
            frame['brs'] = True
        elif flag == 'b':
            frame['brs'] = False
        elif flag == 'F':
            frame['fd'] = True
        elif flag == 'f':
            frame['fd'] = False
        elif flag == 'R':
            frame['rtr'] = True
        elif flag == 'r':
            frame['rtr'] = False
        elif flag.startswith('x') and len(flag) > 1 and flag[1].isdigit():
            frame['filter_id'] = int(flag[1:])

    return frame


class CanInterface:
    """Manages a SocketCAN interface for testing."""

    def __init__(self, ifname: str = "can0"):
        """
        Initialize CAN interface manager.

        Args:
            ifname: Interface name (e.g., "can0")
        """
        self.ifname = ifname
        self.was_up = False

    def is_available(self) -> bool:
        """Check if the CAN interface exists."""
        result = subprocess.run(
            ['ip', 'link', 'show', self.ifname],
            capture_output=True,
            text=True
        )
        return result.returncode == 0

    def is_up(self) -> bool:
        """Check if the CAN interface is up."""
        result = subprocess.run(
            ['ip', 'link', 'show', self.ifname],
            capture_output=True,
            text=True
        )
        if result.returncode != 0:
            return False
        return 'UP' in result.stdout

    def bring_down(self):
        """Bring the CAN interface down."""
        subprocess.run(
            ['sudo', 'ip', 'link', 'set', self.ifname, 'down'],
            check=True
        )

    def configure(self,
                  bitrate: int = 1000000,
                  dbitrate: int = 5000000,
                  sjw: int = 10,
                  dsjw: int = 5,
                  sample_point: float = 0.666,
                  dsample_point: float = 0.666,
                  restart_ms: int = 1000,
                  fd: bool = True):
        """
        Configure and bring up the CAN interface.

        Args:
            bitrate: Nominal bitrate in bps
            dbitrate: Data phase bitrate in bps (for CAN-FD)
            sjw: Synchronization jump width
            dsjw: Data SJW
            sample_point: Nominal sample point
            dsample_point: Data phase sample point
            restart_ms: Auto-restart timeout
            fd: Enable CAN-FD mode
        """
        # Bring down first if already up (to allow reconfiguration)
        if self.is_up():
            try:
                self.bring_down()
            except:
                pass  # Ignore errors

        # Build command
        cmd = [
            'sudo', 'ip', 'link', 'set', self.ifname, 'type', 'can',
            'bitrate', str(bitrate),
            'dbitrate', str(dbitrate),
            'sjw', str(sjw),
            'dsjw', str(dsjw),
            'sample-point', str(sample_point),
            'dsample-point', str(dsample_point),
            'restart-ms', str(restart_ms)
        ]
        if fd:
            cmd.append('fd')
            cmd.append('on')

        subprocess.run(cmd, check=True)

        # Bring up
        subprocess.run(
            ['sudo', 'ip', 'link', 'set', self.ifname, 'up'],
            check=True
        )

    def send_frame(self, can_id: int, data: bytes,
                   fd: bool = False, brs: bool = False):
        """
        Send a CAN frame using cansend.

        Args:
            can_id: CAN identifier
            data: Payload bytes
            fd: Use CAN-FD format
        """
        data_hex = data.hex().upper()
        separator = '##' if fd else '#'
        flags = '' if not fd else '1' if brs else '0'
        frame_str = f"{can_id:X}{separator}{flags}{data_hex}"

        subprocess.run(
            ['cansend', self.ifname, frame_str],
            check=True
        )

    def start_capture(self, filter_id: Optional[int] = None):
        """
        Start capturing CAN frames in the background.

        Args:
            filter_id: Optional CAN ID filter

        Returns:
            Popen process object that can be passed to wait_for_frame()
        """
        # Build interface spec with optional filter
        if filter_id is not None:
            # Filter format: ifname,id:mask
            ifspec = f"{self.ifname},{filter_id:X}:7FF"
        else:
            ifspec = self.ifname

        cmd = ['candump', '-x', '-n', '1', ifspec]

        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return process

    def wait_for_frame(self, process, timeout: float = 1.0) -> Optional[Dict]:
        """
        Wait for a CAN frame from a background candump process.

        Args:
            process: Process object from start_capture()
            timeout: Maximum time to wait

        Returns:
            Dictionary with frame data or None if timeout
        """
        try:
            stdout, stderr = process.communicate(timeout=timeout)

            if process.returncode != 0:
                return None

            if not stdout:
                return None

            # Parse candump output: "  can0  RX B -  123   [8]  01 02 03 04 05 06 07 08"
            match = re.search(r'(\w+)\s+([A-Z]+)\s+(.)\s+(.)\s+([0-9A-F]+)\s+\[(\d+)\]\s+(.*)', stdout)
            if not match:
                return None

            rx_tx = match.group(2)
            brs = match.group(3)
            esi = match.group(4)
            can_id = int(match.group(5), 16)
            data_str = match.group(7).replace(' ', '')
            data = bytes.fromhex(data_str) if data_str else b''

            extra_flags = match.group(6)

            extended = len(match.group(5)) > 3

            fd = len(extra_flags) > 1

            return {
                'can_id': can_id,
                'data': data,
                'extended': extended,
                'brs': brs != '-',
                'esi': esi != '-',
                'fd': fd,
            }

        except subprocess.TimeoutExpired:
            process.kill()
            return None

    def __enter__(self):
        """Context manager entry - save state."""
        self.was_up = self.is_up()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - restore state."""
        if not self.was_up and self.is_up():
            self.bring_down()


def find_device_by_serial(partial_serial: str) -> Optional[str]:
    """
    Find a device path by partial serial number.

    Args:
        partial_serial: Partial serial number to match

    Returns:
        Full device path or None if not found
    """
    dev_by_id = '/dev/serial/by-id'
    if not os.path.exists(dev_by_id):
        return None

    for entry in os.listdir(dev_by_id):
        if partial_serial in entry:
            return os.path.join(dev_by_id, entry)

    return None


def wait_for_device(partial_serial: str, timeout: float = 5.0) -> Optional[str]:
    """
    Wait for a device to appear.

    Args:
        partial_serial: Partial serial number to match
        timeout: Maximum time to wait

    Returns:
        Full device path or None if timeout
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        dev = find_device_by_serial(partial_serial)
        if dev:
            return dev
        time.sleep(0.1)
    return None


def find_can_interface_by_serial(serial: str) -> Optional[str]:
    """
    Find a SocketCAN interface by USB device serial number.

    This works on Linux by reading the USB serial number from sysfs
    for each can* network interface.

    Args:
        serial: USB serial number to match (e.g., "79678ECE")

    Returns:
        Interface name (e.g., "can0") or None if not found
    """
    net_dir = '/sys/class/net'
    if not os.path.exists(net_dir):
        return None

    for ifname in os.listdir(net_dir):
        # Only check CAN interfaces
        if not ifname.startswith('can'):
            continue

        # Try to read the USB serial number from sysfs.
        #
        # The gs_usb driver creates the interface at: /sys/class/net/canX/device
        #
        # which points to the USB interface (3-4.4.3:1.0). The
        # serial is one level up at the USB device ( 3-4.4.3/serial)
        serial_path = os.path.join(net_dir, ifname, 'device', '..', 'serial')
        try:
            # Use realpath to resolve the .. references
            serial_path = os.path.realpath(serial_path)
            if os.path.exists(serial_path):
                with open(serial_path, 'r') as f:
                    device_serial = f.read().strip()
                    if device_serial == serial:
                        return ifname
        except (OSError, IOError):
            # Skip interfaces where we can't read the serial
            continue

    return None
