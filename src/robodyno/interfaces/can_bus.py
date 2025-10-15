# -*-coding:utf-8 -*-
#
# Apache License, Version 2.0
#
# Copyright (c) 2023 Robottime(Beijing) Technology Co., Ltd
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

"""This module provides CAN bus interface for robodyno devices.

Examples:

    >>> from robodyno.interfaces import CanBus
    >>> can_bus = CanBus()
    >>> can_bus.send(0x10, 0x0e, 'fee', 50, 0.02, 0.1)
    >>> can_bus.get(0x10, 0x0d, 'fee')
    (50.0, 0.0200042724609375, 0.0999755859375)
    >>> can_bus.subscribe(callback, 0x10, 0x02)
    >>> can_bus.unsubscribe(callback, 0x10, 0x02)
    >>> can_bus.disconnect()
"""

import sys
from time import time, sleep
from threading import Lock, Thread, Event
from queue import Queue, Empty
from collections import defaultdict
import struct
from typing import Optional, Callable

import can


class CanBus(object):
    """CAN bus interface for robodyno devices.

    This class provides CAN bus interface for robodyno devices. It supports
    sending and receiving data from CAN bus. It also supports subscribing data
    from CAN bus.

    A thread is created to receive data from CAN bus. When a device ID and a
    command ID is subscribed, the callback function will be called when the
    corresponding data is received.
    """

    def __init__(
        self,
        bitrate: int = 1000000,
        channel: str = 'can0',
        connect: bool = True,
    ):
        """Initialize CAN bus interface with given channel and bitrate.

        Args:
            bitrate (int): CAN bus bitrate, e.g. 1000000.
            channel (str): CAN bus channel name, e.g. 'can0'.
            connect (bool): Whether to connect to CAN bus.

        Raises:
            NotImplementedError: If the platform is not supported.
        """
        self._channel = channel
        self._bitrate = bitrate
        if sys.platform == 'win32':
            self._bus_type = 'candle'
        elif sys.platform.startswith('linux'):
            self._bus_type = 'socketcan'
        else:
            raise NotImplementedError(f'Unsupported platform: {sys.platform}')
        self._bus = None
        self._recv_callbacks = defaultdict(lambda: defaultdict(set))
        self._recv_lock = Lock()
        self._send_lock = Lock()
        self._get_lock = Lock()

        self._recv_event = Event()
        self._listening_event = Event()
        self._rx_queue = Queue()

        if connect:
            self.connect()
            sleep(0.1)

    def connect(self) -> None:
        """Connect to CAN bus."""
        self._bus = can.interface.Bus(
            channel=self._channel, bustype=self._bus_type, bitrate=self._bitrate
        )
        self._recv_thread = Thread(target=self._recv_loop, daemon=True)
        self._recv_event.set()
        self._recv_thread.start()

    def disconnect(self) -> None:
        """Disconnect from CAN bus."""
        if self._recv_thread is not None:
            self._recv_event.clear()
            self._recv_thread.join()
            self._recv_thread = None
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def _combine_ids(self, device_id: int, cmd_id: int) -> int:
        """Combine device ID and command ID to arbitration ID.

        Args:
            device_id (int): Device ID.
            cmd_id (int): Command ID.

        Returns:
            (int): Arbitration ID.
        """
        return (device_id << 5) + cmd_id

    def _split_ids(self, arbitration_id: int) -> tuple:
        """Split arbitration ID to device ID and command ID.

        Args:
            arbitration_id (int): Combined ID.

        Returns:
            (tuple): Device ID and command ID.
        """
        device_id = arbitration_id >> 5
        cmd_id = arbitration_id & 0x1F
        return device_id, cmd_id

    def send(self, device_id: int, cmd_id: int, fmt: str, *args) -> None:
        """Send data to CAN bus.

        Args:
            device_id (int): Device ID.
            cmd_id (int): Command ID.
            fmt (str): Format string for packing data. See [struct module](
                https://docs.python.org/3/library/struct.html#format-characters)
                for details.
            *args (): Data to be sent.
        """
        with self._send_lock:
            arbitration_id = self._combine_ids(device_id, cmd_id)
            data = struct.pack('<' + fmt, *args)
            msg = can.Message(
                arbitration_id=arbitration_id,
                is_extended_id=False,
                data=data,
            )
            self._bus.send(msg)

    def _send_remote(self, device_id: int, cmd_id: int) -> None:
        """Send remote frame to CAN bus.

        Args:
            device_id (int): Device ID.
            cmd_id (int): Command ID.
        """
        with self._send_lock:
            arbitration_id = self._combine_ids(device_id, cmd_id)
            msg = can.Message(
                arbitration_id=arbitration_id,
                is_extended_id=False,
                is_remote_frame=True,
            )
            self._bus.send(msg)

    def get(
        self, device_id: int, cmd_id: int, fmt: str, timeout: Optional[float] = None
    ) -> tuple:
        """Get data from CAN bus.

        Args:
            device_id (int): Device ID.
            cmd_id (int): Command ID.
            fmt (str): Format string for unpacking data. See [struct module](
                https://docs.python.org/3/library/struct.html#format-characters)
                for details.
            timeout (float): Timeout for receiving data.

        Returns:
            (tuple): Received data.

        Raises:
            TimeoutError: If timeout.
        """
        with self._get_lock:
            start_time = time()
            arbitration_id = self._combine_ids(device_id, cmd_id)
            with self._rx_queue.mutex:
                self._rx_queue.queue.clear()
            self._listening_event.set()
            self._send_remote(device_id, cmd_id)
            while True:
                if timeout is not None and time() - start_time > timeout:
                    self._listening_event.clear()
                    raise TimeoutError('Timeout when receiving data.')
                try:
                    msg = self._rx_queue.get(timeout=0.001)
                except Empty:
                    continue
                if msg.arbitration_id == arbitration_id:
                    self._listening_event.clear()
                    return struct.unpack('<' + fmt, msg.data)

    def mi_motor_send(
        self,
        msg: can.Message,
        timeout: Optional[float] = None,
        master_id: int = 0xFF,
    ) -> Optional[can.Message]:
        """Send a CAN message and wait for the matching response from an MI motor.

        Args:
            msg: The CAN message to send.
            timeout: Timeout in seconds. If 0, return immediately without waiting.
            master_id: Master device ID used for response filtering.

        Returns:
            The matching response message, or None if no response is expected.

        Raises:
            TimeoutError: If no response is received within the timeout.
            ValueError: If the sent command is not recognized.
        """
        with self._get_lock:
            start_time = time()
            sent_command = (msg.arbitration_id >> 24) & 0x1F

            command_response_map = {
                0x0: {'commands': [0x0], 'check_bit0_7': lambda b: b == 0xFE},
                0x1: {'commands': [0x2], 'check_bit0_7': lambda b: b == master_id},
                0x2: {'commands': [0x2], 'check_bit0_7': lambda b: b == master_id},
                0x3: {'commands': [0x2], 'check_bit0_7': lambda b: b == master_id},
                0x4: {'commands': [0x2], 'check_bit0_7': lambda b: b == master_id},
                0x5: {'commands': [0x2], 'check_bit0_7': lambda b: b == master_id},
                0x6: {'commands': [0x2], 'check_bit0_7': lambda b: b == master_id},
                0x7: {'commands': [0x0], 'check_bit0_7': lambda b: b == 0xFE},
                0x8: {'commands': [0x8], 'check_bit0_7': lambda b: True},
                0x9: {'commands': [0x9], 'check_bit0_7': lambda b: b == master_id},
                0x11: {'commands': [0x11], 'check_bit0_7': lambda b: b == master_id},
                0x12: {'commands': [0x2], 'check_bit0_7': lambda b: b == master_id},
                0x16: {'commands': [0x2], 'check_bit0_7': lambda b: b == master_id},
            }

            expected = command_response_map.get(sent_command)
            if not expected:
                raise ValueError(f"Unknown command: 0x{sent_command:02X}")

            with self._rx_queue.mutex:
                self._rx_queue.queue.clear()

            self._listening_event.set()
            self._bus.send(msg)

            if timeout == 0:
                self._listening_event.clear()
                return None

            while True:
                if timeout is not None and time() - start_time > timeout:
                    self._listening_event.clear()
                    raise TimeoutError("Timeout waiting for response")

                try:
                    rx_msg = self._rx_queue.get(timeout=0.001)
                except Empty:
                    continue

                if not rx_msg.is_extended_id:
                    continue

                command = (rx_msg.arbitration_id >> 24) & 0x1F
                bit0_7 = rx_msg.arbitration_id & 0xFF

                if command not in expected['commands']:
                    continue
                if not expected['check_bit0_7'](bit0_7):
                    continue

                self._listening_event.clear()
                return rx_msg

    def subscribe(
        self, callback: Callable, device_id: int = -1, cmd_id: int = -1
    ) -> None:
        """Subscribe data from CAN bus.

        Args:
            callback (func): Callback function.

                The function should accept four arguments,
                e.g. `callback(data, timestamp, device_id, cmd_id)`.

            device_id (int): Device ID. -1 for all devices.
            cmd_id (int): Command ID. -1 for all commands.
        """
        with self._recv_lock:
            self._recv_callbacks[device_id][cmd_id].add(callback)

    def unsubscribe(
        self, callback: Callable, device_id: int = -1, cmd_id: int = -1
    ) -> None:
        """Unsubscribe data from CAN bus.

        Args:
            callback (func): Callback function.
            device_id (int): Device ID. -1 for all devices.
            cmd_id (int): Command ID. -1 for all commands.
        """
        with self._recv_lock:
            self._recv_callbacks[device_id][cmd_id].remove(callback)

    def _notify(self, msg: can.Message) -> None:
        """Notify received data."""
        if self._listening_event.is_set():
            self._rx_queue.put(msg)
        device_id, cmd_id = self._split_ids(msg.arbitration_id)
        for callback in self._recv_callbacks[device_id][cmd_id]:
            callback(msg.data, msg.timestamp, device_id, cmd_id)
        for callback in self._recv_callbacks[device_id][-1]:
            callback(msg.data, msg.timestamp, device_id, cmd_id)
        for callback in self._recv_callbacks[-1][cmd_id]:
            callback(msg.data, msg.timestamp, device_id, cmd_id)
        for callback in self._recv_callbacks[-1][-1]:
            callback(msg.data, msg.timestamp, device_id, cmd_id)

    def _recv_loop(self) -> None:
        """Receive data from CAN bus."""
        while self._recv_event.is_set():
            try:
                msg = self._bus.recv(timeout=1.0)
            except TimeoutError:
                continue
            if msg is None or msg.is_remote_frame:
                continue
            with self._recv_lock:
                self._notify(msg)
