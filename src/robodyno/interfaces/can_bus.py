# -*-coding:utf-8 -*-
#
# The MIT License (MIT)
#
# Copyright (c) 2023 Robottime(Beijing) Technology Co., Ltd
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""This module provides CAN bus interface for robodyno devices.

Examples:

    >>> from robodyno.interfaces import CanBus
    >>> can_bus = CanBus()
    >>> can.send(0x10, 0x0e, 'fee', 50, 0.02, 0.1)
    >>> can.get(0x10, 0x0d, 'fee')
    (50.0, 0.0200042724609375, 0.0999755859375)
    >>> can_bus.subscribe(callback, 0x10, 0x02)
    >>> can_bus.unsubscribe(callback, 0x10, 0x02)
    >>> can_bus.disconnect()
"""

import sys
from time import time, sleep
from threading import Lock, Thread, Event
from queue import Queue
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

        self._recv_event = Event()
        self._listening_event = Event()
        self._rx_queue = Queue()

        self._recv_thread = Thread(target=self._recv_loop, daemon=True)

        if connect:
            self.connect()
            sleep(0.1)

    def connect(self) -> None:
        """Connect to CAN bus."""
        self._bus = can.interface.Bus(
            channel=self._channel, bustype=self._bus_type, bitrate=self._bitrate
        )
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
            *args: Data to be sent.
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
            (tuple): Received data. None if timeout.
        """
        start_time = time()
        with self._rx_queue.mutex:
            self._rx_queue.queue.clear()
        self._listening_event.set()
        self._send_remote(device_id, cmd_id)
        while True:
            arbitration_id = self._combine_ids(device_id, cmd_id)
            msg = self._rx_queue.get()
            if msg.arbitration_id == arbitration_id:
                self._listening_event.clear()
                return struct.unpack('<' + fmt, msg.data)
            if timeout is not None and time() - start_time > timeout:
                self._listening_event.clear()
                break
        return None

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
