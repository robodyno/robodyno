#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""can_bus.py
Time    :   2023/01/02
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno can bus interface

  Typical usage example:

  can = CanBus(bitrate = 'CAN_1M', channel = 'can0', auto_connect = True)
  can.disconnect()
"""

import sys
import time
import threading
import can
from collections import defaultdict
from enum import Enum
import struct

if sys.platform == 'win32':
    BUS_TYPE = 'candle'
elif sys.platform.startswith('linux'):
    BUS_TYPE = 'socketcan'

class CanBus(object):
    """Can bus interface manage can id, rx thread and listeners."""
    
    class CanSpeed(Enum):
        CAN_1M    = 1000000
        CAN_1000K = 1000000
        CAN_500K  = 500000
        CAN_250K  = 250000

    MAX_DEVICE_NUM = 0x40
    DEVICE_ID_ALL = MAX_DEVICE_NUM

    MAX_COMMAND_NUM = 0x20
    COMMAND_ID_ALL = MAX_COMMAND_NUM

    def __init__(self, bitrate = 'CAN_1000K', channel = 'can0', auto_connect = True, *args, **kwargs):
        """Init can driver interface.
        
        Args:
            bitrate: can bus bandwidth, choose from 'CAN_1M', CAN_1000K', 'CAN_500K', 'CAN_250K'
            channel: can bus channel name, default 'can0'
            auto_connect: if set to False, you need to call connect() manually
        """
        self._bitrate = self.CanSpeed[bitrate].value
        self._channel = channel
        self._bus = None
        self._listeners = defaultdict(lambda: defaultdict(list))
        self._send_lock = threading.Lock()
        self._thread_running = False
        self._rx_thread = threading.Thread(
            target = self._rx_thread_method,
            name = 'can.rx',
            daemon = True,
        )
        if auto_connect:
            self.connect()
            time.sleep(0.1)

    def pack_can_id(self, device_id, command_id):
        """calculate can frame id from device id and command id
        
        Args:
            device_id: robodyno device id
            command_id: device command id
        
        Returns:
            can frame id
        """
        return device_id << 5 | command_id
    
    def unpack_can_id(self, can_id):
        """split can frame id into device id and command id
        
        Args:
            can_id: can frame id
        
        Returns:
            (device_id, command_id)
        """
        return ((can_id & 0x7ff) >> 5, can_id & 0x1f)
      
    def connect(self):
        """Connect to can bus."""
        try:
            self._bus = can.interface.Bus(
                channel=self._channel,
                bitrate = self._bitrate, 
                bustype=BUS_TYPE
            )
            self._thread_running = True
            self._rx_thread.start()
        except:
            raise IOError('Failed to connect can bus.')
    
    def disconnect(self):
        """Disconnect to can bus."""
        self._thread_running = False
        self._rx_thread.join(1)
        try:
            self._bus.shutdown()
        except:
            raise IOError('Failed to disconnect can bus.')

    def send_can_msg(self, device_id, command_id, data = b'', remote = False):
        """Send message from can bus.
        
        Args:
            device_id: robodyno device id, upper 6 bits of can frame id
            command_id: device command id, lower 5 bits of can frame id
            data: bytes of can frame payload
            remotr: can frame rtr bit
        """
        with self._send_lock:
            can_frame_id = self.pack_can_id(device_id, command_id)
            msg = can.Message(arbitration_id=can_frame_id, data=data, is_extended_id=False, is_remote_frame=remote)
            self._bus.send(msg)

    def subscribe(self, device_id, command_id, callback):
        """Subscribe specific device id and command id from can bus.
        
        Args:
            device_id: robodyno device id or Interface.DEVICE_ID_ALL
            command_id: device command id or Interface.COMMAND_ID_ALL
            callback: func(device_id, command_id, data, timestamp)
        """
        if callback not in self._listeners[device_id][command_id]:
            self._listeners[device_id][command_id].append(callback)
        
    def unsubscribe(self, device_id, command_id, callback = None):
        """Unsubscribe specific device id and command id from can bus.
        
        Args:
            device_id: robodyno device id or Interface.DEVICE_ID_ALL
            command_id: device command id or Interface.COMMAND_ID_ALL
            callback: callback to unsubscribe or leave None to unsubscribe all callbacks
        """
        if callback:
            try:
                self._listeners[device_id][command_id].remove(callback)
            except:
                raise ValueError('Failed to find callback to unsubscribe.')
        else:
            self._listeners[device_id][command_id] = []
    
    def _notify(self, frame_id, data, timestamp):
        """Protected method, notify listeners when can frame received.
        
        Args:
            frame_id: can frame id
            data: can frame payload
            timestamp: can frame timestamp
        """
        device_id, command_id = self.unpack_can_id(frame_id)
        for callback in self._listeners[device_id][command_id]:
            callback(device_id, command_id, data, timestamp)
        for callback in self._listeners[device_id][self.COMMAND_ID_ALL]:
            callback(device_id, command_id, data, timestamp)
        for callback in self._listeners[self.DEVICE_ID_ALL][command_id]:
            callback(device_id, command_id, data, timestamp)
        for callback in self._listeners[self.DEVICE_ID_ALL][self.COMMAND_ID_ALL]:
            callback(device_id, command_id, data, timestamp)
        
    def _rx_thread_method(self):
        """read can bus thread method"""
        while self._thread_running:
            try:
                msg = self._bus.recv(0.1)
                self._notify(msg.arbitration_id, msg.data, msg.timestamp)
            except:
                pass

    @classmethod
    def get_from_bus(cls, command_id, format):
        """Decorator for getting data from can bus.
        
        Args:
            command_id: command id
            format: python struct format to unpack msg bytes
        
        Returns:
            decorated function
        """
        def wrapper(func):
            def innerwrapper(self, timeout = 0):
                try:
                    def callback(device_id, command_id, data, timestamp):
                        callback.data = data
                        callback.updated = True
                    callback.updated = False

                    self._iface.subscribe(
                        device_id = self.id, 
                        command_id = command_id, 
                        callback = callback
                    )
                    self._iface.send_can_msg(self.id, command_id, b'', True)

                    start = time.time()
                    while timeout == 0 or time.time() - start < timeout:
                        if callback.updated:
                            try:
                                return func(self, *struct.unpack(format, callback.data))
                            except:
                                raise ValueError()
                    self._iface.unsubscribe(
                        device_id = self.id,
                        command_id = command_id,
                        callback = callback
                    )
                except:
                    raise RuntimeError('Failed to get data from can bus.')
            return innerwrapper
        return wrapper

    @classmethod
    def send_to_bus(cls, command_id, format = '', remote = False):
        """Decorator for sending data from can bus.
        
        Args:
            command_id: command id
            format: python struct format to pack msg bytes
            remote: can frame rtr bit
        
        Returns:
            decorated function
        """
        def wrapper(func):
            def inner_wrapper(self, *args, **kwargs):
                try:
                    data = b''
                    if len(format) > 0:
                        data = struct.pack(format, *func(self, *args, **kwargs))
                    self._iface.send_can_msg(self.id, command_id, data, remote)
                except:
                    raise RuntimeError('Failed to send data from can bus.')
            return inner_wrapper
        return wrapper
