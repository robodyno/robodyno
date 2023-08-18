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

"""GPS sensor driver.

This module provides a class to get the GPS data from the GPS sensor which
is based on a GT-U12 module.

Examples:

    >>> from robodyno.components import GpsSensor
    >>> from robodyno.interfaces import CanBus
    >>> can = CanBus()
    >>> gps = GpsSensor(can)
    >>> gps.get_position()
    {'timestamp': datetime.time(0, 0, 0, 0), 'longitude': 0.0, 'latitude': 0.0}
    >>> can.disconnect()
"""

import struct
import datetime
from typing import Optional

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model


class GpsSensor(CanBusDevice):
    """GPS sensor driver.

    Attributes:
        id (int): Device id.
        type (Model): Device type.
        fw_ver (float): Firmware version.
        timestamp (float): Timestamp of the last received data.
    """

    _CMD_HEARTBEAT = 0x02
    _CMD_CONFIG_CAN = 0x03
    _CMD_GET_POSITION = 0x04

    def __init__(self, can: CanBus, id_: int = 0x32):
        """Initialize the GPS sensor driver.

        Args:
            can (CanBus): CanBus object.
            id_ (int, optional): Device id. Defaults to 0x32.

        Raises:
            ValueError: If the device type is not GPS sensor.
        """
        super().__init__(can, id_)
        self.get_version(timeout=0.015)
        if self.type is None or self.type != Model.ROBODYNO_GPS_SENSOR:
            raise ValueError(f'GPS sensor init failed, id: {id_}')
        self.timestamp = None
        self._can.subscribe(
            callback=self._on_receive_timestamp,
            device_id=self.id,
            cmd_id=self._CMD_HEARTBEAT,
        )

    def __del__(self):
        try:
            self._can.unsubscribe(
                callback=self._on_receive_timestamp,
                device_id=self.id,
                cmd_id=self._CMD_HEARTBEAT,
            )
        except KeyError:
            pass

    def _on_receive_timestamp(
        self, data: bytes, timestamp: float, device_id: int, cmd_id: int
    ) -> None:
        """Callback function when receiving timestamp data.

        Args:
            data (bytes): Data bytes.
            timestamp (float): Can frame timestamp.
            device_id (int): Device id.
            cmd_id (int): Command id.
        """
        del timestamp, device_id, cmd_id
        hour, minute, second = struct.unpack('<HHf', data)
        self.timestamp = datetime.time(
            hour, minute, int(second), int(second * 1000) % 1000
        )

    def get_position(self, timeout: Optional[float] = None) -> Optional[dict]:
        """Get the position of the GPS sensor.

        Args:
            timeout (float, optional): Timeout. Defaults to None.

        Returns:
            Optional[dict]: Timestamp, longitude and latitude.
        """
        try:
            latitude, longitude = self._can.get(
                self.id,
                self._CMD_GET_POSITION,
                'ff',
                timeout=timeout,
            )
        except TimeoutError:
            return None

        return {
            'timestamp': self.timestamp,
            'longitude': longitude,
            'latitude': latitude,
        }

    def get_position_str(self, timeout: Optional[float] = None) -> Optional[str]:
        """Get the position of the GPS sensor in string format.

        Args:
            timeout (float, optional): Timeout. Defaults to None.

        Returns:
            Optional[str]: Position (timestamp, longitude, latitude) in string format.
        """
        position = self.get_position(timeout)
        if position is None:
            return None
        longitude_str = 'E' if position['longitude'] >= 0 else 'W'
        latitude_str = 'N' if position['latitude'] >= 0 else 'S'
        return (
            f'{self.timestamp} {abs(position.get("longitude"))}°{longitude_str} '
            f'{abs(position.get("latitude"))}°{latitude_str}'
        )

    def config_can_bus(self, new_id: int = None, bitrate: int = 1000000) -> None:
        """Configures the can bus.

        Args:
            new_id (int): New device id.
            bitrate (int): Can bus bitrate.

        Raises:
            ValueError: If the new id is invalid.
        """
        if new_id is None:
            new_id = self.id
        if not 0x01 <= new_id <= 0x3F:
            raise ValueError('New CAN id must be in the range of 0x01-0x3f.')
        bitrate_id = {
            250000: 0,
            500000: 1,
            1000000: 2,
        }.get(bitrate, 2)
        self._can.send(self.id, self._CMD_CONFIG_CAN, 'HH', new_id, bitrate_id)
