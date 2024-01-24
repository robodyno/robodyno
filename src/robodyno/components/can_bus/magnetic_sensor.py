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

"""Magnetic sensor driver

The magnetic guide sensor can be used to get the position status.

Examples:

    >>> from robodyno.components import MagneticSensor
    >>> from robodyno.interfaces import CanBus
    >>> can = CanBus()
    >>> mag = MagneticSensor(can)
    >>> print(mag.get_position_status())
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0]
"""

from typing import Optional
from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice


class MagneticSensor(CanBusDevice):
    """Magnetic sensor driver

    Attributes:
        id(int): Device id.
    """

    _CMD_GET_POSITION = 0x00
    _CMD_SET_POSITION = 0x01
    _CMD_ZERO_POINT = 0x02

    def __init__(self, can: CanBus, id_: int = 0x02):
        """Initializes the magnetic sensor driver.

        Args:
            can(CanBus): Can bus instance.
            id_(int): Device id.
        """
        super().__init__(can, id_)

    def get_position_status(self, timeout: Optional[float] = None) -> Optional[list]:
        """Reads the magnetic sensor position status.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (list | None): The position status value read by the magnetic sensor.
            Contains a list of 16 position status
        """
        mag_data = [0 for i in range(16)]
        try:
            self._can.send(
                self.id, 0, 'BBBBBBBB', 1, 2, 3, 4, 5, self._CMD_SET_POSITION, 0, 0
            )
            _, _, data_high, data_low = self._can.get(
                self.id, self._CMD_GET_POSITION, 'IHBB', timeout
            )
        except TimeoutError:
            return None
        sensor_data = data_high << 8 | data_low
        for i in range(16):
            mag_data[i] = sensor_data & 1
            sensor_data = sensor_data >> 1
        return mag_data

    def calibrate_magnetic_field(self):
        """In the new environment, magnetic sensors need to be calibrated."""
        self._can.send(
            self.id, 0, 'BBBBBBBB', 1, 2, 3, 4, 5, self._CMD_ZERO_POINT, 0, 0
        )
