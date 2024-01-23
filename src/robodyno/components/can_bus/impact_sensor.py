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

"""Impact sensor driver

The impact sensor can be used to get the trigger status. 

Examples:
    >>> from robodyno.components import ImpactSensor
    >>> from robodyno.interfaces import CanBus
    >>> can = CanBus()
    >>> impact_sensor = ImpactSensor(can)
    >>> print(impact_sensor.get_status())
    1

"""
from typing import Optional

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model


class ImpactSensor(CanBusDevice):
    """Impact sensor driver

    Attributes:
      id(int):Device id.
      type(Model):Device type.
      fw_ver(float):Firmware version.
    """

    _CMD_GET_API_VER = 0x01
    _CMD_GET_DISTANCE = 0x02
    _CMD_CONFIG_CAN = 0x03

    def __init__(self, can: CanBus, id_: int = 0x34):
        """Initializes the impact driver.
        Args:
            can(CanBus):Can bus instance.
            id_(int):Device id.

        Raises:
            ValueError:If the device id is invalid.
        """
        super().__init__(can, id_)
        self.get_version(timeout=0.015)
        if self.type is None or self.type != Model.ROBODYNO_IMPACT_SENSOR:
            raise ValueError(f'Device of id {id_} is not a impact driver.')

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

    def get_status(self, timeout: Optional[float] = 0.02) -> Optional[float]:
        """Reads the distance.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (float | None):The  trigger status read by the impact sensor. Returns
                None if the read times out.
        """
        try:
            pos = self._can.get(self.id, self._CMD_GET_DISTANCE, 'b', timeout)[0]
        except TimeoutError:
            return None
        return pos
