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

"""Base class for all robodyno devices which use can bus as communication

All robodyno devices which use can bus as communication interface should
inherit this class.

Examples:

    >>> from robodyno.components import CanBusDevice
    >>> from robodyno.interfaces import CanBus
    >>> can_bus = CanBus()
    >>> device = CanBusDevice(can_bus, 0x01)
    >>> device.get_version()
    {'main_version': 1, 'sub_version': 0, 'type': <Model.THIRD_PARTY: 255>}
    >>> device.fw_ver
    1.0
    >>> device.type
    <Model.THIRD_PARTY: 255>
    >>> can_bus.disconnect()
"""

from typing import Optional

from robodyno.components.config.model import Model
from robodyno.interfaces import InterfaceType, get_interface_type


class CanBusDevice(object):
    """Base class for all robodyno devices which use can bus as communication
    interface.

    This class provides basic functions for all robodyno devices which use can
    bus as communication interface. It provides functions to get device
    firmware version and device type.

    Attributes:
        id (int): Device id.
        type (Model): Device type.
        fw_ver (float): Device firmware version.
    """

    _CMD_GET_VERSION = 0x01

    def __init__(self, can, device_id):
        """Init device with interface and id.

        Args:
            can (CanBus): Can bus interface.
            device_id (int): Device id.

        Raises:
            ValueError: If device id is not in range [0x01, 0x40).
            TypeError: If can is not CanBus.
        """
        if not 0x01 <= device_id < 0x40:
            raise ValueError('Device id should be in range [0x01, 0x40).')
        if get_interface_type(can) != InterfaceType.CAN_BUS:
            raise TypeError('CanBusDevice can only be initialized with CanBus.')
        self._can = can
        self.id = device_id
        self.fw_ver = None
        self.type = Model.THIRD_PARTY

    def get_version(self, timeout: Optional[float] = None) -> Optional[dict]:
        """Get device firmware version and type.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (dict | None): Device firmware version and type.
        """
        try:
            main_ver, sub_ver, data, type_ = self._can.get(
                self.id, self._CMD_GET_VERSION, 'HHeH', timeout
            )
        except TimeoutError:
            return None
        self.fw_ver = float(f'{main_ver}.{sub_ver}')
        self.type = Model(type_)
        return {
            'main_version': main_ver,
            'sub_version': sub_ver,
            'data': data,
            'type': Model(type_),
        }
