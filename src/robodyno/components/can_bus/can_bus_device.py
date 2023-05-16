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

"""Base class for all robodyno devices.

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
from robodyno.interfaces import CanBus


class CanBusDevice(object):
    """Base class for all robodyno devices which use can bus as communication
    interface.

    This class provides basic functions for all robodyno devices which use can
    bus as communication interface. It provides functions to get device
    firmware version and device type.

    Attributes:
        id: Device id.
        type: Device type.
        fw_ver: Device firmware version.
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
        if not isinstance(can, CanBus):
            raise TypeError('CanBusDevice can only be initialized with CanBus.')
        self._can = can
        self.id = device_id
        self.fw_ver = None
        self.type = Model.THIRD_PARTY

    def get_version(self, timeout: Optional[float] = None):
        """Get device firmware version and type.

        Args:
            timeout (float, optional): Timeout in seconds.

        Returns:
            (dict): Device firmware version and type.
        """
        main_ver, sub_ver, type_ = self._can.get(
            self.id, self._CMD_GET_VERSION, 'HHI', timeout
        )
        self.fw_ver = float(f'{main_ver}.{sub_ver}')
        self.type = Model(type_)
        return {
            'main_version': main_ver,
            'sub_version': sub_ver,
            'type': Model(type_),
        }
