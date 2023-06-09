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

"""Led driver.

The led driver is a device that provides a simple interface to control the
ws2812b led strip.

Examples:

    >>> from robodyno.components import LedDriver
    >>> from robodyno.interfaces import CanBus
    >>> can = CanBus()
    >>> strip = LedDriver(can)
    >>> strip.set_color(id_=0, color=(255, 0, 0))
    >>> strip.blink(1, color=(0, 255, 0), period=0.5)
    >>> strip.breathe((2, 3), color=(0, 0, 255), period=0.5)
    >>> strip.marquee((4, 7), color=(255, 255, 0), period=1.0)
    >>> strip.clear()
    >>> can.disconnect()
"""

from typing import Union

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model


class LedDriver(CanBusDevice):
    """Led driver.

    Attributes:
        id (int): Device id.
        type (Model): Device type.
        fw_ver (float): Firmware version.
    """

    _CMD_GET_VERSION = 0x01
    _CMD_SET_COLOR = 0x02
    _CMD_CONFIG_CAN = 0x03

    def __init__(self, can: CanBus, id_: int = 0x33):
        """Initializes the led driver.

        Args:
            can (CanBus): Can bus instance.
            id_ (int): Device id.

        Raises:
            ValueError: If the device id is invalid.
        """
        super().__init__(can, id_)
        self.get_version(timeout=0.015)
        if self.type is None or self.type != Model.ROBODYNO_LED_DRIVER:
            raise ValueError(f'Device of id {id_} is not a led driver.')

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

    def set_color(self, id_: Union[int, tuple], color: tuple) -> None:
        """Sets the color of the led.

        Args:
            id_ (int | tuple): Led id(s). If it is a tuple, it will set the
                color of multiple leds starting from the first id in the tuple
                to the second id in the tuple.
            color (tuple): Led(s) color.
        """
        if isinstance(id_, int):
            id_ = (id_, id_)
        self._can.send(self.id, self._CMD_SET_COLOR, 'BBBBBBe', *id_, *color, 0, 0)

    def blink(self, id_: Union[int, tuple], color: tuple, period: float) -> None:
        """Lets the led blink.

        Args:
            id_ (int | tuple): Led id(s). If it is a tuple, it will blink
                multiple leds starting from the first id in the tuple to the
                second id in the tuple.
            color (tuple): Led(s) color.
            period (float): Half of the blink period in seconds. The led will
                be on for period seconds and off for period seconds.
        """
        if isinstance(id_, int):
            id_ = (id_, id_)
        self._can.send(self.id, self._CMD_SET_COLOR, 'BBBBBBe', *id_, *color, 1, period)

    def breathe(self, id_: Union[int, tuple], color: tuple, period: float) -> None:
        """Lets the led breathe.

        Args:
            id_ (int | tuple): Led id(s). If it is a tuple, it will breath
                multiple leds starting from the first id in the tuple to the
                second id in the tuple.
            color (tuple): Led(s) color.
            period (float): Half of the breath period in seconds. The led will
                be on for period seconds and off for period seconds.
        """
        if isinstance(id_, int):
            id_ = (id_, id_)
        self._can.send(self.id, self._CMD_SET_COLOR, 'BBBBBBe', *id_, *color, 2, period)

    def marquee(self, id_: tuple, color: tuple, period: float) -> None:
        """Turns on the led one by one and then turns off one by one.

        Args:
            id_ (tuple): Led ids. It will control multiple leds turning on and
                off one by one starting from the first id in the tuple to the
                second id in the tuple.
            color (tuple): Led(s) color.
            period (float): Half of the marquee period in seconds. The led will
                be on for period seconds and off for period seconds.

        Raises:
            ValueError: If the id_ is not a tuple.
        """
        if not isinstance(id_, tuple):
            raise ValueError('id_ must be a tuple.')
        self._can.send(self.id, self._CMD_SET_COLOR, 'BBBBBBe', *id_, *color, 3, period)

    def clear(self) -> None:
        """Turns off all the leds."""
        self._can.send(self.id, self._CMD_SET_COLOR, 'BBBBBBe', 0, 255, 0, 0, 0, 0, 0)
