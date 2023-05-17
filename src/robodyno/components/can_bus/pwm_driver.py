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

"""Pwm develop board driver.

The pwm develop board has a 8-bit pwm output or servo output on the signal pin.
It can be used to control the servo or pwm device.

Examples:

    >>> from robodyno.components import PwmDriver
    >>> from robodyno.interfaces import CanBus
    >>> can = CanBus()
    >>> pwm = PwmDriver(can)
    >>> pwm.set_pwm(100)
    >>> pwm.set_servo(100)
    >>> can.disconnect()
"""

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model


class PwmDriver(CanBusDevice):
    """Pwm develop board driver.

    Attributes:
        id (int): Device id.
        type (Model): Device type.
        fw_ver (float): Firmware version.
    """

    _CMD_SET_NODE_ID = 0x02
    _CMD_SET_SERVO = 0x03
    _CMD_SET_PWM = 0x04

    def __init__(self, can: CanBus, id_: int = 0x21):
        """Initializes the pwm develop board.

        Args:
            can (CanBus): Can bus instance.
            id_ (int): Device id.

        Raises:
            ValueError: If the device is not a pwm develop board.
        """
        super().__init__(can, id_)
        self.get_version(timeout=0.15)
        if self.type is None or self.type != Model.ROBODYNO_D_VAC01:
            raise ValueError('The device is not a pwm develop board.')
        self.off()
        self._pwm = 0
        self._servo = None

    @property
    def pwm(self) -> int:
        """Gets the pwm value.

        Returns:
            (int): 8-bit pwm value. The range is 0-255. None if the device is in
                servo mode.
        """
        return self._pwm

    @property
    def servo_pos(self) -> int:
        """Gets the servo position.

        Returns:
            (int): Position value. The range is 0-255 (0-180 degree). None if the
                device is in pwm mode.
        """
        return self._servo

    def config_can_bus(self, new_id: int) -> None:
        """Configures the can bus.

        Args:
            new_id (int): New device id.

        Raises:
            ValueError: If the new id is not in the range of 0x01-0x3f.
        """
        if not 0x01 <= new_id <= 0x3F:
            raise ValueError('New CAN id must be in the range of 0x01-0x3f.')
        self._can.send(self.id, self._CMD_SET_NODE_ID, 'B', new_id)

    def on(self) -> None:
        """Turns on the pwm output."""
        self.set_pwm(255)

    def off(self) -> None:
        """Turns off the pwm output."""
        self.set_pwm(0)

    def set_servo(self, pos: int) -> None:
        """Sets the servo position.

        Args:
            pos (int): Position value. The range is 0-255 (0-180 degree).

        Raises:
            ValueError: If the position is not in the range of 0-255.
        """
        if not 0 <= pos <= 255:
            raise ValueError('Servo position must be in the range of 0-255.')
        self._servo = pos
        self._pwm = None
        self._can.send(self.id, self._CMD_SET_SERVO, 'B', pos)

    def set_pwm(self, pwm: int) -> None:
        """Sets the pwm value.

        Args:
            pwm (int): 8-bit pwm value. The range is 0-255.

        Raises:
            ValueError: If the pwm value is not in the range of 0-255.
        """
        if not 0 <= pwm <= 255:
            raise ValueError('PWM value must be in the range of 0-255.')
        self._pwm = pwm
        self._servo = None
        self._can.send(self.id, self._CMD_SET_PWM, 'B', pwm)
