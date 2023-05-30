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

"""Stepper develop board driver.

The stepper develop board provides a stepper motor driver with a CAN bus interface.
It can be used to configure the stepper motor driver and control the stepper motor.

Examples:

    >>> from robodyno.components import StepperDriver
    >>> from robodyno.interfaces import CanBus
    >>> can = CanBus()
    >>> stepper = StepperDriver(can)
    >>> stepper.enable()
    >>> stepper.set_vel(1)
    >>> can.disconnect()
"""

from math import pi
from typing import Optional

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model


class StepperDriver(CanBusDevice):
    """Stepper develop board driver.

    Attributes:
        id (int): Device id.
        type (Model): Device type.
        fw_ver (float): Firmware version.
    """

    _CMD_SET_NODE_ID = 0x02
    _CMD_SET_SUBDIVISION = 0x03
    _CMD_SET_VEL_ACC_LIMIT = 0x04
    _CMD_ENABLE = 0x05
    _CMD_DISABLE = 0x06
    _CMD_STOP = 0x07
    _CMD_SET_POSITION = 0x08
    _CMD_SET_VELOCITY = 0x09
    _CMD_GET_POSITION = 0x0A
    _CMD_GET_VELOCITY = 0x0B

    def __init__(self, can: CanBus, id_: int = 0x22, reduction: float = 1):
        """Initializes the stepper develop board driver.

        Args:
            can (CanBus): Can bus interface.
            id_ (int): Device id.
            reduction (float): The reduction ratio of the stepper motor.

        Raises:
            ValueError: If the device is not a stepper develop board.
        """
        super().__init__(can, id_)
        self.get_version(timeout=0.015)
        if self.type is None or self.type != Model.ROBODYNO_STEPPER_DRIVER:
            raise ValueError('The device is not a stepper develop board.')
        self._reduction = reduction
        self.set_subdivision(8)

    def config_can_bus(self, new_id: int) -> None:
        """Configures the CAN bus.

        Args:
            new_id (int): New device id.

        Raises:
            ValueError: If the new id is not in the range of 0x01-0x3f.
        """
        if not 0x01 <= new_id <= 0x3F:
            raise ValueError('New CAN id must be in the range of 0x01-0x3f.')
        self._can.send(self.id, self._CMD_SET_NODE_ID, 'B', new_id)

    def set_subdivision(self, subdivision: int) -> None:
        """Sets the subdivision of the stepper motor.

        Args:
            subdivision (int): The subdivision of the stepper motor.

        Raises:
            ValueError: If the subdivision is not 8, 16, 32 or 64.
        """
        if subdivision not in [8, 16, 32, 64]:
            raise ValueError('Subdivision must be 8, 16, 32 or 64.')
        self._subdivision = subdivision
        self._factor = 200.0 * subdivision * self._reduction / 2.0 / pi
        self._can.send(self.id, self._CMD_SET_SUBDIVISION, 'B', subdivision)

    def set_vel_acc_limit(self, max_vel: float, acc: Optional[float] = None) -> None:
        """Sets the maximum velocity and acceleration of the stepper motor.

        Args:
            max_vel (float): The maximum velocity of the stepper motor.
            acc (float): The acceleration of the stepper motor. If None, the
                acceleration is set to 4 times the maximum velocity.
        """
        if acc is None:
            acc = max_vel * 4
        self._can.send(
            self.id,
            self._CMD_SET_VEL_ACC_LIMIT,
            'ff',
            max_vel * self._factor,
            acc * self._factor,
        )

    def enable(self) -> None:
        """Enables the stepper motor."""
        self._can.send(self.id, self._CMD_ENABLE, '')

    def disable(self) -> None:
        """Disables the stepper motor."""
        self._can.send(self.id, self._CMD_DISABLE, '')

    def stop(self) -> None:
        """Stops the stepper motor."""
        self._can.send(self.id, self._CMD_STOP, '')

    def set_pos(self, pos: float) -> None:
        """Sets the position of the stepper motor.

        Args:
            pos (float): The position of the stepper motor in radians.
        """
        self._can.send(self.id, self._CMD_SET_POSITION, 'i', int(pos * self._factor))

    def set_vel(self, vel: float) -> None:
        """Sets the velocity of the stepper motor.

        Args:
            vel (float): The velocity of the stepper motor in rad/s.
        """
        self._can.send(self.id, self._CMD_SET_VELOCITY, 'f', vel * self._factor)

    def get_pos(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the position of the stepper motor.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (float | None): The position of the stepper motor in radians. Returns
                None if the read times out.
        """
        try:
            pos = self._can.get(self.id, self._CMD_GET_POSITION, 'i', timeout)[0]
        except TimeoutError:
            return None
        return pos / self._factor

    def get_vel(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the velocity of the stepper motor.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (float | None): The velocity of the stepper motor in rad/s. Returns
                None if the read times out.
        """
        try:
            vel = self._can.get(self.id, self._CMD_GET_VELOCITY, 'f', timeout)[0]
        except TimeoutError:
            return None
        return vel / self._factor
