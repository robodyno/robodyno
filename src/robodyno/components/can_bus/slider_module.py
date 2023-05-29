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

"""Slider module.

This module provides functions to control slider module. The slider module is a
lead screw driven linear module with a robodyno motor.

Examples:

    >>> from robodyno.components import SliderModule
    >>> from robodyno.interfaces import CanBus
    >>> can_bus = CanBus()
    >>> slider = SliderModule(can_bus, 0x10)
    >>> slider.enable()
    >>> slider.set_pos(0.1)
    >>> slider.get_pos()
    0.1
    >>> slider.disable()
    >>> can_bus.disconnect()
"""

from math import fabs, pi
from typing import Optional

from robodyno.interfaces import CanBus
from robodyno.components.can_bus.motor import Motor


class SliderModule(object):
    """Slider module.

    Attributes:
        motor (Motor): Motor object of the slider module.
    """

    def __init__(
        self,
        can: CanBus,
        id_: int = 0x10,
        type_: Optional[str] = None,
        max_vel: float = 0.02,
        lead: float = 0.01,
    ):
        """Inits SliderModule with can bus object and device id.

        This function initializes the slider module by creating a motor object
        and configuring the motor to position track mode with max velocity.

        Args:
            can (CanBus): CanBus object.
            id_ (int): Motor id of the slider module.
            type_ (string): Motor type of the slider module.
            max_vel (float): Max velocity of the slider module in m/s.
            lead (float): Lead of the slider module in m/round.
        """
        self._lead = lead
        self.motor = Motor(can, id_, type_)
        self._reduction = self._lead / 2 / pi  # in m/rad
        self.set_max_vel(max_vel)

    def set_max_vel(self, max_vel: float) -> None:
        """Sets max velocity of the slider module.

        This function configures the motor to position track mode with max
        velocity.

        Args:
            max_vel (float): Max velocity of the slider module in m/s.
        """
        self._max_vel = fabs(max_vel)
        self.motor.position_track_mode(self._max_vel / self._reduction, 40, 40)

    def enable(self) -> None:
        """Enables the slider module."""
        self.motor.enable()

    def disable(self) -> None:
        """Disables the slider module."""
        self.motor.disable()

    def init_pos(self, initial_pos: float = 0) -> None:
        """Initializes position of the slider module.

        Args:
            initial_pos (float): Initial position of the slider module in m.
        """
        self.motor.init_pos(initial_pos / self._reduction)

    def init_abs_pos(self, initial_pos: float = 0, save: bool = True) -> None:
        """Initializes absolute position of the slider module.

        Args:
            initial_pos (float): Initial absolute position of the slider module in m.
            save (bool): Whether to save the configuration.
        """
        self.motor.init_abs_pos(initial_pos / self._reduction)
        if save:
            self.motor.save()

    def set_pos(self, pos: float) -> None:
        """Sets position of the slider module.

        Args:
            pos (float): Position of the slider module in m.
        """
        self.motor.set_pos(pos / self._reduction)

    def set_abs_pos(self, pos: float) -> None:
        """Sets absolute position of the slider module.

        Args:
            pos (float): Absolute position of the slider module in m.
        """
        self.motor.set_abs_pos(pos / self._reduction)

    def get_pos(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads position of the slider module.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (float | None): Position of the slider module in m. Returns None if
                the read times out.
        """
        pos = self.motor.get_pos(timeout)
        if pos is None:
            return None
        return pos * self._reduction

    def get_abs_pos(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads absolute position of the slider module.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (float | None): Absolute position of the slider module in m. Returns
                None if the read times out.
        """
        pos = self.motor.get_abs_pos(timeout)
        if pos is None:
            return None
        return pos * self._reduction
