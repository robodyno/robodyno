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

This module provides functions to control slider module in Webots. The slider module 
is a lead screw driven linear module with a robodyno motor.

Examples:

```python
from robodyno.components import SliderModule
from robodyno.interfaces import Webots
webots = Webots()
slider = SliderModule(webots, 0x10)
slider.enable()
slider.set_pos(0.1)
webots.step(10)
print(slider.get_pos())
slider.disable()
webots.stop()
```
"""

from math import fabs, pi
from typing import Optional

from robodyno.interfaces import Webots
from robodyno.components.webots.motor import Motor


class SliderModule(object):
    """Slider module.

    Attributes:
        motor (Motor): Motor object of the slider module.
    """

    def __init__(
        self,
        webots: Webots,
        id_: int = 0x10,
        type_: Optional[str] = None,
        max_vel: float = 0.02,
        lead: float = 0.01,
    ):
        """Initialize the slider module.

        Args:
            webots (Webots): Webots object.
            id_ (int): ID of the slider module.
            type_ (string): Type of the slider module.
            max_vel (float): Maximum velocity of the slider module in m/s.
            lead (float): Lead of the slider module in meter.
        """
        self._lead = lead
        self.motor = Motor(webots, id_, type_)
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

        Timeout is not supported in Webots.

        Returns:
            (float): Position of the slider module in m.
        """
        del timeout
        pos = self.motor.get_pos()
        return pos * self._reduction

    def get_abs_pos(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads absolute position of the slider module.

        Timeout is not supported in Webots.

        Returns:
            (float): Absolute position of the slider module in m.
        """
        del timeout
        pos = self.motor.get_abs_pos()
        return pos * self._reduction
