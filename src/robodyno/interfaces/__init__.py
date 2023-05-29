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

"""Robodyno interfaces for robodyno devices."""

from robodyno.interfaces.can_bus import CanBus

try:
    from robodyno.interfaces.webots import Webots

    WEBOTS_AVAILABLE = True
except RuntimeError as e:
    WEBOTS_AVAILABLE = False

from enum import Enum


class InterfaceType(Enum):
    """Robodyno supported interface types."""

    UNKNOWN = 0
    CAN_BUS = 1
    WEBOTS = 2


def get_interface_type(iface: object) -> InterfaceType:
    if isinstance(iface, CanBus):
        return InterfaceType.CAN_BUS
    elif WEBOTS_AVAILABLE and isinstance(iface, Webots):
        return InterfaceType.WEBOTS
    else:
        return InterfaceType.UNKNOWN
