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
