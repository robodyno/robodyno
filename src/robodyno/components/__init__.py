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

"""Components of robottime devices.

Loads components from entry_points and provides a factory function for
components.
"""

import sys

from robodyno.interfaces import InterfaceType, get_interface_type, WEBOTS_AVAILABLE
from robodyno.components.can_bus.can_bus_device import CanBusDevice

if WEBOTS_AVAILABLE:
    from robodyno.components.webots.webots_device import WebotsDevice

if sys.version_info < (3, 10):
    from importlib_metadata import entry_points
else:
    from importlib.metadata import entry_points

__thismodule = sys.modules[__name__]
__components = {}


def _load_components(eps: list, interface_type: InterfaceType) -> None:
    for ep in eps:
        if ep.name in __components:
            __components[ep.name].update({interface_type: ep.load()})
        else:
            __components.update({ep.name: {interface_type: ep.load()}})


_load_components(
    entry_points(group='robodyno.components.can_bus'), InterfaceType.CAN_BUS
)

if WEBOTS_AVAILABLE:
    _load_components(
        entry_points(group='robodyno.components.webots'), InterfaceType.WEBOTS
    )


def _component_factory(component_name: str) -> callable:
    def construct_component(interface: InterfaceType, *args, **kwargs) -> callable:
        interface_type = get_interface_type(interface)
        if component_name in __components:
            if interface_type in __components[component_name]:
                return __components[component_name][interface_type](
                    interface, *args, **kwargs
                )
            else:
                raise ValueError(
                    f'Component {component_name} is not available for interface '
                    f'type {interface_type}.'
                )
        else:
            raise ValueError(f'Component {component_name} is not available.')

    return construct_component


for name in __components:
    setattr(__thismodule, name, _component_factory(name))
