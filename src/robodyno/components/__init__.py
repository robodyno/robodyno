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

The components are loaded by entry_points, and the entry_points are defined in
setup.py.
"""

import sys

from robodyno.interfaces import InterfaceType, GET_IFACE_TYPE
from robodyno.components.can_bus.can_bus_device import CanBusDevice

if sys.version_info < (3, 10):
    from importlib_metadata import entry_points
else:
    from importlib.metadata import entry_points
__thismodule = sys.modules[__name__]

__components = {}


can_eps = entry_points(group='robodyno.components.can_bus')
for ep in can_eps:
    if ep.name in __components:
        __components[ep.name].update({InterfaceType.CanBus: ep.load()})
    else:
        __components.update({ep.name: {InterfaceType.CanBus: ep.load()}})

try:
    from robodyno.components.webots.webots_device import WebotsDevice

    wb_eps = entry_points(group='robodyno.components.webots')
    for ep in wb_eps:
        if ep.name in __components:
            __components[ep.name].update({InterfaceType.Webots: ep.load()})
        else:
            __components.update({ep.name: {InterfaceType.Webots: ep.load()}})
except ImportError as e:
    pass


def component_factory(cls_name):
    def constructor(interface, *args, **kwargs):
        if cls_name not in __components:
            raise ValueError(f'Component {cls_name} not found.')
        if GET_IFACE_TYPE(interface) not in __components[cls_name]:
            raise ValueError(
                f'Component {cls_name} does not support interface {interface}.'
            )
        return __components[cls_name][GET_IFACE_TYPE(interface)](
            interface, *args, **kwargs
        )
    return constructor


for component in __components:
    setattr(__thismodule, component, component_factory(component))
