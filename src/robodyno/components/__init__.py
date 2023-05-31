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
