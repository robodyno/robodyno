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


"""Provides base class for all robodyno devices in webots.

Examples:

```python
from robodyno.components import WebotsDevice
from robodyno.interfaces import Webots
webots = Webots()

class MyDevice(WebotsDevice):
    def __init__(self, webots, device_id):
        super().__init__(webots, device_id)
        self.type = Model.MY_DEVICE
        self.register()

    def __del__(self):
        self.deregister()

    def step(self):
        pass
        
    def parallel_step(self):
        pass
```
"""

from robodyno.components.config.model import Model
from robodyno.interfaces import InterfaceType, get_interface_type


class WebotsDevice(object):
    """Base class for all robodyno devices in webots.

    All robodyno devices in webots should inherit from this class.

    Attributes:
        id (int): Device id.
        type (Model): Device type.
    """

    def __init__(self, webots: object, device_id: int):
        """Initializes WebotsDevice with webots and device id.

        Args:
            webots (Webots): Webots instance.
            device_id (int): Device id.

        Raises:
            TypeError: If webots is not an instance of Webots class.
        """
        if get_interface_type(webots) != InterfaceType.WEBOTS:
            raise TypeError(
                f'interface must be an instance of Webots class, not {type(webots)}.'
            )
        self._webots = webots
        self.id = device_id
        self.type = Model.THIRD_PARTY

    def __del__(self):
        self._webots.deregister(self)

    def register(self) -> None:
        """Registers node to webots."""
        self._webots.register(self)

    def deregister(self) -> None:
        """Deregisters node from webots."""
        self._webots.deregister(self)

    def get_version(self) -> dict:
        """Gets version of the device.

        Returns:
            dict: Version of the device.
        """
        return {'main_version': 1, 'sub_version': 0, 'type': self.type}

    def parallel_step(self) -> None:
        """Parallel step function.

        This function is called in parallel with the webots simulation step.
        """
        pass

    def step(self) -> None:
        """Step function.

        This function is called after the webots simulation step.
        """
        pass
