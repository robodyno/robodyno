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
