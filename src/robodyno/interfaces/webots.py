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

"""This module provides Webots interface for robodyno devices.

Examples:

```python
from robodyno.interfaces import Webots

def step_callback(webots):
    print(webots.time())

webots = Webots(step_callback=step_callback, start=False)
webots.start()

while True:
    # do something

    if webots.sleep(1) == -1:
        break

webots.stop()
```

"""

from threading import Thread, Lock, Event
from typing import Optional, Callable
import time

try:
    from controller import Robot
except ModuleNotFoundError as e:
    raise RuntimeError('Please run this script in Webots.') from e


class Webots(object):
    """Webots interface for robodyno devices.

    This class provides Webots interface for robodyno devices. It supports
    managing the time and running the simulation.

    A thread is created to run the simulation. When the simulation is running,
    the callback function will be called every time step.

    Attributes:
        robot (Robot): The Webots robot instance.
    """

    def __init__(
        self,
        time_step: Optional[int] = None,
        step_callback: Optional[Callable] = None,
        start: bool = True,
    ):
        """Initialize the Webots interface.

        Args:
            time_step (int, optional): The time step of the simulation. If not
                specified, the default time step of the robot will be used.
            step_callback (callable, optional): The callback function that will
                be called every time step. 
                
                The function should accept one argument, which is the webots
                interface instance. 
                
                The function should not block for too long,
                otherwise the simulation will be delayed.

            start (bool, optional): Whether to start the simulation
                automatically. Defaults to True.
        """
        self.robot = Robot()
        if time_step is None:
            self._time_step = int(self.robot.getBasicTimeStep())
        else:
            self._time_step = int(time_step)

        self._step_callback = step_callback
        self._step_lock = Lock()
        self._step_event = Event()
        self._step_thread = None

        self._devices = set()

        if start:
            self.start()

    @property
    def time_step(self) -> int:
        """Get the time step of the simulation.

        Returns:
            int: The time step of the simulation.
        """
        return self._time_step

    @property
    def step_lock(self) -> Lock:
        """Get the step lock of the simulation.

        Returns:
            Lock: The step lock of the simulation.
        """
        return self._step_lock

    def start(self) -> None:
        """Start the simulation."""
        if self._step_thread is not None:
            return

        self._step_thread = Thread(target=self._step_loop)
        self._step_event.set()
        self._step_thread.start()

    def stop(self) -> None:
        """Stop the simulation."""
        if self._step_thread is None:
            return

        self._step_event.clear()
        self._step_thread.join()
        self._step_thread = None

    def time(self) -> float:
        """Get the current time of the simulation.

        Returns:
            float: The current time of the simulation.
        """
        return self.robot.getTime()

    def sleep(self, seconds: float) -> int:
        """Sleep for a specified amount of time.

        Args:
            seconds (float): The amount of time to sleep.

        Returns:
            (int): 0 if the simulation is running, -1 if the simulation is
                stopped.
        """
        start_time = self.time()
        while self._step_event.is_set():
            if self.time() - start_time >= seconds:
                break
        else:
            return -1
        return 0

    def register(self, device) -> None:
        """Register a device to the simulation.

        Args:
            device (WebotsDevice): The device to register.
        """
        self._devices.add(device)

    def deregister(self, device) -> None:
        """Deregister a device from the simulation.

        Args:
            device (WebotsDevice): The device to deregister.
        """
        if device in self._devices:
            self._devices.remove(device)

    def _step_loop(self) -> None:
        """The loop function of the simulation."""
        while self._step_event.is_set():
            self._step_lock.acquire()

            if self.robot.stepBegin(self._time_step) == -1:
                break
            for device in self._devices:
                device.parallel_step()
            if self.robot.stepEnd() == -1:
                break
            for device in self._devices:
                device.step()

            self._step_lock.release()

            if self._step_callback is not None:
                self._step_callback(self)

            time.sleep(0.001)  # sleep for 1ms to avoid busy loop

        if self._step_lock.locked():
            self._step_lock.release()

        self._step_event.clear()
