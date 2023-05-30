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

"""Provides a class to control webots motor.

The webots motor has the same interface as the can bus motor. The webots motor
is used for simulation.

Examples:

```python
from robodyno.components import Motor
from robodyno.interfaces import Webots
webots = Webots()
motor = Motor(webots, 0x10)

motor.position_track_mode(5, 2, 2)
motor.enable()
motor.set_pos(31.4)

while True:
    print(f'pos: {motor.get_pos():.2f}, vel: {motor.get_vel():.2f}')
    if webots.sleep(0.5) == -1:
        break
```
"""

from math import pi, fabs, copysign, sqrt
from enum import Enum
from typing import Optional

from robodyno.components import WebotsDevice
from robodyno.components.can_bus.motor import Motor as CanMotor
from robodyno.components.config.model import Model
from robodyno.components.config.robottime_config import ROBOTTIME_PARAMS
from robodyno.interfaces import Webots, CanBus


class _MotorPositionTrackController(object):
    """Motor position track controller."""

    def __init__(self, max_vel, max_acc, max_dec):
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_dec = max_dec
        self.target = 0
        self.pos = 0
        self.vel = 0
        self.top_vel = 0
        self.acc = 0
        self.dec = 0
        self.t = 0
        self.t_acc = 0
        self.t_dec = 0
        self.t_vel = 0

    def set_pos(self, target_pos, pos, vel):
        self.t = 0
        self.target = target_pos
        self.pos = pos
        self.vel = vel

        dx = self.target - self.pos
        self.top_vel = self.max_vel if dx >= 0 else -self.max_vel
        dx_stop = self.vel * fabs(self.vel) / (2.0 * self.max_dec)
        s = copysign(1, dx - dx_stop)
        self.top_vel = s * self.max_vel
        self.acc = s * self.max_acc
        self.dec = -s * self.max_dec
        if s * self.vel > s * self.top_vel:
            self.acc = -self.acc
        self.t_acc = (self.top_vel - self.vel) / self.acc
        self.t_dec = -self.top_vel / self.dec
        dx_min = (
            0.5 * self.t_acc * (self.top_vel + self.vel)
            + 0.5 * self.t_dec * self.top_vel
        )
        if s * dx < s * dx_min:
            self.top_vel = s * sqrt(
                max(
                    (self.dec * self.vel * self.vel + 2 * self.acc * self.dec * dx)
                    / (self.dec - self.acc),
                    0,
                )
            )
            self.t_acc = max(0, (self.top_vel - self.vel) / self.acc)
            self.t_dec = max(0, -self.top_vel / self.dec)
            self.t_vel = 0
        else:
            self.t_vel = (dx - dx_min) / self.top_vel

    def update(self, dt):
        self.t += dt
        if self.t < 0:
            return (self.pos, self.vel)
        if self.t < self.t_acc:
            return (
                self.pos + self.vel * self.t + 0.5 * self.acc * self.t * self.t,
                self.vel + self.acc * self.t,
            )
        if self.t < self.t_acc + self.t_vel:
            return (
                self.pos
                + self.vel * self.t_acc
                + 0.5 * self.acc * self.t_acc * self.t_acc
                + self.top_vel * (self.t - self.t_acc),
                self.top_vel,
            )
        if self.t < self.t_acc + self.t_vel + self.t_dec:
            td = self.t - (self.t_acc + self.t_vel + self.t_dec)
            return (self.target + 0.5 * self.dec * td * td, self.dec * td)
        else:
            return (self.target, self.max_vel)


class _MotorPositionFilterController(object):
    """Motor position filter controller."""

    def __init__(self, bandwidth):
        self.bandwidth = bandwidth
        self.target = 0

    def set_pos(self, target_pos):
        self.target = target_pos

    def update(self, dt, pos, vel):
        bandwidth = min(self.bandwidth, 0.25 / dt)
        input_filter_ki = 2.0 * bandwidth
        input_filter_kp = 0.25 * (input_filter_ki * input_filter_ki)
        delta_pos = self.target - pos
        delta_vel = -vel
        accel = input_filter_kp * delta_pos + input_filter_ki * delta_vel
        new_vel = vel + dt * accel
        new_pos = pos + dt * new_vel
        return (new_pos, new_vel)


class Motor(WebotsDevice):
    """Webots motor component.

    Attributes:
        id (int): ID of the motor.
        type (Model): Type of the motor.
        reduction (float): Reduction of the motor.
        available_velocity (float): The maximum velocity of the motor.
        available_torque (float): The maximum torque of the motor.
        available_current (float): The maximum phase current of the motor.
        torque_constant (float):
            The torque constant of the motor in Nm/A. This is the torque provided by
            the motor when the current is 1A.
        with_brake (bool): Whether the motor has a brake.
        state (Motor.MotorState): The working state of the motor.
        mode (Motor.MotorControlMode): The control mode of the motor.
    """

    def __init__(
        self,
        webots: Webots,
        id_: int = 0x10,
        type_: Optional[str] = None,
        twin: Optional[CanBus] = None,
    ):
        """Initialize the motor.

        Args:
            webots (Webots): The instance of Webots class.
            id_ (int, optional): ID of the motor. Defaults to 0x10.
            type_ (Optional[str], optional): Force the motor type. Defaults to None.
            twin (Optional[CanBus], optional): The twin motor. Defaults to None.

        Raises:
            ValueError: The motor type is invalid.
        """
        webots.step_lock.acquire()
        super().__init__(webots, id_)
        try:
            self._name = f'0x{id_:02X}::motor'
        except TypeError:
            self._name = id_
        self._motor = self._webots.robot.getDevice(self._name)
        self._pos_sensor = self._motor.getPositionSensor()
        self._twin_motor = None
        if twin and isinstance(twin, CanBus):
            self._twin_motor = CanMotor(twin, id_, type_)
            self._init_twin_offset()
            self.type = self._twin_motor.type
        elif type_:
            self.type = getattr(Model, type_.upper(), None)
        else:
            self.type = getattr(Model, self._pos_sensor.getName(), None)
        if self.type not in ROBOTTIME_PARAMS['motor']:
            raise ValueError('Motor type is invalid')
        self.mode = self.MotorControlMode.UNKNOWN
        self.__dict__.update(ROBOTTIME_PARAMS['motor'].get(self.type, None))
        self.state = self.MotorState.DISABLED
        self._rot_factor = self.reduction / 2.0 / pi

        self._motor.enableTorqueFeedback(int(self._webots.time_step))
        self._pos_sensor.enable(int(self._webots.time_step))

        self._track_controller = _MotorPositionTrackController(
            fabs(4 * 2 * pi / self.reduction),
            fabs(10 * 2 * pi / self.reduction),
            fabs(10 * 2 * pi / self.reduction),
        )
        self._filter_controller = _MotorPositionFilterController(10)

        self._input_pos = 0
        self._input_vel = 0
        self._input_torque = 0
        self._max_vel = self.available_velocity
        self._pos_feedback = 0
        self._vel_feedback = 0
        self._torque_feedback = 0
        self._prev_pos_feedback = 0
        self._prev_update_time = self._webots.time()
        self._webots.register(self)
        webots.step_lock.release()
        if self._twin_motor:
            self.twin_mode()
        else:
            self.position_mode()

    def _init_twin_offset(self) -> bool:
        """Initialize the twin motor offset.

        Returns:
            bool: Whether the twin motor offset is initialized successfully.
        """
        self._twin_offset = 0
        if self._twin_motor:
            if self._twin_motor.fw_ver >= 1:
                abs_pos = self._twin_motor.get_abs_pos(0.5)
                if abs_pos:
                    self._twin_offset = abs_pos
                    return True
                else:
                    return False
            else:
                pos = self._twin_motor.get_pos(0.5)
                if pos:
                    self._twin_offset = pos
                    return True
                else:
                    return False

    def _set_velocity(self, velocity: float) -> None:
        """Set the velocity of the motor.

        Args:
            velocity (float): The velocity of the motor.
        """
        self._motor.setVelocity(min(velocity, self._max_vel))

    def parallel_step(self) -> None:
        """See base class."""
        if self.mode == self.MotorControlMode.TWIN_MODE:
            feedback = self._twin_motor.get_feedback(0.1)
            if feedback is not None:
                self._pos_feedback, self._vel_feedback, self._torque_feedback = feedback
                self._pos_feedback -= self._twin_offset
            if self._twin_motor.fw_ver >= 1:
                abs_pos = self._twin_motor.get_abs_pos(0.1)
                if abs_pos:
                    self._pos_feedback = abs_pos - self._twin_offset

    def step(self) -> None:
        """See base class."""
        if self.mode == self.MotorControlMode.TWIN_MODE:
            self._motor.setPosition(self._pos_feedback)
            return

        t = self._webots.time()
        dt = t - self._prev_update_time
        self._prev_update_time = t
        if dt == 0:
            return
        self._pos_feedback = self._pos_sensor.getValue()
        self._vel_feedback = (self._pos_feedback - self._prev_pos_feedback) / dt
        self._prev_pos_feedback = self._pos_feedback
        self._torque_feedback = self._motor.getTorqueFeedback()
        if self.state == self.MotorState.DISABLED:
            self._motor.setTorque(0)
        elif self.state == self.MotorState.ENABLED:
            if self.mode == self.MotorControlMode.TORQUE_MODE:
                self._motor.setTorque(self._input_torque)
            elif (
                self.mode == self.MotorControlMode.VELOCITY_MODE
                or self.mode == self.MotorControlMode.VELOCITY_RAMP_MODE
            ):
                self._set_velocity(self._input_vel)
            elif self.mode == self.MotorControlMode.POSITION_MODE:
                self._motor.setPosition(self._input_pos)
            elif self.mode == self.MotorControlMode.POSITION_FILTER_MODE:
                p, v = self._filter_controller.update(
                    dt, self._pos_feedback, self._vel_feedback
                )
                self._motor.setPosition(p)
                self._set_velocity(fabs(v))
            elif self.mode == self.MotorControlMode.POSITION_TRACK_MODE:
                p, v = self._track_controller.update(dt)
                self._motor.setPosition(p)
                self._set_velocity(fabs(v))

    def enable(self) -> None:
        """Enables the motor."""
        self._webots.step_lock.acquire()
        self.state = self.MotorState.ENABLED
        self._webots.step_lock.release()

    def disable(self) -> None:
        """Disables the motor."""
        self._webots.step_lock.acquire()
        self.state = self.MotorState.DISABLED
        self._webots.step_lock.release()

    def twin_mode(self) -> None:
        """Sets the motor to digital twin mode."""
        self._webots.step_lock.acquire()
        self._motor.setPosition(self._pos_feedback)
        self._set_velocity(self._max_vel)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.TWIN_MODE
        self.state = self.MotorState.ENABLED
        self._webots.step_lock.release()

    def position_mode(self) -> None:
        """Sets the motor to position mode."""
        self._webots.step_lock.acquire()
        self._motor.setPosition(self._pos_feedback)
        self._set_velocity(self._max_vel)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.POSITION_MODE
        self._webots.step_lock.release()

    def position_filter_mode(self, bandwidth: float) -> None:
        """Sets the motor to position filter mode.

        Args:
            bandwidth (float): The bandwidth of the filter in Hz. The value is suggested
                to be the control loop frequency in Hz.
        """
        self._webots.step_lock.acquire()
        self._filter_controller = _MotorPositionFilterController(bandwidth)
        self._filter_controller.set_pos(self._pos_feedback)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.POSITION_FILTER_MODE
        self._webots.step_lock.release()

    def position_track_mode(self, vel: float, acc: float, dec: float) -> None:
        """Sets the motor to position track mode.

        Args:
            vel (float): The velocity limit in rad/s.
            acc (float): The acceleration in rad/s^2. The value must be positive.
            dec (float): The deceleration in rad/s^2. The value must be positive.
        """
        self._webots.step_lock.acquire()
        self._track_controller = _MotorPositionTrackController(vel, acc, dec)
        self._track_controller.set_pos(
            self._pos_feedback, self._pos_feedback, self._vel_feedback
        )
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.POSITION_TRACK_MODE
        self._webots.step_lock.release()

    def velocity_mode(self) -> None:
        """Sets the motor to velocity mode."""
        self._webots.step_lock.acquire()
        self._motor.setPosition(float('+inf'))
        self._set_velocity(self._input_vel)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.VELOCITY_MODE
        self._webots.step_lock.release()

    def velocity_ramp_mode(self, ramp: float) -> None:
        """Sets the motor to velocity ramp mode.

        Args:
            ramp (float): The velocity ramp in rad/s^2. The value must be positive.
        """
        self._webots.step_lock.acquire()
        self._motor.setPosition(float('+inf'))
        self._set_velocity(self._input_vel)
        self._motor.setAcceleration(ramp)
        self.mode = self.MotorControlMode.VELOCITY_RAMP_MODE
        self._webots.step_lock.release()

    def torque_mode(self) -> None:
        """Sets the motor to torque mode."""
        self._webots.step_lock.acquire()
        self._motor.setTorque(self._input_torque)
        self.mode = self.MotorControlMode.TORQUE_MODE
        self._webots.step_lock.release()

    def set_pos(self, pos: float, vel_ff: float = 0, torque_ff: float = 0) -> None:
        """Sets the target position of the motor.

        velocity feedforward and torque feedforward are not supported in Webots.

        Args:
            pos (float): The target position in rad.
        """
        del vel_ff, torque_ff
        self._webots.step_lock.acquire()
        self._input_pos = pos
        self._filter_controller.set_pos(pos)
        self._track_controller.set_pos(pos, self._pos_feedback, self._vel_feedback)
        self._webots.step_lock.release()

    def set_abs_pos(self, pos: float, vel_ff: float = 0, torque_ff: float = 0) -> None:
        """Sets the target absolute position of the motor.

        velocity feedforward and torque feedforward are not supported in Webots.

        Args:
            pos (float): The target absolute position in rad.
        """
        del vel_ff, torque_ff
        self._webots.step_lock.acquire()
        self._input_pos = pos
        self._filter_controller.set_pos(pos)
        self._track_controller.set_pos(pos, self._pos_feedback, self._vel_feedback)
        self._webots.step_lock.release()

    def set_vel(self, vel: float, torque_ff: float = 0) -> None:
        """Sets the target velocity of the motor.

        Args:
            vel (float): The target velocity in rad/s.
        """
        del torque_ff
        self._webots.step_lock.acquire()
        self._input_vel = vel
        self._webots.step_lock.release()

    def set_torque(self, torque: float) -> None:
        """Sets the target torque of the motor.

        Args:
            torque (float): The target torque in Nm.
        """
        self._webots.step_lock.acquire()
        self._input_torque = torque
        self._webots.step_lock.release()

    def set_vel_limit(self, vel_lim: float) -> None:
        """Sets the velocity limit of the motor.

        Args:
            vel_lim (float): The velocity limit in rad/s. The value must be positive.

        Raises:
            ValueError: If the velocity limit is out of range.
        """
        self._webots.step_lock.acquire()
        if not 0 < fabs(vel_lim) <= self.available_velocity:
            raise ValueError(f'Velocity limit {vel_lim} is out of range.')
        self._max_vel = fabs(vel_lim)
        if self.mode in [
            self.MotorControlMode.POSITION_MODE,
            self.MotorControlMode.POSITION_FILTER_MODE,
            self.MotorControlMode.POSITION_TRACK_MODE,
        ]:
            self._set_velocity(fabs(vel_lim))
        self._webots.step_lock.release()

    def set_current_limit(self, current_lim: float) -> None:
        """Sets the current limit of the motor.

        Args:
            current_lim (float): The current limit in amps. The value must be positive.

        Raises:
            ValueError: If the current limit is out of range.
        """
        self._webots.step_lock.acquire()
        if not 0 < fabs(current_lim) <= self.available_current:
            raise ValueError(f'Current limit {current_lim} is out of range.')
        self._motor.setAvailableTorque(fabs(current_lim) * self.torque_constant)
        self._webots.step_lock.release()

    def get_mode(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the control mode and the mode parameters of the motor.

        Timeout is not supported in Webots.

        Returns:
            (tuple): The control mode and the mode parameters.
        """
        del timeout
        ret = None
        if self.mode == self.MotorControlMode.POSITION_FILTER_MODE:
            ret = (self.mode, {'bandwidth': self._filter_controller.bandwidth})
        elif self.mode == self.MotorControlMode.POSITION_TRACK_MODE:
            ret = (
                self.mode,
                {
                    'vel': fabs(self._track_controller.max_vel / self._rot_factor),
                    'acc': fabs(self._track_controller.max_acc / self._rot_factor),
                    'dec': fabs(self._track_controller.max_dec / self._rot_factor),
                },
            )
        elif self.mode == self.MotorControlMode.VELOCITY_RAMP_MODE:
            ret = (self.mode, {'ramp': self._motor.getAcceleration()})
        else:
            ret = (self.mode,)
        return ret

    def get_pos(self, timeout: Optional[float] = None) -> float:
        """Reads the position of the motor.

        Timeout is not supported in Webots.

        Returns:
            (float): The position in rad.
        """
        del timeout
        return self._pos_feedback

    def get_abs_pos(self, timeout: Optional[float] = None) -> float:
        """Reads the absolute position of the motor.

        Timeout is not supported in Webots.

        Returns:
            (float): The absolute position in rad.
        """
        del timeout
        return self._pos_feedback

    def get_vel(self, timeout: Optional[float] = None) -> float:
        """Reads the velocity of the motor.

        Timeout is not supported in Webots.

        Returns:
            (float): The velocity in rad/s.
        """
        del timeout
        return self._vel_feedback

    def get_torque(self, timeout: Optional[float] = None) -> float:
        """Reads the torque of the motor.

        Timeout is not supported in Webots.

        Returns:
            (float): The torque in Nm.
        """
        del timeout
        return self._torque_feedback

    def get_vel_limit(self, timeout: Optional[float] = None) -> float:
        """Reads the velocity limit of the motor.

        Timeout is not supported in Webots.

        Returns:
            (float): The velocity limit in rad/s.
        """
        del timeout
        return self._max_vel

    def get_current_limit(self, timeout: Optional[float] = None) -> float:
        """Reads the current limit of the motor.

        Timeout is not supported in Webots.

        Returns:
            (float): The current limit in amps.
        """
        del timeout
        return self._motor.getAvailableTorque() / self.torque_constant

    class MotorState(Enum):
        """Enumerates motor working states."""

        UNKNOWN = -1
        """The state of the motor is unknown."""
        DISABLED = 1
        """The motor is disabled."""
        CALIBRATE = 3
        """The motor is preparing for calibration."""
        MOTOR_CALIBRATING = 4
        """The motor is calibrating."""
        OFFSET_CALIBRATING = 7
        """The motor is calibrating the encoder."""
        ENABLED = 8
        """The motor is enabled."""
        HOMING = 11
        """The motor is aligning the encoders."""

    class MotorControlMode(Enum):
        """Enumerates motor control modes."""

        UNKNOWN = (0, 0)
        """The motor control mode is unknown."""
        POSITION_MODE = (3, 1)
        """Control the motor in position mode."""
        POSITION_FILTER_MODE = (3, 3)
        """Control the motor in position filter mode."""
        POSITION_TRACK_MODE = (3, 5)
        """Control the motor in position track mode."""
        VELOCITY_MODE = (2, 1)
        """Control the motor in velocity mode."""
        VELOCITY_RAMP_MODE = (2, 2)
        """Control the motor in velocity ramp mode."""
        TORQUE_MODE = (1, 1)
        """Control the motor in torque mode."""
        TWIN_MODE = (255, 255)
        """Digital twin mode."""
