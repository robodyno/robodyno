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

"""This module provides a class for controlling Robodyno motors.

The Motor class provided by this module is used to control Robodyno motors
through the CAN bus. It provides methods for setting motor parameters, reading motor
states, and controlling the motor to run in different modes.

Examples:

    >>> from robodyno.components import Motor
    >>> from robodyno.interfaces import CanBus
    >>> can_bus = CanBus()
    >>> motor = Motor(can_bus)
    >>> print(motor)
    Motor(id=0x10, type=ROBODYNO_PRO_P44, state=DISABLED, error=NONE)
    >>> can_bus.disconnect()
"""

from time import sleep
from math import pi, fabs
from enum import Enum
from struct import unpack
from typing import Optional

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model
from robodyno.components.config.robottime_config import ROBOTTIME_PARAMS


class Motor(CanBusDevice):
    """Controls Robodyno motors through the CAN bus.

    Attributes:
        id (int): The ID of the motor device on the CAN bus.
        type (Model): The model type of the motor.
        fw_ver (float): The firmware version of the motor.
        reduction (float): The reduction ratio of the motor.
        available_velocity (float): The maximum velocity of the motor.
        available_torque (float): The maximum torque of the motor.
        available_current (float): The maximum phase current of the motor.
        torque_constant (float):
            The torque constant of the motor in Nm/A. This is the torque provided by
            the motor when the current is 1A.
        with_brake (bool): Whether the motor has a brake.
        state (Motor.MotorState): The working state of the motor.
        error (Dict): The error information of the motor.
        mode (Motor.MotorControlMode): The control mode of the motor.
        sync_time (int): The synchronization time of the motor in milliseconds.
    """

    _CMD_HEARTBEAT = 0x02
    _CMD_ESTOP = 0x03
    _CMD_REBOOT = 0x04
    _CMD_CLEAR_ERRORS = 0x05
    _CMD_SAVE = 0x06
    _CMD_CONFIG_CAN = 0x07
    _CMD_SET_STATE = 0x08
    _CMD_GET_HARDWARE_STATUS = 0x09
    _CMD_GET_MOTOR_FEEDBACK = 0x0A
    _CMD_GET_MODE = 0x0B
    _CMD_SET_MODE = 0x0C
    _CMD_GET_PID = 0x0D
    _CMD_SET_PID = 0x0E
    _CMD_GET_LIMITS = 0x0F
    _CMD_SET_LIMITS = 0x10
    _CMD_SET_POS = 0x11
    _CMD_SET_VEL = 0x12
    _CMD_SET_TORQUE = 0x13
    _CMD_RESET = 0x14
    _CMD_UNLOCK = 0x15
    _CMD_GET_ABS_POS = 0x16
    _CMD_SET_ABS_POS = 0x17
    _CMD_INIT_POS = 0x18
    _CMD_INIT_ABS_POS = 0x19
    _CMD_GET_STATE = 0x1A

    def __init__(self, can: CanBus, id_: int = 0x10, type_: Optional[str] = None):
        """Initializes the Robodyno motor based on the device id and type.

        Args:
            can (CanBus): The CAN bus interface.
            id_ (int): The device id. The default factory id of the motor is 0x10.
            type_ (string): Forcing the motor type. The default is None.

        Raises:
            ValueError: If the motor type is invalid.
        """
        super().__init__(can, id_)
        version = self.get_version(timeout=0.015)
        if type_:
            self.type = getattr(Model, type_.upper(), None)
        if self.type not in ROBOTTIME_PARAMS['motor']:
            valid_models = ', '.join(
                map(lambda model: model.name, ROBOTTIME_PARAMS['motor'].keys())
            )
            raise ValueError(
                f'Invalid motor type. Valid motor types are: {valid_models}.'
            )

        self.__dict__.update(ROBOTTIME_PARAMS['motor'].get(self.type, None))

        if self.fw_ver >= 2.0:
            self.reduction = version['data']
            if Model.is_pro(self.type):
                self.__dict__.update(ROBOTTIME_PARAMS['pro_motor_common'])
            elif Model.is_plus(self.type):
                self.__dict__.update(ROBOTTIME_PARAMS['plus_motor_common'])
            self.available_velocity = fabs(
                self.hw_max_vel_limit / self.reduction * 2.0 * pi
            )
            self.available_torque = fabs(
                self.available_current * self.hw_torque_constant * self.reduction
            )
            self.torque_constant = fabs(self.hw_torque_constant * self.reduction)
            self.default_vel_limit = fabs(
                self.hw_default_vel_limit / self.reduction * 2.0 * pi
            )

        self.state = self.MotorState.UNKNOWN
        self.error = {
            'error': self.MotorError.NONE,
            'motor_err': self.MotorMotorError.NONE,
            'encoder_err': self.MotorEncoderError.NONE,
            'controller_err': self.MotorControllerError.NONE,
        }
        self.mode = self.MotorControlMode.UNKNOWN
        self.sync_time = 0
        if self.fw_ver >= 1:
            self.get_state(1.5)

        if self.fw_ver >= 2.0:
            self._rot_factor = 1
            self._torque_factor = 1
        elif self.fw_ver >= 1.0:
            self._rot_factor = -self.reduction / 2.0 / pi
            self._torque_factor = -self.reduction
        elif self.fw_ver <= 0.3:
            self._rot_factor = self.reduction / 2.0 / pi
            self._torque_factor = self.reduction
        self._can.subscribe(
            self._heartbeat_callback,
            device_id=self.id,
            cmd_id=self._CMD_HEARTBEAT,
        )

    def __repr__(self) -> str:
        return (
            f'{self.__class__.__name__}'
            f'(id=0x{self.id:02X}, type={self.type.name}, '
            f'state={self.state.name}, error={self.error["error"].name})'
        )

    def __del__(self):
        try:
            self._can.unsubscribe(
                self._heartbeat_callback,
                device_id=self.id,
                cmd_id=self._CMD_HEARTBEAT,
            )
        except KeyError:
            pass

    def _heartbeat_callback(self, data, timestamp, device_id, command_id) -> None:
        """Updates the motor attributes when receiving the heartbeat command."""
        del device_id, command_id
        state, err, merr, eerr, cerr, cm, im, _ = unpack('<BBBBBBBB', data)
        self.state = self.MotorState(state)
        if self.fw_ver < 1:
            self.error.update(
                {
                    'error': self.MotorError(err + 1 if err > 0 else 0),
                    'motor_err': self.MotorMotorError(merr + 1 if merr > 0 else 0),
                    'encoder_err': self.MotorEncoderError(eerr + 1 if eerr > 0 else 0),
                    'controller_err': self.MotorControllerError(
                        cerr + 1 if cerr > 0 else 0
                    ),
                }
            )
        else:
            self.error.update(
                {
                    'error': self.MotorError(err),
                    'motor_err': self.MotorMotorError(merr),
                    'encoder_err': self.MotorEncoderError(eerr),
                    'controller_err': self.MotorControllerError(cerr),
                }
            )
        self.mode = self.MotorControlMode((cm, im))
        self.sync_time = timestamp

    def _set_state(self, state) -> None:
        """Sets the motor working state.

        Args:
            state (Motor.MotorState): The motor working state to set.
        """
        self._can.send(self.id, self._CMD_SET_STATE, 'I', state.value)

    def enable(self) -> None:
        """Enables the motor.

        After enabled, the motor can be controlled.
        """
        self._set_state(self.MotorState.ENABLED)
        sleep(0.02)

    def disable(self) -> None:
        """Disables the motor.

        This is the default state after power on. After disabled, the motor
        will stop working and loose torque.
        """
        self._set_state(self.MotorState.DISABLED)

    def unlock(self) -> None:
        """Unlocks the motor.

        This command only works on motors with brake. After unlocked, the motor
        can be rotated freely.

        Raises:
            NotImplementedError: If the motor does not support brake.
        """
        if not self.with_brake:
            raise NotImplementedError('unlock is not supported on motors without brake')
        self._can.send(self.id, self._CMD_UNLOCK, '')

    def init_pos(self, initial_pos: float = 0) -> None:
        """Sets the current position of the motor as the initial position.

        Args:
            initial_pos (float): The initial position in rad.

        Raises:
            NotImplementedError: If the firmware version is < 1.0.
        """
        if self.fw_ver < 1.0:
            raise NotImplementedError(
                'Initial position is not supported on firmware < 1.0'
            )
        self._can.send(self.id, self._CMD_INIT_POS, 'f', initial_pos * self._rot_factor)

    def init_abs_pos(self, initial_pos: float = 0) -> None:
        """Sets the current absolute position of the motor as the initial position.

        Args:
            initial_pos (float): The initial absolute position in rad.

        Raises:
            NotImplementedError: If the firmware version is < 1.0.
        """
        if self.fw_ver < 1.0:
            raise NotImplementedError(
                'Absolute position is not supported on firmware < 1.0'
            )
        self._can.send(
            self.id, self._CMD_INIT_ABS_POS, 'f', initial_pos * self._rot_factor
        )

    def calibrate(self) -> None:
        """Calibrates the motor.

        Save the configurations after calibration manually by calling
        `save_configuration()` if needed.
        """
        self._set_state(self.MotorState.CALIBRATE)

    def config_can_bus(
        self, new_id: int = None, heartbeat: int = 1000, bitrate: int = 1000000
    ) -> None:
        """Configures the CAN bus settings of the motor.

        Args:
            new_id (int): The new device id.
            heartbeat (int): The heartbeat period in milliseconds.
            bitrate (int): The bitrate of the CAN bus. Choose from 250000,
                500000, 1000000.

        Raises:
            ValueError: If the new CAN id is not in the range of 0x01-0x3f.
            ValueError: If the bitrate is not one of 250000, 500000, 1000000.
        """
        if new_id is None:
            new_id = self.id
        if not 0x01 <= new_id <= 0x3F:
            raise ValueError('New CAN id must be in the range of 0x01-0x3f.')

        bitrate_id = {
            250000: 0,
            500000: 1,
            1000000: 2,
        }
        if bitrate not in bitrate_id:
            raise ValueError('Bitrate must be one of 250000, 500000, 1000000.')
        self._can.send(
            self.id,
            self._CMD_CONFIG_CAN,
            'HHI',
            new_id,
            bitrate_id[bitrate],
            int(heartbeat),
        )

    def save(self) -> None:
        """Saves the motor configurations.

        If the motor is in a running state, the motor will stop and save the
        configurations.
        """
        self._can.send(self.id, self._CMD_SAVE, '')
        sleep(0.1)
        self.clear_errors()

    def save_configuration(self) -> None:
        """Saves the motor configurations.

        This method will be deprecated. Use `save` instead.
        """
        self.save()

    def clear_errors(self) -> None:
        """Clears the motor errors.

        Most errors are cleared automatically if the cause is resolved.
        """
        self._can.send(self.id, self._CMD_CLEAR_ERRORS, '')

    def estop(self) -> None:
        """Requests an emergency stop.

        The motor will loose torque immediately and report an ESTOP error.
        """
        self._can.send(self.id, self._CMD_ESTOP, '')

    def reboot(self) -> None:
        """Reboots the motor."""
        self._can.send(self.id, self._CMD_REBOOT, '')

    def reset(self) -> None:
        """Resets the motor.

        This command will reset the motor to the factory settings. All
        configurations will be lost and the id will be reset to 0x10.
        The motor must be calibrated again after reset.
        """
        self._can.send(self.id, self._CMD_RESET, '')

    def position_mode(self) -> None:
        """Sets the motor to position mode."""
        self._can.send(
            self.id,
            self._CMD_SET_MODE,
            'BB',
            *self.MotorControlMode.POSITION_MODE.value,
        )

    def position_filter_mode(self, bandwidth: float) -> None:
        """Sets the motor to position filter mode.

        Args:
            bandwidth (float): The bandwidth of the filter in Hz. The value is suggested
                to be the control loop frequency in Hz.
        """
        cmode, imode = self.MotorControlMode.POSITION_FILTER_MODE.value
        self._can.send(self.id, self._CMD_SET_MODE, 'BBf', cmode, imode, bandwidth)

    def position_track_mode(self, vel: float, acc: float, dec: float) -> None:
        """Sets the motor to position track mode.

        Args:
            vel (float): The velocity limit in rad/s.
            acc (float): The acceleration in rad/s^2. The value must be positive.
            dec (float): The deceleration in rad/s^2. The value must be positive.
        """
        cmode, imode = self.MotorControlMode.POSITION_TRACK_MODE.value
        self._can.send(
            self.id,
            self._CMD_SET_MODE,
            'BBeee',
            cmode,
            imode,
            fabs(vel * self._rot_factor),
            fabs(acc * self._rot_factor),
            fabs(dec * self._rot_factor),
        )

    def velocity_mode(self) -> None:
        """Sets the motor to velocity mode."""
        self._can.send(
            self.id,
            self._CMD_SET_MODE,
            'BB',
            *self.MotorControlMode.VELOCITY_MODE.value,
        )

    def velocity_ramp_mode(self, ramp: float) -> None:
        """Sets the motor to velocity ramp mode.

        Args:
            ramp (float): The velocity ramp in rad/s^2. The value must be positive.
        """
        cmode, imode = self.MotorControlMode.VELOCITY_RAMP_MODE.value
        self._can.send(
            self.id,
            self._CMD_SET_MODE,
            'BBf',
            cmode,
            imode,
            fabs(ramp * self._rot_factor),
        )

    def torque_mode(self) -> None:
        """Sets the motor to torque mode."""
        self._can.send(
            self.id, self._CMD_SET_MODE, 'BB', *self.MotorControlMode.TORQUE_MODE.value
        )

    def set_pos(self, pos: float, vel_ff: float = 0, torque_ff: float = 0) -> None:
        """Sets the target position of the motor.

        Args:
            pos (float): The target position in rad.
            vel_ff (float): The velocity feedforward in rad/s.
            torque_ff (float): The torque feedforward in Nm.
        """
        self._can.send(
            self.id,
            self._CMD_SET_POS,
            'fee',
            pos * self._rot_factor,
            vel_ff * self._rot_factor,
            torque_ff / self._torque_factor,
        )

    def set_abs_pos(self, pos: float, vel_ff: float = 0, torque_ff: float = 0) -> None:
        """Sets the target absolute position of the motor.

        Args:
            pos (float): The target absolute position in rad.
            vel_ff (float): The velocity feedforward in rad/s.
            torque_ff (float): The torque feedforward in Nm.

        Raises:
            NotImplementedError: If the firmware version is < 1.0.
        """
        if self.fw_ver < 1.0:
            raise NotImplementedError(
                'Absolute position is not supported on firmware < 1.0'
            )
        self._can.send(
            self.id,
            self._CMD_SET_ABS_POS,
            'fee',
            pos * self._rot_factor,
            vel_ff * self._rot_factor,
            torque_ff / self._torque_factor,
        )

    def set_vel(self, vel: float, torque_ff: float = 0) -> None:
        """Sets the target velocity of the motor.

        Args:
            vel (float): The target velocity in rad/s.
            torque_ff (float): The torque feedforward in Nm.
        """
        self._can.send(
            self.id,
            self._CMD_SET_VEL,
            'ff',
            vel * self._rot_factor,
            torque_ff / self._torque_factor,
        )

    def set_torque(self, torque: float) -> None:
        """Sets the target torque of the motor.

        Args:
            torque (float): The target torque in Nm.
        """
        self._can.send(self.id, self._CMD_SET_TORQUE, 'f', torque / self._torque_factor)

    def set_pid(self, pos_kp: float, vel_kp: float, vel_ki: float) -> None:
        """Sets the PID parameters of the motor.

        Args:
            pos_kp (float): The Kp of the position control loop.
            vel_kp (float): The Kp of the velocity control loop.
            vel_ki (float): The Ki of the velocity control loop.
        """
        self._can.send(self.id, self._CMD_SET_PID, 'fee', pos_kp, vel_kp, vel_ki)

    def set_vel_limit(self, vel_lim: float) -> None:
        """Sets the velocity limit of the motor.

        Args:
            vel_lim (float): The velocity limit in rad/s. The value must be positive.

        Raises:
            ValueError: If the velocity limit is out of range.
        """
        if not 0 < vel_lim <= self.available_velocity:
            raise ValueError(f'Velocity limit {vel_lim} is out of range.')
        self._can.send(
            self.id, self._CMD_SET_LIMITS, 'ff', fabs(vel_lim * self._rot_factor), 0
        )

    def set_current_limit(self, current_lim: float) -> None:
        """Sets the current limit of the motor.

        Args:
            current_lim (float): The current limit in amps. The value must be positive.

        Raises:
            ValueError: If the current limit is out of range.
        """
        if not 0 < current_lim <= self.available_current:
            raise ValueError(f'Current limit {current_lim} is out of range.')
        self._can.send(self.id, self._CMD_SET_LIMITS, 'ff', 0, current_lim)

    def get_state(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the motor state and error.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): The motor state, error, and control mode. Returns None if
                the read times out.

        Raises:
            NotImplementedError: If the firmware version is < 1.0.
        """
        if self.fw_ver < 1.0:
            raise NotImplementedError('get_state is not supported on firmware < 1.0')
        try:
            state, err, merr, eerr, cerr, cm, im, _ = self._can.get(
                self.id, self._CMD_GET_STATE, 'B' * 8, timeout
            )
        except TimeoutError:
            return None
        self.state = self.MotorState(state)
        self.error.update(
            {
                'error': self.MotorError(err),
                'motor_err': self.MotorMotorError(merr),
                'encoder_err': self.MotorEncoderError(eerr),
                'controller_err': self.MotorControllerError(cerr),
            }
        )
        self.mode = self.MotorControlMode((cm, im))
        return (self.state, self.error, self.mode)

    def get_voltage(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the DC bus voltage of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The DC bus voltage in volts. Returns None if the read
                times out.
        """
        try:
            vbus, _ = self._can.get(
                self.id, self._CMD_GET_HARDWARE_STATUS, 'ff', timeout
            )
        except TimeoutError:
            return None
        return vbus

    def get_temperature(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the temperature of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The temperature in degrees Celsius. Returns None if the
                read times out.
        """
        try:
            _, temperature = self._can.get(
                self.id, self._CMD_GET_HARDWARE_STATUS, 'ff', timeout
            )
        except TimeoutError:
            return None
        return temperature

    def get_mode(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the control mode and the mode parameters of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): The control mode and the mode parameters. Returns None if
                the read times out.
        """
        try:
            payload = self._can.get(self.id, self._CMD_GET_MODE, '8s', timeout)[0]
        except TimeoutError:
            return None
        control_mode, input_mode = unpack('<BB', payload[:2])
        mode = self.MotorControlMode((control_mode, input_mode))
        self.mode = mode
        if mode == self.MotorControlMode.POSITION_FILTER_MODE:
            (bandwidth,) = unpack('<f', payload[2:6])
            return (mode, {'bandwidth': bandwidth})
        elif mode == self.MotorControlMode.POSITION_TRACK_MODE:
            vel, acc, dec = unpack('<eee', payload[2:8])
            return (
                mode,
                {
                    'vel': fabs(vel / self._rot_factor),
                    'acc': fabs(acc / self._rot_factor),
                    'dec': fabs(dec / self._rot_factor),
                },
            )
        elif mode == self.MotorControlMode.VELOCITY_RAMP_MODE:
            (ramp,) = unpack('<f', payload[2:6])
            return (mode, {'ramp': fabs(ramp / self._rot_factor)})
        else:
            return (mode,)

    def get_feedback(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the feedbacks of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): The position, velocity, and torque. Returns None if the
                read times out.
        """
        try:
            pos, vel, torque = self._can.get(
                self.id, self._CMD_GET_MOTOR_FEEDBACK, 'fee', timeout
            )
        except TimeoutError:
            return None
        return (
            pos / self._rot_factor,
            vel / self._rot_factor,
            torque * self._torque_factor,
        )

    def get_pos(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the position of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The position in rad. Returns None if the read times
                out.
        """
        feedback = self.get_feedback(timeout)
        if feedback is None:
            return None
        return feedback[0]

    def get_abs_pos(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the absolute position of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The absolute position in rad. Returns None if the
                read times out.

        Raises:
            NotImplementedError: If the firmware version is < 1.0.
        """
        if self.fw_ver < 1.0:
            raise NotImplementedError(
                'Absolute position is not supported on firmware < 1.0'
            )
        try:
            pos = self._can.get(self.id, self._CMD_GET_ABS_POS, 'f', timeout)[0]
        except TimeoutError:
            return None
        return pos / self._rot_factor

    def get_vel(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the velocity of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The velocity in rad/s. Returns None if the read times
                out.
        """
        feedback = self.get_feedback(timeout)
        if feedback is None:
            return None
        return feedback[1]

    def get_torque(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the torque of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The torque in Nm. Returns None if the read times out.
        """
        feedback = self.get_feedback(timeout)
        if feedback is None:
            return None
        return feedback[2]

    def get_pid(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the PID parameters of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): The position KP, velocity KP, and velocity KI. Returns
                None if the read times out.
        """
        try:
            pos_kp, vel_kp, vel_ki = self._can.get(
                self.id, self._CMD_GET_PID, 'fee', timeout
            )
        except TimeoutError:
            return None
        return (pos_kp, vel_kp, vel_ki)

    def get_vel_limit(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the velocity limit of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The velocity limit in rad/s. Returns None if the read
                times out.
        """
        try:
            vel_limit, _ = self._can.get(self.id, self._CMD_GET_LIMITS, 'ff', timeout)
        except TimeoutError:
            return None
        return vel_limit / fabs(self._rot_factor)

    def get_current_limit(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the current limit of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The current limit in A. Returns None if the read
                times out.
        """
        try:
            _, current_limit = self._can.get(
                self.id, self._CMD_GET_LIMITS, 'ff', timeout
            )
        except TimeoutError:
            return None
        return current_limit

    class MotorState(Enum):
        """Enumerates motor working states."""

        UNKNOWN = -1
        """The state of the motor is unknown."""
        BOOTING = 0
        """The motor is booting."""
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

    class MotorError(Enum):
        """Enumerates motor errors."""

        NONE = 0
        """No error."""
        INVALID_STATE = 1
        """The motor is in an invalid state."""
        UNDER_VOLTAGE = 2
        """The motor is under voltage."""
        OVER_VOLTAGE = 3
        """The motor is over voltage."""
        CURRENT_MEASUREMENT_TIMEOUT = 4
        """The motor current measurement timed out."""
        BRAKE_RESISTOR_DISARMED = 5
        """The brake resistor is disarmed."""
        MOTOR_DISARMED = 6
        """The motor is disarmed."""
        MOTOR_FAILED = 7
        """The driver of the motor failed."""
        SENSORLESS_ESTIMATOR_FAILED = 8
        """The sensorless estimator of the motor failed."""
        ENCODER_FAILED = 9
        """The encoder of the motor failed."""
        CONTROLLER_FAILED = 10
        """The controller of the motor failed."""
        POS_CTRL_DURING_SENSORLESS = 11
        """The position controller of the motor is running during sensorless mode."""
        WATCHDOG_TIMER_EXPIRED = 12
        """The watchdog timer of the motor expired."""
        MIN_ENDSTOP_PRESSED = 13
        """The minimum endstop of the motor is pressed."""
        MAX_ENDSTOP_PRESSED = 14
        """The maximum endstop of the motor is pressed."""
        ESTOP_REQUESTED = 15
        """The motor is requested to emergency stop."""
        HOMING_WITHOUT_ENDSTOP = 16
        """The motor is homing without endstop."""
        OVER_TEMP = 17
        """The motor is over temperature."""

    class MotorMotorError(Enum):
        """Enumerates motor driver errors."""

        NONE = 0
        """No error."""
        PHASE_RESISTANCE_OUT_OF_RANGE = 1
        """The phase resistance of the motor is out of range."""
        PHASE_INDUCTANCE_OUT_OF_RANGE = 2
        """The phase inductance of the motor is out of range."""
        ADC_FAILED = 3
        """The ADC of the motor failed."""
        DRV_FAULT = 4
        """The driver of the motor failed."""
        CONTROL_DEADLINE_MISSED = 5
        """The control deadline of the motor is missed."""
        NOT_IMPLEMENTED_MOTOR_TYPE = 6
        """The motor type is not implemented."""
        BRAKE_CURRENT_OUT_OF_RANGE = 7
        """The brake current of the motor is out of range."""
        MODULATION_MAGNITUDE = 8
        """The low level modulation failed."""
        BRAKE_DEADTIME_VIOLATION = 9
        """The brake deadtime of the motor is violated."""
        UNEXPECTED_TIMER_CALLBACK = 10
        """The timer callback of the motor is unexpected."""
        CURRENT_SENSE_SATURATION = 11
        """The current sense of the motor is saturated."""
        CURRENT_LIMIT_VIOLATION = 13
        """The current limit of the motor is violated."""
        BRAKE_DUTY_CYCLE_NAN = 14
        """The brake duty cycle of the motor is NaN."""
        DC_BUS_OVER_REGEN_CURRENT = 15
        """The DC bus is over allowable regenerative current."""
        DC_BUS_OVER_CURRENT = 16
        """The DC bus is over current."""

    class MotorEncoderError(Enum):
        """Enumerates motor encoder errors."""

        NONE = 0
        """No error."""
        UNSTABLE_GAIN = 1
        """The PLL gain of the encoder is unstable."""
        CPR_POLEPAIRS_MISMATCH = 2
        """The CPR and pole pairs of the encoder mismatch."""
        NO_RESPONSE = 3
        """The encoder does not respond."""
        UNSUPPORTED_ENCODER_MODE = 4
        """The encoder mode is not supported."""
        ILLEGAL_HALL_STATE = 5
        """The hall state of the encoder is illegal."""
        INDEX_NOT_FOUND_YET = 6
        """The index of the encoder is not found yet."""
        ABS_SPI_TIMEOUT = 7
        """The SPI of the encoder timed out."""
        ABS_SPI_COM_FAIL = 8
        """The SPI of the encoder failed."""
        ABS_SPI_NOT_READY = 9
        """The SPI of the encoder is not ready."""

    class MotorControllerError(Enum):
        """Enumerates motor controller errors."""

        NONE = 0
        """No error."""
        OVERSPEED = 1
        """The motor is overspeed."""
        INVALID_INPUT_MODE = 2
        """The input mode of the motor is invalid."""
        UNSTABLE_GAIN = 3
        """The gain of the sensorless estimator is unstable."""
        INVALID_MIRROR_AXIS = 4
        """The mirror axis of the motor is invalid."""
        INVALID_LOAD_ENCODER = 5
        """The encoder of the motor is invalid."""
        INVALID_ESTIMATE = 6
        """The estimate feedbacks of the motor is invalid."""

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
