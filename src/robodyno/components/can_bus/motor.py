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

import time
from math import pi, fabs
from enum import Enum
import struct
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
        error (Dict[str, Any]): The error information of the motor.
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
    _CMD_GET_STATE = 0x20

    def __init__(self, can: CanBus, id_: int = 0x10, type_: Optional[str] = None):
        """Initializes the Robodyno motor based on the device id and type.

        Args:
            can: The CAN bus interface.
            id_: The device id. The default factory id of the motor is 0x10.
            type_: The motor model type.

        Raises:
            ValueError: If the motor type is invalid.
        """
        super().__init__(can, id_)
        self.get_version(timeout=0.15)
        if type_:
            self.type = Model(type_)
        if self.type not in ROBOTTIME_PARAMS['motor']:
            valid_models = list(
                map(lambda model: model.name, ROBOTTIME_PARAMS['motor'].keys())
            )
            raise ValueError(
                f'Invalid motor type. Valid motor types are: {valid_models}.'
            )

        self.__dict__.update(ROBOTTIME_PARAMS['motor'].get(self.type, None))

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

        self._rot_factor = -self.reduction / 2.0 / pi
        if self.fw_ver <= 0.3:
            self._rot_factor = -self._rot_factor
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
        self._can.unsubscribe(
            self._heartbeat_callback,
            device_id=self.id,
            cmd_id=self._CMD_HEARTBEAT,
        )

    def _heartbeat_callback(self, data, timestamp, device_id, command_id):
        """Updates the motor attributes when receiving the heartbeat command."""
        del device_id, command_id
        state, err, merr, eerr, cerr, cm, im, _ = struct.unpack('<BBBBBBBB', data)
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

    def _set_state(self, state):
        """Sets the motor working state.

        Args:
            state (Motor.MotorState): The motor working state to set.
        """
        self._can.send(self.id, self._CMD_SET_STATE, 'I', state.value)

    def enable(self):
        """Enables the motor.

        After enabled, the motor can be controlled.
        """
        self._set_state(self.MotorState.ENABLED)
        time.sleep(0.02)

    def disable(self):
        """Disables the motor.

        This is the default state after power on. After disabled, the motor
        will stop working and loose torque.
        """
        self._set_state(self.MotorState.DISABLED)

    def unlock(self):
        """Unlocks the motor.

        This command only works on motors with brake. After unlocked, the motor
        can be rotated freely.

        Raises:
            NotImplementedError: If the motor does not support brake.
        """
        if not self.with_brake:
            raise NotImplementedError('unlock is not supported on motors without brake')
        self._can.send(self.id, self._CMD_UNLOCK, '')

    def init_pos(self, initial_pos: float = 0):
        """Sets the current position of the motor as the initial position.

        Args:
            initial_pos (float): The initial position in rad.
        """
        self._can.send(self.id, self._CMD_INIT_POS, 'f', initial_pos * self._rot_factor)

    def init_abs_pos(self, initial_pos: float = 0):
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

    def calibrate(self):
        """Calibrates the motor.

        Save the configurations after calibration manually by calling
        `save_configuration()` if needed.
        """
        self._set_state(self.MotorState.CALIBRATE)

    def config_can_bus(
        self, new_id: int, heartbeat: int = 1000, bitrate: int = 1000000
    ):
        """Configures the CAN bus settings of the motor.

        Args:
            new_id (int): The new device id.
            heartbeat (int): The heartbeat period in milliseconds.
            bitrate (int): The bitrate of the CAN bus. Choose from 250000,
                500000, 1000000.

        Raises:
            ValueError: If the new CAN id is not in the range of 0x01-0x3f.
        """
        if not 0x01 <= new_id <= 0x3f:
            raise ValueError('New CAN id must be in the range of 0x01-0x3f.')

        bitrate_id = {
            250000: 0,
            500000: 1,
            1000000: 2,
        }.get(bitrate, 2)
        self._can.send(
            self.id, self._CMD_CONFIG_CAN, 'HHI', new_id, bitrate_id, int(heartbeat)
        )

    def save(self):
        """Saves the motor configurations.

        If the motor is in a running state, the motor will stop and save the
        configurations.
        """
        self._can.send(self.id, self._CMD_SAVE, '')
        self.clear_errors()

    def save_configuration(self):
        """Saves the motor configurations.

        This method will be deprecated. Use `save` instead.
        """
        self.save()

    def clear_errors(self):
        """Clears the motor errors.

        Most errors are cleared automatically if the cause is resolved except
        for the ESTOP error.
        """
        self._can.send(self.id, self._CMD_CLEAR_ERRORS, '')

    def estop(self):
        """Requests an emergency stop.

        The motor will loose torque immediately and report an ESTOP error.
        """
        self._can.send(self.id, self._CMD_ESTOP, '')

    def reboot(self):
        """Reboots the motor."""
        self._can.send(self.id, self._CMD_REBOOT, '')

    def reset(self):
        """Resets the motor.

        This command will reset the motor to the factory settings. All
        configurations will be lost and the id will be reset to 0x10.
        The motor must be calibrated again after reset.
        """
        self._can.send(self.id, self._CMD_RESET, '')

    def position_mode(self):
        """Sets the motor to position mode."""
        self._can.send(
            self.id,
            self._CMD_SET_MODE,
            'BB',
            *self.MotorControlMode.POSITION_MODE.value,
        )

    def position_filter_mode(self, bandwidth: float):
        """Sets the motor to position filter mode.

        Args:
            bandwidth (float): The bandwidth of the filter in Hz. The value is suggested
                to be the control loop frequency in Hz.
        """
        cmode, imode = self.MotorControlMode.POSITION_FILTER_MODE.value
        self._can.send(self.id, self._CMD_SET_MODE, 'BBf', cmode, imode, bandwidth)

    def position_track_mode(self, vel: float, acc: float, dec: float):
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

    def velocity_mode(self):
        """Sets the motor to velocity mode."""
        self._can.send(
            self.id,
            self._CMD_SET_MODE,
            'BB',
            *self.MotorControlMode.VELOCITY_MODE.value,
        )

    def velocity_ramp_mode(self, ramp: float):
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

    def torque_mode(self):
        """Sets the motor to torque mode."""
        self._can.send(
            self.id, self._CMD_SET_MODE, 'BB', *self.MotorControlMode.TORQUE_MODE.value
        )

    def set_pos(self, pos: float, vel_ff: float = 0, torque_ff: float = 0):
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
            torque_ff / self.reduction,
        )

    def set_abs_pos(self, pos: float, vel_ff: float = 0, torque_ff: float = 0):
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
            torque_ff / self.reduction,
        )

    def set_vel(self, vel: float, torque_ff: float = 0):
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
            torque_ff / self.reduction,
        )

    def set_torque(self, torque: float):
        """Sets the target torque of the motor.

        Args:
            torque (float): The target torque in Nm.
        """
        self._can.send(self.id, self._CMD_SET_TORQUE, 'f', torque / self.reduction)

    def set_pid(self, pos_kp: float, vel_kp: float, vel_ki: float):
        """Sets the PID parameters of the motor.

        Args:
            pos_kp (float): The Kp of the position control loop.
            vel_kp (float): The Kp of the velocity control loop.
            vel_ki (float): The Ki of the velocity control loop.
        """
        self._can.send(self.id, self._CMD_SET_PID, 'fee', pos_kp, vel_kp, vel_ki)

    def set_vel_limit(self, vel_lim: float):
        """Sets the velocity limit of the motor.

        Args:
            vel_lim (float): The velocity limit in rad/s. The value must be positive.

        Raises:
            ValueError: If the velocity limit is out of range.
        """
        if not 0 <= vel_lim <= self.available_velocity:
            raise ValueError(f'Velocity limit {vel_lim} is out of range.')
        self._can.send(
            self.id, self._CMD_SET_LIMITS, 'ff', fabs(vel_lim * self._rot_factor), 0
        )

    def set_current_limit(self, current_lim: float):
        """Sets the current limit of the motor.

        Args:
            current_lim (float): The current limit in amps. The value must be positive.

        Raises:
            ValueError: If the current limit is out of range.
        """
        if not 0 <= current_lim <= self.available_current:
            raise ValueError(f'Current limit {current_lim} is out of range.')
        self._can.send(self.id, self._CMD_SET_LIMITS, 'ff', 0, current_lim)

    def get_state(self, timeout: Optional[float] = None):
        """Reads the motor state and error.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (tuple): The motor state, error, and control mode. Returns None if the
                     request timed out.

        Raises:
            NotImplementedError: If the firmware version is < 1.0.
        """
        if self.fw_ver < 1.0:
            raise NotImplementedError('get_state is not supported on firmware < 1.0')
        state, err, merr, eerr, cerr, cm, im, _ = self._can.get(
            self.id, self._CMD_GET_STATE, 'B' * 8, timeout
        )
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

    def get_voltage(self, timeout: Optional[float] = None):
        """Reads the DC bus voltage of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float): The DC bus voltage in volts. Returns None if the request
                timed out.
        """
        vbus = self._can.get(self.id, self._CMD_GET_HARDWARE_STATUS, 'ff', timeout)[0]
        return vbus

    def get_temperature(self, timeout: Optional[float] = None):
        """Reads the temperature of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float): The temperature in degrees Celsius. Returns None if the
                request timed out.
        """
        temperature = self._can.get(
            self.id, self._CMD_GET_HARDWARE_STATUS, 'ff', timeout
        )[1]
        return temperature

    def get_mode(self, timeout: Optional[float] = None):
        """Reads the control mode and the mode parameters of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (MotorControlMode, dict): The control mode and the mode parameters.
        """
        payload = self._can.get(self.id, self._CMD_GET_MODE, '8s', timeout)[0]
        control_mode, input_mode = struct.unpack('<BB', payload[:2])
        mode = self.MotorControlMode((control_mode, input_mode))
        self.mode = mode
        if mode == self.MotorControlMode.POSITION_FILTER_MODE:
            (bandwidth,) = struct.unpack('<f', payload[2:6])
            return (mode, {'bandwidth': bandwidth})
        elif mode == self.MotorControlMode.POSITION_TRACK_MODE:
            vel, acc, dec = struct.unpack('<eee', payload[2:8])
            return (
                mode,
                {
                    'vel': fabs(vel / self._rot_factor),
                    'acc': fabs(acc / self._rot_factor),
                    'dec': fabs(dec / self._rot_factor),
                },
            )
        elif mode == self.MotorControlMode.VELOCITY_RAMP_MODE:
            (ramp,) = struct.unpack('<f', payload[2:6])
            return (mode, {'ramp': fabs(ramp / self._rot_factor)})
        else:
            return (mode,)

    def get_feedback(self, timeout: Optional[float] = None):
        """Reads the feedbacks of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float, float, float): The position(rad), velocity(rad/s) and torque(Nm)
                of the motor. Returns None if the request timed out.
        """
        pos, vel, torque = self._can.get(
            self.id, self._CMD_GET_MOTOR_FEEDBACK, 'fee', timeout
        )
        return pos / self._rot_factor, vel / self._rot_factor, torque * self.reduction

    def get_pos(self, timeout: Optional[float] = None):
        """Reads the position of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float): The position in rad. Returns None if the request timed out.
        """
        return self.get_feedback(timeout)[0]

    def get_abs_pos(self, timeout: Optional[float] = None):
        """Reads the absolute position of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float): The absolute position in rad. Returns None if the request timed
                out.

        Raises:
            NotImplementedError: If the firmware version is < 1.0.
        """
        if self.fw_ver < 1.0:
            raise NotImplementedError(
                'Absolute position is not supported on firmware < 1.0'
            )
        pos = self._can.get(self.id, self._CMD_GET_ABS_POS, 'f', timeout)[0]
        return pos / self._rot_factor

    def get_vel(self, timeout: Optional[float] = None):
        """Reads the velocity of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float): The velocity in rad/s. Returns None if the request timed out.
        """
        return self.get_feedback(timeout)[1]

    def get_torque(self, timeout: Optional[float] = None):
        """Reads the torque of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float): The torque in Nm. Returns None if the request timed out.
        """
        return self.get_feedback(timeout)[2]

    def get_pid(self, timeout: Optional[float] = None):
        """Reads the PID parameters of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float, float, float): The Kp of the position control loop, Kp of the
                velocity control loop, and Ki of the velocity control loop. Returns
                None if the request timed out.
        """
        pos_kp, vel_kp, vel_ki = self._can.get(
            self.id, self._CMD_GET_PID, 'fee', timeout
        )
        return (pos_kp, vel_kp, vel_ki)

    def get_vel_limit(self, timeout: Optional[float] = None):
        """Reads the velocity limit and the current limit of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float, float): The velocity limit in rad/s and the current limit in amps.
                Returns None if the request timed out.
        """
        vel_limit = self._can.get(self.id, self._CMD_GET_LIMITS, 'ff', timeout)[0]
        return vel_limit / fabs(self._rot_factor)

    def get_current_limit(self, timeout: Optional[float] = None):
        """Reads the current limit of the motor.

        Args:
            timeout (float, optional): The timeout in seconds.

        Returns:
            (float): The current limit in amps. Returns None if the request timed out.
        """
        current_limit = self._can.get(self.id, self._CMD_GET_LIMITS, 'ff', timeout)[1]
        return current_limit

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
