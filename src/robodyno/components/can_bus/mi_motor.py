import can
from time import sleep
from math import pi, fabs
from enum import Enum
from struct import unpack, pack, calcsize
from typing import Optional
import numpy as np

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model
from robodyno.interfaces import InterfaceType, get_interface_type

ROBOTTIME_PARAMS = {
    'mi_cybergear': {
        'reduction': 7.75,
        'available_velocity': 30,
        'available_torque': 12,
        'available_current': 23,
        'torque_constant': 0.87,
        'with_brake': False,
        'absolution_encoder': False,
        'default_vel_limit': 2,
        'default_current_limit': 23.0,
        'default_pos_kp': 39.0,
        'default_vel_kp': 1,
        'default_vel_ki': 0.002,
        # 'hw_torque_constant': 8.27 / 240.0,
        # 'hw_default_vel_limit': 40,
        # 'hw_max_vel_limit': 60,
    }
}

MiMotor_Version = {
    'main': 1,
    'sub': 0,
}


class MiMotor(CanBusDevice):
    master_id_ = 0xFF

    def __init__(self, can: CanBus, id_: int = 0x7F):
        """Initializes the Robodyno motor based on the device id and type.

        Args:
            can (CanBus): The CAN bus interface.
            id_ (int): The device id. The default factory id of the mi_motor is 0x7F.

        Raises:
            ValueError: If device id is not in range [0x01, 0x80).
            TypeError: If can is not CanBus.
        """
        mcu_uuid = MiMotor.get_mcu_uuid(can, id_, 0.05)
        if mcu_uuid != None:
            self.type = Model.CYBERGEAR
            self.mcu_uuid = mcu_uuid

        # CanBusDevice __init__
        if not 0x01 <= id_ < 0x80:
            raise ValueError('Device id should be in range [0x01, 0x80).')
        if get_interface_type(can) != InterfaceType.CAN_BUS:
            raise TypeError('CanBusDevice can only be initialized with CanBus.')
        self._can = can
        self.id = id_
        self.fw_ver = None
        version = self.get_version(0.05)

        self.__dict__.update(ROBOTTIME_PARAMS["mi_cybergear"])
        self.reduction = version['data']

        self.state = self.MotorState.UNKNOWN
        self.error = {
            'error': self.MotorError.NONE,
            'motor_err': self.MotorMotorError.NONE,
            'encoder_err': self.MotorEncoderError.NONE,
            'controller_err': self.MotorControllerError.NONE,
        }
        self.mode = self.MotorControlMode.UNKNOWN
        self.sync_time = 0

        # update state, error, mode
        self.get_state()

    @staticmethod
    def get_mcu_uuid(can: CanBus, can_id: int, timeout: float = 0.1) -> Optional[int]:
        """
        Retrieves the MCU UUID of a Xiaomi motor from the CAN bus for a specified device ID.
        This is commonly used to check if the device is online.

        Args:
            can: The CAN bus instance used to send and receive messages.
            can_id: The target device ID on the CAN bus to query.
            timeout: The time to wait for a response, in seconds (default is 0.1s).

        Returns:
            The MCU UUID of the device if successful, otherwise None if the request times out
            or the device ID does not match the expected CAN ID.
        """
        try:
            msg = MiMotorProtocol.command0(MiMotor.master_id_, can_id)
            rx_msg = can.mi_motor_send(msg, timeout)
        except TimeoutError:
            return None

        device_id, mcu_uuid = MiMotorProtocol.parse_command0_response(rx_msg)
        if device_id == can_id:
            return mcu_uuid
        # If device IDs do not match, return None
        return None

    def get_version(self, timeout: Optional[float] = 0.1) -> Optional[dict]:
        """Get device firmware version and type.

        Args:
            timeout (float): The time to wait for a response, in seconds (default is 0.1s).

        Returns:
            (dict | None): Device firmware version and type.
        """
        main_ver = MiMotor_Version["main"]
        sub_ver = MiMotor_Version["sub"]
        data = self._get_data_from_command9(
            MiMotorProtocol.PARAM_TABLE["GearRatio"], timeout
        )  # gear ratio

        self.fw_ver = float(f'{main_ver}.{sub_ver}')
        return {
            'main_version': main_ver,
            'sub_version': sub_ver,
            'data': data,
            'type': self.type,
        }

    def enable(self) -> None:
        """Enables the motor.

        After enabled, the motor can be controlled.
        """
        msg = MiMotorProtocol.command3(self.master_id_, self.id)
        self._can.mi_motor_send(msg, timeout=0)
        sleep(0.02)  # todo: check if this is necessary

    def disable(self) -> None:
        """Disables the motor.

        This is the default state after power on. After disabled, the motor
        will stop working and loose torque.
        """
        msg = MiMotorProtocol.command4(self.master_id_, self.id)
        self._can.mi_motor_send(msg, timeout=0)

    def unlock(self) -> None:
        """Unlocks the motor.

        This command only works on motors with brake. After unlocked, the motor
        can be rotated freely.

        Raises:
            NotImplementedError: If the motor does not support brake.
        """
        if not self.with_brake:
            raise NotImplementedError('unlock is not supported on motors without brake')

    def init_pos(self) -> None:
        """Sets the current position of the motor as the initial position."""
        msg = MiMotorProtocol.command6(self.master_id_, self.id)
        self._can.mi_motor_send(msg, timeout=0)

    def init_abs_pos(self) -> None:
        """Sets the current absolute position of the motor as the initial position.

        Raises:
            NotImplementedError: If the device is mi motor.
        """
        raise NotImplementedError('Absolute position is not supported on mi motor')

    def calibrate(self) -> None:
        """Calibrates the motor."""
        msg = MiMotorProtocol.command5(self.master_id_, self.id)
        self._can.mi_motor_send(msg, timeout=0)

    def config_can_bus(self, new_id: int):
        """Configures the CAN bus settings of the motor.

        Args:
            new_id (int): The new device id.

        Raises:
            ValueError: If the new CAN id is not in the range of 0x01-0x7f.
        """
        if not 0x01 <= new_id <= 0x7F:
            raise ValueError('New CAN id must be in the range of 0x01-0x7f.')

        msg = MiMotorProtocol.command7(self.master_id_, self.id, new_id)
        self._can.mi_motor_send(msg, timeout=0)

    def save(self) -> None:
        """Saves the motor configurations."""
        msg = MiMotorProtocol.command8(self.master_id_, self.id, cmd=2)
        self._can.mi_motor_send(msg, timeout=0)

    def save_configuration(self) -> None:
        """Saves the motor configurations.

        This method will be deprecated. Use `save` instead.
        """
        self.save()

    def clear_errors(self) -> None:
        """Clears the motor errors.

        Most errors are cleared automatically if the cause is resolved.
        """

        msg = MiMotorProtocol.command4(self.master_id_, self.id, 1)
        self._can.mi_motor_send(msg, timeout=0)

    def estop(self) -> None:
        """Requests an emergency stop.

        The motor will loose torque immediately and report an ESTOP error.

        Raises:
            NotImplementedError: If the device is mi motor.
        """
        raise NotImplementedError('estop is not supported on mi motor')

    def reboot(self) -> None:
        """Reboots the motor.

        Raises:
            NotImplementedError: If the device is mi motor.
        """
        raise NotImplementedError('reboot is not supported on mi motor')

    def reset(self) -> None:
        """Resets the motor.

        This command will reset the motor to the factory settings. All
        configurations will be lost and the id will be reset to 0x7F.
        The motor must be calibrated again after reset.
        """
        msg = MiMotorProtocol.command8(self.master_id_, self.id, cmd=3)
        self._can.mi_motor_send(msg, timeout=0)

    def mit_mode(self) -> None:
        """Sets the motor to mit mode."""
        msg = MiMotorProtocol.command18(
            self.master_id_, self.id, MiMotorProtocol.PARAMETERS["run_mode"], 0
        )
        self._can.mi_motor_send(msg, timeout=0)

    def position_mode(self) -> None:
        """Sets the motor to position mode."""
        msg = MiMotorProtocol.command18(
            self.master_id_, self.id, MiMotorProtocol.PARAMETERS["run_mode"], 1
        )
        self._can.mi_motor_send(msg, timeout=0)

    def position_filter_mode(self, bandwidth: float) -> None:
        """Sets the motor to position filter mode.
        todo: implement the position filter mode
        Note:
            The position track mode has not been implemented yet. The current implementation
            only sets the motor to position mode.

        Args:
            bandwidth (float): The bandwidth of the filter in Hz. The value is suggested
                to be the control loop frequency in Hz.
        """
        self.position_mode()

    def position_track_mode(self, vel: float, acc: float, dec: float) -> None:
        """Sets the motor to position track mode.
        todo: implement the position track mode
        Note:
            The position track mode has not been implemented yet. The current implementation
            only sets the motor to position mode.

        Args:
            vel (float): The velocity limit in rad/s.
            acc (float): The acceleration in rad/s^2. The value must be positive.
            dec (float): The deceleration in rad/s^2. The value must be positive.
        """
        self.position_mode()

    def velocity_mode(self) -> None:
        """Sets the motor to velocity mode."""
        msg = MiMotorProtocol.command18(
            self.master_id_, self.id, MiMotorProtocol.PARAMETERS["run_mode"], 2
        )
        self._can.mi_motor_send(msg, timeout=0)

    def velocity_ramp_mode(self, ramp: float) -> None:
        """Sets the motor to velocity ramp mode.
        todo: implement the velocity ramp mode

        Note:
            The velocity ramp mode has not been implemented yet. The current implementation
            only sets the motor to velocity mode.

        Args:
            ramp (float): The velocity ramp in rad/s^2. The value must be positive.
        """
        self.velocity_mode()

    def torque_mode(self) -> None:
        """Sets the motor to torque mode."""
        msg = MiMotorProtocol.command18(
            self.master_id_, self.id, MiMotorProtocol.PARAMETERS["run_mode"], 3
        )
        self._can.mi_motor_send(msg, timeout=0)

    def set_mit(
        self, torque: float, position: float, velocity: float, kp: float, kd: float
    ) -> None:
        """Sets the MIT control parameters of the motor.

        Args:
            torque (float): The torque reference in Nm.
            position (float): The position reference in rad.
            velocity (float): The velocity reference in rad/s.
            kp (float): The proportional gain.
            kd (float): The derivative gain.
        """
        msg = MiMotorProtocol.command1(torque, position, velocity, kp, kd, self.id)
        self._can.mi_motor_send(msg, timeout=0)

    def set_pos(self, position: float) -> None:
        """Sets the target position of the motor.

        Args:
            position (float): The target position in rad.
        """
        msg = MiMotorProtocol.command18(
            self.master_id_,
            self.id,
            parameter=MiMotorProtocol.PARAMETERS["loc_ref"],
            data=position,
        )
        self._can.mi_motor_send(msg, timeout=0)

    def set_abs_pos(self, pos: float, vel_ff: float = 0, torque_ff: float = 0) -> None:
        """Sets the target absolute position of the motor.

        Args:
            pos (float): The target absolute position in rad.
            vel_ff (float): The velocity feedforward in rad/s.
            torque_ff (float): The torque feedforward in Nm.

        Raises:
            NotImplementedError: If the motor is mi motor.
        """
        raise NotImplementedError('Absolute position is not supported on mi motor')

    def set_vel(self, velocity: float) -> None:
        """Sets the target velocity of the motor.

        Args:
            vel (float): The target velocity in rad/s.
        """
        msg = MiMotorProtocol.command18(
            self.master_id_,
            self.id,
            parameter=MiMotorProtocol.PARAMETERS["spd_ref"],
            data=velocity,
        )
        self._can.mi_motor_send(msg, timeout=0)

    def set_torque(self, torque: float) -> None:
        """Sets the target torque of the motor.

        Args:
            torque (float): The target torque in Nm.
        """
        TORQUE_TO_CURRENT = 1 / 0.6
        iq = torque * TORQUE_TO_CURRENT
        msg = MiMotorProtocol.command18(
            self.master_id_,
            self.id,
            parameter=MiMotorProtocol.PARAMETERS["iq_ref"],
            data=iq,
        )
        self._can.mi_motor_send(msg, timeout=0)

    def set_pid(self, pos_kp: float, vel_kp: float, vel_ki: float) -> None:
        """Sets the PID parameters of the motor.

        Args:
            pos_kp (float): The Kp of the position control loop.
            vel_kp (float): The Kp of the velocity control loop.
            vel_ki (float): The Ki of the velocity control loop.
        """
        msg_1 = MiMotorProtocol.command8(
            self.master_id_, self.id, 0, MiMotorProtocol.PARAM_TABLE["loc_kp"], pos_kp
        )
        msg_2 = MiMotorProtocol.command8(
            self.master_id_, self.id, 0, MiMotorProtocol.PARAM_TABLE["spd_kp"], vel_kp
        )
        msg_3 = MiMotorProtocol.command8(
            self.master_id_, self.id, 0, MiMotorProtocol.PARAM_TABLE["spd_ki"], vel_ki
        )

        self._can.mi_motor_send(msg_1, timeout=1)
        self._can.mi_motor_send(msg_2, timeout=1)
        self._can.mi_motor_send(msg_3, timeout=1)

    def set_vel_limit(self, velocity_limit: float) -> None:
        """Sets the velocity limit of the motor.

        Args:
            vel_lim (float): The velocity limit in rad/s. The value must be positive.

        Raises:
            ValueError: If the velocity limit is out of range.
        """

        if not 0 < velocity_limit <= self.available_velocity:
            raise ValueError(f'Velocity limit {velocity_limit} is out of range.')

        msg = MiMotorProtocol.command18(
            self.master_id_,
            self.id,
            parameter=MiMotorProtocol.PARAMETERS["limit_spd"],
            data=velocity_limit,
        )
        self._can.mi_motor_send(msg, timeout=0)

    def set_current_limit(self, current_limit: float) -> None:
        """Sets the current limit of the motor.

        Args:
            current_lim (float): The current limit in amps. The value must be positive.

        Raises:
            ValueError: If the current limit is out of range.
        """
        if not 0 < current_limit <= self.available_current:
            raise ValueError(f'Current limit {current_limit} is out of range.')
        msg = MiMotorProtocol.command18(
            self.master_id_,
            self.id,
            parameter=MiMotorProtocol.PARAMETERS["limit_cur"],
            data=current_limit,
        )
        self._can.mi_motor_send(msg, timeout=0)

    def _mi_get_state(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Get the state of the mi motor.

        Returns:
            (tuple | None): The motor state, error, pos, vel, tor, and temperature. Returns None if
                the read times out.
                - master_id (int): The master id.
                - state (int): The motor state.
                - error_mask (int): The error mask.
                - position (float): The position in rad.
                - velocity (float): The velocity in rad/s.
                - torque (float): The torque in Nm.
                - temperature (float): The temperature in degree Celsius.
        """
        try:
            msg = MiMotorProtocol.command2(self.master_id_, self.id)
            rx_msg = self._can.mi_motor_send(msg, timeout)
        except TimeoutError:
            return None
        if rx_msg is None:
            return None

        return MiMotorProtocol.parse_command2_response(rx_msg)

    # cmd 9 是从 param_table 中的param 获取值
    def _get_data_from_command9(self, param: dict, timeout: int) -> any:
        """Retrieve data for a specified param using Command 9.

        This function sends a Command 9 request to the motor to read data for a given parameter
        and processes the response to extract the desired information.

        Args:
            param (dict): The parameter to read, represented as a dictionary with the following keys:
                - "index" (int): The index of the parameter to be read, specifying its location.
                - "format" (str): The format of the parameter's data (e.g., "B" for unsigned byte, "f" for float).
            timeout (int): The maximum time to wait (in seconds) for the response. If the response is not received
                        within the timeout period, the function returns `None`.

        Returns:
            any: The value associated with the requested parameter, formatted based on the "format" key.
                Returns `None` if the response is not received or an error occurs.

        Raises:
            TimeoutError: If the request times out while waiting for a response.

        Example:
            param = {"index": 0x7005, "format": "B"}
            result = self._get_data_from_command9(param, timeout=2)
            if result is not None:
                print(f"Parameter data: {result}")
            else:
                print("Failed to retrieve parameter data.")
        """
        msg = MiMotorProtocol.command9(self.master_id_, self.id, param)
        try:
            rx_msg = self._can.mi_motor_send(msg, timeout)
        except TimeoutError:
            return None
        if rx_msg is not None:
            master_id, device_id, rx_param, data = (
                MiMotorProtocol.parse_command9_response(rx_msg)
            )
            if rx_param == param:
                return data
        else:
            return None

    def _get_data_from_command17(self, parameter: dict, timeout: int) -> any:
        """Retrieve data for a specified parameter using Command 17.

        This function sends a Command 17 request to the motor to read data for a given parameter
        and processes the response to extract the desired information.

        Args:
            parameter (dict): The parameter to read, represented as a dictionary with the following keys:
                - "index" (int): The index of the parameter to be read, specifying its location.
                - "format" (str): The format of the parameter's data (e.g., "B" for unsigned byte, "f" for float).
            timeout (int): The maximum time to wait (in seconds) for the response. If the response is not received
                        within the timeout period, the function returns `None`.

        Returns:
            any: The value associated with the requested parameter, formatted based on the "format" key.
                Returns `None` if the response is not received or an error occurs.

        Raises:
            TimeoutError: If the request times out while waiting for a response.

        Example:
            parameter = {"index": 0x7005, "format": "B"}
            result = self._get_data_from_command17(parameter, timeout=2)
            if result is not None:
                print(f"Parameter data: {result}")
            else:
                print("Failed to retrieve parameter data.")
        """
        msg = MiMotorProtocol.command17(self.master_id_, self.id, parameter)
        try:
            rx_msg = self._can.mi_motor_send(msg, timeout)
        except TimeoutError:
            return None
        if rx_msg is not None:
            master_id, device_id, rx_param, data = (
                MiMotorProtocol.parse_command17_response(rx_msg)
            )
            if rx_param == parameter:
                return data
        else:
            return None

    def get_state(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the motor state and error.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): The motor state, error, and control mode. Returns None if
                the read times out.
        """
        _, state, error_mask, _, _, _, _ = self._mi_get_state(timeout)
        self.state = self.MotorState(state)
        self.error.update(
            {
                'error': self.MotorError(error_mask),
                'motor_err': self.MotorMotorError(0),
                'encoder_err': self.MotorEncoderError(0),
                'controller_err': self.MotorControllerError(0),
            }
        )
        mode = self._get_data_from_command17(
            MiMotorProtocol.PARAMETERS["run_mode"], timeout
        )
        self.mode = self.MotorControlMode(mode)
        return (self.state, self.error, self.mode)

    def get_voltage(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the DC bus voltage of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The DC bus voltage in volts. Returns None if the read
                times out.
        """
        voltage = self._get_data_from_command17(
            MiMotorProtocol.PARAMETERS["VBUS"], timeout
        )
        return voltage

    def get_temperature(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the temperature of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The temperature in degrees Celsius. Returns None if the
                read times out.
        """
        return self._mi_get_state(timeout)[6]

    def get_mode(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the control mode and the mode parameters of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): The control mode and the mode parameters. Returns None if
                the read times out.
        """

        mode = self._get_data_from_command17(
            MiMotorProtocol.PARAMETERS["run_mode"], timeout
        )
        self.mode = self.MotorControlMode(mode)
        return (self.mode,)

    def get_feedback(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the feedbacks of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): The position, velocity, and torque. Returns None if the
                read times out.
        """
        pos = self.get_pos(timeout)
        vel, torque = self._mi_get_state(timeout)[4:6]
        return (pos, vel, torque)

    def get_pos(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the position of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The position in rad. Returns None if the read times
                out.
        """
        return self._get_data_from_command17(
            MiMotorProtocol.PARAMETERS["mechPos"], timeout
        )

    def get_abs_pos(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the absolute position of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The absolute position in rad. Returns None if the
                read times out.

        Raises:
            NotImplementedError: If the device is mi motor.
        """
        raise NotImplementedError('Absolute position is not supported on mi motor')

    def get_vel(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the velocity of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The velocity in rad/s. Returns None if the read times
                out.
        """
        return self._get_data_from_command17(
            MiMotorProtocol.PARAMETERS["mechVel"], timeout
        )

    def get_torque(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the torque of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The torque in Nm. Returns None if the read times out.
        """
        r = self._mi_get_state(timeout)
        if r is None:
            return None
        _, _, _, _, _, torque, _ = r
        return torque

    def get_pid(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the PID parameters of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): The position KP, velocity KP, and velocity KI. Returns
                None if the read times out.
        """

        loc_kp = self._get_data_from_command9(
            MiMotorProtocol.PARAM_TABLE["loc_kp"], timeout
        )
        spd_kp = self._get_data_from_command9(
            MiMotorProtocol.PARAM_TABLE["spd_kp"], timeout
        )
        spd_ki = self._get_data_from_command9(
            MiMotorProtocol.PARAM_TABLE["spd_ki"], timeout
        )
        return (loc_kp, spd_kp, spd_ki)

    def get_vel_limit(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the velocity limit of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The velocity limit in rad/s. Returns None if the read
                times out.
        """
        return self._get_data_from_command17(
            MiMotorProtocol.PARAMETERS["limit_spd"], timeout
        )

    def get_current_limit(self, timeout: Optional[float] = None) -> Optional[float]:
        """Reads the current limit of the motor.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): The current limit in A. Returns None if the read
                times out.
        """
        return self._get_data_from_command17(
            MiMotorProtocol.PARAMETERS["limit_cur"], timeout
        )

    class MotorState(Enum):
        """Enumerates motor working states."""

        UNKNOWN = -1
        """The state of the motor is unknown."""
        DISABLED = 0
        """The motor is disabled."""
        CALIBRATE = 1
        """The motor is calibrating."""
        ENABLED = 2
        """The motor is enabled."""

    class MotorError(Enum):
        """Enumerates motor errors."""

        NONE = 0
        """No error."""
        UNDER_VOLTAGE = 1
        """The motor is under voltage."""
        OVER_CURRENT = 2
        """The motor is over current."""
        OVER_TEMP = 4
        """The motor is over temperature."""
        MAGNETIC_ENCODER = 8
        """The magnetic encoder is error."""
        HALL_ENCODER = 16
        """The hall encoder is error."""
        UNCALIBRATEED = 32
        """The motor is uncalibrated ."""

    class MotorMotorError(Enum):
        """Enumerates motor driver errors."""

        NONE = 0
        """No error."""

    class MotorEncoderError(Enum):
        """Enumerates motor encoder errors."""

        NONE = 0
        """No error."""

    class MotorControllerError(Enum):
        """Enumerates motor controller errors."""

        NONE = 0
        """No error."""

    class MotorControlMode(Enum):
        """Enumerates motor control modes."""

        UNKNOWN = -1
        """The motor control mode is unknown."""
        MIT_MODE = 0
        """Control the motor in MIT mode."""
        POSITION_MODE = 1
        """Control the motor in position mode."""
        VELOCITY_MODE = 2
        """Control the motor in velocity mode."""
        TORQUE_MODE = 3
        """Control the motor in torque mode."""


class MiMotorProtocol:

    PARAM_TABLE = {
        "Name": {
            "index": 0x0000,
            "format": "S",
            "permission": "rw",
            "max_value": None,
            "min_value": None,
        },
        "BarCode": {
            "index": 0x0001,
            "format": "S",
            "permission": "rw",
            "max_value": None,
            "min_value": None,
        },
        "BootCodeVersion": {
            "index": 0x1000,
            "format": "S",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "BootBuildDate": {
            "index": 0x1001,
            "format": "S",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "BootBuildTime": {
            "index": 0x1002,
            "format": "S",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "AppCodeVersion": {
            "index": 0x1003,
            "format": "S",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "AppGitVersion": {
            "index": 0x1004,
            "format": "S",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "AppBuildDate": {
            "index": 0x1005,
            "format": "S",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "AppBuildTime": {
            "index": 0x1006,
            "format": "S",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "AppCodeName": {
            "index": 0x1007,
            "format": "S",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "echoPara1": {
            "index": 0x2000,
            "format": "H",
            "permission": "config",
            "max_value": 74,
            "min_value": 5,
        },
        "echoPara2": {
            "index": 0x2001,
            "format": "H",
            "permission": "config",
            "max_value": 74,
            "min_value": 5,
        },
        "echoPara3": {
            "index": 0x2002,
            "format": "H",
            "permission": "config",
            "max_value": 74,
            "min_value": 5,
        },
        "echoPara4": {
            "index": 0x2003,
            "format": "H",
            "permission": "config",
            "max_value": 74,
            "min_value": 5,
        },
        "echoFreHz": {
            "index": 0x2004,
            "format": "I",
            "permission": "rw",
            "max_value": 10000,
            "min_value": 1,
        },
        "MechOffset": {
            "index": 0x2005,
            "format": "f",
            "permission": "setting",
            "max_value": 7,
            "min_value": -7,
        },
        "MechPos_init": {
            "index": 0x2006,
            "format": "f",
            "permission": "rw",
            "max_value": 50,
            "min_value": -50,
        },
        "limit_torque": {
            "index": 0x2007,
            "format": "f",
            "permission": "rw",
            "max_value": 12,
            "min_value": 0,
        },
        "I_FW_MAX": {
            "index": 0x2008,
            "format": "f",
            "permission": "rw",
            "max_value": 33,
            "min_value": 0,
        },
        "motor_index": {
            "index": 0x2009,
            "format": "B",
            "permission": "setting",
            "max_value": 20,
            "min_value": 0,
        },
        "CAN_ID": {
            "index": 0x200A,
            "format": "B",
            "permission": "setting",
            "max_value": 127,
            "min_value": 0,
        },
        "CAN_MASTER": {
            "index": 0x200B,
            "format": "B",
            "permission": "setting",
            "max_value": 127,
            "min_value": 0,
        },
        "CAN_TIMEOUT": {
            "index": 0x200C,
            "format": "I",
            "permission": "rw",
            "max_value": 1000000,
            "min_value": 0,
        },
        "motorOverTemp": {
            "index": 0x200D,
            "format": "H",
            "permission": "rw",
            "max_value": 1500,
            "min_value": 0,
        },
        "overTempTime": {
            "index": 0x200E,
            "format": "I",
            "permission": "rw",
            "max_value": 1000000,
            "min_value": 1000,
        },
        "GearRatio": {
            "index": 0x200F,
            "format": "f",
            "permission": "rw",
            "max_value": 64,
            "min_value": 1,
        },
        "Tq_caliType": {
            "index": 0x2010,
            "format": "B",
            "permission": "rw",
            "max_value": 1,
            "min_value": 0,
        },
        "cur_filt_gain": {
            "index": 0x2011,
            "format": "f",
            "permission": "rw",
            "max_value": 1,
            "min_value": 0,
        },
        "cur_kp": {
            "index": 0x2012,
            "format": "f",
            "permission": "rw",
            "max_value": 200,
            "min_value": 0,
        },
        "cur_ki": {
            "index": 0x2013,
            "format": "f",
            "permission": "rw",
            "max_value": 200,
            "min_value": 0,
        },
        "spd_kp": {
            "index": 0x2014,
            "format": "f",
            "permission": "rw",
            "max_value": 200,
            "min_value": 0,
        },
        "spd_ki": {
            "index": 0x2015,
            "format": "f",
            "permission": "rw",
            "max_value": 200,
            "min_value": 0,
        },
        "loc_kp": {
            "index": 0x2016,
            "format": "f",
            "permission": "rw",
            "max_value": 200,
            "min_value": 0,
        },
        "spd_filt_gain": {
            "index": 0x2017,
            "format": "f",
            "permission": "rw",
            "max_value": 1,
            "min_value": 0,
        },
        "limit_spd": {
            "index": 0x2018,
            "format": "f",
            "permission": "rw",
            "max_value": 200,
            "min_value": 0,
        },
        "limit_cur": {
            "index": 0x2019,
            "format": "f",
            "permission": "rw",
            "max_value": 23,
            "min_value": 0,
        },
        "timeUse0": {
            "index": 0x3000,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "timeUse1": {
            "index": 0x3001,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "timeUse2": {
            "index": 0x3002,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "timeUse3": {
            "index": 0x3003,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "encoderRaw": {
            "index": 0x3004,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "mcuTemp": {
            "index": 0x3005,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "motorTemp": {
            "index": 0x3006,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "vBus(mv)": {
            "index": 0x3007,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "adc1Offset": {
            "index": 0x3008,
            "format": "I",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "adc2Offset": {
            "index": 0x3009,
            "format": "I",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "adc1Raw": {
            "index": 0x300A,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "adc2Raw": {
            "index": 0x300B,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "VBUS": {
            "index": 0x300C,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "cmdId": {
            "index": 0x300D,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "cmdIq": {
            "index": 0x300E,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "cmdlocref": {
            "index": 0x300F,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "cmdspdref": {
            "index": 0x3010,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "cmdTorque": {
            "index": 0x3011,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "cmdPos": {
            "index": 0x3012,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "cmdVel": {
            "index": 0x3013,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "rotation": {
            "index": 0x3014,
            "format": "H",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "modPos": {
            "index": 0x3015,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "mechPos": {
            "index": 0x3016,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
        "mechVel": {
            "index": 0x3017,
            "format": "f",
            "permission": "r",
            "max_value": None,
            "min_value": None,
        },
    }

    PARAMETERS = {
        "run_mode": {"index": 0x7005, "format": "B"},
        "iq_ref": {"index": 0x7006, "format": "f"},
        "spd_ref": {"index": 0x700A, "format": "f"},
        "limit_torque": {"index": 0x700B, "format": "f"},
        "cur_kp": {"index": 0x7010, "format": "f"},
        "cur_ki": {"index": 0x7011, "format": "f"},
        "cur_filt_gain": {"index": 0x7014, "format": "f"},
        "loc_ref": {"index": 0x7016, "format": "f"},
        "limit_spd": {"index": 0x7017, "format": "f"},
        "limit_cur": {"index": 0x7018, "format": "f"},
        "mechPos": {"index": 0x7019, "format": "f"},
        "iqf": {"index": 0x701A, "format": "f"},
        "mechVel": {"index": 0x701B, "format": "f"},
        "VBUS": {"index": 0x701C, "format": "f"},
        "rotation": {"index": 0x701D, "format": "h"},
    }

    @staticmethod
    def command0(master_id, id):
        """Get device ID"""
        arbitration_id = (0x0 << 24) | (master_id << 8) | id
        msg = can.Message(arbitration_id=arbitration_id, data=None, is_extended_id=True)
        return msg

    @staticmethod
    def parse_command0_response(msg):
        """Parse command0 response"""
        command = (msg.arbitration_id >> 24) & 0x1F
        fixed_data = (msg.arbitration_id >> 0) & 0xFF
        device_id = (msg.arbitration_id >> 8) & 0xFF
        mcu_uuid = unpack("<Q", msg.data)[0]

        # Check if command or fixed data is invalid
        if command != 0 or fixed_data != 0xFE:
            raise Exception("Invalid command or fixed data")

        return device_id, mcu_uuid

    @staticmethod
    def command1(torque, position, velocity, kp, kd, id):
        """Set MIT control command, device will return the current state by command 2 when received"""
        if (
            not (-12 <= torque <= 12)
            or not (-4 * pi <= position <= 4 * pi)
            or not (-30 <= velocity <= 30)
            or not (0 <= kp <= 500)
            or not (0 <= kd <= 5)
        ):
            raise ValueError("Invalid value")

        torque = int(np.interp(torque, [-12, 12], [0, 65535]))
        position = int(np.interp(position, [-4 * pi, 4 * pi], [0, 65535]))
        velocity = int(np.interp(velocity, [-30, 30], [0, 65535]))
        kp = int(np.interp(kp, [0, 500], [0, 65535]))
        kd = int(np.interp(kd, [0, 5], [0, 65535]))

        arbitration_id = (0x1 << 24) | (torque << 8) | id
        data = pack(">HHHH", position, velocity, kp, kd)
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        return msg

    @staticmethod
    def command2(master_id, id):
        """Get current state, device will return the current state by command 2 when received"""
        arbitration_id = (0x2 << 24) | (master_id << 8) | id
        msg = can.Message(arbitration_id=arbitration_id, data=None, is_extended_id=True)
        return msg

    @staticmethod
    def parse_command2_response(msg):
        """Parse command0 response

        Returns:
            - master_id (int): The ID of the master device.
            - mode (int): The mode of the motor.
                - 0: Reset mode
                - 1: Calibration mode
                - 2: Motor mode
            - error_mask (int): The error mask of the motor.
            - position (float): The position of the motor in radians.
            - velocity (float): The velocity of the motor in rad/s.
            - torque (float): The torque of the motor in Nm.
            - temperature (float): The temperature of the motor in degrees
        """
        command = (msg.arbitration_id >> 24) & 0x1F
        if command != 2:
            raise ValueError("Invalid command")

        master_id = msg.arbitration_id & 0xFF
        mode = (msg.arbitration_id >> 22) & 0x3
        error_mask = (msg.arbitration_id >> 16) & 0x3F

        position, velocity, torque, temperature = unpack(">HHHH", msg.data)

        position = np.interp(position, [0, 65535], [-4 * pi, 4 * pi])
        velocity = np.interp(velocity, [0, 65535], [-30, 30])
        torque = np.interp(torque, [0, 65535], [-12, 12])
        temperature = temperature / 10

        return master_id, mode, error_mask, position, velocity, torque, temperature

    @staticmethod
    def command3(master_id, id):
        """enable motor, device will return the current state by command 2 when received"""
        arbitration_id = (0x3 << 24) | (master_id << 8) | id
        msg = can.Message(arbitration_id=arbitration_id, data=None, is_extended_id=True)
        return msg

    @staticmethod
    def command4(master_id, id, data_cmd=0):
        """disable motor, device will return the current state by command 2 when received

        Args:
            - data_cmd (int, optional): Command data for disabling motor. Default is 0
                - 0: Disable motor
                - 1: Disable motor and clear error
        """
        arbitration_id = (0x4 << 24) | (master_id << 8) | id
        data = pack("<B", data_cmd)
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        return msg

    @staticmethod
    def command5(master_id, id, data_cmd=0):
        """Encoder Calibration"""
        arbitration_id = (0x5 << 24) | (master_id << 8) | id
        data = bytes(8)
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        return msg

    @staticmethod
    def command6(master_id, id):
        """set mechanical zero, device will return the current state by command 2 when received"""
        arbitration_id = (0x6 << 24) | (master_id << 8) | id
        data = b'\x01'
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        return msg

    @staticmethod
    def command7(master_id, id, new_id):
        """set device id, device will return the mcu uuid by command 0 when received"""
        arbitration_id = (0x7 << 24) | (new_id << 16) | (master_id << 8) | id
        msg = can.Message(arbitration_id=arbitration_id, data=None, is_extended_id=True)
        return msg

    @staticmethod
    def command8(
        master_id: int,
        id: int,
        cmd: int,
        param: Optional[dict] = None,
        param_data: any = None,
    ) -> can.Message:
        """
        Creates a CAN message to set, save, or reset a parameter on a device.

        This function generates a CAN message with a specific command (`cmd`) to instruct
        the device to perform a parameter-related action. The device will respond based on
        the command type:
            - Write parameter (cmd = 0)
            - Save parameter (cmd = 2)
            - Reset parameter (cmd = 3)

        Parameters:
            master_id (int): The ID of the master device.
            id (int): The ID of the target device (e.g., motor).
            cmd (int): The command type. Valid values are:
                - 0: Write parameter
                - 2: Save parameter
                - 3: Reset parameter
            param (dict, optional): A dictionary containing parameter details,
                required for write commands (`cmd == 0`).
                Expected keys:
                    - 'format' (str): The format string for packing parameter data (e.g., 'H' for unsigned short).
                    - 'index' (int): The index of the parameter to write or read.
            param_data (any, optional): The data to write to the parameter, required for `cmd == 0`.

        Returns:
            can.Message: The generated CAN message with the appropriate arbitration ID
                         and data payload based on the command.

        Example:
            `msg = command8(0xFF, 0x7F, cmd=0, param={"format": "B", "index": 0x200A}, param_data=0x70)`

            Sends a write command with parameter index 0x200A and data 0x70 to device 0x7F
        """
        # Construct the arbitration ID using master_id, id, and cmd
        arbitration_id = (0x08 << 24) | (cmd << 16) | (master_id << 8) | id

        # Prepare the data payload based on the command type
        if cmd == 0 and param_data is not None:  # Write parameter
            fmt = "<HH" + param["format"]
            data = pack(fmt, param["index"], 6, param_data)
        elif cmd == 2:  # Save parameter
            data = bytes(8)  # 8 bytes of zero for save
        elif cmd == 3:  # Reset parameter
            data = bytes(8)  # 8 bytes of zero for reset
        else:
            raise ValueError("Invalid command value")

        # Return the generated CAN message with the computed arbitration ID and data
        return can.Message(
            arbitration_id=arbitration_id, data=data, is_extended_id=True
        )

    @staticmethod
    def command9(
        master_id: int,
        id: int,
        param: Optional[dict] = None,
    ):
        """read param, device will return the param value by command 9 when received"""
        arbitration_id = (0x9 << 24) | (master_id << 8) | id
        data = pack("<HH", param["index"], 0)
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        return msg

    @staticmethod
    def parse_command9_response(msg):
        """Parse command9 response"""
        command = (msg.arbitration_id >> 24) & 0x1F
        if command != 9:
            raise ValueError("Invalid command: " + str(command))
        device_id = (msg.arbitration_id >> 8) & 0xFF
        master_id = msg.arbitration_id & 0xFF

        format = None
        index = unpack("<H", msg.data[0:2])[0]  # 1 byte
        for name, value in MiMotorProtocol.PARAM_TABLE.items():
            if value["index"] == index:
                format = value["format"]
                break
        if MiMotorProtocol.PARAM_TABLE[name] == None:
            raise ValueError("Invalid parameter index")
        data = unpack("<" + format, msg.data[4 : 4 + calcsize(format)])[0]
        return master_id, device_id, MiMotorProtocol.PARAM_TABLE[name], data

    @staticmethod
    def command17(master_id, id, parameter):
        """read parameter, device will return the parameter value by command 17 when received"""
        arbitration_id = (0x11 << 24) | (master_id << 8) | id
        data = pack("<HH", parameter["index"], 0)
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        return msg

    @staticmethod
    def parse_command17_response(msg):
        """Parse command17 response"""
        command = (msg.arbitration_id >> 24) & 0x1F
        if command != 17:
            raise ValueError("Invalid command: " + str(command))
        device_id = (msg.arbitration_id >> 8) & 0xFF
        master_id = msg.arbitration_id & 0xFF

        format = None
        index = unpack("<H", msg.data[0:2])[0]  # 1 byte
        for name, value in MiMotorProtocol.PARAMETERS.items():
            if value["index"] == index:
                format = value["format"]
                break
        if MiMotorProtocol.PARAMETERS[name] == None:
            raise ValueError("Invalid parameter index")
        data = unpack("<" + format, msg.data[4 : 4 + calcsize(format)])[0]
        return master_id, device_id, MiMotorProtocol.PARAMETERS[name], data

    @staticmethod
    def command18(master_id, id, parameter, data):
        """write parameter, device will return the parameter value by command 18 when received"""
        arbitration_id = (0x12 << 24) | (master_id << 8) | id
        fmt = "<HH" + parameter["format"]
        data = pack(fmt, parameter["index"], 0, data)
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        return msg
