#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""motor.py
Time    :   2023/01/03
Author  :   song 
Version :   1.0
Contact :   zhaosongy@CanBus.126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno motor can bus driver

  Typical usage example:

  from robodyno.interfaces import CanBus, Webots
  from robodyno.components import Motor

  can = CanBus()
  webots = Webots()
  
  motor_can = Motor(iface = can, id = 0x10, type = 'ROBODYNO_PRO_44')
  motor_webots = Motor(iface = webots, id = 0x11, type = 'ROBODYNO_PRO_12')

  can.disconnect()
"""

import time
from math import pi, fabs
from enum import Enum
import struct

from robodyno.interfaces import CanBus
from robodyno.components import DeviceType, ROBODYNO_MOTOR_SPECS, CanBusDevice

class Motor(CanBusDevice):
    """Robodyno Motor
    
    Attributes:
        id: correspond with motor id in real world or simulation.
        type: motor type enum
        reduction: motor reduction
        available_velocity: theoretical max velocity(rad/s)
        available_torque: theoretical max torque(Nm)
        available_current: theoretical max phase current(A)
        torque_constant: theoretical torque provided by unit current(Nm/A)
        with_brake: is motor has a brake(bool)
        state: motor state read from heartbeat pack
        error: motor error read from heartbeat pack
        mode: motor mode read from heartbeat pack
        sync_time: last timestamp when receive heartbeat pack
    """

    class MotorApiRuntimeError(RuntimeError):
        def __init__(self):
            super().__init__('Failed to invoke motor api from interface.')

    class MotorState(Enum):
        UNKNOWN = -1
        DISABLED = 1
        CALIBRATE = 3
        MOTOR_CALIBRATING = 4
        OFFSET_CALIBRATING = 7
        ENABLED = 8

    class MotorError(Enum):
        NONE = 0
        INVALID_STATE = 1
        UNDER_VOLTAGE = 2
        OVER_VOLTAGE = 3
        CURRENT_MEASUREMENT_TIMEOUT = 4
        BRAKE_RESISTOR_DISARMED = 5
        MOTOR_DISARMED = 6
        MOTOR_FAILED = 7
        SENSORLESS_ESTIMATOR_FAILED = 8
        ENCODER_FAILED = 9
        CONTROLLER_FAILED = 10
        POS_CTRL_DURING_SENSORLESS = 11
        WATCHDOG_TIMER_EXPIRED = 12
        MIN_ENDSTOP_PRESSED = 13
        MAX_ENDSTOP_PRESSED = 14
        ESTOP_REQUESTED = 15
        HOMING_WITHOUT_ENDSTOP = 16
        OVER_TEMP = 17

    class MotorMotorError(Enum):
        NONE = 0
        PHASE_RESISTANCE_OUT_OF_RANGE = 1
        PHASE_INDUCTANCE_OUT_OF_RANGE = 2
        ADC_FAILED = 3
        DRV_FAULT = 4
        CONTROL_DEADLINE_MISSED = 5
        NOT_IMPLEMENTED_MOTOR_TYPE = 6
        BRAKE_CURRENT_OUT_OF_RANGE = 7
        MODULATION_MAGNITUDE = 8
        BRAKE_DEADTIME_VIOLATION = 9
        UNEXPECTED_TIMER_CALLBACK = 10
        CURRENT_SENSE_SATURATION = 11
        CURRENT_LIMIT_VIOLATION = 13
        BRAKE_DUTY_CYCLE_NAN = 14
        DC_BUS_OVER_REGEN_CURRENT = 15
        DC_BUS_OVER_CURRENT = 16

    class MotorEncoderError(Enum):
        NONE = 0
        UNSTABLE_GAIN = 1
        CPR_POLEPAIRS_MISMATCH = 2
        NO_RESPONSE = 3
        UNSUPPORTED_ENCODER_MODE = 4
        ILLEGAL_HALL_STATE = 5
        INDEX_NOT_FOUND_YET = 6
        ABS_SPI_TIMEOUT = 7
        ABS_SPI_COM_FAIL = 8
        ABS_SPI_NOT_READY = 9
        
    class MotorControllerError(Enum):
        NONE = 0
        OVERSPEED = 1
        INVALID_INPUT_MODE = 2
        UNSTABLE_GAIN = 3
        INVALID_MIRROR_AXIS = 4
        INVALID_LOAD_ENCODER = 5
        INVALID_ESTIMATE = 6

    class MotorControlMode(Enum):
        UNKNOWN = -1
        POSITION_MODE = 0
        POSITION_FILTER_MODE = 1
        POSITION_TRACK_MODE = 2
        VELOCITY_MODE = 3
        VELOCITY_RAMP_MODE = 4
        TORQUE_MODE = 5

    CMD_HEARTBEAT = 0x02
    CMD_ESTOP = 0x03
    CMD_REBOOT = 0x04
    CMD_CLEAR_ERRORS = 0x05
    CMD_SAVE = 0x06
    CMD_CONFIG_CAN = 0x07
    CMD_SET_STATE = 0x08
    CMD_GET_HARDWARE_STATUS = 0x09
    CMD_GET_MOTOR_FEEDBACK = 0x0A
    CMD_GET_MODE = 0x0B
    CMD_SET_MODE = 0x0C
    CMD_GET_PID = 0x0D
    CMD_SET_PID = 0x0E
    CMD_GET_LIMITS = 0x0F
    CMD_SET_LIMITS = 0x10
    CMD_SET_POS = 0x11
    CMD_SET_VEL = 0x12
    CMD_SET_TORQUE = 0x13
    CMD_RESET = 0x14
    CMD_UNLOCK = 0x15
    CMD_GET_ABS_POS = 0x16
    CMD_SET_ABS_POS = 0x17
    CMD_INIT_POS = 0x18
    CMD_INIT_ABS_POS = 0x19
    CMD_GET_STATE = 0x20

    def __init__(self, iface, id = 0x10, type = None):
        """Init motor from interface, id and type.
        
        Args:
            iface: robodyno interface object
            id: range from 0x01 to 0x40
            type: Motor type string
        """
        super().__init__(iface, id)
        valid_motor_types = [
            DeviceType.ROBODYNO_PRO_44,
            DeviceType.ROBODYNO_PRO_12,
            DeviceType.ROBODYNO_PRO_50 ,
            DeviceType.ROBODYNO_PRO_100,
            DeviceType.ROBODYNO_PRO_DIRECT,
            DeviceType.ROBODYNO_PLUS_50 ,
            DeviceType.ROBODYNO_PLUS_100,
            DeviceType.ROBODYNO_PLUS_12 ,
            DeviceType.ROBODYNO_PLUS_DIRECT,
            DeviceType.ROBODYNO_NANO_100,
        ]
        self.get_version(timeout=0.15)
        if type:
            self.type = DeviceType[type]
        if self.type not in valid_motor_types:
            raise ValueError('Motor type is invalid and can not distinguish automatically.')

        self.__dict__.update(ROBODYNO_MOTOR_SPECS.get(self.type, None))
        
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
        self._iface.subscribe(
            device_id = self.id, 
            command_id = self.CMD_HEARTBEAT, 
            callback = self._heartbeat_callback
        )
    
    def __del__(self):
        """Collect node from memory."""
        self._iface.unsubscribe(
            device_id = self.id, 
            command_id = self.CMD_HEARTBEAT, 
            callback = self._heartbeat_callback
        )

    def _heartbeat_callback(self, device_id, command_id, data, timestamp):
        """Update motor attributes from heartbeat."""
        state, err, merr, eerr, cerr, cm, im, _ = struct.unpack('<BBBBBBBB', data)
        self.state = self.MotorState(state)
        if self.fw_ver < 1:
            self.error.update({
                'error': self.MotorError(err+1 if err > 0 else 0),
                'motor_err': self.MotorMotorError(merr+1 if merr > 0 else 0),
                'encoder_err': self.MotorEncoderError(eerr+1 if eerr > 0 else 0),
                'controller_err': self.MotorControllerError(cerr+1 if cerr > 0 else 0),
            })
        else:
            self.error.update({
                'error': self.MotorError(err),
                'motor_err': self.MotorMotorError(merr),
                'encoder_err': self.MotorEncoderError(eerr),
                'controller_err': self.MotorControllerError(cerr),
            })
        self.mode = self._ctrl_mode_from_raw(cm, im)
        self.sync_time = timestamp
    
    @CanBus.get_from_bus(CMD_GET_STATE, '<BBBBBBBB')
    def get_state(self, state, err, merr, eerr, cerr, cm, im, type):
        """Get motor state.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            (state, error, mode)
            None if timeout
        """
        self.state = self.MotorState(state)
        if self.fw_ver < 1:
            self.error.update({
                'error': self.MotorError(err+1 if err > 0 else 0),
                'motor_err': self.MotorMotorError(merr+1 if merr > 0 else 0),
                'encoder_err': self.MotorEncoderError(eerr+1 if eerr > 0 else 0),
                'controller_err': self.MotorControllerError(cerr+1 if cerr > 0 else 0),
            })
        else:
            self.error.update({
                'error': self.MotorError(err),
                'motor_err': self.MotorMotorError(merr),
                'encoder_err': self.MotorEncoderError(eerr),
                'controller_err': self.MotorControllerError(cerr),
            })
        self.mode = self._ctrl_mode_from_raw(cm, im)
        return (self.state, self.error, self.mode)

    def _ctrl_mode_to_raw(self, mode):
        """Translate from control mode enum to control mode raw msg
        
        Args:
            mode: control mode enum
        
        Returns:
            tuple of raw msg: (control mode, input mode)
            None when mode not recognized
        """
        if mode == self.MotorControlMode.POSITION_MODE:
            return (3, 1)
        elif mode == self.MotorControlMode.POSITION_FILTER_MODE:
            return (3, 3)
        elif mode == self.MotorControlMode.POSITION_TRACK_MODE:
            return (3, 5)
        elif mode == self.MotorControlMode.VELOCITY_MODE:
            return (2, 1)
        elif mode == self.MotorControlMode.VELOCITY_RAMP_MODE:
            return (2, 2)
        elif mode == self.MotorControlMode.TORQUE_MODE:
            return (1, 1)
        else:
            return None

    def _ctrl_mode_from_raw(self, cmode, imode):
        """Translate from control mode raw msg to control mode enum
        
        Args:
            cmode: raw control mode
            imode: raw input mode
        
        Returns:
            control mode enum
        """
        if cmode == 3:
            if imode == 1:
                return self.MotorControlMode.POSITION_MODE
            elif imode == 3:
                return self.MotorControlMode.POSITION_FILTER_MODE
            elif imode == 5:
                return self.MotorControlMode.POSITION_TRACK_MODE
        if cmode == 2:
            if imode == 1:
                return self.MotorControlMode.VELOCITY_MODE
            elif imode == 2:
                return self.MotorControlMode.VELOCITY_RAMP_MODE
        if cmode == 1 and imode == 1:
            return self.MotorControlMode.TORQUE_MODE
        return self.MotorControlMode.UNKNOWN
    
    @CanBus.send_to_bus(CMD_ESTOP)
    def estop(self):
        """Emergency stop motor."""
        pass

    @CanBus.send_to_bus(CMD_REBOOT)
    def reboot(self):
        """Reboot motor."""
        pass

    @CanBus.send_to_bus(CMD_CLEAR_ERRORS)
    def clear_errors(self):
        """Try to clear motor errors."""
        pass

    @CanBus.send_to_bus(CMD_SAVE)
    def _save(self):
        """Save configurations to hardware."""
        pass

    def save_configuration(self):
        """Save configurations to hardware."""
        self._save()
        self.clear_errors()

    @CanBus.send_to_bus(CMD_CONFIG_CAN, '<HHI')
    def config_can_bus(self, new_id, heartbeat = 1000, bitrate = 'CAN_1M'):
        """Configure motor can bus settings.
        
        Args:
            new_id: motor device id(0x01-0x3f)
            heartbeat: heartbeat period(ms)
            bitrate: choose from 'CAN_1000K', 'CAN_500K', 'CAN_250K',
                     (save and reboot to take effect)
        """
        if new_id < 0x01 or new_id > 0x3f:
            raise ValueError('Unavailable can id. Choose from 0x01-0x3f')

        bitrate = CanBus.CanSpeed[bitrate].value
        if bitrate == 250000:
            bitrate_id = 0
        elif bitrate == 500000:
            bitrate_id = 1
        else:
            bitrate_id = 2
        return (new_id, bitrate_id, int(heartbeat))

    @CanBus.send_to_bus(CMD_SET_STATE, '<I')
    def set_state(self, state):
        """Cange motor state.
        
        Args:
            state: motor state(MotorState)
        """
        return (state.value,)

    def enable(self):
        """Enable motor. Set motor state to CLOSED_LOOP."""
        try:
            self.set_state(self.MotorState.ENABLED)
            time.sleep(0.02)
        except:
            raise self.MotorApiRuntimeError
    
    def calibrate(self):
        """Calibrate motor. Set motor state to CALIBRATE."""
        try:
            self.set_state(self.MotorState.CALIBRATE)
        except:
            raise self.MotorApiRuntimeError
    
    def disable(self):
        """Disable motor. Set motor state to IDLE."""
        try:
            self.set_state(self.MotorState.DISABLED)
        except:
            raise self.MotorApiRuntimeError

    @CanBus.get_from_bus(CMD_GET_HARDWARE_STATUS, '<ff')
    def get_voltage(self, vbus, temperature):
        """Get motor vbus voltage.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            voltage(V)
            None if timeout
        """
        return vbus
    
    @CanBus.get_from_bus(CMD_GET_HARDWARE_STATUS, '<ff')
    def get_temperature(self, vbus, temperature):
        """Get motor temperature.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            temperature(°C)
            None if timeout
        """
        return temperature
    
    @CanBus.get_from_bus(CMD_GET_MOTOR_FEEDBACK, '<fee')
    def get_feedback(self, pos, vel, torque):
        """Get motor position, velocity and torque feedback.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            position(rad), velocity(rad/s), torque(Nm)
            None if timeout
        """
        return pos / self._rot_factor, vel / self._rot_factor, torque * self.reduction

    def get_pos(self, timeout = 0):
        """Get motor position.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            position(rad)
            None if timeout
        """
        try:
            return self.get_feedback(timeout)[0]
        except:
            raise self.MotorApiRuntimeError

    @CanBus.get_from_bus(CMD_GET_ABS_POS, '<f')
    def get_abs_pos(self, pos):
        """Get motor absolute position.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            position(rad)
            None if timeout
        """
        return pos / self._rot_factor
    
    def get_vel(self, timeout = 0):
        """Get motor velocity.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            velocity(rad/s)
            None if timeout
        """
        try:
            return self.get_feedback(timeout)[1]
        except:
            raise self.MotorApiRuntimeError
    
    def get_torque(self, timeout = 0):
        """Get motor torque.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            torque(Nm)
            None if timeout
        """
        try:
            return self.get_feedback(timeout)[2]
        except:
            raise self.MotorApiRuntimeError
    
    @CanBus.get_from_bus(CMD_GET_MODE, '8s')
    def get_mode(self, payload):
        """Get motor control mode and params.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            control mode(MotorControlMode), control params dict
            None if timeout
        """
        control_mode, input_mode = struct.unpack('<BB', payload[:2])
        mode = self._ctrl_mode_from_raw(control_mode, input_mode)
        self.mode = mode
        if mode == self.MotorControlMode.POSITION_FILTER_MODE:
            bandwidth, = struct.unpack('<f', payload[2:6])
            return (mode, {'bandwidth': bandwidth})
        elif mode == self.MotorControlMode.POSITION_TRACK_MODE:
            vel, acc, dec = struct.unpack('<eee', payload[2:8])
            return (mode, {
                'vel': fabs(vel / self._rot_factor),
                'acc': fabs(acc / self._rot_factor),
                'dec': fabs(dec / self._rot_factor),
            })
        elif mode == self.MotorControlMode.VELOCITY_RAMP_MODE:
            ramp, = struct.unpack('<f', payload[2:6])
            return (mode, {'ramp': fabs(ramp / self._rot_factor)})
        else:
            return (mode, )
    
    @CanBus.send_to_bus(CMD_SET_MODE, '<BB')
    def position_mode(self):
        """Enter position pid mode."""
        return self._ctrl_mode_to_raw(self.MotorControlMode.POSITION_MODE)

    @CanBus.send_to_bus(CMD_SET_MODE, '<BBf')
    def position_filter_mode(self, bandwidth):
        """Enter position filter mode.
        
        Args:
            bandwidth: filter bandwith, equals to control frequency(Hz)
        """
        cmode, imode = self._ctrl_mode_to_raw(self.MotorControlMode.POSITION_FILTER_MODE)
        return (cmode, imode, bandwidth)
    
    @CanBus.send_to_bus(CMD_SET_MODE, '<BBeee')
    def position_track_mode(self, vel, acc, dec):
        """Enter position track mode.
        
        Args:
            vel: motion max vel(rad/s)
            acc: motion acceleration(rad/s^2)
            dec: motion deceleration(rad/s^2)
        """
        cmode, imode = self._ctrl_mode_to_raw(self.MotorControlMode.POSITION_TRACK_MODE)
        return (
            cmode, 
            imode, 
            fabs(vel * self._rot_factor), 
            fabs(acc * self._rot_factor), 
            fabs(dec * self._rot_factor)
        )
    
    @CanBus.send_to_bus(CMD_SET_MODE, '<BB')
    def velocity_mode(self):
        """Enter velocity mode."""
        return self._ctrl_mode_to_raw(self.MotorControlMode.VELOCITY_MODE)
    
    @CanBus.send_to_bus(CMD_SET_MODE, '<BBf')
    def velocity_ramp_mode(self, ramp):
        """Enter velocity ramp mode.
        
        Args:
            ramp: motion acceleration(rad/s^2)
        """
        cmode, imode = self._ctrl_mode_to_raw(self.MotorControlMode.VELOCITY_RAMP_MODE)
        return (cmode, imode, fabs(ramp * self._rot_factor))
    
    @CanBus.send_to_bus(CMD_SET_MODE, '<BB')
    def torque_mode(self):
        """Enter torque mode"""
        return self._ctrl_mode_to_raw(self.MotorControlMode.TORQUE_MODE)
    
    @CanBus.get_from_bus(CMD_GET_PID, '<fee')
    def get_pid(self, pos_kp, vel_kp, vel_ki):
        """Get motor pid params.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            pos_kp, vel_kp, vel_ki
            None if timeout
        """
        return (pos_kp, vel_kp, vel_ki)
    
    @CanBus.send_to_bus(CMD_SET_PID, '<fee')
    def set_pid(self, pos_kp, vel_kp, vel_ki):
        """Set motor pid params.
        
        Args:
            pos_kp: kp of position control
            vel_kp: kp of velocity control
            vel_ki: ki of velocity control
        """
        return (pos_kp, vel_kp, vel_ki)

    @CanBus.get_from_bus(CMD_GET_LIMITS, '<ff')
    def get_vel_limit(self, vel_limit, current_limit):
        """Get motor volocity limit.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            velocity limit(rad/s)
            None if timeout
        """
        return vel_limit / fabs(self._rot_factor)
    
    @CanBus.get_from_bus(CMD_GET_LIMITS, '<ff')
    def get_current_limit(self, vel_limit, current_limit):
        """Get motor current limit.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            current limit(A)
            None if timeout
        """
        return current_limit

    @CanBus.send_to_bus(CMD_SET_LIMITS, '<ff')
    def set_vel_limit(self, vel_lim):
        """Set motor velocity limit.
        
        Args:
            vel_lim: velocity limit(rad/s)
        """
        return (fabs(vel_lim * self._rot_factor), 0)
    
    @CanBus.send_to_bus(CMD_SET_LIMITS, '<ff')
    def set_current_limit(self, current_lim):
        """Set motor current limit.
        
        Args:
            current_lim: current limit(A)
        """
        return (0, current_lim)
    
    @CanBus.send_to_bus(CMD_SET_POS, '<fee')
    def set_pos(self, pos, vel_ff = 0, torque_ff = 0):
        """Set motor target position.
        
        Args:
            pos: target position(rad)
            vel_ff: velocity feed forward(rad/s)
            torque_ff:torque feed forward(Nm)
        """
        return (pos * self._rot_factor, vel_ff * self._rot_factor, torque_ff / self.reduction)
    
    @CanBus.send_to_bus(CMD_SET_ABS_POS, '<fee')
    def set_abs_pos(self, pos, vel_ff = 0, torque_ff = 0):
        """Set motor target absolute position.
        
        Args:
            pos: target absolute position(rad)
            vel_ff: velocity feed forward(rad/s)
            torque_ff:torque feed forward(Nm)
        """
        return (pos * self._rot_factor, vel_ff * self._rot_factor, torque_ff / self.reduction)

    @CanBus.send_to_bus(CMD_INIT_POS, '<f')
    def init_pos(self, offset = 0):
        """Init motor pos with current position.
        
        Args:
            offset: current position(rad)
        """
        return (offset * self._rot_factor, )

    @CanBus.send_to_bus(CMD_INIT_ABS_POS, '<f')
    def init_abs_pos(self, offset = 0):
        """Init motor absolute pos with current position.
        
        Args:
            offset: current absolute position(rad)
        """
        return (offset * self._rot_factor, )

    @CanBus.send_to_bus(CMD_SET_VEL, '<ff')
    def set_vel(self, vel, torque_ff = 0):
        """Set motor velocity.
        
        Args:
            vel: target velocity(rad)
            torque_ff: torque feed forward(Nm)
        """
        return (vel * self._rot_factor, torque_ff / self.reduction)
    
    @CanBus.send_to_bus(CMD_SET_TORQUE, '<f')
    def set_torque(self, torque):
        """Set motor torque.
        
        Args:
            torque: target torque(Nm)
        """
        return (torque / self.reduction, )

    @CanBus.send_to_bus(CMD_RESET)
    def reset(self):
        """Reset motor with factory configurations."""
        pass

    @CanBus.send_to_bus(CMD_UNLOCK)
    def unlock(self):
        """Unlock motor brake if motor has one."""
        if not self.with_brake:
            raise RuntimeError('Motor do not have a brake.')
