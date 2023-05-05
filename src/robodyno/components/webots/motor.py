#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""motor.py
Time    :   2023/01/05
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno motor webots bridge

  Typical usage example:

  from robodyno.interfaces import CanBus, Webots
  from robodyno.components import Motor

  can = CanBus()
  webots = Webots()
  
  motor_can = Motor(iface = can, id = 0x10, type = 'ROBODYNO_PRO_44')
  motor_webots = Motor(iface = webots, id = 0x11, type = 'ROBODYNO_PRO_12')

  can.disconnect()
"""

from math import pi, fabs, copysign, sqrt
from enum import Enum
from robodyno.components import DeviceType, ROBODYNO_MOTOR_SPECS, WebotsDevice
from robodyno.components.can_bus.motor import Motor as CanMotor
from robodyno.interfaces import InterfaceType, GET_IFACE_TYPE

class MotorPositionTrackController(object):
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
        dx_min = 0.5 * self.t_acc * (self.top_vel + self.vel) + 0.5 * self.t_dec * self.top_vel
        if s * dx < s * dx_min:
            self.top_vel = s * sqrt(max((self.dec*self.vel*self.vel + 2*self.acc*self.dec*dx) / (self.dec-self.acc), 0))
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
                self.vel + self.acc * self.t
            )
        if self.t < self.t_acc + self.t_vel:
            return (
                self.pos + self.vel*self.t_acc + 0.5*self.acc*self.t_acc*self.t_acc + self.top_vel*(self.t - self.t_acc),
                self.top_vel
            )
        if self.t < self.t_acc + self.t_vel + self.t_dec:
            td = self.t - (self.t_acc + self.t_vel + self.t_dec)
            return (
                self.target + 0.5 * self.dec * td * td,
                self.dec * td
            )
        else:
            return (self.target, self.max_vel)

class MotorPositionFilterController(object):
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
        state: motor state
        mode: motor mode
    """

    class MotorState(Enum):
        UNKNOWN = -1
        DISABLED = 1
        CALIBRATE = 3
        MOTOR_CALIBRATING = 4
        OFFSET_CALIBRATING = 7
        ENABLED = 8

    class MotorControlMode(Enum):
        UNKNOWN = -1
        POSITION_MODE = 0
        POSITION_FILTER_MODE = 1
        POSITION_TRACK_MODE = 2
        VELOCITY_MODE = 3
        VELOCITY_RAMP_MODE = 4
        TORQUE_MODE = 5
        TWIN_MODE = 255
    
    def __init__(self, iface, id = 0x10, type = None, twin = None):
        """Init motor from interface, id and type.
        
        Args:
            iface: robodyno interface object
            id: range from 0x01 to 0x40 or motor name in webots
            type: Motor type string
        """
        iface._mutex.acquire()
        super().__init__(iface, id, auto_register=False)
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
        try:
            self.name = '0x{:02X}::motor'.format(id)
        except:
            self.name = id
        self._motor = self._iface.robot.getDevice(self.name)
        self._pos_sensor = self._motor.getPositionSensor()
        self._twin_motor = None
        if twin and GET_IFACE_TYPE(twin) == InterfaceType.CanBus:
            self._twin_motor = CanMotor(twin, id, type)
            self.init_twin_offset()
            self.type = self._twin_motor.type
        elif type:
            self.type = DeviceType[type]
        else:
            self.type = DeviceType[self._pos_sensor.getName()]
        if self.type not in valid_motor_types:
            raise ValueError('Motor type is invalid')
        self.mode = self.MotorControlMode.UNKNOWN
        self.__dict__.update(ROBODYNO_MOTOR_SPECS.get(self.type, None))
        self.state = self.MotorState.DISABLED
        self._rot_factor = self.reduction / 2.0 / pi

        self._motor.enableTorqueFeedback(int(self._iface.time_step))
        self._pos_sensor.enable(int(self._iface.time_step))

        self._track_controller = MotorPositionTrackController(
            fabs(4 * 2 * pi / self.reduction),
            fabs(10 * 2 * pi / self.reduction),
            fabs(10 * 2 * pi / self.reduction)
        )
        self._filter_controller = MotorPositionFilterController(10)

        self._input_pos = 0
        self._input_vel = 0
        self._input_torque = 0
        self._max_vel = self.available_velocity
        self._pos_feedback = 0
        self._vel_feedback = 0
        self._torque_feedback = 0
        self._prev_pos_feedback = 0
        self._prev_update_time = self._iface.time()
        self._iface.register(self)
        iface._mutex.release()
        if self._twin_motor:
            self.twin_mode()
        else:
            self.position_mode()

    def init_twin_offset(self):
        """Read twin motor position set set it as an offset.

        Returns:
            True if success else False.
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

    def parallel_update(self):
        """Update parallel with simulation step."""
        if self.mode == self.MotorControlMode.TWIN_MODE:
            feedback = self._twin_motor.get_feedback(0.1)
            if feedback is not None:
                self._pos_feedback, self._vel_feedback, self._torque_feedback = feedback
                self._pos_feedback -= self._twin_offset
            if self._twin_motor.fw_ver >= 1:
                abs_pos = self._twin_motor.get_abs_pos(0.1)
                if abs_pos:
                    self._pos_feedback = abs_pos - self._twin_offset

    def update(self):
        """Simulation update callback."""
        if self.mode == self.MotorControlMode.TWIN_MODE:
            self._motor.setPosition(self._pos_feedback)
            return

        t = self._iface.time()
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
            elif self.mode == self.MotorControlMode.VELOCITY_MODE or self.mode == self.MotorControlMode.VELOCITY_RAMP_MODE:
                self._motor.setVelocity(self._input_vel)
            elif self.mode == self.MotorControlMode.POSITION_MODE:
                self._motor.setPosition(self._input_pos)
            elif self.mode == self.MotorControlMode.POSITION_FILTER_MODE:
                p, v = self._filter_controller.update(dt, self._pos_feedback, self._vel_feedback)
                self._motor.setPosition(p)
                self._motor.setVelocity(fabs(v))
            elif self.mode == self.MotorControlMode.POSITION_TRACK_MODE:
                p, v = self._track_controller.update(dt)
                self._motor.setPosition(p)
                self._motor.setVelocity(fabs(v))

    def enable(self):
        """Enable motor. Set motor state to CLOSED_LOOP."""
        self._iface._mutex.acquire()
        self.state = self.MotorState.ENABLED
        self._iface._mutex.release()
    
    def disable(self):
        """Disable motor. Set motor state to IDLE."""
        self._iface._mutex.acquire()
        self.state = self.MotorState.DISABLED
        self._iface._mutex.release()

    def get_pos(self, timeout = 0):
        """Get motor position.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            position(rad)
            None if timeout
        """
        return self._pos_feedback * copysign(1, self.reduction)

    def get_abs_pos(self, timeout = 0):
        """Get motor absolute position.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            position(rad)
            None if timeout
        """
        return self._pos_feedback * copysign(1, self.reduction)


    def get_vel(self, timeout = 0):
        """Get motor velocity.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            velocity(rad/s)
            None if timeout
        """
        return self._vel_feedback * copysign(1, self.reduction)

    def get_torque(self, timeout = 0):
        """Get motor torque.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            torque(Nm)
            None if timeout
        """
        return self._torque_feedback * copysign(1, self.reduction)

    def get_mode(self, payload):
        """Get motor control mode and params.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            control mode(MotorControlMode), control params dict
            None if timeout
        """
        ret = None
        if self.mode == self.MotorControlMode.POSITION_FILTER_MODE:
            ret = (self.mode, {'bandwidth': self._filter_controller.bandwidth})
        elif self.mode == self.MotorControlMode.POSITION_TRACK_MODE:
            ret = (self.mode, {
                'vel': fabs(self._track_controller.max_vel / self._rot_factor),
                'acc': fabs(self._track_controller.max_acc / self._rot_factor),
                'dec': fabs(self._track_controller.max_dec / self._rot_factor),
            })
        elif self.mode == self.MotorControlMode.VELOCITY_RAMP_MODE:
            ret = (self.mode, {'ramp': self._motor.getAcceleration()})
        else:
            ret = (self.mode, )
        return ret
    
    def twin_mode(self):
        self._iface._mutex.acquire()
        self._motor.setPosition(self._pos_feedback)
        self._motor.setVelocity(self._max_vel)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.TWIN_MODE
        self.state = self.MotorState.ENABLED
        self._iface._mutex.release()

    def position_mode(self):
        """Enter position pid mode."""
        self._iface._mutex.acquire()
        self._motor.setPosition(self._pos_feedback)
        self._motor.setVelocity(self._max_vel)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.POSITION_MODE
        self._iface._mutex.release()

    def position_filter_mode(self, bandwidth):
        """Enter position filter mode.
        
        Args:
            bandwidth: filter bandwith, equals to control frequency(Hz)
        """
        self._iface._mutex.acquire()
        self._filter_controller = MotorPositionFilterController(bandwidth)
        self._filter_controller.set_pos(self._pos_feedback)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.POSITION_FILTER_MODE
        self._iface._mutex.release()

    def position_track_mode(self, vel, acc, dec):
        """Enter position track mode.
        
        Args:
            vel: motion max vel(rad/s)
            acc: motion acceleration(rad/s^2)
            dec: motion deceleration(rad/s^2)
        """
        self._iface._mutex.acquire()
        self._track_controller = MotorPositionTrackController(vel, acc, dec)
        self._track_controller.set_pos(self._pos_feedback, self._pos_feedback, self._vel_feedback)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.POSITION_TRACK_MODE
        self._iface._mutex.release()
    
    def velocity_mode(self):
        """Enter velocity mode."""
        self._iface._mutex.acquire()
        self._motor.setPosition(float('+inf'))
        self._motor.setVelocity(self._input_vel)
        self._motor.setAcceleration(-1)
        self.mode = self.MotorControlMode.VELOCITY_MODE
        self._iface._mutex.release()

    def velocity_ramp_mode(self, ramp):
        """Enter velocity ramp mode.
        
        Args:
            ramp: motion acceleration(rad/s^2)
        """
        self._iface._mutex.acquire()
        self._motor.setPosition(float('+inf'))
        self._motor.setVelocity(self._input_vel)
        self._motor.setAcceleration(ramp)
        self.mode = self.MotorControlMode.VELOCITY_RAMP_MODE
        self._iface._mutex.release()

    def torque_mode(self):
        """Enter torque mode"""
        self._iface._mutex.acquire()
        self._motor.setTorque(self._input_torque)
        self.mode = self.MotorControlMode.TORQUE_MODE
        self._iface._mutex.release()
    
    def get_vel_limit(self, vel_limit, current_limit):
        """Get motor volocity limit.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            velocity limit(rad/s)
            None if timeout
        """
        return self._max_vel
    
    def get_current_limit(self, vel_limit, current_limit):
        """Get motor current limit.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            current limit(A)
            None if timeout
        """
        return self._motor.getAvailableTorque() / self.torque_constant
    
    def set_vel_limit(self, vel_lim):
        """Set motor velocity limit.
        
        Args:
            vel_lim: velocity limit(rad/s)
        """
        self._iface._mutex.acquire()
        if (fabs(vel_lim) > self.available_velocity):
            raise ValueError('Velocity limit exceeded available value.')
        self._max_vel = fabs(vel_lim)
        if self.mode in [self.MotorControlMode.POSITION_MODE, self.MotorControlMode.POSITION_FILTER_MODE, self.MotorControlMode.POSITION_TRACK_MODE]:
            self._motor.setVelocity(fabs(vel_lim))
        self._iface._mutex.release()
    
    def set_current_limit(self, current_lim):
        """Set motor current limit.
        
        Args:
            current_lim: current limit(A)
        """
        self._iface._mutex.acquire()
        if fabs(current_lim) * self.torque_constant > self.available_current:
            raise ValueError('Current limit exceed available value.')
        self._motor.setAvailableTorque(fabs(current_lim) * self.torque_constant)
        self._iface._mutex.release()

    def set_pos(self, pos, vel_ff = 0, torque_ff = 0):
        """Set motor target position.
        
        Args:
            pos: target position(rad)
            vel_ff: velocity feed forward(rad/s)
            torque_ff:torque feed forward(Nm)
        """
        self._iface._mutex.acquire()
        pos = pos * copysign(1, self.reduction)
        self._input_pos = pos
        self._filter_controller.set_pos(pos)
        self._track_controller.set_pos(pos, self._pos_feedback, self._vel_feedback)
        self._iface._mutex.release()
    
    def set_abs_pos(self, pos, vel_ff = 0, torque_ff = 0):
        """Set motor target absolute position.
        
        Args:
            pos: target position(rad)
            vel_ff: velocity feed forward(rad/s)
            torque_ff:torque feed forward(Nm)
        """
        self._iface._mutex.acquire()
        pos = pos * copysign(1, self.reduction)
        self._input_pos = pos
        self._filter_controller.set_pos(pos)
        self._track_controller.set_pos(pos, self._pos_feedback, self._vel_feedback)
        self._iface._mutex.release()
    
    def set_vel(self, vel, torque_ff = 0):
        """Set motor velocity.
        
        Args:
            vel: target velocity(rad)
            torque_ff: torque feed forward(Nm)
        """
        self._iface._mutex.acquire()
        vel = vel * copysign(1, self.reduction)
        self._input_vel = vel
        self._iface._mutex.release()
    
    def set_torque(self, torque):
        """Set motor torque.
        
        Args:
            torque: target torque(Nm)
        """
        self._iface._mutex.acquire()
        torque = torque * copysign(1, self.reduction)
        self._input_torque = torque
        self._iface._mutex.release()
