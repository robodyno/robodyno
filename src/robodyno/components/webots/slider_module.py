#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""slider_module.py
Time    :   2023/04/02
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno slider module can bus driver

  Typical usage example:

  from robodyno.interfaces import Webots
  from robodyno.components import SliderModule

  webots = Webots()
  
  slider = SliderModule(webots, id = 0x10, max_vel = 0.02)
"""

import time
from math import fabs, pi
from robodyno.components.webots.motor import Motor

class SliderModule():
    """Robodyno slider module driver.
    
    Attributes:
        motor: robodyno motor integrated in module
        max_vel: slider's max velocity (m/s)
        LEAD: linear travel the nut makes per one screw revolution (m)
    """

    def __init__(self, iface, id = 0x10, type = None, max_vel = 0.02, twin = None, *args, **kwargs):
        """Init slider module from interface, motor id, motor type, slider's max vel
        
        Args:
            iface: robodyno interface
            id: robodyno motor id
            type: robodyno motor type
            max_vel: slider
        """
        self.LEAD = 0.01
        self.motor = Motor(iface, id, type, twin)
        self._reduction = self.LEAD / 2 / pi # mm/rad
        if self.motor.mode != self.motor.MotorControlMode.TWIN_MODE:
            self.set_max_vel(max_vel)
    
    def set_max_vel(self, max_vel):
        """Change slider's max velocity.
        
        Args:
            max_vel: slider's max velocity, should be always positive (m/s)
        """
        self.max_vel = fabs(max_vel)
        if self.motor.mode != self.motor.MotorControlMode.TWIN_MODE:
            self.motor.position_track_mode(self.max_vel / self._reduction, 40, 40)
    
    def enable(self):
        """Enable slider module."""
        if self.motor.mode != self.motor.MotorControlMode.TWIN_MODE:
            self.motor.enable()

    def disable(self):
        """Disable slider module."""
        if self.motor.mode != self.motor.MotorControlMode.TWIN_MODE:
            self.motor.disable()

    def set_pos(self, pos):
        """Set slider's position.
        
        Args:
            pos: slider's position (m)
        """
        if self.motor.mode != self.motor.MotorControlMode.TWIN_MODE:
            self.motor.set_pos(pos / self._reduction)
        
    def set_abs_pos(self, pos):
        """Set slider's absolute position.
        
        Args:
            pos: slider's absolute position (m)
        """
        if self.motor.mode != self.motor.MotorControlMode.TWIN_MODE:
            self.motor.set_abs_pos(pos / self._reduction)

    def get_pos(self, timeout = 0):
        """Read slider's position.
        
        Args:
            timeout: 0 indicates unlimited timeout (s)
        
        Returns:
            position (m) / None if timeout
        """
        pos = self.motor.get_pos(timeout)
        return None if pos is None else pos * self._reduction

    def get_abs_pos(self, timeout = 0):
        """Read slider's absolute position.
        
        Args:
            timeout: 0 indicates unlimited timeout (s)
        
        Returns:
            absolute position (m) / None if timeout
        """
        pos = self.motor.get_abs_pos(timeout)
        return None if pos is None else pos * self._reduction
