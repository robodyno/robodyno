#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""slider_module.py
Time    :   2023/01/30
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno slider module can bus driver

  Typical usage example:

  from robodyno.interfaces import CanBus
  from robodyno.components import SliderModule

  can = CanBus()
  
  slider = SliderModule(can, id = 0x10, type = 'ROBODYNO_PRO_44', max_vel = 0.02)

  can.disconnect()
"""

import time
from math import fabs, pi
from robodyno.components.can_bus.motor import Motor

class SliderModule():
    """Robodyno slider module driver.
    
    Attributes:
        motor: robodyno motor integrated in module
        max_vel: slider's max velocity (m/s)
        LEAD: linear travel the nut makes per one screw revolution (m)
    """

    def __init__(self, iface, id = 0x10, type = None, max_vel = 0.02, *args, **kwargs):
        """Init slider module from interface, motor id, motor type, slider's max vel
        
        Args:
            iface: robodyno interface
            id: robodyno motor id
            type: robodyno motor type
            max_vel: slider
        """
        self.LEAD = 0.01
        self.motor = Motor(iface, id, type)
        self._reduction = self.LEAD / 2 / pi # mm/rad
        self.set_max_vel(max_vel)
    
    def set_max_vel(self, max_vel):
        """Change slider's max velocity.
        
        Args:
            max_vel: slider's max velocity, should be always positive (m/s)
        """
        self.max_vel = fabs(max_vel)
        self.motor.position_track_mode(self.max_vel / self._reduction, 40, 40)
    
    def enable(self):
        """Enable slider module."""
        self.motor.enable()

    def disable(self):
        """Disable slider module."""
        self.motor.disable()

    def set_pos(self, pos):
        """Set slider's position.
        
        Args:
            pos: slider's position (m)
        """
        self.motor.set_pos(pos / self._reduction)
        
    def set_abs_pos(self, pos):
        """Set slider's absolute position.
        
        Args:
            pos: slider's absolute position (m)
        """
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

    def init_pos(self, offset = 0):
        """Init slider pos with current position
        
        Args:
            offset: current position (m)
        """
        self.motor.init_pos(offset / self._reduction)

    def init_abs_pos(self, offset = 0, save = True):
        """Init slider absolute pos with current position
        
        Args:
            offset: current absolute position (m)
        """
        self.motor.init_abs_pos(offset / self._reduction)
        if save:
            self.motor.save_configuration()
            time.sleep(0.1)
