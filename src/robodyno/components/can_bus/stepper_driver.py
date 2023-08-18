#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""stepper_driver.py
Time    :   2023/03/31
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno stepper driver can bus driver

  Typical usage example:

  from robodyno.interfaces import CanBus
  from robodyno.components import StepperDriver

  can = CanBus()
  
  stepper = StepperDriver(iface = can)
  stepper.enable()
  stepper.set_vel(1)

  can.disconnect()
"""

from math import pi
from robodyno.interfaces import CanBus
from robodyno.components import DeviceType, CanBusDevice

class StepperDriver(CanBusDevice):
    """Robodyno stepper driver

    Attributes:
        id: can bus device id.
        type: device type enum
        fw_ver: firmware version
    """

    CMD_SET_NODE_ID = 0x02
    CMD_SET_SUBDIVISION = 0x03
    CMD_SET_VEL_ACC_LIMIT = 0x04
    CMD_ENABLE = 0x05
    CMD_DISABLE = 0x06
    CMD_STOP = 0x07
    CMD_SET_POSITION = 0x08
    CMD_SET_VELOCITY = 0x09
    CMD_GET_POSITION = 0x0a
    CMD_GET_VELOCITY = 0x0b

    def __init__(self, iface, id = 0x22, type = None, reduction = 10):
        """Init stepper driver from interface, id and type
        
        Args:
            iface: robodyno interface
            id: range from 0x01 to 0x40, default 0x21
            type: stepper driver type(ROBODYNO_STEPPER_DRIVER)
        """
        super().__init__(iface, id)
        self.get_version(timeout=0.15)
        if type:
            self.type = DeviceType[type]
        if self.type != DeviceType.ROBODYNO_STEPPER_DRIVER:
            raise ValueError('Stepper Driver type is invalid and can not distinguish automatically.')
        self._reduction = reduction
        self._set_subdivision(8)
    
    def _set_subdivision(self, subdivision):
        """Validate stepper subdivision and set locally
        
        Args:
            subdivision: 8/16/32/64
        
        Returns:
            True if subdivision is valid else False
        """
        if subdivision in [8, 16, 32, 64]:
            self._subdivision = subdivision
            self._factor = 200.0 * subdivision * self._reduction / 2.0 / pi
            return True
        return False

    @CanBus.send_to_bus(CMD_SET_NODE_ID, '<B')
    def config_can_bus(self, new_id):
        """Change driver device id.
        
        Args:
            new_id: node new device id
        """
        return (new_id,)
    
    @CanBus.send_to_bus(CMD_SET_SUBDIVISION, '<B')
    def set_subdivision(self, subdivision):
        """Cange sub division of stepper
        
        Args:
            subdivision: 8/16/32/64
        """
        if not self._set_subdivision(subdivision):
            raise ValueError('Subdivision is invalid.')
        return (subdivision,)

    @CanBus.send_to_bus(CMD_SET_VEL_ACC_LIMIT, '<ff')
    def set_vel_acc_limit(self, max_vel, acc = None):
        """Set stepper vel limit and acceleration
        
        Args:
            max_vel: max rotate speed, rad/s
            acc: acceleration, rad/s^2, default: max_vel * 4
        """
        if acc is None:
            acc = max_vel * 4
        return (max_vel * self._factor, acc * self._factor)
    
    @CanBus.send_to_bus(CMD_ENABLE)
    def enable(self):
        """Enable stepper motor."""
        pass

    @CanBus.send_to_bus(CMD_DISABLE)
    def disable(self):
        """Disable stepper motor."""
        pass

    @CanBus.send_to_bus(CMD_STOP)
    def stop(self):
        """Stop stepper motor."""
        pass

    @CanBus.send_to_bus(CMD_SET_POSITION, '<i')
    def set_pos(self, pos):
        """Set stepper motor position.
        
        Args:
            pos: target position, rad
        """
        return (int(pos * self._factor),)
        
    @CanBus.send_to_bus(CMD_SET_VELOCITY, '<f')
    def set_vel(self, vel):
        """Set stepper motor velocity.
        
        Args:
            vel: target velocity, rad/s
        """
        return (vel * self._factor,)
    
    @CanBus.get_from_bus(CMD_GET_POSITION, '<i')
    def get_pos(self, pos):
        """Get stepper motor position.
        
        Returns:
            motor current position, rad
        """
        return pos / self._factor
    
    @CanBus.get_from_bus(CMD_GET_VELOCITY, '<f')
    def get_vel(self, vel):
        """Get stepper motor velocity.
        
        Returns:
            motor current velocity, rad/s
        """
        return vel / self._factor
