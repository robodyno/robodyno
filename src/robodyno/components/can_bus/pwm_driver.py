#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""pwm_driver.py
Time    :   2023/03/31
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno pwm driver can bus driver

  Typical usage example:

  from robodyno.interfaces import CanBus
  from robodyno.components import PwmDriver

  can = CanBus()
  
  pwm = PwmDriver(iface = can)
  pwm.set_servo(255)
  pwm.set_pwm(127)

  can.disconnect()
"""

from robodyno.interfaces import CanBus
from robodyno.components import DeviceType, CanBusDevice

class PwmDriver(CanBusDevice):
    """Robodyno pwm driver

    Attributes:
        id: can bus device id.
        type: device type enum
        fw_ver: firmware version
    """

    CMD_SET_NODE_ID = 0x02
    CMD_SET_SERVO = 0x03
    CMD_SET_PWM = 0x04

    def __init__(self, iface, id = 0x21, type = None):
        """Init pwm driver from interface, id and type
        
        Args:
            iface: robodyno interface
            id: range from 0x01 to 0x40, default 0x21
            type: pwm driver type(ROBODYNO_PWM_DRIVER)
        """
        super().__init__(iface, id)
        self.get_version(timeout=0.15)
        if type:
            self.type = DeviceType[type]
        if self.type != DeviceType.ROBODYNO_PWM_DRIVER:
            raise ValueError('Pwm Driver type is invalid and can not distinguish automatically.')

    @CanBus.send_to_bus(CMD_SET_NODE_ID, '<B')
    def config_can_bus(self, new_id):
        """Change driver device id.
        
        Args:
            new_id: node new device id
        """
        return (new_id,)

    @CanBus.send_to_bus(CMD_SET_SERVO, '<B')
    def set_servo(self, pos):
        """Set servo pos
        
        Args:
            pos: range from 0 to 255
        """
        return (pos,)

    @CanBus.send_to_bus(CMD_SET_PWM, '<B')
    def set_pwm(self, pwm):
        """Set pwm
        
        Args:
            pwm: range from 0 to 255
        """
        return (pwm,)
