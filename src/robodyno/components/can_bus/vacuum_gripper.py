#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""vacuum_gripper.py
Time    :   2023/01/30
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno vacuum gripper can bus driver

  Typical usage example:

  from robodyno.interfaces import CanBus
  from robodyno.components import VGripper

  can = CanBus()
  
  gripper = VGripper(iface = can)
  gripper.on()
  gripper.off()
  gripper.set_pwm(127)

  can.disconnect()
"""

from robodyno.interfaces import CanBus
from robodyno.components import DeviceType, CanBusDevice

class VGripper(CanBusDevice):
    """Robodyno Vacuum Gripper Driver

    Attributes:
        id: can bus device id.
        type: device type enum
        fw_ver: firmware version
    """

    CMD_SET_NODE_ID = 0x02
    CMD_SET_PWM = 0x04

    def __init__(self, iface, id = 0x21, type = None):
        """Init vacuum gripper from interface, id and type
        
        Args:
            iface: robodyno interface
            id: range from 0x01 to 0x40, default 0x21
            type: vacuum gripper type(ROBODYNO_PWM_DRIVER / ROBODYNO_VACUUM_GRIPPER)
        """
        super().__init__(iface, id)
        self.get_version(timeout=0.15)
        if type:
            self.type = DeviceType[type]
        if self.type not in [DeviceType.ROBODYNO_PWM_DRIVER, DeviceType.ROBODYNO_VACUUM_GRIPPER]:
            raise ValueError('Vacuum gripper type is invalid and can not distinguish automatically.')
        self.status = False
        self.off()

    @CanBus.send_to_bus(CMD_SET_NODE_ID, '<B')
    def config_can_bus(self, new_id):
        """Change gripper device id.
        
        Args:
            new_id: gripper new device id
        """
        return (new_id,)

    @CanBus.send_to_bus(CMD_SET_PWM, '<B')
    def set_pwm(self, pwm):
        """Change gripper strength.
        
        Args:
            pwm: range from 0 to 255
        """
        return (pwm,)

    def on(self):
        """Turn on vacuum gripper."""
        self.set_pwm(255)
        self.status = True
    
    def off(self):
        """Turn off vacuum gripper."""
        self.set_pwm(0)
        self.status = False

