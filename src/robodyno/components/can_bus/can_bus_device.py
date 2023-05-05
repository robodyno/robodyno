#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""can_bus_device.py
Time    :   2023/01/02
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Can bus device base class.
"""

from enum import Enum
from robodyno.components import DeviceType
from robodyno.interfaces import CanBus

class CanBusDevice(object):
    """Can bus robodyno common device node."""
    
    CMD_GET_VERSION = 0x01

    def __init__(self, iface, id = 0x10):
        """Init device with interface and id
        
        Args:
            iface: can bus interface
            id: device id
        """
        if id < 0x01 or id >= 0x40:
            raise ValueError('Use a valid device id range from 0x01 to 0x40.')
        if not isinstance(iface, CanBus):
            raise ValueError('Use a can bus interface to init a can bus device.')
        self._iface = iface
        self.id = id
        self.fw_ver = None
        self.type = DeviceType.ROBODYNO_THIRD_PARTY
    
    @CanBus.get_from_bus(CMD_GET_VERSION, '<HHI')
    def get_version(self, main_ver, sub_ver, type):
        """Get device firmware version.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            dictionary of device version
            None if timeout
        """
        self.fw_ver = float('{}.{}'.format(main_ver, sub_ver))
        self.type = DeviceType(type)
        return {
            'main_version': main_ver,
            'sub_version': sub_ver,
            'type': DeviceType(type)
        }
