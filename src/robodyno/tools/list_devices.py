#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""list.py
Time    :   2022/10/17
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

List robodyno devices on can bus.
"""

from .utils import colored
from ..components import CanBusDevice

def get_device_version(bus, id):
    """Read robodyno device version from bus with id
    
    Args:
        bus: robodyno can bus
        id: robodyno device id
    
    Returns:
        robodyno device version dict
    """
    device = CanBusDevice(bus, id)
    return device.get_version(timeout = 0.1)

def display_ver(id, ver):
    """Display robodyno device version
    
    Args:
        id: robodyno device id
        ver: robodyno device version dict
    """
    print(colored('[0x{:02X}]'.format(id), 'cyan') + ' ' +
          colored(ver['type'].name, 'green') + ', ' +
          'ver: {}.{}'.format(ver['main_version'], ver['sub_version']))

def list_devices(can, id_list):
    """Print all device and version with id selected on can bus.
    
    Args:
        can: can bus interface
        id_list: selected id list
    """
    if len(id_list) > 0:
        for id in id_list:
            ver = get_device_version(can, id)
            if ver:
                display_ver(id, ver)
    else:
        for id in range(1, 0x40):
            ver = get_device_version(can, id)
            if ver:
                display_ver(id, ver)
    can.disconnect()