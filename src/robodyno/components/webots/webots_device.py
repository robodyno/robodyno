#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""webots_device.py
Time    :   2023/01/05
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Webots device node base class.
"""

from enum import Enum
from robodyno.interfaces import Webots

class WebotsDevice(object):
    """Webots common device node."""
    
    def __init__(self, iface, id = 0x10, auto_register = True):
        """Init node with webots interface and id.
        
        Args:
            iface: webots interface
            id: node id
            type: robodyno device type name
            auto_register: auto register self to webots interface
        """
        self.id = id
        if not isinstance(iface, Webots):
            raise ValueError('Use a webots interface to init a webots device.')
        self._iface = iface
        if auto_register:
            iface.register(self)
    
    def __del__(self):
        """Collect node from memory."""
        self._iface.deregister(self)
    
    def get_version(self):
        """Get device simulation version.
        
        Returns:
            dictionary of device version
        """
        return {
            'main_version': 4,
            'sub_version': 0,
            'type': self.type
        }
    
    def parallel_update(self):
        """Update parallel with simulation step."""
        pass

    def update(self):
        """after simulation step update callback."""
        pass