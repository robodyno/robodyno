#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""monitor.py
Time    :   2022/10/17
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Monitor messages on can bus.
"""

import threading
from .utils import colored

rx_lock = threading.Lock()

def rx_callback(device_id, command_id, data, timestamp):
    """Thread safe can bus rx callback, print message to console.
    
    Args:
        device_id: robodyno device id
        command_id: message command id
        data: message payload
        timestamp: time when message received
    """
    with rx_lock:
        payload = ''
        for byte in data:
            payload += '0x{:02x} '.format(byte)
        if len(payload) > 0:
            payload = payload[:-1]
        print('[{time}] {dev_id} | {cmd_id} [{payload}]'.format(
            time = '{}.{}'.format(int(timestamp/1000000), int(timestamp%1000000/1000)),
            dev_id = colored('0x{:02X}'.format(device_id), 'cyan'),
            cmd_id = colored('0x{:02X}'.format(command_id), 'magenta'),
            payload = colored(payload, 'yellow'),
        ))

def monitor(can, id_list):
    """Display messages from can bus to console.
    
    Args:
        can: can bus interface
        id_list: selected id list
    """
    if len(id_list) > 0:
        for id in id_list:
            can.subscribe(id, can.COMMAND_ID_ALL, rx_callback)
    else:
        can.subscribe(can.DEVICE_ID_ALL, can.COMMAND_ID_ALL, rx_callback)
    
    try:
        while True:
            pass
    except KeyboardInterrupt:
        if len(id_list) > 0:
            for id in id_list:
                can.unsubscribe(id, can.COMMAND_ID_ALL, rx_callback)
        else:
            can.unsubscribe(can.DEVICE_ID_ALL, can.COMMAND_ID_ALL, rx_callback)
        can.disconnect()
