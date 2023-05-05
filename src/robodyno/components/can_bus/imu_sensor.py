#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""imu_sensor.py
Time    :   2023/04/20
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Robodyno IMU sensor

  Typical usage example:

  from robodyno.interfaces import CanBus
  from robodyno.components import ImuSensor

  can = CanBus()
  imu = ImuSensor(can)
  imu.get_gyro(timeout = 1)

  can.disconnect()
"""

import math
import struct

from robodyno.interfaces import CanBus
from robodyno.components import DeviceType, CanBusDevice

class ImuSensor(CanBusDevice):
    """Robodyno IMU sensor
    
    Attributes:
        id: can bus device id
        type: robodyno device type
        fw_ver: firmware version
    """
    
    FC_HEARTBEAT = 0x02
    FC_CONFIG_CAN = 0x03
    FC_GET_ORIENTATION = 0x04
    FC_GET_GYROSCOPE = 0x05
    FC_GET_ACCELERATOR = 0x06
    FC_SET_RANGES = 0x07

    def __init__(self, iface, id = 0x31):
        """Init imu sensor node
        
        Args:
            iface: can bus interface
            id: robodyno node id, default = 0x31
            quaternion: imu orientation in quaternion
            euler: imu orientation in rpy
        """
        super().__init__(iface, id)
        self.get_version(timeout = 0.2)
        if not self.type or self.type != DeviceType.ROBODYNO_IMU_SENSOR:
            raise TypeError('There is not a imu driver on can bus with id 0x{:02X}.'.format(id))
        self.quaternion = (0,0,0,1)
        self.euler = (0, 0, 0)
        self._gyro_factor = math.pi / 23592.96 # raw to rad/s
        self._accel_factor = 9.80665 / 16384.0 # raw to m/s^2
        self._iface.subscribe(
            device_id = self.id, 
            command_id = self.FC_HEARTBEAT, 
            callback = self._heartbeat_callback
        )

    def __del__(self):
        """Collect node from memory."""
        self._iface.unsubscribe(
            device_id = self.id, 
            command_id = self.FC_HEARTBEAT, 
            callback = self._heartbeat_callback
        )

    @classmethod
    def quat_to_euler(cls, x, y, z, w):
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return (roll, pitch, yaw)

    def _heartbeat_callback(self, device_id, command_id, data, timestamp):
        """Update orientation from heartbeat."""
        self.quaternion = struct.unpack('<eeee', data)
        self.euler = self.quat_to_euler(*self.quaternion)

    @CanBus.send_to_bus(FC_CONFIG_CAN, '<HHI')
    def config_can_bus(self, new_id, heartbeat = 1000, bitrate = 'CAN_1M'):
        """Configure can bus settings.
        
        Args:
            new_id: device id(0x01-0x3f)
            heartbeat: heartbeat period(ms)
            bitrate: choose from 'CAN_1000K', 'CAN_500K', 'CAN_250K'
        """
        if new_id < 0x01 or new_id > 0x3f:
            raise ValueError('Can id is not available. Choose from 0x01-0x3f')

        bitrate = CanBus.CanSpeed[bitrate].value
        if bitrate == 250000:
            bitrate_id = 0
        elif bitrate == 500000:
            bitrate_id = 1
        else:
            bitrate_id = 2
        return (new_id, bitrate_id, int(heartbeat))
    
    @CanBus.get_from_bus(FC_GET_ORIENTATION, '<eeee')
    def get_quaternion(self, x, y, z, w):
        """Query Quaternion from imu.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            imu quaternion(x, y, z, w)
        """
        self.quaternion = (x, y, z, w)
        self.euler = self.quat_to_euler(x, y, z, w)
        return self.quaternion
    
    @CanBus.get_from_bus(FC_GET_ORIENTATION, '<eeee')
    def get_euler(self, x, y, z, w):
        """Query euler angle from imu.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            imu euler angle(roll, pitch, yaw)
        """
        self.quaternion = (x, y, z, w)
        self.euler = self.quat_to_euler(x, y, z, w)
        return self.euler
    
    @CanBus.get_from_bus(FC_GET_GYROSCOPE, '<hhhH')
    def get_gyro(self, x, y, z, range):
        """Query gyroscope from imu.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            imu gyroscope(gyro_x, gyro_y, gyro_z) in rad/s
        """
        k = 2 ** range
        return (x * self._gyro_factor / k, y * self._gyro_factor / k, z * self._gyro_factor / k)
        
    @CanBus.get_from_bus(FC_GET_ACCELERATOR, '<hhhH')
    def get_accel(self, x, y, z, range):
        """Query acceleration from imu.
        
        Args:
            timeout: 0 indicates unlimited timeout(s)
        
        Returns:
            imu acceleration(accel_x, accel_y, accel_z) in m/s^2
        """
        k = 2 ** range
        return (x * self._accel_factor / k, y * self._accel_factor / k, z * self._accel_factor / k)
        
    @CanBus.send_to_bus(FC_SET_RANGES, '<HH')
    def set_ranges(self, gyro_range, accel_range):
        return (gyro_range, accel_range)