# -*-coding:utf-8 -*-
#
# Apache License, Version 2.0
#
# Copyright (c) 2023 Robottime(Beijing) Technology Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Imu sensor driver.

The imu sensor is a 6-axis inertial measurement unit (IMU) that provides
accurate acceleration and angular rate (gyroscope) measurement data for
space-constrained applications. This device also calculates and outputs
quaternions that can be used to track the absolute orientation of the sensor.

Examples:

    >>> from robodyno.components import ImuSensor
    >>> from robodyno.interfaces import CanBus
    >>> can = CanBus()
    >>> imu = ImuSensor(can)
    >>> imu.get_euler()
    (-0.6930452576214104, -0.5058813752928343, 0.33744507654497824)
    >>> can.disconnect()
    
"""

from math import pi, atan2, asin
import struct
from typing import Optional

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model


class ImuSensor(CanBusDevice):
    """Imu sensor driver.

    Attributes:
        id (int): Device id.
        type (Model): Device type.
        fw_ver (float): Firmware version.
    """

    _FC_HEARTBEAT = 0x02
    _FC_CONFIG_CAN = 0x03
    _FC_GET_ORIENTATION = 0x04
    _FC_GET_GYROSCOPE = 0x05
    _FC_GET_ACCELERATOR = 0x06
    _FC_SET_RANGES = 0x07

    def __init__(self, can: CanBus, id_: int = 0x31):
        """Initializes the imu sensor.

        Args:
            can (CanBus): Can bus instance.
            id_ (int): Device id. The default factory id is 0x31.

        Raises:
            ValueError: If the device is not a imu sensor.
        """
        super().__init__(can, id_)
        self.get_version(timeout=0.015)
        if self.type is None or self.type != Model.ROBODYNO_IMU_SENSOR:
            raise ValueError(f'Device of id {id_} is not a imu sensor')
        self._quaternion = (0, 0, 0, 1)
        self._euler = (0, 0, 0)
        self._gyro_factor = pi / 23592.96  # raw to rad/s
        self._accel_factor = 9.80665 / 16384.0  # raw to m/s^2
        self._can.subscribe(
            callback=self._heartbeat_callback,
            device_id=self.id,
            cmd_id=self._FC_HEARTBEAT,
        )

    def __del__(self):
        try:
            self._can.unsubscribe(
                callback=self._heartbeat_callback,
                device_id=self.id,
                cmd_id=self._FC_HEARTBEAT,
            )
        except KeyError:
            pass

    @property
    def quaternion(self) -> tuple:
        """Quaternion of imu sensor.

        Returns:
            (tuple): Quaternion of imu sensor.
        """
        return self._quaternion

    @property
    def euler(self) -> tuple:
        """Euler angles of imu sensor.

        Returns:
            (tuple): Euler angles of imu sensor.
        """
        return self._euler

    def _update_quaternion(self, value: tuple) -> None:
        """Updates the quaternion of imu sensor.

        Args:
            value (tuple): Quaternion of imu sensor.
        """
        self._quaternion = value
        self._euler = self.quat_to_euler(*value)

    @classmethod
    def quat_to_euler(cls, x: float, y: float, z: float, w: float) -> tuple:
        """Convert quaternion to euler angles.

        Args:
            x (float): x of quaternion.
            y (float): y of quaternion.
            z (float): z of quaternion.
            w (float): w of quaternion.

        Returns:
            (tuple): euler angles(roll, pitch, yaw).
        """
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = asin(2 * (w * y - x * z))
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return (roll, pitch, yaw)

    def _heartbeat_callback(self, data, timestamp, device_id, command_id) -> None:
        """Updates the imu sensor state."""
        del timestamp, device_id, command_id
        self._update_quaternion(struct.unpack('<eeee', data))

    def config_can_bus(
        self, new_id: int, heartbeat: int = 1000, bitrate=1000000
    ) -> None:
        """Configures the CAN bus settings.

        Args:
            new_id (int): The new device id.
            heartbeat (int): The heartbeat period in milliseconds.
            bitrate (int): The bitrate of the CAN bus. Choose from 250000,
                500000, 1000000.

        Raises:
            ValueError: If the new CAN id is not in the range of 0x01-0x3f.
        """
        if not 0x01 <= new_id <= 0x3F:
            raise ValueError('New CAN id must be in the range of 0x01-0x3f.')
        bitrate_id = {
            250000: 0,
            500000: 1,
            1000000: 2,
        }.get(bitrate, 2)
        self._can.send(
            self.id, self._FC_CONFIG_CAN, 'HHI', new_id, bitrate_id, int(heartbeat)
        )

    def set_ranges(self, gyro_range: int, accel_range: int) -> None:
        """Set gyro and accel range.

        Args:
            gyro_range: gyro range, 0: 250dps, 1: 500dps, 2: 1000dps, 3: 2000dps
            accel_range: accel range, 0: 2g, 1: 4g, 2: 8g, 3: 16g

        Raises:
            ValueError: If gyro_range or accel_range is not in the range of 0-3.
        """
        if not 0 <= gyro_range <= 3:
            raise ValueError('gyro_range must be in the range of 0-3.')
        if not 0 <= accel_range <= 3:
            raise ValueError('accel_range must be in the range of 0-3.')
        self._can.send(self.id, self._FC_SET_RANGES, 'HH', gyro_range, accel_range)

    def get_quaternion(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the quaternion of imu sensor.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (tuple | None): Quaternion of imu sensor. (x, y, z, w)
        """
        try:
            quat = self._can.get(self.id, self._FC_GET_ORIENTATION, 'eeee', timeout)
        except TimeoutError:
            return None
        self._update_quaternion(quat)
        return self._quaternion

    def get_euler(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the euler angles of imu sensor.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (tuple | None): Euler angles of imu sensor. (roll, pitch, yaw)
        """
        try:
            quat = self._can.get(self.id, self._FC_GET_ORIENTATION, 'eeee', timeout)
        except TimeoutError:
            return None
        self._update_quaternion(quat)
        return self._euler

    def get_gyro(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the gyro of imu sensor.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (tuple | None): Gyroscope of imu sensor. (gyro_x, gyro_y, gyro_z)
        """
        try:
            x, y, z, gyro_range = self._can.get(
                self.id, self._FC_GET_GYROSCOPE, 'hhhH', timeout
            )
        except TimeoutError:
            return None
        k = 2**gyro_range
        return (
            x * self._gyro_factor / k,
            y * self._gyro_factor / k,
            z * self._gyro_factor / k,
        )

    def get_accel(self, timeout: Optional[float] = None) -> Optional[tuple]:
        """Reads the accel of imu sensor.

        Args:
            timeout (float): Timeout in seconds.

        Returns:
            (tuple | None): Acceleration of imu sensor. (accel_x, accel_y, accel_z)
        """
        try:
            x, y, z, accel_range = self._can.get(
                self.id, self._FC_GET_ACCELERATOR, 'hhhH', timeout
            )
        except TimeoutError:
            return None
        k = 2**accel_range
        return (
            x * self._accel_factor / k,
            y * self._accel_factor / k,
            z * self._accel_factor / k,
        )
