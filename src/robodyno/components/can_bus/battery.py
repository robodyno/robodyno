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

"""Battery driver.

The battery driver can be used to get the battery voltage and current.

Examples:

    >>> from robodyno.components import Battery
    >>> from robodyno.interfaces import CanBus
    >>> can = CanBus()
    >>> battery = Battery(can)
    >>> print(battery)
    Battery(id=0x0F, voltage=27.66, current=0.0, error=0, rsoc=0.88, fet=3)
"""

from struct import unpack
from typing import Optional
from datetime import datetime

from robodyno.interfaces import CanBus
from robodyno.components import CanBusDevice
from robodyno.components.config.model import Model


class Battery(CanBusDevice):
    """Battery driver.

    Attributes:
        id (int): Device id.
        type (Model): Device type.
        fw_ver (float): Firmware version.
        voltage (float): Battery voltage.
        current (float): Battery current.
        error (int): Error code.
        rsoc (float): Relative state of charge.
        fet (int): FET status.
    """

    _CMD_HEARTBEAT = 0x02
    _CMD_CONFIG_CAN = 0x03
    _CMD_GET_VOLTAGE = 0x04
    _CMD_GET_CURRENT = 0x05
    _CMD_GET_CAPACITY = 0x06
    _CMD_GET_CYTLE_COUNT = 0x07
    _CMD_GET_PRODUCTION_DATE = 0x08
    _CMD_GET_BALANCE_STATUS = 0x09
    _CMD_GET_PROTECTION_STATUS = 0x0A
    _CMD_GET_HARDWARE_VERSION = 0x0B
    _CMD_GET_RSOC = 0x0C
    _CMD_GET_FET = 0x0D
    _CMD_GET_STRING_COUNT = 0x0E
    _CMD_GET_NTC = 0x0F

    def __init__(self, can: CanBus, id_: int = 0x0F):
        """Initializes the battery driver.

        Args:
            can (CanBus): Can bus instance.
            id_ (int): Device id.

        Raises:
            ValueError: If the device id is invalid.
        """
        super().__init__(can, id_)
        self.get_version(timeout=0.015)
        if self.type is None or self.type != Model.ROBODYNO_BATTERY:
            raise ValueError('The device is not a battery supported by robodyno.')
        self.voltage = 0.0
        self.current = 0.0
        self.error = 0
        self.rsoc = 0.0
        self.fet = 0

        self._can.subscribe(
            self._on_hearbeat,
            device_id=self.id,
            cmd_id=self._CMD_HEARTBEAT,
        )

    def __repr__(self) -> str:
        """Gets the string representation of the battery driver.

        Returns:
            str: String representation of the battery driver.
        """
        return (
            f'Battery(id=0x{self.id:02X}, voltage={self.voltage}, '
            f'current={self.current}, error={self.error}, '
            f'rsoc={self.rsoc}, fet={self.fet})'
        )

    def _on_hearbeat(self, data, timestamp, device_id, command_id) -> None:
        """Handles the heartbeat message."""
        del timestamp, device_id, command_id
        voltage, current, error, rsoc, fet = unpack('<HhHBB', data)
        self.voltage = voltage / 100.0
        self.current = current / 100.0
        self.error = error
        self.rsoc = rsoc / 100.0
        self.fet = fet

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
            self.id, self._CMD_CONFIG_CAN, 'HHI', new_id, bitrate_id, int(heartbeat)
        )

    def get_voltage(self, timeout: Optional[float] = None) -> float:
        """Reads the voltage of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): the voltage(V). Returns None if times out.
        """
        try:
            (voltage,) = self._can.get(self.id, self._CMD_GET_VOLTAGE, 'H', timeout)
        except TimeoutError:
            return None
        return voltage / 100.0

    def get_current(self, timeout: Optional[float] = None) -> float:
        """Reads the current of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): the current(A). Positive value means charging.
                Returns None if times out.
        """
        try:
            (current,) = self._can.get(self.id, self._CMD_GET_CURRENT, 'h', timeout)
        except TimeoutError:
            return None
        return current / 100.0

    def get_capacity(self, timeout: Optional[float] = None) -> tuple:
        """Reads the capacity of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): (remaining capacity(Ah), standard_capacity).
                Returns None if times out.
        """
        try:
            (remaining, standard) = self._can.get(
                self.id, self._CMD_GET_CAPACITY, 'HH', timeout
            )
        except TimeoutError:
            return None
        return (remaining / 100.0, standard / 100.0)

    def get_cycle_count(self, timeout: Optional[float] = None) -> int:
        """Reads the cycle count of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (int | None): the cycle count. Returns None if times out.
        """
        try:
            (cycle_count,) = self._can.get(
                self.id, self._CMD_GET_CYTLE_COUNT, 'H', timeout
            )
        except TimeoutError:
            return None
        return cycle_count

    def get_production_date(self, timeout: Optional[float] = None) -> datetime:
        """Reads the production date of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (datetime | None): the production date. Returns None if times out.
        """
        try:
            (time_data,) = self._can.get(
                self.id, self._CMD_GET_PRODUCTION_DATE, 'H', timeout
            )
        except TimeoutError:
            return None
        year = time_data >> 9
        month = (time_data >> 5) & 0x0F
        day = time_data & 0x1F
        return datetime(year, month, day)

    def get_balance_status(self, timeout: Optional[float] = None) -> int:
        """Reads the balance status of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (int | None): the balance status. Returns None if times out.
        """
        try:
            (status,) = self._can.get(
                self.id, self._CMD_GET_BALANCE_STATUS, 'I', timeout
            )
        except TimeoutError:
            return None
        return status

    def get_protection_status(self, timeout: Optional[float] = None) -> int:
        """Reads the protection status of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (int | None): the protection status. Returns None if times out.
        """
        try:
            (status,) = self._can.get(
                self.id, self._CMD_GET_PROTECTION_STATUS, 'H', timeout
            )
        except TimeoutError:
            return None
        return status

    def get_hardware_version(self, timeout: Optional[float] = None) -> str:
        """Reads the hardware version of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (str | None): the hardware version. Returns None if times out.
        """
        try:
            (version,) = self._can.get(
                self.id, self._CMD_GET_HARDWARE_VERSION, 'B', timeout
            )
        except TimeoutError:
            return None
        return f'{version >> 4}.{version & 0x0F}'

    def get_rsoc(self, timeout: Optional[float] = None) -> float:
        """Reads the rsoc of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (float | None): the remaining state of charge. Returns None if times out.
        """
        try:
            (rsoc,) = self._can.get(self.id, self._CMD_GET_RSOC, 'B', timeout)
        except TimeoutError:
            return None
        return rsoc / 100.0

    def get_fet(self, timeout: Optional[float] = None) -> int:
        """Reads the fet status of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (int | None): the fet status. Returns None if times out.
        """
        try:
            (fet,) = self._can.get(self.id, self._CMD_GET_FET, 'B', timeout)
        except TimeoutError:
            return None
        return fet

    def get_string_count(self, timeout: Optional[float] = None) -> int:
        """Reads the string count of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (int | None): the string count. Returns None if times out.
        """
        try:
            (string_count,) = self._can.get(
                self.id, self._CMD_GET_STRING_COUNT, 'B', timeout
            )
        except TimeoutError:
            return None
        return string_count

    def get_temperature(self, timeout: Optional[float] = None) -> tuple:
        """Reads the temperature of the battery.

        Args:
            timeout (float): The timeout in seconds.

        Returns:
            (tuple | None): the temperatures. Returns None if times out.
        """
        try:
            (_, temp1, temp2) = self._can.get(
                self.id, self._CMD_GET_NTC, 'HHH', timeout
            )
        except TimeoutError:
            return None
        return ((temp1 - 2731) / 10.0, (temp2 - 2731) / 10.0)
