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

"""Provides a CAN bus monitor tool."""

from queue import Queue

from rich.console import Console
from rich.table import Table
from rich.live import Live

from robodyno.interfaces import CanBus


class CanBusMonitor(object):
    """Monitors all messages on the CAN bus."""

    def __init__(self, can: CanBus, group: bool = False, console: Console = Console()):
        """Initializes the monitor.

        Args:
            can: The CAN bus to monitor.
            group: Whether to group messages by device and command.
            console: The console to use.
        """
        self._can = can
        self._group = group
        self._console = console
        self._msg_queue = Queue()

    def _msg_callback(self, data, us, device_id, command_id) -> None:
        data_list = [f'0x{byte:02X}' for byte in data]
        ts = f'{us // 1000000}.{int((us % 1000000) // 1000):03d}'
        self._msg_queue.put((ts, device_id, command_id, data_list))

    def _generate_table(self, data_dict: dict) -> Table:
        table = Table()
        table.add_column('Timestamp', style='green', justify='center', no_wrap=True)
        table.add_column('ID', style='cyan', justify='center', no_wrap=True)
        table.add_column('Command', style='magenta', justify='center', no_wrap=True)
        table.add_column('Data', style='yellow', justify='left', no_wrap=True)

        for (device_id, command_id), (ts, data_list) in data_dict.items():
            table.add_row(
                ts,
                f'0x{device_id:02X}',
                f'0x{command_id:02X}',
                ' '.join(data_list),
            )
        return table

    def _monitor_grouped(self) -> None:
        data_dict = {}

        with Live(
            self._generate_table(data_dict),
            auto_refresh=False,
            vertical_overflow='visible',
        ) as live:
            while True:
                if self._msg_queue.empty():
                    continue
                ts, device_id, command_id, data_list = self._msg_queue.get()
                data_dict[(device_id, command_id)] = (ts, data_list)
                live.update(self._generate_table(data_dict), refresh=True)

    def _monitor(self) -> None:
        while True:
            if self._msg_queue.empty():
                continue
            ts, device_id, command_id, data_list = self._msg_queue.get()
            self._console.print(
                f'[green not bold][{ts}][/] '
                f'[cyan not bold]0x{device_id:02X}[/] | '
                f'[magenta not bold]0x{command_id:02X}[/] '
                f'[yellow not bold][{" ".join(data_list)}][/]',
            )

    def monitor(self) -> None:
        """Starts monitoring the CAN bus.

        This method blocks until the user presses Ctrl-C.
        """
        self._can.subscribe(self._msg_callback)
        try:
            if self._group:
                self._monitor_grouped()
            else:
                self._monitor()
        except KeyboardInterrupt:
            self._can.unsubscribe(self._msg_callback)
