# -*-coding:utf-8 -*-
#
# The MIT License (MIT)
#
# Copyright (c) 2023 Robottime(Beijing) Technology Co., Ltd
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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
        ts = f'{us // 1000000}.{((us % 1000000) // 1000):03d}'
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
                ts, device_id, command_id, data_list = self._msg_queue.get()
                data_dict[(device_id, command_id)] = (ts, data_list)
                live.update(self._generate_table(data_dict), refresh=True)

    def _monitor(self) -> None:
        while True:
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
