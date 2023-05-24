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

"""Provides a function to generate a table of motor information."""

from time import time
from enum import Enum

from rich.table import Table
from rich.style import Style

from robodyno.components.can_bus.motor import Motor

def error_text(error: Enum) -> str:
    """Returns a rich text object for an error.

    Args:
        error (Enum): The error to get text for.

    Returns:
        (string): The rich text object.
    """
    if error is None:
        return ''
    if error.value:
        return f'[red]{error.name}[/]'
    return f'[green]{error.name}[/]'


def motor_info_table(motor: Motor) -> Table:
    """Returns a table of motor information.

    Args:
        motor (Motor): The motor to get information from.
    Returns:
        (Table): The table of motor information.
    """
    table = Table(
        title=(
            f'[cyan][0x{motor.id:02X}][/] '
            f'[green]{motor.type.name}[/] '
            f'[yellow]{motor.fw_ver}[/]'
        ),
        title_style=Style(italic=False, bold=True)
    )
    table.add_column('Property', style='cyan', justify='left', no_wrap=True)
    table.add_column('Value', style='green', justify='right', no_wrap=True)
    table.add_column(style='yellow', justify='left', no_wrap=True)

    if motor.fw_ver < 1:
        sync_time = motor.sync_time
        t = time()
        timeout = 1
        while motor.sync_time == sync_time and time() - t < timeout:
            pass
        state = motor.state
        error = motor.error
        mode = motor.mode
    else:
        state, error, mode = motor.get_state(timeout = 0.1)
    error_detail = None
    if error['error'] == motor.MotorError.MOTOR_FAILED:
        error_detail = error['motor_err']
    elif error['error'] == motor.MotorError.ENCODER_FAILED:
        error_detail = error['encoder_err']
    elif error['error'] == motor.MotorError.CONTROLLER_FAILED:
        error_detail = error['controller_err']
    table.add_row('State', f'{state.name}', '')
    table.add_row('Error', error_text(error['error']), error_text(error_detail))
    table.add_row('Mode', f'{mode.name}', '')

    voltage = motor.get_voltage(timeout = 0.1)
    table.add_row('Voltage', f'{voltage:.2f}', 'V')
    temperature = motor.get_temperature(timeout = 0.1)
    table.add_row('Temperature', f'{temperature:.2f}', '℃')

    position = motor.get_pos(timeout = 0.1)
    table.add_row('Position', f'{position:.4f}', 'rad')
    if motor.fw_ver >= 1:
        absolute_position = motor.get_abs_pos(timeout = 0.1)
        table.add_row('Absolute Position', f'{absolute_position:.4f}', 'rad')
    velocity = motor.get_vel(timeout = 0.1)
    table.add_row('Velocity', f'{velocity:.4f}', 'rad/s')
    torque = motor.get_torque(timeout = 0.1)
    table.add_row('Torque', f'{torque:.4f}', 'N*m')
    return table
