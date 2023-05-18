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

"""Provides a cli for robodyno devices."""

from time import sleep

import click
from rich.console import Console
from rich.table import Table
from rich.columns import Columns
from rich.traceback import install

from robodyno import __version__
from robodyno.interfaces import CanBus
from robodyno.components.can_bus.motor import Motor
from robodyno.components.config.model import Model
from robodyno.tools.can_bus_monitor import CanBusMonitor
from robodyno.tools.motor_info_table import motor_info_table
from robodyno.tools.cli_param_types import ID_LIST, BASED_INT

console = Console()


@click.group()
@click.version_option(__version__)
@click.option('--can-bus', '-c', default='can0', help='CAN bus interface channel.')
@click.option('--baud-rate', '-b', default=1000000, help='CAN bus baud rate.')
@click.pass_context
def cli(ctx: click.Context, can_bus: str, baud_rate: int) -> None:
    """Robodyno command line interface."""
    install(suppress=[click])
    ctx.ensure_object(dict)
    ctx.obj['can'] = CanBus(baud_rate, can_bus)


@cli.command(name='list')
@click.pass_context
def list_devices(ctx: click.Context) -> None:
    """List all devices on the CAN bus."""
    table = Table()
    table.add_column('ID', style='cyan', justify='right', no_wrap=True)
    table.add_column('Model', style='green', justify='left', no_wrap=True)
    table.add_column('Version', style='yellow', justify='left', no_wrap=True)

    for device_id in range(0x3F):
        try:
            main_ver, sub_ver, type_ = ctx.obj['can'].get(
                device_id, 0x01, 'HHI', timeout=0.015
            )
            table.add_row(
                f'0x{device_id:02X}', Model(type_).name, f'{main_ver}.{sub_ver}'
            )
        except TimeoutError:
            continue
    if table.rows:
        console.print(table)
    else:
        console.print('[i]No device found...[/i]')


@cli.command()
@click.option('--group', '-g', is_flag=True, help='Group messages by CAN ID.')
@click.pass_context
def monitor(ctx: click.Context, group: bool) -> None:
    """Monitor all devices on the CAN bus."""
    console.print('Press Ctrl+C to exit.')
    m = CanBusMonitor(ctx.obj['can'], group, console)
    m.monitor()


@cli.group()
@click.option(
    '--id', '-i', 'id_set', type=ID_LIST, default=0x10, help='Device ID list.'
)
@click.pass_context
def motor(ctx: click.Context, id_set) -> None:
    """Motor commands."""
    ctx.obj['motors'] = []
    can = ctx.obj['can']
    for device_id in id_set:
        try:
            m = Motor(can, device_id)
            ctx.obj['motors'].append(m)
        except ValueError:
            console.print(f'[i]Device 0x{device_id:02X} not found.[/i]')
            continue


@motor.command()
@click.pass_context
def info(ctx: click.Context) -> None:
    """Get motor info."""
    motors = ctx.obj['motors']
    tables = []
    for m in motors:
        tables.append(motor_info_table(m))
    console.print(Columns(tables, equal=True))


@motor.command()
@click.pass_context
def enable(ctx: click.Context) -> None:
    """Enable motor."""
    motors = ctx.obj['motors']
    for m in motors:
        m.enable()


@motor.command()
@click.pass_context
def disable(ctx: click.Context) -> None:
    """Disable motor."""
    motors = ctx.obj['motors']
    for m in motors:
        m.disable()


@motor.command(name='init')
@click.option('--position', '-p', type=float, default=0.0, help='Initial position.')
@click.option('--absolute', '-a', is_flag=True, help='Use absolute position.')
@click.option(
    '--save/--not-save',
    ' /-S',
    default=True,
    help='Save the absolute initial position.',
)
@click.pass_context
def init_pos(ctx: click.Context, position: float, absolute: bool, save: bool) -> None:
    """Initialize motor."""
    motors = ctx.obj['motors']
    for m in motors:
        if m.fw_ver < 1:
            console.print(
                f'[i]Absolute position is not supported on '
                f'firmware version {m.fw_ver}.[/i]'
            )
            return
        if absolute:
            m.init_abs_pos(position)
            if save:
                m.save()
        else:
            m.init_pos(position)


@motor.command(name='pos')
@click.argument('position', type=float)
@click.option(
    '--mode',
    '-m',
    type=click.Choice(['direct', 'filter', 'track']),
    default='filter',
    help='Position control mode. Default is filter mode.',
)
@click.option(
    '--bandwidth', '-b', type=float, default=10.0, help='Bandwidth of the filter.'
)
@click.option(
    '--traj-vel',
    '-v',
    type=float,
    default=5.0,
    help='Trajectory velocity of the position tracking control.',
)
@click.option(
    '--traj-acc',
    '-a',
    type=float,
    default=10.0,
    help='Trajectory acceleration of the position tracking control.',
)
@click.option(
    '--traj-dec',
    '-d',
    type=float,
    default=10.0,
    help='Trajectory deceleration of the position tracking control.',
)
@click.option('--vel-ff', type=float, default=0.0, help='Velocity feedforward.')
@click.option('--torque-ff', type=float, default=0.0, help='Torque feedforward.')
@click.option('--absolute', '-a', is_flag=True, help='Use absolute position.')
@click.pass_context
def set_pos(
    ctx: click.Context,
    position: float,
    mode: str,
    bandwidth: float,
    traj_vel: float,
    traj_acc: float,
    traj_dec: float,
    vel_ff: float,
    torque_ff: float,
    absolute: bool,
) -> None:
    """Set motor position."""
    motors = ctx.obj['motors']
    for m in motors:
        if mode == 'direct':
            m.position_mode()
        elif mode == 'filter':
            m.position_filter_mode(bandwidth)
        elif mode == 'track':
            m.position_track_mode(traj_vel, traj_acc, traj_dec)
        m.enable()
        if absolute and m.fw_ver >= 1:
            m.set_abs_pos(position, vel_ff, torque_ff)
        else:
            m.set_pos(position, vel_ff, torque_ff)


@motor.command(name='vel')
@click.argument('velocity', type=float)
@click.option(
    '--mode',
    '-m',
    type=click.Choice(['direct', 'ramp']),
    default='ramp',
    help='Velocity control mode. Default is ramp mode.',
)
@click.option(
    '--ramp',
    '-r',
    type=float,
    default=10.0,
    help='Ramp rate of the velocity ramp control.',
)
@click.option('--torque-ff', type=float, default=0.0, help='Torque feedforward.')
@click.pass_context
def set_vel(
    ctx: click.Context, velocity: float, mode: str, ramp: float, torque_ff: float
) -> None:
    """Set motor velocity."""
    motors = ctx.obj['motors']
    for m in motors:
        if mode == 'direct':
            m.velocity_mode()
        elif mode == 'ramp':
            m.velocity_ramp_mode(ramp)
        m.enable()
        m.set_vel(velocity, torque_ff)


@motor.command(name='torque')
@click.argument('torque', type=float)
@click.pass_context
def set_torque(ctx: click.Context, torque: float) -> None:
    """Set motor torque."""
    motors = ctx.obj['motors']
    for m in motors:
        m.torque_mode()
        m.enable()
        m.set_torque(torque)


@motor.command(name='id')
@click.argument('new_id', type=BASED_INT)
@click.option('--save', '-s', is_flag=True, help='Save the new ID.')
@click.pass_context
def set_id(ctx: click.Context, new_id: int, save: bool) -> None:
    """Set motor ID."""
    motors = ctx.obj['motors']
    if len(motors) != 1:
        console.print('[i]Please specify only [yellow bold]one[/] motor.[/i]')
        return
    motors[0].config_can_bus(new_id)
    if save:
        m = Motor(ctx.obj['can'], new_id)
        m.save()


@motor.command(name='reset')
@click.pass_context
def reset(ctx: click.Context) -> None:
    motors = ctx.obj['motors']
    if len(motors) != 1:
        console.print('[i]Please specify only [yellow bold]one[/] motor.[/i]')
        return
    m = motors[0]
    with console.status(f'[bold green]Resetting motor 0x{m.id:02X}...'):
        m.reset()
        sleep(4)
        m = Motor(ctx.obj['can'], 0x10)
        console.print(f'[i]Calibrating motor 0x{m.id:02X}...[/i]')
        m.calibrate()
        sleep(1)
        while m.state != Motor.MotorState.DISABLED:
            pass
        if m.error['error'] != Motor.MotorError.NONE:
            console.print(f'[red bold]Error: {m.error}[/]')
            return
        sleep(0.2)
        m.save()
        sleep(0.2)
        console.print(f'[i]Rebooting motor 0x{m.id:02X}...[/i]')
        m.reboot()
        sleep(3)
        console.print('[i]Done. The new ID is [yellow bold]0x10[/].[/i]')
