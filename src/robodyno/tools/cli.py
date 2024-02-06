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

"""Provides a cli for robodyno devices."""

from time import sleep

import click
from rich.console import Console
from rich.table import Table
from rich.columns import Columns
from rich.traceback import install
from rich.prompt import Prompt, Confirm, IntPrompt, FloatPrompt

from robodyno import __version__
from robodyno.interfaces import CanBus
from robodyno.components.can_bus.motor import Motor
from robodyno.components.config.model import Model
from robodyno.tools.can_bus_monitor import CanBusMonitor
from robodyno.tools.motor_info_table import motor_info_table
from robodyno.tools.cli_param_types import ID_LIST, BasedIntPrompt


@click.group()
@click.version_option(__version__)
@click.option('--can-bus', '-c', default='can0', help='CAN bus interface channel.')
@click.option('--baudrate', '-b', default=1000000, help='CAN bus baud rate.')
@click.pass_context
def robodyno(ctx: click.Context, can_bus: str, baudrate: int) -> None:
    """Robodyno command line interface."""
    install(suppress=[click])
    ctx.ensure_object(dict)
    ctx.obj['console'] = Console()
    ctx.obj['can'] = CanBus(baudrate, can_bus)
    ctx.obj['can_bitrate'] = baudrate


@robodyno.command(name='list')
@click.pass_context
def list_devices(ctx: click.Context) -> None:
    """List all devices on the CAN bus."""
    console = ctx.obj['console']

    table = Table()
    table.add_column('ID', style='cyan', justify='right', no_wrap=True)
    table.add_column('Model', style='green', justify='left', no_wrap=True)
    table.add_column('Version', style='yellow', justify='left', no_wrap=True)

    for device_id in range(0x3F):
        try:
            main_ver, sub_ver, _, type_ = ctx.obj['can'].get(
                device_id, 0x01, 'HHeH', timeout=0.015
            )
            table.add_row(
                f'0x{device_id:02X}', Model(type_).name, f'{main_ver}.{sub_ver}'
            )
        except TimeoutError:
            continue
    if table.rows:
        console.print(table)
    else:
        console.print('No device found...')


@robodyno.command()
@click.option('--group', '-g', is_flag=True, help='Group messages by CAN ID.')
@click.pass_context
def monitor(ctx: click.Context, group: bool) -> None:
    """Monitor all devices on the CAN bus."""
    console = ctx.obj['console']
    console.print('Press Ctrl+C to exit.')
    m = CanBusMonitor(ctx.obj['can'], group, console)
    m.monitor()


@robodyno.group()
@click.option(
    '--id', '-i', 'id_set', type=ID_LIST, default=0x10, help='Device ID list.'
)
@click.pass_context
def motor(ctx: click.Context, id_set) -> None:
    """Motor commands."""
    ctx.obj['motors'] = []
    console = ctx.obj['console']
    can = ctx.obj['can']
    for device_id in id_set:
        try:
            m = Motor(can, device_id)
            ctx.obj['motors'].append(m)
        except ValueError:
            console.print(f'Device 0x{device_id:02X} not found.')
            continue


@motor.command()
@click.pass_context
def info(ctx: click.Context) -> None:
    """Get motor info."""
    console = ctx.obj['console']
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
@click.option('--pos', '-p', type=float, default=0.0, help='Initial position.')
@click.option('--absolute', '-a', is_flag=True, help='Use absolute position.')
@click.option(
    '--save/--not-save',
    ' /-S',
    default=True,
    help='Save the absolute initial position.',
)
@click.pass_context
def init_pos(ctx: click.Context, pos: float, absolute: bool, save: bool) -> None:
    """Initialize motor."""
    console = ctx.obj['console']
    motors = ctx.obj['motors']
    for m in motors:
        if m.fw_ver < 1:
            console.print(
                f'Absolute position is not supported on '
                f'firmware version {m.fw_ver}.'
            )
            return
        if absolute:
            m.init_abs_pos(pos)
            if save:
                m.save()
        else:
            m.init_pos(pos)


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
    '--acc',
    type=float,
    default=10.0,
    help='Trajectory acceleration of the position tracking control.',
)
@click.option(
    '--traj-dec',
    '--dec',
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


@motor.command(name='config')
@click.option('--vel-limit', '-v', is_flag=True, help='Set velocity limit.')
@click.option('--current-limit', '-c', is_flag=True, help='Set current limit.')
@click.option('--pid', '-p', is_flag=True, help='Set PID gains.')
@click.option('--heartbeat-rate', '-h', is_flag=True, help='Set heartbeat rate.')
@click.option('--new-id', '-i', is_flag=True, help='Set new ID.')
@click.option('--can-baudrate', '-b', is_flag=True, help='Set CAN baudrate.')
@click.pass_context
def config(
    ctx: click.Context,
    vel_limit: bool,
    current_limit: bool,
    pid: bool,
    heartbeat_rate: bool,
    new_id: bool,
    can_baudrate: bool,
) -> None:
    """Configure motor parameters."""
    console = ctx.obj['console']
    motors = ctx.obj['motors']
    if len(motors) != 1:
        console.print('Please specify only [yellow bold]one[/yellow bold] motor.')
        return
    m = motors[0]
    if not (
        vel_limit or current_limit or pid or heartbeat_rate or new_id or can_baudrate
    ):  # if no option is specified, set all options except can bus configuartions.
        vel_limit = True
        current_limit = True
        pid = True

    if vel_limit:
        vel_limit_now = m.get_vel_limit(0.1)
        try:
            m.set_vel_limit(
                FloatPrompt.ask(
                    (
                        f'[green bold]Enter[/] new velocity limit, '
                        f'now: [green bold]{vel_limit_now:.2f} rad/s[/]'
                    ),
                    console=console,
                    default=m.default_vel_limit,
                )
            )
        except ValueError as e:
            console.print(f'[red bold]Error:[/] failed to set velocity limit: {e}')
    if current_limit:
        current_limit_now = m.get_current_limit(0.1)
        try:
            m.set_current_limit(
                FloatPrompt.ask(
                    (
                        f'[green bold]Enter[/] new current limit, '
                        f'now: [green bold]{current_limit_now:.1f} A[/]'
                    ),
                    console=console,
                    default=m.default_current_limit,
                )
            )
        except ValueError as e:
            console.print(f'[red bold]Error:[/] failed to set current limit: {e}')
    if pid:
        pos_kp, vel_kp, vel_ki = m.get_pid(0.1)
        m.set_pid(
            FloatPrompt.ask(
                (
                    f'[green bold]Enter[/] new position kp, '
                    f'now: [green bold]{pos_kp:.2f}[/]'
                ),
                console=console,
                default=m.default_pos_kp,
            ),
            FloatPrompt.ask(
                (
                    f'[green bold]Enter[/] new velocity kp, '
                    f'now: [green bold]{vel_kp:.2f}[/]'
                ),
                console=console,
                default=m.default_vel_kp,
            ),
            FloatPrompt.ask(
                (
                    f'[green bold]Enter[/] new velocity ki, '
                    f'now: [green bold]{vel_ki:.2f}[/]'
                ),
                console=console,
                default=m.default_vel_ki,
            ),
        )
    heartbeat = 1000
    bitrate = ctx.obj['can_bitrate']
    if heartbeat_rate:
        heartbeat = IntPrompt.ask(
            '[green bold]Enter[/] new heartbeat rate im ms',
            console=console,
            default=1000,
        )
        m.config_can_bus(heartbeat=heartbeat, bitrate=bitrate)
    if can_baudrate:
        console.print(
            '[magenta bold]WARNING[/] This will change the CAN baudrate '
            'of the motor [bold]after a reboot[/bold].'
        )
        bitrate = int(
            Prompt.ask(
                '[green bold]Enter[/] new CAN bitrate',
                console=console,
                choices=['1000000', '500000', '250000'],
                default=str(bitrate),
            )
        )
        m.config_can_bus(
            heartbeat=heartbeat,
            bitrate=bitrate,
        )
    if new_id:
        console.print(
            '[yellow bold]NOTE[/] This will change the device ID of the '
            'motor [bold]immediately[/bold].'
        )
        new_id = BasedIntPrompt.ask(
            '[green bold]Enter[/] new ID',
            console=console,
            default=m.id,
        )
        m.config_can_bus(new_id=new_id, heartbeat=heartbeat, bitrate=bitrate)
        m = Motor(ctx.obj['can'], new_id)
    if Confirm.ask('[cyan bold]Save[/] the new parameters to flash?'):
        m.save()


@motor.command(name='calibrate')
@click.pass_context
def calibrate(ctx: click.Context) -> None:
    """Calibrate motor."""
    console = ctx.obj['console']
    motors = ctx.obj['motors']
    if len(motors) != 1:
        console.print('Please specify only [yellow bold]one[/yellow bold] motor.')
        return
    if not Confirm.ask(
        '[magenta bold]WARNING[/] Calibration will move the motor and '
        'reset the encoders. Make sure the motor is free to move.'
    ):
        return
    m = motors[0]
    with console.status(f'[bold green]Calibrating motor 0x{m.id:02X}...'):
        m.calibrate()
        sleep(1)
        while m.state != Motor.MotorState.DISABLED:
            pass
        if m.error['error'] != Motor.MotorError.NONE:
            console.print(f'[red bold]Error:[/] {m.error}')
            return
        sleep(0.2)
        console.print('[green bold]Done![/]')
    if Confirm.ask('[cyan bold]Save[/] the result to flash?', default=True):
        m.save()


@motor.command(name='clear-errors')
@click.pass_context
def clear_errors(ctx: click.Context) -> None:
    """Clear errors on motors."""
    motors = ctx.obj['motors']
    for m in motors:
        m.clear_errors()


@motor.command(name='reboot')
@click.pass_context
def reboot(ctx: click.Context) -> None:
    """Reboot motors."""
    motors = ctx.obj['motors']
    for m in motors:
        m.reboot()


@motor.command(name='estop')
@click.pass_context
def estop(ctx: click.Context) -> None:
    """Emergency stop motors."""
    motors = ctx.obj['motors']
    for m in motors:
        m.estop()


@motor.command(name='reset')
@click.pass_context
def reset(ctx: click.Context) -> None:
    """Reset motors."""
    console = ctx.obj['console']
    motors = ctx.obj['motors']
    if len(motors) != 1:
        console.print('Please specify only [yellow bold]one[/yellow bold] motor.')
        return
    if not Confirm.ask(
        '[magenta bold]WARNING[/] Resetting the motor will reset '
        '[red bold]ALL[/red bold] parameters to their default values, calibrate '
        'the motor and reboot it. Continue?'
    ):
        return
    m = motors[0]
    with console.status(f'[bold green]Resetting motor 0x{m.id:02X}...'):
        m.reset()
        sleep(4)
        m = Motor(ctx.obj['can'], 0x10)
        console.print(f'[green bold]Calibrating[/green bold] motor 0x{m.id:02X}...')
        m.calibrate()
        sleep(1)
        while m.state != Motor.MotorState.DISABLED:
            pass
        if m.error['error'] != Motor.MotorError.NONE:
            console.print(f'[red bold]Error:[/] {m.error}')
            return
        sleep(0.2)
        m.save()
        sleep(0.2)
        console.print(f'[green bold]Rebooting[/green bold] motor 0x{m.id:02X}...')
        m.reboot()
        sleep(3)
        console.print('[green bold]Done.[/]')
        console.print('The ID of the motor is now [green bold]0x10[/green bold].')
