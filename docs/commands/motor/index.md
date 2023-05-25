
# robodyno motor

Motor commands.

## Usage

```
Usage: robodyno motor [OPTIONS] COMMAND [ARGS]...
```

## Options
* `id_set`: 
  * Type: <robodyno.tools.cli_param_types.DeviceIdListParamType object at 0x00000219F58236D0> 
  * Default: `16`
  * Usage: `--id
-i`

  Device ID list.


* `help`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--help`

  Show this message and exit.



## CLI Help

```
Usage: robodyno motor [OPTIONS] COMMAND [ARGS]...

  Motor commands.

Options:
  -i, --id DEVICE_ID_LIST  Device ID list.
  --help                   Show this message and exit.

Commands:
  calibrate     Calibrate motor.
  clear-errors  Clear errors on motors.
  config        Configure motor parameters.
  disable       Disable motor.
  enable        Enable motor.
  estop         Emergency stop motors.
  info          Get motor info.
  init          Initialize motor.
  pos           Set motor position.
  reboot        Reboot motors.
  reset
  torque        Set motor torque.
  vel           Set motor velocity.
```

