# robodyno motor

电机控制命令。

## 使用

```bash
robodyno motor [OPTIONS] COMMAND [ARGS]...
```

## Options（选项）

### `-i`, `--id`

设备 ID 列表。

- 类型: 设备 ID 列表，用逗号分隔，可以是单个 ID，ID 范围（`start-end`），比如 `0x10,0x11,0x12-0x15`。
- 默认值: 0x10

### `--help`

显示帮助信息。

- 类型: BOOL
- 默认值: `false`

## Commands（命令）

### [info](info/)

获取电机信息。

### [enable](enable/)

使能电机。

### [disable](disable/)

失能电机。

### [init](init/)

初始化电机位置。

### [pos](pos/)

控制电机按位置转动。

### [vel](vel/)

控制电机按速度转动。

### [torque](torque/)

控制电机按力矩转动。

### [config](config/)

配置电机参数。

### [calibrate](calibrate/)

校准电机。

### [clear-errors](clear-errors/)

清除电机错误。

### [reboot](reboot/)

重启电机。

### [estop](estop/)

急停电机。

### [reset](reset/)

恢复出厂设置。

## CLI 帮助信息

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
  reset         Reset motors.
  torque        Set motor torque.
  vel           Set motor velocity.
```
