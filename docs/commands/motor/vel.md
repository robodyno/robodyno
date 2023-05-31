# motor vel

控制电机按速度转动。

## 使用

```bash
robodyno motor vel [OPTIONS] VELOCITY
```

## Options（选项）

### `VELOCITY` （必需）

电机目标速度。

- 类型: FLOAT
- 用法：

```bash
robodyno motor vel 3.14
robodyno motor vel -- -3.14
```

### `-m`, `--mode`

指定速度控制模式。

- 类型: Choice(['direct', 'ramp'])
- 默认值: `ramp`

### `-r`, `--ramp`

速度坡度模式的坡度（加速度及减速度）。

- 类型: FLOAT
- 默认值: `10.0`

### `--torque-ff`

力矩前馈。

- 类型: FLOAT
- 默认值: `0.0`

### `--help`

显示帮助信息。

- 类型: BOOL
- 默认值: `false`

## CLI 帮助信息

```
Usage: robodyno motor vel [OPTIONS] VELOCITY

  Set motor velocity.

Options:
  -m, --mode [direct|ramp]  Velocity control mode. Default is ramp mode.
  -r, --ramp FLOAT          Ramp rate of the velocity ramp control.
  --torque-ff FLOAT         Torque feedforward.
  --help                    Show this message and exit.
```
