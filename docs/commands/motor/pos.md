# motor pos

控制电机按位置转动。

## 使用

```bash
robodyno motor pos [OPTIONS] POSITION
```

## Options（选项）

### `POSITION` （必需）

电机目标位置。

- 类型: FLOAT
- 用法:

```bash
robodyno motor pos 3.14
robodyno motor pos -- -3.14
```

### `-m`, `--mode`

指定位置控制模式。

- 类型: Choice(['direct', 'filter', 'track'])
- 默认值: `filter`

### `-b`, `--bandwidth`

滤波位置模式中滤波器带宽。

- 类型: FLOAT
- 默认值: `10.0`

### `-v`, `--traj-vel`

位置追踪模式的最高轨迹速度。

- 类型: FLOAT
- 默认值: `5.0`

### `--acc`, `--traj-acc`

位置追踪模式的轨迹加速度。

- 类型: FLOAT
- 默认值: `10.0`

### `--dec`, `--traj-dec`

位置追踪模式的轨迹减速度。

- 类型: FLOAT
- 默认值: `10.0`

### `--vel-ff`

速度前馈。

- 类型: FLOAT
- 默认值: `0.0`

### `--torque-ff`

力矩前馈。

- 类型: FLOAT
- 默认值: `0.0`

### `-a`, `--absolute`

使用绝对位置。

- 类型: BOOL
- 默认值: `false`

### `--help`

显示帮助信息。

- 类型: BOOL
- 默认值: `false`

## CLI 帮助信息

```
Usage: robodyno motor pos [OPTIONS] POSITION

  Set motor position.

Options:
  -m, --mode [direct|filter|track]
                                  Position control mode. Default is filter
                                  mode.
  -b, --bandwidth FLOAT           Bandwidth of the filter.
  -v, --traj-vel FLOAT            Trajectory velocity of the position tracking
                                  control.
  --acc, --traj-acc FLOAT         Trajectory acceleration of the position
                                  tracking control.
  --dec, --traj-dec FLOAT         Trajectory deceleration of the position
                                  tracking control.
  --vel-ff FLOAT                  Velocity feedforward.
  --torque-ff FLOAT               Torque feedforward.
  -a, --absolute                  Use absolute position.
  --help                          Show this message and exit.
```
