# motor config

配置电机参数。没有指定选项时，默认设置电机常规参数。

## 使用

```bash
robodyno motor config [OPTIONS]
```

## Options（选项）

### `-v`, `--vel-limit`

设置速度限制。

- 类型: BOOL
- 默认值: `false`

### `-c`, `--current-limit`

设置电流限制。

- 类型: BOOL
- 默认值: `false`

### `-p`, `--pid`

设置 PID 参数。

- 类型: BOOL
- 默认值: `false`

### `-h`, `--heartbeat-rate`

设置心跳包发送频率。

- 类型: BOOL
- 默认值: `false`

### `-i`, `--new-id`

设置新的电机 ID。

- 类型: BOOL
- 默认值: `false`

### `-b`, `--can-baudrate`

设置 CAN 波特率。

- 类型: BOOL
- 默认值: `false`

### `--help`

显示帮助信息。

- 类型: BOOL
- 默认值: `false`

## CLI 帮助信息

```
Usage: robodyno motor config [OPTIONS]

  Configure motor parameters.

Options:
  -v, --vel-limit       Set velocity limit.
  -c, --current-limit   Set current limit.
  -p, --pid             Set PID gains.
  -h, --heartbeat-rate  Set heartbeat rate.
  -i, --new-id          Set new ID.
  -b, --can-baudrate    Set CAN baudrate.
  --help                Show this message and exit.
```
