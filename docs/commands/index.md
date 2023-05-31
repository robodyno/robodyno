# Robodyno CLI 命令行工具

## 使用

```bash
robodyno [OPTIONS] COMMAND [ARGS]...
```

## Options（选项）

### `--version`

显示 Robodyno 版本信息。

- 类型: BOOL
- 默认值: `false`

### `-c`, `--can-bus`

CAN 总线接口通道选择。

- 类型: STRING
- 默认值: `can0`

### `-b`, `--baudrate`

CAN 总线波特率，从 250000，500000，1000000 中选择。

- 类型: INT
- 默认值: 1000000

### `--help`

显示帮助信息。

- 类型: BOOL
- 默认值: `false`

## Commands（命令）

### [list](list/)

列出 CAN 总线上所有设备。

### [monitor](monitor/)

监控 CAN 总线上所有消息。

### [motor](motor/)

电机控制命令。

## CLI 帮助信息

```
Usage: robodyno [OPTIONS] COMMAND [ARGS]...

  Robodyno command line interface.

Options:
  --version               Show the version and exit.
  -c, --can-bus TEXT      CAN bus interface channel.
  -b, --baudrate INTEGER  CAN bus baud rate.
  --help                  Show this message and exit.

Commands:
  list     List all devices on the CAN bus.
  monitor  Monitor all devices on the CAN bus.
  motor    Motor commands.
```
