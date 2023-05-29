# robodyno monitor

监控 CAN 总线上所有消息。

## 使用

```bash
robodyno monitor [OPTIONS]
```

## Options（选项）

### `-g`, `--group`

按 CAN ID 分组消息。

- 类型: BOOL
- 默认值: `false`

### `--help`

显示帮助信息。

- 类型: BOOL
- 默认值: `false`

## CLI 帮助信息

```
Usage: robodyno monitor [OPTIONS]

  Monitor all devices on the CAN bus.

Options:
  -g, --group  Group messages by CAN ID.
  --help       Show this message and exit.
```
