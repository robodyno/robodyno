# motor torque

控制电机按力矩转动。

## 使用

```bash
robodyno motor torque [OPTIONS] TORQUE
```

## Options（选项）

### `TORQUE` (必需)

电机目标力矩。

- 类型: FLOAT
- 用法：

```bash
robodyno motor torque 0.1
```

### `--help`

显示帮助信息。

- 类型: BOOL
- 默认值: `false`

## CLI 帮助信息

```
Usage: robodyno motor torque [OPTIONS] TORQUE

  Set motor torque.

Options:
  --help  Show this message and exit.
```
