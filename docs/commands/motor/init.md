# motor init

初始化电机位置。

## 使用

```bash
robodyno motor init [OPTIONS]
```

## Options（选项）

### `-p`, `--pos`

设定的新初始位置，单位：弧度。

- 类型: FLOAT
- 默认值: 0.0

### `-a`, `--absolute`

设定绝对位置的初始位置。

- 类型: BOOL
- 默认值: `false`

### `--save / -S`, `--not-save`

保存绝对位置的初始位置。

- 类型: BOOL
- 默认值: `true`

### `--help`

显示帮助信息。

- 类型: BOOL
- 默认值: `false`

## CLI 帮助信息

```
Usage: robodyno motor init [OPTIONS]

  Initialize motor.

Options:
  -p, --pos FLOAT          Initial position.
  -a, --absolute           Use absolute position.
  --save / -S, --not-save  Save the absolute initial position.
  --help                   Show this message and exit.
```
