# 快速开始

## 连接硬件

Robodyno 提供的通信模块可以将 USB 接口转换为 CAN 总线接口，同时也能够为 CAN 总线上的模组提供电源。因此，你可以将外接电源（比如 12V 5A 的电源适配器）连接到通信模块上的电源接口，然后通过 Type-C 数据线将通信模块连接到电脑或树莓派上，即可开始操作 Robodyno 的硬件模组。

!!! note

    通信模块的电源接口是 5.5mm x 2.1mm 的 DC 接口，如果你使用的是 12V 动力电池，需要制作 5.5mm x 2.1mm 的 DC 转接头。

## 使用命令行工具

Robodyno 的命令行工具提供了一些简单的命令，可以帮助你快速使用 Robodyno 的硬件模组。

可以通过以下命令查看 CAN 总线上的设备：

```bash
robodyno list
```

查看 ID 默认为 0x10 的电机的详细信息：

```bash
robodyno motor info
```

控制电机转动 1 圈：

!!! warning

    在控制电机转动之前，你需要确保电机所处的环境是安全的，否则可能会造成人身伤害或财产损失。

```bash
robodyno motor pos 6.28
```

全部可用的命令请查看 [命令行说明文档](../../commands)，你也通过以下命令查看 Robodyno 的命令行工具提供了哪些命令：

```bash
robodyno --help
```

如果你想要查看某个命令的详细用法，可以通过以下命令查看：

```bash
robodyno <command> --help
```

## 使用软件包

Robodyno 的软件包提供了一些 Python 类，可以帮助你快速使用 Robodyno 的硬件模组。

可以通过以下代码初始化 CAN 总线：

```python
from robodyno.interfaces import CanBus

can_bus = CanBus()
```

如果你想要控制 ID 默认为 0x10 的电机，可以通过以下代码初始化电机：

```python
from robodyno.components import Motor

motor = Motor(can_bus)
```

控制这个电机转动 1 圈：

!!! warning

    在控制电机转动之前，你需要确保电机所处的环境是安全的，否则可能会造成人身伤害或财产损失。

```python
motor.position_mode()
motor.enable()
motor.set_pos(6.28)
```

软件包的完整说明请查看 [API 文档](../../references) 或 [源代码](https://github.com/robodyno/robodyno)。
