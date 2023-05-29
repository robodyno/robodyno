# 深入了解

## 接口

Robodyno 的硬件模组是基于 CAN 总线的，因此你可以通过 CAN 总线来控制 Robodyno 的硬件模组。CAN 总线是一种串行通信协议，它可以在不同的设备之间进行通信，详细的介绍请参考 [CAN 总线](interfaces/can-bus/)。

于此同时，Robodyno 还提供了基于 Webots 的仿真环境，你可以在仿真环境中测试你的代码，详细的介绍请参考 [Webots 仿真环境](interfaces/webots/)。

## 组件

Robodyno 会尽可能地保证组件通过不同接口连接时的兼容性，例如，你可以使用几乎相同的代码来通过 CAN 总线和 Webots 仿真环境来控制伺服减速电机：

```python
from robodyno.interfaces import CanBus, Webots
from robodyno.components import Motor

# 通过 CAN 总线连接伺服减速电机
can_bus = CanBus()
motor = Motor(can_bus)

# 通过 Webots 仿真环境连接伺服减速电机
webots = Webots()
motor = Motor(webots)

# 控制伺服减速电机
motor.enable()
motor.set_position(0.5)
```

当前 Robodyno 的硬件模组清单及相关资料请参考 [硬件模组](components/)。
