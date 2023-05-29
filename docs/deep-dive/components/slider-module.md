# 直线运动模组

直线运动模组是 Robodyno 提供的一种基于伺服减速电机的直线运动机构，可以通过伺服减速电机的转动实现直线运动。

## 使用

默认使用伺服减速电机的轨迹追踪模式，直线运动模组的运动单位是米，而不是弧度。

示例代码：

``` python
from robodyno.interfaces import CanBus
from robodyno.components import SliderModule

can_bus = CanBus()
slider_module = SliderModule(can_bus)

# 设置速度为 0.01m/s
slider_module.set_max_vel(0.01)

# 使能直线运动模组
slider_module.enable()

# 直线运动模组移动到 0.05m 的位置
slider_module.set_pos(0.05)
```

!!! note

    通过模组的 `motor` 属性可以获取直线运动模组的电机对象，在确保操作安全的前提下，可以通过电机对象的 API 直接控制直线运动模组的电机。

## 硬件参数

| 参数 | 值 |
| ---- | ---- |
| 行程 | 0.2m |
| 导程 | 0.01m |

## API

| 组件 | 说明 |
| ---- | ---- |
| [can_bus.slider_module](../../../references/components/can_bus/slider_module) | CAN 总线直线运动模组类 |
| [webots.slider_module](../../../references/components/webots/slider_module) | Webots 直线运动模组类 |