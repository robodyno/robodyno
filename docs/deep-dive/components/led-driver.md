# LED 驱动器

Robodyno LED 驱动器模块提供了一个最多可控制 256 个基于 WS2812B 的 RGB LED 灯珠的驱动器，可以通过 CAN 总线控制 LED 灯珠的颜色和闪烁模式。

## 使用

### 设置 LED 常亮颜色

通过以下代码可以设置 LED 常亮颜色：

```python
from robodyno.interfaces import CanBus
from robodyno.components import LedDriver

can_bus = CanBus()
strip = LedDriver(can_bus)
strip.set_color(0, (0, 255, 0)) # 设置第 0 个灯珠为绿色
strip.set_color((1, 3), (255, 0, 0)) # 设置第 1 ~ 3 个灯珠为红色
```

### 设置 LED 闪烁模式

通过以下代码可以设置 LED 闪烁模式：

```python
from robodyno.interfaces import CanBus
from robodyno.components import LedDriver

can_bus = CanBus()
strip = LedDriver(can_bus)

# 设置第 0 个灯珠为闪烁模式，闪烁频率为 1 Hz，闪烁颜色为红色
strip.blink(0, (255, 0, 0), 1.0 / 2)
```

### 设置 LED 呼吸模式

通过以下代码可以设置 LED 呼吸模式：

```python
from robodyno.interfaces import CanBus
from robodyno.components import LedDriver

can_bus = CanBus()
strip = LedDriver(can_bus)

# 设置第 0 个灯珠为呼吸模式，呼吸频率为 1 Hz，呼吸颜色为红色
strip.breathe(0, (255, 0, 0), 1.0 / 2)
```

### 设置跑马灯模式

通过以下代码可以设置跑马灯模式：

```python
from robodyno.interfaces import CanBus
from robodyno.components import LedDriver

can_bus = CanBus()
strip = LedDriver(can_bus)

# 设置第 0 ~ 3 个灯珠为跑马灯模式，跑马灯频率为 1 Hz，跑马灯颜色为红色
strip.marquee((0, 3), (255, 0, 0), 1.0 / 2)
```

### 关闭所有 LED

通过以下代码可以设置 LED 关闭：

```python
from robodyno.interfaces import CanBus
from robodyno.components import LedDriver

can_bus = CanBus()
strip = LedDriver(can_bus)

strip.clear() # 关闭所有 LED
```

## 设置

### 设置 ID

LED 驱动器模块的 ID 默认为 0x33，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import LedDriver

can_bus = CanBus()
strip = LedDriver(can_bus)

# 设置 ID 为 0x34
strip.config_can_bus(0x34)
```

## 硬件参数

| 参数         | 值         |
| ------------ | ---------- |
| 电源输入     | 5V ~ 29.4V |
| 最大灯珠数量 | 256        |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.led_driver](../../../references/components/can_bus/led_driver) | CAN 总线 LED 驱动器模块类 |
