# UltrasonicSensor 超声波传感器

Robodyno UltrasonicSensor 传感器可以通过 CAN 总线读取超声波传感器的数据。

## 使用

通过以下代码可以读取超声波传感器读取到的数值：

```python
from robodyno.interfaces import CanBus
from robodyno.components import UltrasonicSensor

can_bus = CanBus()
ultrasonic_sensor = UltrasonicSensor(can_bus)

# 读取超声波传感器数值
print(ultrasonic_sensor.get_distance())
```

## 设置

### 设置 ID

UltrasonicSensor 传感器模块的 ID 默认为 0x35，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import UltrasonicSensor

can_bus = CanBus()
ultrasonic_sensor = UltrasonicSensor(can_bus)

# 设置 ID 为 0x32
ultrasonic_sensor.config_can_bus(0x32)
```


## 硬件参数

| 参数       | 值                         |
| ---------- | --------------------------|
| 电源输入    | 5V ~ 29.4V                 |
| 模块量程    | 20cm ~ 250cm               |
| 处理频率    | 6.6667HZ                   |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.ultrasonic_sensor](../../../references/components/can_bus/ultrasonic_sensor) | CAN 总线 UltrasonicSensor 传感器模块类 |
