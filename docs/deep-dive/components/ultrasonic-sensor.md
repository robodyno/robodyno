# Ultrasonic 传感器（超声波）

Robodyno Ultrasonic 传感器可以通过 CAN 总线读取 Ultrasonic 的数据。Ultrasonic 传感器的有效量程为 20cm～250cm,至少 0.126 秒读取到超声波传感器距离值。

## 使用

通过以下代码可以读取超声波传感器读取到的数值：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Ultrasonic

can_bus = CanBus()
ultrasonic_sensor = Ultrasonic(can_bus)

# 读取超声波传感器数值
print(ultrasonic_sensor.get_distance())
```

## 设置

### 设置 ID

Ultrasonic 传感器模块的 ID 默认为 0x31，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Ultrasonic

can_bus = CanBus()
ultrasonic_sensor = ImuSensor(can_bus)

# 设置 ID 为 0x32
ultrasonic_sensor.config_can_bus(0x32)
```


## 硬件参数

| 参数       | 值                         |
| ---------- | -------------------------- |
| 电源输入    | 5V ~ 29.4V                 |
| 模块量程    | 20cm ~ 250cm               |
| 模块超时时间 | 0.126秒                    |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.imu_sensor](../../../references/components/can_bus/ultrasonic_sensor) | CAN 总线 Ultrasonic 传感器模块类 |
