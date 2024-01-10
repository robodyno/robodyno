# CliffSensor 悬崖传感器

Robodyno CliffSensor 传感器可以通过 CAN 总线读取悬崖传感器的数据。


## 使用

通过以下代码可以读取悬崖传感器读取到的数值：

```python
from robodyno.interfaces import CanBus
from robodyno.components import CliffSensor

can_bus = CanBus()
cliff_sensor = CliffSensor(can_bus)

# 读取悬崖传感器数值
print(cliff_sensor.get_distance())
```

## 设置

### 设置 ID

CliffSensor 传感器模块的 ID 默认为 0x30，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import CliffSensor

can_bus = CanBus()
cliff_sensor = CliffSensor(can_bus)

# 设置 ID 为 0x32
cliff_sensor.config_can_bus(0x32)
```


## 硬件参数

| 参数       | 值                          |
| ---------- | -------------------------- |
| 电源输入    | 5V ~ 29.4V                 |
| 模块量程    | 2cm ~ 63cm                 |
| 处理频率    | 50HZ                       |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.imu_sensor](../../../references/components/can_bus/cliff_sensor) | CAN 总线 CliffSensor 传感器模块类 |
