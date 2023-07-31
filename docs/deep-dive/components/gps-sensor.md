# GPS 传感器

Robodyno GPS 传感器模块提供了一个基于 GT-U12 的 GPS 定位模块，可以通过 CAN 总线读取 GPS 的数据。

## 使用

通过以下代码可以读取 GPS 传感器的当前位置：

```python
from robodyno.interfaces import CanBus
from robodyno.components import GpsSensor

can_bus = CanBus()
gps_sensor = GpsSensor(can_bus)

# 读取当前位置
print(gps_sensor.get_position())
```

返回的位置数据为一个字典，包含以下字段：

| 字段       | 说明     |
| ---------- | -------- |
| timestamp  | 时间戳   |
| longitude  | 经度     |
| latitude   | 纬度     |

## 设置

### 设置 ID

GPS 传感器模块的 ID 默认为 0x32，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import GpsSensor

can_bus = CanBus()
gps_sensor = GpsSensor(can_bus)

# 设置 ID 为 0x33
gps_sensor.config_can_bus(0x33)
```

## 硬件参数

| 参数     | 值         |
| -------- | ---------- |
| 电源输入 | 5V ~ 29.4V  |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.gps_sensor](../../../references/components/can_bus/gps_sensor) | CAN 总线 GPS 传感器模块类 |
