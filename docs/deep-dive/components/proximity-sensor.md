# ProximitySensor 接近开关传感器

Robodyno ProximitySensor 传感器可以通过 CAN 总线读取接近开关传感器的数据。

## 使用

通过以下代码可以读取接近开关传感器读取到的数值：

```python
from robodyno.interfaces import CanBus
from robodyno.components import ProximitySensor

can_bus = CanBus()
proximity_sensor = ProximitySensor(can_bus)

# 读取接近开关传感器数值
print(proximity_sensor.get_data())
```

## 设置

### 设置 ID

ProximitySensor 传感器模块的 ID 默认为 0x36，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import ProximitySensor

can_bus = CanBus()
proximity_sensor = ProximitySensor(can_bus)

# 设置 ID 为 0x32
proximity_sensor.config_can_bus(0x32)
```


## 硬件参数

| 参数       | 值                         |
| ---------- | --------------------------|
| 电源输入    | 5V ~ 29.4V                |
| 模块取值    | 0 / 1  /  None            |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.proximity_sensor](../../../references/components/can_bus/proximity_sensor) | CAN 总线 ProximitySensor 传感器模块类 |
