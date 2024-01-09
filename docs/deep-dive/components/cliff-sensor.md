# Cliff 传感器（悬崖）

Robodyno Cliff 传感器可以通过 CAN 总线读取 Cliff 的数据。Cliff 传感器的有效量程为 2cm～63cm,至少 0.0016 秒读取到悬崖传感器距离值。

## 使用

通过以下代码可以读取悬崖传感器读取到的数值：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Cliff

can_bus = CanBus()
cliff_sensor = Cliff(can_bus)

# 读取悬崖传感器数值
print(cliff_sensor.get_distance())
```

## 设置

### 设置 ID

Cliff 传感器模块的 ID 默认为 0x38，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Cliff

can_bus = CanBus()
cliff_sensor = Cliff(can_bus)

# 设置 ID 为 0x32
cliff_sensor.config_can_bus(0x32)
```


## 硬件参数

| 参数       | 值                         |
| ---------- | -------------------------- |
| 电源输入    | 5V ~ 29.4V                 |
| 模块量程    | 2cm ~ 63cm               |
| 模块超时时间 | 0.0016秒                    |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.imu_sensor](../../../references/components/can_bus/cliff_sensor) | CAN 总线 Cliff 传感器模块类 |
