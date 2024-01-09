# Impact 传感器（防撞条）

Robodyno Impact 传感器可以通过 CAN 总线读取 Impact 的数据。至少 0.0016 秒读取到给模块触发值。

## 使用

通过以下代码可以读取防撞条传感器读取到的数值：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Impact

can_bus = CanBus()
impact_sensor = Impact(can_bus)

# 读取超声波传感器数值
print(impact_sensor.get_status())
```

## 设置

### 设置 ID

IMU 传感器模块的 ID 默认为 0x31，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Impact

can_bus = CanBus()
impact_sensor = Impact(can_bus)

# 设置 ID 为 0x32
impact_sensor.config_can_bus(0x32)
```


## 硬件参数

| 参数       | 值                         |
| ---------- | -------------------------- |
| 电源输入    | 5V ~ 29.4V                 |
| 模块数值范围 | 高电平：1 低电平：0           |
| 模块超时时间 | 0.0016秒                    |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.imu_sensor](../../../references/components/can_bus/impact_sensor) | CAN 总线 Impact 传感器模块类 |
