# MagneticSensor 磁导航传感器

Robodyno MagneticSensor 传感器可以通过 CAN 总线读取磁导航传感器的位置状态数据。

## 使用

### 输出位置状态

通过以下代码可以读取磁导航传感器检测到的位置状态值：

```python
from robodyno.interfaces import CanBus
from robodyno.components import MagneticSensor

can_bus = CanBus()
mag = MagneticSensor(can_bus)

# 读取磁导航传感器的位置状态值
print(mag.get_position_status())
```

### 磁场校准

通过以下代码可以校准磁导航传感器的磁场：

```python
from robodyno.interfaces import CanBus
from robodyno.components import MagneticSensor

can_bus = CanBus()
mag = MagneticSensor(can_bus)
mag.calibrate_magnetic_field()
```
!!! note

    在新的使用环境下，磁导航传感器需重新校准磁场。磁场校准是记录新环境在没有磁条情况下的磁场大小。

## 硬件参数

| 参数           | 值                         |
| :----------:  | :--------------------------:|
| 工作电压        | 10V ~ 30V     |
| 工作温度        | -20 ~ 50度    |
| 检测距离        | 0 ~ 30mm      |
| 检测极性        | N、S          |
| 建议安装高度     | 20 ~ 30mm     |
| 磁条规格     | 宽 30mm、50mm，建议使用表面磁场大于 100 GS 的磁条或磁钉  |


## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.magnetic_sensor](../../../references/components/can_bus/magnetic_sensor) | CAN 总线 MagneticSensor 传感器模块类 |