# IMU 传感器

Robodyno IMU 传感器模块提供了一个 6 轴 IMU 传感器，可以通过 CAN 总线读取 IMU 的数据。

## 使用

通过以下代码可以读取 IMU 传感器的当前姿态、角速度和加速度：

```python
from robodyno.interfaces import CanBus
from robodyno.components import ImuSensor

can_bus = CanBus()
imu_sensor = ImuSensor(can_bus)

# 读取当前姿态（欧拉角）
print(imu_sensor.get_euler())

# 读取当前姿态（四元数）
print(imu_sensor.get_quaternion())

# 读取当前角速度
print(imu_sensor.get_gyro())

# 读取当前加速度
print(imu_sensor.get_accel())
```

## 设置

### 设置 ID

IMU 传感器模块的 ID 默认为 0x31，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import ImuSensor

can_bus = CanBus()
imu_sensor = ImuSensor(can_bus)

# 设置 ID 为 0x32
imu_sensor.config_can_bus(0x32)
```

### 设置量程

IMU 传感器模块的角速度量程默认为 250 deg/s，加速度量程默认为 2 g，可以在 250、500、1000、2000 deg/s 和 2、4、8、16 g 之间设置。

参数设置示例：

```python
from robodyno.interfaces import CanBus
from robodyno.components import ImuSensor

can_bus = CanBus()
imu_sensor = ImuSensor(can_bus)

# 设置角速度量程为 500 deg/s，加速度量程为 8 g
imu_sensor.set_ranges(1, 2)
```

## 硬件参数

| 参数       | 值                         |
| ---------- | -------------------------- |
| 电源输入   | 5V ~ 29.4V                 |
| 角速度量程 | 250, 500, 1000, 2000 deg/s |
| 加速度量程 | 2, 4, 8, 16 g              |

## API

| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.imu_sensor](../../../references/components/can_bus/imu_sensor) | CAN 总线 IMU 传感器模块类 |
