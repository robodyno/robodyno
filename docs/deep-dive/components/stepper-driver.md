# 步进电机驱动模块

Robodyno 步进电机驱动模块提供了一路步进电机驱动，可以通过 CAN 总线控制步进电机的运动。

!!! note
    当前 `ROBODYNO_D_CNV02` 模组也临时使用了这个模块的固件，因此会被识别为步进电机驱动模块。

## 引脚定义

| 引脚 | 功能 |
| :--: | :--: |
| A1 | 步进电机 A+ |
| A2 | 步进电机 A- |
| B1 | 步进电机 B+ |
| B2 | 步进电机 B- |

## 运动控制

### 位置控制

```python
from robodyno.interfaces import CanBus
from robodyno.components import StepperDriver

can_bus = CanBus()
stepper_driver = StepperDriver(can_bus)

stepper_driver.enable()
stepper_driver.set_pos(6.28) # 旋转1圈
```

### 速度控制

```python
from robodyno.interfaces import CanBus
from robodyno.components import StepperDriver

can_bus = CanBus()
stepper_driver = StepperDriver(can_bus)

stepper_driver.enable()
stepper_driver.set_vel(3.14) # 以 3.14 rad/s 的速度旋转
```

### 停止运动

```python
from robodyno.interfaces import CanBus
from robodyno.components import StepperDriver

can_bus = CanBus()
stepper_driver = StepperDriver(can_bus)

stepper_driver.stop()
```

## 设置

### 设置 ID

步进电机驱动模块的 ID 默认为 0x22，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import StepperDriver

can_bus = CanBus()
stepper_driver = StepperDriver(can_bus)

# 设置 ID 为 0x23
stepper_driver.config_can_bus(0x23)
```

### 设置运动参数

步进电机驱动模块的细分值默认为 8，可以在 8、16、32、64 之间设置。速度上限默认为 4000 step/s，加速度上限默认为 16000 step/s^2，在程序中设置的速度和加速度会根据细分值和减速比进行换算。

参数设置示例：

```python
from robodyno.interfaces import CanBus
from robodyno.components import StepperDriver

can_bus = CanBus()
stepper_driver = StepperDriver(can_bus, reduction = 1)

# 设置细分为 8
stepper.set_subdivision(8)

# 设置速度上限为 5 rad/s，加速度上限为 10 rad/s^2
stepper.set_vel_acc_limit(5, 10)
```

## 硬件参数

| 参数 | 值 |
| ---- | ---- |
| 电源输入 | 5V ~ 29.4V |
| 最大驱动电流 | 2.8A |
| 电机细分 | 8, 16, 32, 64 |
| 电机步距角 | 1.8° |

## API

| 组件 | 说明 |
| ---- | ---- |
| [can_bus.stepper_driver](../../../references/can_bus/stepper_driver) | CAN 总线步进电机驱动模块类 |
