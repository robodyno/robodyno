# 伺服减速电机

伺服减速电机是 Robodyno 的核心硬件模组，它可以作为机器人的关节或者驱动轮，也可以以其他形式作为机器人的动力源。

Robodyno 的伺服减速电机根据减速比和电机参数的不同，分为两种型号：`ROBODYNO_PRO_P44` 和 `ROBODYNO_PRO_P12`。`ROBODYNO_PRO_P44` 的减速比为 44:1，拥有较大的扭矩，适合作为机器人的关节；`ROBODYNO_PRO_P12` 的减速比为 12.45:1，拥有较大的转速，适合作为机器人的驱动轮。

## 控制

伺服减速电机可以通过命令行工具或 Python API 进行控制。

### 状态

电机需要进入使能（`ENABLE`）状态才能进行控制，否则会被动力系统忽略。

通过以下方式进入使能状态：

=== "命令行工具"

    ``` bash
    robodyno motor enable
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    motor.enable()
    ```

通过以下方式退出使能状态：

=== "命令行工具"

    ``` bash
    robodyno motor disable
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    motor.disable()
    ```

通过以下方式查看电机的状态：

=== "命令行工具"

    ``` bash
    robodyno motor info
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    print(motor.state)
    ```

### 转动

通过以下方式可以控制电机转动到指定的目标位置：

=== "命令行工具"

    ``` bash
    robodyno motor pos 6.28
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    motor.position_filter_mode(10)
    motor.enable()
    motor.set_pos(6.28)
    ```

通过以下方式可以控制电机按照指定的目标速度转动：

=== "命令行工具"

    ``` bash
    robodyno motor vel 3.14
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    motor.velocity_ramp_mode(10)
    motor.enable()
    motor.set_vel(3.14)
    ```

Robodyno 的伺服减速电机的运动模式分为以下几种：

- 位置模式（`POSITION`）
- 位置滤波模式（`POSITION_FILTER`）
- 位置追踪模式（`POSITION_TRACK`）
- 速度模式（`VELOCITY`）
- 速度坡度模式（`VELOCITY_RAMP`）
- 力矩模式（`TORQUE`）

关于运动模式的详细说明请参考 [电机运动模式](../../motor-modes/)。

## 绝对位置及位置初始化

固件版本 1.0 及以上的电机支持绝对位置，电机在断电后仍然可以记录当前的绝对位置，重新上电后可以查询到正确的绝对位置。这个特性可以用于机械臂的关节控制，机械臂的关节可以在断电后重新上电，而不需要重新初始化位置。

通过以下方式设置电机的绝对位置：

=== "命令行工具"

    ``` bash
    robodyno motor pos 3.14 -a
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    motor.set_abs_pos(3.14)
    ```

固件版本 1.0 及以上同时支持电机初始化位置，电机初始化位置是将电机的当前位置设置为指定的弧度值。

通过以下方式将电机（绝对）位置初始化：

=== "命令行工具"

    ``` bash
    # 初始化当前位置为 3.14
    robodyno motor init -p 3.14
    # 初始化当前绝对位置为 0
    robodyno motor init -a -p 0
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    # 初始化当前位置为 3.14
    motor.init_pos(3.14)

    # 初始化当前绝对位置为 0
    motor.init_abs_pos(0)
    motor.save()
    ```

## 设置

### 电机 ID

通过以下方式设置电机 ID：

=== "命令行工具"

    ``` bash
    robodyno motor --id 0x10 config -i
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()

    current_id = 0x10
    new_id = 0x11

    motor = Motor(can_bus, current_id)
    motor.config_can_bus(new_id = new_id)

    # 保存设置
    motor = Motor(can_bus, new_id)
    motor.save()
    ```

### 电机运行参数

通过以下方式设置电机运行参数：

=== "命令行工具"

    ``` bash
    robodyno motor --id 0x10 config
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    # 设置 PID
    motor.set_pid(100, 0.02, 0.1)
    print(motor.get_pid())

    # 设置电流限制
    motor.set_current_limit(10)
    print(motor.get_current_limit())

    # 设置速度限制
    motor.set_vel_limit(5)
    print(motor.get_vel_limit())

    # 保存设置
    motor.save()
    ```

## 错误

如果电机运行过程中出现异常，可以通过以下方式查看错误信息：

=== "命令行工具"

    ``` bash
    robodyno motor info
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    print(motor.error)
    ```

完整的错误列表请参考 [电机错误](../../../references/components/can_bus/motor/#robodyno.components.can_bus.motor.Motor.MotorError)。

在确保电机没有异常的情况下，可以通过以下方式清除错误：

=== "命令行工具"

    ``` bash
    robodyno motor clear-errors
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    motor.clear_errors()
    ```

### 紧急停止

如果电机运行过程中出现紧急情况，可以通过以下方式紧急停止电机：

!!! warning

    紧急停止并不会让电机锁定，相反，电机会进入错误状态并且失去动力，此时电机会变成一个惯性负载，需要通过其他方式停止电机。

=== "命令行工具"

    ``` bash
    robodyno motor estop
    ```

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor

    can_bus = CanBus()
    motor = Motor(can_bus)

    motor.estop()
    ```

紧急停止后，电机会进入错误状态，可以清除错误后重新使用电机。

## 数字孪生

Robodyno 电机组件提供了基于 Webots 的仿真模型，可以通过在 Webots 环境中初始化电机时添加 `twin` 参数来启用数字孪生功能。

```python
from robodyno.components import Motor
from robodyno.interfaces import Webots, CanBus
webots = Webots()
can_bus = CanBus()
motor = Motor(webots, 0x10, twin = can_bus)

while True:
    print(motor.get_pos())
    if webots.sleep(1) == -1:
        break
```

!!! note

    使用数字孪生功能时，会占用 CAN 总线实时读取电机的位置信息。因此，在 Windows 系统上，无法同时在 Webots 环境外使用 CAN 总线实时控制电机；在 Linux 系统上，可能会影响其他程序通过 CAN 总线实时控制电机的性能。

仿真模型的下载方式和详细使用方法请参考 [Webots 仿真接口](../../interfaces/webots/)。

## 硬件参数

| 参数       | `ROBODYNO_PRO_P44` | `ROBODYNO_PRO_P12` |
| ---------- | ------------------ | ------------------ |
| 电源输入   | 11.1V ~ 25.2V      |
| 额定功率   | 35W                |
| 最大相电流 | 15A                |
| 减速比     | 44:1               | 12.45:1            |
| 额定转速   | 52RPM              | 190RPM             |
| 额定扭矩   | 6.57Nm             | 1.86Nm             |
| 最大转速   | 85RPM              | 310RPM             |
| 最大扭矩   | 9.86Nm             | 2.79Nm             |

## API

| 组件                                                          | 说明                      |
| ------------------------------------------------------------- | ------------------------- |
| [can_bus.motor](../../../references/components/can_bus/motor) | CAN 总线伺服减速电机类    |
| [webots.motor](../../../references/components/webots/motor)   | Webots 仿真伺服减速电机类 |

## 命令行工具

| 命令                                      | 说明                   |
| ----------------------------------------- | ---------------------- |
| [robodyno motor](../../../commands/motor) | 伺服减速电机命令行工具 |
