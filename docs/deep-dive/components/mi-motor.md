# 小米赛博电机

小米赛博电机是 Robodyno 的第三方模组，它可以作为机器人的关节或者驱动轮，也可以以其他形式作为机器人的动力源。

## 控制

小米赛博电机可以通过Python API 进行控制。

### 状态

电机需要进入使能（`ENABLE`）状态才能进行控制，否则会被动力系统忽略。

通过以下方式进入使能状态：

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()
    motor = MiMotor(can_bus)

    motor.enable()
    ```

通过以下方式退出使能状态：

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()
    motor = MiMotor(can_bus)

    motor.disable()
    ```

通过以下方式查看电机的状态：

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()
    motor = MiMotor(can_bus)

    print(motor.state)
    ```

### 转动

通过以下方式可以控制电机转动到指定的目标位置：

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()
    motor = MiMotor(can_bus)

    motor.position_mode()
    motor.enable()
    motor.set_pos(6.28)
    ```

通过以下方式可以控制电机按照指定的目标速度转动：

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()
    motor = MiMotor(can_bus)

    motor.velocity_mode()
    motor.enable()
    motor.set_vel(3.14)
    ```

Robodyno 的小米赛博电机的运动模式分为以下几种：

- MIT模式（`MIT`）
- 位置模式（`POSITION`）
- 速度模式（`VELOCITY`）
- 力矩模式（`TORQUE`）

## 设置

### 电机 ID

通过以下方式设置电机 ID：

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()

    current_id = 0x7F
    new_id = 0x77

    motor = MiMotor(can_bus, current_id)
    motor.config_can_bus(new_id = new_id)

    # 保存设置
    motor = MiMotor(can_bus, new_id)
    motor.save()
    ```

### 电机运行参数

通过以下方式设置电机运行参数：

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()
    motor = MiMotor(can_bus)

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

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()
    motor = MiMotor(can_bus)

    print(motor.error)
    ```

完整的错误列表请参考 [电机错误](../../../references/components/can_bus/mi_motor/#robodyno.components.can_bus.mi_motor.MiMotor.MotorError)。

在确保电机没有异常的情况下，可以通过以下方式清除错误：

=== "Python API"

    ``` python
    from robodyno.interfaces import CanBus
    from robodyno.components import MiMotor

    can_bus = CanBus()
    motor = MiMotor(can_bus)

    motor.clear_errors()
    ```

## 硬件参数

| 参数       | `小米 Cyber Gear` |
| ---------- | -------------------- |
| 电源输入   | 11.1V ~ 24V           |
| 额定功率   | 102W                  |
| 最大相电流 | 23A                   |
| 减速比     | 7.75:1                |
| 额定扭矩   | 4Nm                   |
| 最大扭矩   | 12Nm                  |
| 最大转速   | 296RPM                |

## API

| 组件                                                          | 说明                      |
| ------------------------------------------------------------- | ------------------------- |
| [can_bus.mi_motor](../../../references/components/can_bus/mi_motor) | CAN 总线小米赛博电机类    |
