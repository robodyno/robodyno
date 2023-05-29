# PWM 扩展模块

Robodyno PWM 扩展模块同时提供一路可调的电源输出和一路可调的 PWM 输出。可以应用于小功率电机或气动元件的控制以及舵机的控制。

## 引脚定义

| 引脚 |   功能   |           说明           |
| :--: | :------: | :----------------------: |
|  S   | 信号输出 |  通过 CAN 总线调节输出   |
|  V   | 电源输出 | 通过拨码开关调节输出电压 |
|  G   |   GND    |            地            |

## 信号输出

### 输出数字信号

```python
from robodyno.interfaces import CanBus
from robodyno.components import PwmDriver

can_bus = CanBus()
pwm_driver = PwmDriver(can_bus)

# 设置输出信号为高电平
pwm_driver.on()

# 设置输出信号为低电平
pwm_driver.off()
```

### 输出 PWM 信号

```python
from robodyno.interfaces import CanBus
from robodyno.components import PwmDriver

can_bus = CanBus()
pwm_driver = PwmDriver(can_bus)

# 设置输出信号的占空比为 50%
pwm_driver.set_pwm(128)
```

程序中传入的参数为 0 ~ 255 之间的整数，对应的占空比为 0% ~ 100%。

### 输出舵机信号

```python
from robodyno.interfaces import CanBus
from robodyno.components import PwmDriver

can_bus = CanBus()
pwm_driver = PwmDriver(can_bus)

# 设置舵机的角度为 180°
pwm_driver.set_servo(255)
```

程序中传入的参数为 0 ~ 255 之间的整数，对应的角度为 0° ~ 180°。

## 设置 ID

PWM 扩展模块的 ID 默认为 0x21，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import PwmDriver

can_bus = CanBus()
pwm_driver = PwmDriver(can_bus)

# 设置 ID 为 0x22
pwm_driver.config_can_bus(0x22)
```

## 电源输出

PWM 扩展模块的电源输出可以通过两个拨码开关进行调节，拨码开关的状态如下表所示：

| 拨码开关 1 | 拨码开关 2 | 电源输出 |
| :--------: | :--------: | :------: |
|     0      |     0      |   7.5V   |
|     0      |     1      |    5V    |
|     1      |     0      |   8.5V   |
|     1      |     1      |   5.7V   |

## 硬件参数

| 参数     | 值            |
| -------- | ------------- |
| 电源输入 | 11.1V ~ 29.4V |
| 电源输出 | 5V ~ 8.5V     |
| PWM 输出 | 0 ~ 3.3V      |
| 最大电流 | 3A            |

## API

| 组件                                                                    | 说明                    |
| ----------------------------------------------------------------------- | ----------------------- |
| [can_bus.pwm_driver](../../../references/components/can_bus/pwm_driver) | CAN 总线 PWM 扩展模块类 |
