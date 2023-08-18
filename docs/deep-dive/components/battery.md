# 电池模块

Robodyno 电池模块支持嘉佰达电池软件板 UART 接口通用协议，波特率为 9600。

## 使用

通过以下代码可以读取电池模块的当前电压和电流：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Battery

can_bus = CanBus()
battery = Battery(can_bus)

# 读取当前电压
print(battery.get_voltage())

# 读取当前电流
print(battery.get_current())
```

通过以下代码可以读取电池模块其他信息：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Battery

can_bus = CanBus()
battery = Battery(can_bus)

# 读取当前电池当前容量及标称容量
print(battery.get_capacity())

# 读取当前电池循环次数
print(battery.get_cycle_count())

# 读取当前电池出厂日期
print(battery.get_production_date())

# 读取当前电池均衡状态
print(battery.get_balance_status())

# 读取当前电池保护状态
print(battery.get_protection_status())

# 读取当前电池硬件版本
print(battery.get_hardware_version())

# 读取当前电池剩余电量百分比
print(battery.get_rsoc())

# 读取当前电池 FET 状态
print(battery.get_fet())

# 读取当前电池串数
print(battery.get_string_count())

# 读取当前电池温度
print(battery.get_temperature())
```

## 设置

### 设置 ID

电池模块的 ID 默认为 0x0F，可以通过以下代码修改：

```python
from robodyno.interfaces import CanBus
from robodyno.components import Battery

can_bus = CanBus()
battery = Battery(can_bus)

# 设置 ID 为 0x0E
battery.config_can_bus(0x0E)
```

## API
| 组件                                                                    | 说明                      |
| ----------------------------------------------------------------------- | ------------------------- |
| [can_bus.battery](../../../references/components/can_bus/battery) | CAN 总线 电池模块类 |
