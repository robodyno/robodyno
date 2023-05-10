# Robodyno

## 安装
[安装Python(>=3.7)](https://www.python.org/downloads/)

使用pip安装robodyno库

```
pip install robodyno
````

## Python API

### 1. 初始化电机对象

```python
from robodyno.components import Motor
from robodyno.interfaces import CanBus
can = CanBus()
motor = Motor(can, 0x10)
```
`Motor(iface, id, type)`  
参数 ：
- iface : robodyno 接口对象
- id : 电机ID（ 范围从 0x01 到 0x3F ）
- type : 电机的类型，默认自动识别
    - ROBODYNO_PRO_44
    - ROBODYNO_PRO_12
    - ROBODYNO_PRO_50
    - ROBODYNO_PRO_100

### 2. 读取电机状态

```python
motor.state
```
`state`  
返回值 ：
- 电机状态 （1-空闲，8-使能）

### 3. 读取电机错误

```python
motor.error
```
`error`  
返回值 :  
- 电机错误 dict  
    - error : 错误码（1-电压不足，14-急停）  
    - motor : 电机相关错误码  
    - encoder_err : 编码器相关错误码  
    - controller_err : 控制器相关错误码  



### 4. 读取电机模式

```python
motor.mode
```
`mode`  
返回值：  
- 电机模式 dict （control_mode，input_mode）  
- control_mode : 控制模式（ 1-力矩模式，2-速度模式，3-位置模式 ）  
- input_mode : 输入模式（ 1-直接值，2-带加速度，3-带滤波，5-梯形轨迹 )  

### 5. 读取电机版本

```python
motor.get_version(1)
```
`get_api_version(timeout = 0)`  
参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值 ：
- API版本dict
    - device_uuid : 设备uuid
    - main_version : 主版本号
    - sub_version : 副版本号

### 6. 电机软急停

```python
motor.estop()
```

`estop()`  

### 7. 电机重启

```python
motor.reboot()
```

`reboot()`  

### 8. 清除错误

```python
motor.clear_errors()
```

`clear_errors()`

### 9. 保存设置
```python
motor.save_configuration()
```

`save_configuration()`

设置参数后默认不会保存，直到调用此函数

### 10. 设置电机CAN_ID

```python
motor.config_can_bus(new_id = 0x11, heartbeat = 1)
```

`config_can_bus(new_id, heartbeat = 1, bitrate = 'CAN_1M')`

参数 ：
- new_id ：电机新CAN_ID (0x01~0x3F)
- heartbeat : 心跳包发送周期 （ s ）

### 11. 电机使能
```python
motor.enable()
```

`enable()`

### 12. 电机失能

```python
motor.disable()
```

`disable()`

### 13. 电机校准
```python
motor.calibrate()
```

`calibrate()`

校准后需保存参数

### 14. 读取总线电压
```python
motor.get_voltage(1)
```

`get_voltage(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值 ：
- 总线电压值 ( V ) ，超时则不返回

### 15. 读取电机温度
```python
motor.get_temperature(1)
```

`get_temperature(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 电机温度 ( °C ) ，超时则不返回

### 16. 读取电机状态参数
```python
motor.get_feedback(1)
```

`get_feedback(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值 ：
- 电机状态参数(位置rad, 速度rad/s, 力矩Nm)，超时则不返回


### 17. 读取电机位置
```python
motor.get_pos(1)
```

`get_pos(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 位置(rad)，超时则不返回

### 18. 读取电机绝对位置
```python
motor.get_abs_pos(1)
```

`get_abs_pos(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 绝对位置(rad)，断电不丢失，超时则不返回

### 19. 读取电机速度
```python
motor.get_vel(1)
```

`get_vel(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 速度(rad/s)，超时则不返回

### 20. 读取电机力矩
```python
motor.get_torque(1)
```

`get_torque(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 力矩(Nm)，超时则不返回

### 21. 读取电机控制模式
```python
motor.get_mode(1)
```
`get_mode(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 控制模式(control_mode, input_mode)
    - control_mode: 控制模式（1：力矩控制，2：速度控制，3：位置控制）
    - input_mode: 输入模式（1：直接值，2：带加速度，3：带滤波，5：梯形轨迹)

### 22. 进入直接位置模式
```python
motor.position_mode()
```
`position_mode()`

直接PID控制位置

### 23. 进入滤波位置模式
```python
motor.position_filter_mode(4)
```
`position_filter_mode(bandwidth)`

参数：
- bandwidth: 滤波带宽 / 控制频率(Hz)

### 24. 进入轨迹位置模式
```python
motor.position_track_mode(10,5,5)
```
`position_track_mode(vel, acc, dec)`

参数:
- vel: 运动最高速度 ( rad/s ) 
- acc: 运动加速度 ( rad/s^2 ) 
- dec: 运动减速度 ( rad/s^2 ) 

### 25. 进入直接速度模式
```python
motor.velocity_mode()
```
`velocity_mode()`

速度PID控制

### 26. 进入匀加减速速度模式
```python
motor.velocity_ramp_mode(1.414)
```
`velocity_ramp_mode(ramp)`

参数:
- ramp: 加速度(rad/s^2)

### 27. 进入力矩控制模式
```python
motor.torque_mode()
```
`torque_mode()`

### 28. 读取电机PID参数
```python
motor.get_pid(1)
```
`get_pid(timeout = 0)`

参数：
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 电机PID
    - pos_kp : 位置环P
    - vel_kp : 速度环P
    - vel_ki : 速度环I

### 29. 设置电机PID参数
```python
motor.set_pid(100,0.02,0.005)
```
`set_pid(pos_kp, vel_kp, vel_ki)`

参数:
- pos_kp: 位置环比例系数
- vel_kp: 速度环比例系数
- vel_ki: 速度环积分系数

### 30. 读取电机速度限制
```python
motor.get_vel_limit()
```
`get_vel_limit(timeout = 0)`

参数:
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 输出端最大速度(rad/s)，超时则不返回

### 31. 读取电机电流限制
```python
motor.get_current_limit()
```

`get_current_limit(timeout = 0)`

参数:
- timeout : 请求超时时间(s)，0代表无超时时间

返回值：
- 最大电流(A)，超时则不返回

### 32. 设置电机速度限制
```python
motor.set_vel_limit(3.14)
```

`set_vel_limit(vel_lim)`

参数：
- vel_lim: 输出端最大速度(rad/s)

### 33. 设置电机电流限制
```python
motor.set_current_limit(10)
```
`set_current_limit(current_lim)`

参数:
- current_lim : 最大电流(A)

### 34. 设置位置
```python
motor.set_pos(-1.57)
```
`set_pos(pos)`

参数:
- pos: 目标位置(rad)

### 35. 设置绝对位置
```python
motor.set_abs_pos(-1.57)
```
`set_abs_pos(pos)`

参数:
- pos: 目标绝对位置(rad)

### 36. 设置速度
```python
motor.set_vel(-0.5)
```
`set_vel(velocity)`

参数：
- velocity: 目标速度(rad/s)

### 37. 设置力矩
```python
motor.set_torque(0.0)
```
`set_torque(torque)`

参数：
- torque: 目标力矩(Nm)

### 38. 恢复出厂设置
```python
motor.reset()
```
`reset()`

## 命令行工具

1. robodyno
- -c, --channel: CAN总线通道选项（可选，默认can0）
- -b, --bitrate: CAN总线速率选项（可选，默认CAN_1M）
- -h, --help: 显示帮助信息

    `robodyno -h`

- list: 列举总线上所有robodyno设备

    -   例：当前CAN总线速率为1000K，通道为can0，列举所有robodyno设备

        `robodyno list`

- monitor: 监听总线上所有消息

    - 例：当前CAN总线速率为500K，监听CAN总线上的实时消息

        `robodyno monitor -b CAN_500K`

2. robodyno-motor
- -c, --channel: CAN总线通道选项（可选，默认can0）
- -b, --bitrate: CAN总线速率选项（可选，默认CAN_1M）
- --id: 需要操作的电机ID（可选，可传多个，默认0x10）
- -h, --help: 显示帮助信息

    `robodyno-motor -h`

- enable: 电机使能

    - 例：将ID为0x10的电机使能

        `robodyno-motor enable`

- disable: 电机失能

    - 例：将ID为0x11的电机失能

        `robodyno-motor disable --id 0x11`

- info: 显示电机状态信息

    - 例：显示ID为0x12、0x13及0x14的电机信息

        `robodyno-motor info --id 0x12 0x13 0x14`

- control: 控制电机运动

    - -p, --pos: 指定电机运动位置(rad)

    - 例：将电机转动到相对开机初始位置180°的位置

        `robodyno-motor control -p 3.14`

    - -ap, --abs-pos: 指定电机绝对位置（仅支持部分型号）(rad)

    - 例：将ID为0x10的电机转动到相对复位位置180°的位置

        `robodyno-motor control -ap 3.14`

    - -v, --vel: 指定电机运动速度(rad/s)

    - 例：操作ID为0x11的电机以15rpm的速度转动

        `robodyno-motor control --id 0x11 -v 1.57`

    - -t, --torque: 指定电机输出力矩(Nm)

    - 例：操作ID为0x12, 0x13及0x14的电机输出0.1Nm的扭矩

        `robodyno-motor control --id 0x12 0x13 0x14 -t 0.1`

- config --new-id [-s, --save]: 修改电机ID，只能同时操作一个电机，-s 保存设置

    - 例：将原ID为0x10的电机ID更改为0x11并保存

        `robodyno-motor config --new-id 0x11 -s`
    
    - 例：将原ID为0x11的电机ID临时更改为0x10（断电后ID仍为0x11）

        `robodyno-motor config --id 0x11 --new-id 0x10`

- reset: 将电机恢复出厂设置（重新校准，ID复原为0x10）
    
    - 例：将ID为0x11的电机恢复出厂设置

        `robodyno-motor reset --id 0x11`
