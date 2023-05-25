# 深入了解

## 接口

Robodyno 的硬件模组是基于 CAN 总线的，因此你可以通过 CAN 总线来控制 Robodyno 的硬件模组。CAN 总线是一种串行通信协议，它可以在不同的设备之间进行通信，详细的介绍请参考 [CAN 总线](interfaces/can-bus/)。

于此同时，Robodyno 还提供了基于 Webots 的仿真环境，你可以在仿真环境中测试你的代码，详细的介绍请参考 [Webots仿真环境](interfaces/webots/)。

## 组件

当前 Robodyno 的硬件模组包含了以下组件：

* [伺服减速电机](components/motor/)
* [直线运动模组](components/slider-module/)
* [PWM 扩展模块](components/pwm-driver/)
* [步进电机驱动模块](components/stepper-driver/)
* [IMU 传感器](components/imu-sensor/)

当前，所有组件都支持 CAN 总线，伺服减速电机和直线运动模组支持 Webots 仿真环境。同时，我们在命令行工具中提供了伺服减速电机的完整控制命令，具体请查看 [电机命令行说明文档](../../commands/motor)，其他组件的控制命令将在后续版本中提供。