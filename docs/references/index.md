# Robodyno API 参考

## 源代码

Robodyno 的源代码可以在 [GitHub](https://github.com/robodyno/robodyno) 或 [Gitee](https://gitee.com/robodyno/robodyno) 上找到。

Robodyno 的源代码遵循 [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) 协议。

## API 文档

### interfaces（接口）

- [CanBus（CAN 总线）](interfaces/can_bus/)
- [Webots（Webots 仿真环境）](interfaces/webots/)

### components（组件）

- [CanBusDevice（CAN 总线设备）](components/can_bus/)

    - [Motor（伺服减速电机）](components/can_bus/motor/)
    - [SliderModule（直线运动模组）](components/can_bus/slider_module/)
    - [PwmDriver（PWM 扩展模块）](components/can_bus/pwm_driver/)
    - [StepperDriver（步进电机驱动模块）](components/can_bus/stepper_driver/)
    - [ImuSensor（IMU 传感器）](components/can_bus/imu_sensor/)

- [WebotsDevice（Webots 设备）](components/webots/)

    - [Motor（伺服减速电机）](components/webots/motor/)
    - [SliderModule（直线运动模组）](components/webots/slider_module/)
