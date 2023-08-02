# 组件

Robodyno 的所有硬件模组都可以通过 CAN 总线来控制，同时，部分硬件模组还支持 Webots 仿真环境或者命令行工具。

## 组件列表

| 型号                      | 组件                                | 默认设备 ID | CAN 总线                                                       | Webots                                            | 命令行工具                    |
| ------------------------- | ----------------------------------- | ----------- | -------------------------------------------------------------- | ------------------------------------------------- | ----------------------------- |
| `ROBODYNO_PRO_P44`        | [伺服减速电机](motor/)              | 0x10        | [支持](../../../references/components/can_bus/motor/)          | [支持](../../../references/components/webots/motor/)         | [支持](../../commands/motor/) |
| `ROBODYNO_PRO_P12`        | [伺服减速电机](motor/)              | 0x10        | [支持](../../../references/components/can_bus/motor/)          | [支持](../../../references/components/webots/motor/)         | [支持](../../commands/motor/) |
|                           | [直线运动模组](slider-module/)      | 0x10        | [支持](../../../references/components/can_bus/slider_module/)  | [支持](../../../references/components/webots/slider_module/) |                               |
| `ROBODYNO_D_VAC01`        | [PWM 扩展模块](pwm-driver/)         | 0x21        | [支持](../../../references/components/can_bus/pwm_driver/)     |                                                   |                               |
| `ROBODYNO_STEPPER_DRIVER` | [步进电机驱动模块](stepper-driver/) | 0x22        | [支持](../../../references/components/can_bus/stepper_driver/) |                                                   |                               |
| `ROBODYNO_IMU_SENSOR`     | [IMU 传感器](imu-sersor/)           | 0x23        | [支持](../../../references/components/can_bus/imu_sensor/)     |                                                   |                               |
| `ROBODYNO_LED_DRIVER`     | [LED 驱动模块](led-driver/)         | 0x33        | [支持](../../../references/components/can_bus/led_driver/)     |                                                   |                               |
| `ROBODYNO_BATTERY`        | [电池模块](battery/)               | 0x0F        | [支持](../../../references/components/can_bus/battery/)        |                                                   |                               |

## API

| 组件                                                              | 说明                |
| ----------------------------------------------------------------- | ------------------- |
| [can_bus.can_bus_device](../../../references/components/can_bus/) | CAN 总线设备基类    |
| [webots.webots_device](../../../references/components/webots/)               | Webots 仿真设备基类 |
