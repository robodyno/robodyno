# 组件

Robodyno 的所有硬件模组都可以通过 CAN 总线来控制，同时，部分硬件模组还支持 Webots 仿真环境或者命令行工具。

## 组件列表

| 型号                      | 组件                                | 默认设备 ID | CAN 总线                                                          | Webots                                                       | 命令行工具                    |
| ------------------------- | ----------------------------------- | ----------- | ----------------------------------------------------------------- | ------------------------------------------------------------ | ----------------------------- |
| `ROBODYNO_PRO_01A`        | [伺服减速电机](motor/)              | 0x10        | [支持](../../../references/components/can_bus/motor/)             | [支持](../../../references/components/webots/motor/)         | [支持](../../commands/motor/) |
| `ROBODYNO_PRO_01B`        | [伺服减速电机](motor/)              | 0x10        | [支持](../../../references/components/can_bus/motor/)             | [支持](../../../references/components/webots/motor/)         | [支持](../../commands/motor/) |
| `ROBODYNO_PRO_02A`        | [伺服减速电机](motor/)              | 0x10        | [支持](../../../references/components/can_bus/motor/)             | [支持](../../../references/components/webots/motor/)         | [支持](../../commands/motor/) |
| `ROBODYNO_PRO_02B`        | [伺服减速电机](motor/)              | 0x10        | [支持](../../../references/components/can_bus/motor/)             | [支持](../../../references/components/webots/motor/)         | [支持](../../commands/motor/) |
|                           | [直线运动模组](slider-module/)      | 0x10        | [支持](../../../references/components/can_bus/slider_module/)     | [支持](../../../references/components/webots/slider_module/) |                               |
| `ROBODYNO_D_VAC01`        | [PWM 扩展模块](pwm-driver/)         | 0x21        | [支持](../../../references/components/can_bus/pwm_driver/)        |                                                              |                               |
| `ROBODYNO_STEPPER_DRIVER` | [步进电机驱动模块](stepper-driver/) | 0x22        | [支持](../../../references/components/can_bus/stepper_driver/)    |                                                              |                               |
| `ROBODYNO_IMU_SENSOR`     | [IMU 传感器](imu-sersor/)           | 0x23        | [支持](../../../references/components/can_bus/imu_sensor/)        |                                                              |                               |
| `ROBODYNO_LED_DRIVER`     | [LED 驱动模块](led-driver/)         | 0x33        | [支持](../../../references/components/can_bus/led_driver/)        |                                                              |                               |
| `ROBODYNO_BATTERY`        | [电池模块](battery/)                | 0x0F        | [支持](../../../references/components/can_bus/battery/)           |                                                              |                               |
| `ROBODYNO_GPS_SENSOR`     | [GPS 传感器](gps-sensor/)           | 0x31        | [支持](../../../references/components/can_bus/gps_sensor/)        |                                                              |                               |
| `ROBODYNO_IMPACT_SENSOR`  | [防撞条传感器](impact-sensor/)      | 0x34        | [支持](../../../references/components/can_bus/impact_sensor/)     |                                                              |                               |
| `ROBODYNO_ULTRASONIC`     | [超声波传感器](ultrasonic-sensor/)  | 0x35        | [支持](../../../references/components/can_bus/ultrasonic_sensor/) |                                                              |                               |
| `ROBODYNO_CLIFF_SENSOR`   | [悬崖传感器](cliff-sensor/)         | 0x30        | [支持](../../../references/components/can_bus/cliff_sensor/)      |                                                              |                               |
|                           | [磁导传感器](magnetic-sensor/)      | 0x02        | [支持](../../../references/components/can_bus/magnetic_sensor/)      |                                                              |                               |
| `ROBODYNO_PROXIMITY_SENSOR` | [接近开关传感器](proximity-sensor/)| 0x36        | [支持](../../../references/components/can_bus/proximity_sensor/)      |                                                              |                               |

## API

| 组件                                                              | 说明                |
| ----------------------------------------------------------------- | ------------------- |
| [can_bus.can_bus_device](../../../references/components/can_bus/) | CAN 总线设备基类    |
| [webots.webots_device](../../../references/components/webots/)    | Webots 仿真设备基类 |
