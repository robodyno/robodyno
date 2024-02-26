## 四自由度码垛机械臂
`four_dof_pallet_robot`

### 1. 使用方法
#### 1.1 查看库的使用方法
- 使用下面方法来打印库中对应的API说明注释:
```python
#  从robodyno中导入四自由度码垛机械臂库
from robodyno_robots_python.four_dof_palletizing_robot import four_dof_pallet_robot
```
```python
#  打印整体库的介绍及例子
print(four_dof_pallet_robot.__doc__)
```
```
four_dof_pallet_robot.py
Time    :   2022/10/17
Author  :   ryan 
Version :   1.0
Contact :   ryanzhang@163.com
License :   (C)Copyright 2022, robottime / robodyno

4DoF Pallet Robot Drive

  Typical usage example:

  from robodyno.robots.four_dof_pallet_robot import FourDoFPallet
  robot = FourDoFPallet(
    j1 = base_motor,
    j2 = upperarm_motor,
    j3 = forearm_motor,
    j4 = hand_motor,
    l01 = 0.0794,
    l23 = 0.190,
    l34 = 0.190,
    l45 = 0.010,
    end_effector = None
  )
```

#### 1.2 4-DoF-Pallet-API
##### 1.2.0 导入四自由度码垛机械臂的类
```python
from robodyno.robots.four_dof_pallet_robot import FourDoFPallet
```

##### 1.2.1 获取关节当前位置
`get_joints_poses()`
- 读取机械臂关节的位置
    - 返回值：
        - `poses`: 包含4个关节位置的列表

```python
#  查看使用方法
print(FourDoFPallet.get_joints_pose.__doc__)
```
```
Read joints position to a list.
        
        Returns:
            a list of 4 joint positions
```

##### 1.2.2 机械臂使能
`enable()`
- 使能机械臂所有关节电机

```python
#  查看使用方法
print(FourDoFPallet.enable.__doc__)
```
```
enable joints motors
```

##### 1.2.3 机械臂失能
`disable()`
- 失能机械臂所有关节电机

```python
#  查看使用方法
print(FourDoFPallet.disable.__doc__)
```
```
disable joints motors
```

##### 1.2.4 机械臂初始化
`init(axes_poses)`
- 记录机械臂当前所有关节电机
    - 参数： 
        - `axes_poses`: 包含机械臂当前4个关节坐标位置的列表(rad),默认为0 。

```python
#  查看使用方法
print(FourDoFPallet.init.__doc__)
```
```
Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses: list of 4 axes poses(rad)
```

##### 1.2.5 设置机械臂关节位置
`set_joint_pos(id, pos)`
- 设置机械臂对应关节电机位置
    - 参数： 
        - `id`: 机械臂对应位置关节（0-3）。
        - `pos`: 对应关节的目标位置(rad) 。

```python
#  查看使用方法
print(FourDoFPallet.set_joint_pos.__doc__)
```
```
Set joint angle with joint and position.
        
        Args:
            id: joint id (0-3)
            pos: joint target position(red)
```

##### 1.2.6 关节空间的插值运动
`joint_space_interpolated_motion(target, speeds, duration)`
- 机械臂在关节空间中的插值运动
    - 参数： 
        - `target`: 机械臂运动的目标位置，包含4个关节角度的列表或元组(rad)。
        - `speeds`: 对应关节的插值运动速度列表(rad/s),默认为None,其优先级高于时间插值 。
        - `duration`: 关节空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(FourDoFPallet.joint_space_interpolated_motion.__doc__)
```
```
Robot interpolated motion in joint space.
        
        Args:
            target: iterable of 4 joints target angle(rad)
            speeds: iterable of 4 joints motion speed(rad/s)
            duration: default motion duration(s)
```

##### 1.2.7 笛卡尔空间的插值运动
`cartesian_space_interpolated_motion(self, x, y, z, x_speed, y_speed, z_speed, duration)`
- 机械臂在笛卡尔空间中的插值运动
    - 参数： 
        - `x`: 机械臂运动的目标位置的x值(m) 。
        - `y`: 机械臂运动的目标位置的y值(m) 。 
        - `z`: 机械臂运动的目标位置的z值(m) 。 
        - `yaw`: 机械臂运动的目标姿态的yaw值(rad) 。
        - `x_speed`: 机械臂x方向的插值运动速度(m/s),默认为None 。
        - `y_speed`: 机械臂y方向的插值运动速度(m/s),默认为None 。
        - `z_speed`: 机械臂z方向的插值运动速度(m/s),默认为None 。
        - `yaw_speed`: 机械臂z轴的插值旋转速度(rad/s),默认为None 。
        - `duration`: 笛卡尔空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(FourDoFPallet.cartesian_space_interpolated_motion.__doc__)

```
```
Robot Interpolated motion in cartesian space.
        
        Args:
            x: target robot end x
            y: target robot end y
            z: target robot end z
            yaw: target robot end yaw
            x_speed: speed alone X dimension(m/s)
            y_speed: speed alone Y dimension(m/s)
            z_speed: speed alone Z dimension(m/s)
            yaw_speed: rotation speed on Z axis(rad/s)
            duration: default motion duration(s)
```

##### 1.2.8 机械臂回到初始位姿
`home(duration)`
- 机械臂回的初始位姿
    - 参数： 
        - `duration`: 机械臂运动持续时间(s),默认为5 。

```python
#  查看使用方法
print(FourDoFPallet.home.__doc__)
```
```
Move back to zero position.
        
        Args:
            duration: motion duration(s)
```

##### 1.2.9 机械臂正向运动学
`forward_kinematics(angles)`
- 机械臂回的初始位姿
    - 参数： 
        - `angles`: 包含机械臂3个关节角度的列表(rad) 。
    - 返回值：
        - `(x, y, z, yaw)`: 机械臂末端位姿的元组 。

```python
#  查看使用方法
print(FourDoFPallet.forward_kinematics.__doc__)
```
```
Forward kinematics algorism
        
        Args:
            angles: list of 4 joint angles(rad)
        
        Returns:
            (x, y, z, yaw): tuples of 3 axis position and 1 axis posture
```

##### 1.2.10 机械臂逆向运动学
`inverse_kinematics(x, y, z)`
- 机械臂回的初始位姿
    - 参数： 
        - `x`: 机械臂的目标位置的x值(m) 。
        - `y`: 机械臂的目标位置的y值(m) 。 
        - `z`: 机械臂的目标位置的z值(m) 。 
    - 返回值：
        - `angle`: 包含机械臂3个关节角度的列表(rad) 。

```python
#  查看使用方法
print(FourDoFPallet.inverse_kinematics.__doc__)
```
```
inverse kinematics algorism
        
        Args:
            x: robot end x
            y: robot end y
            z: robot end z
            yaw: robot end yaw
        
        Returns:
            list of joint angles
```

#### 1.3 具体使用实例
*注：本实例基于Jupyter lab*

##### 1.3.1 导入电机库和can总线
```python
from robodyno.components import Motor
from robodyno.interfaces import CanBus
can = CanBus()
```
##### 1.3.2 导入四自由度码垛机械臂的类
```python
from robodyno.robots.four_dof_pallet_robot import FourDoFPallet
```

##### 1.3.3 查看ThreeDoFPallet类需要的参数
- 使用下面方法可以查看`four_dof_pallet_robot`使用实例
```python
print(four_dof_pallet_robot.__doc__)
```
```
4DoF Pallet Robot Drive

  Typical usage example:

  from robodyno.robots.four_dof_pallet_robot import FourDoFPallet
  robot = FourDoFPallet(
    j1 = base_motor,
    j2 = upperarm_motor,
    j3 = forearm_motor,
    j4 = hand_motor,
    l01 = 0.0794,
    l23 = 0.190,
    l34 = 0.190,
    l45 = 0.010,
    end_effector = None
  )
```
- 在上面实例中，可以了解到`FourDoFPallet`类需要我们传入4个关节电机，以及运动学计算的具体尺寸(m)。

- 也可以使用下面的方法，查看类的具体属性
```python
print(FourDoFPallet.__doc__)
```
```
4 DoF pallet robot driver
    
    Attributes:
        joints: list of 4 joint motors
        l01: link from world to joint 1 (m)
        l23: link from joint 2 to joint 3 transverse distance (m)
        l34: link from joint 3 to joint 4 longitudinal distance (m)
        l45: link from joint 4 to joint 5 transverse distance (m)
        end_effector: end effector object
```
- 三自由度码垛机械臂详细的尺寸说明如下图：
![1](../../robodyno_motor/)

##### 1.3.4 构建机械臂的类
- 构建一个自己的类，并继承FourDoFPallet，使用Motor创建4个关节电机，并将关节电机及运动学计算尺寸传给父类。
- 也可以在类中编写自己的方法，如下面例子中的获取机械臂参数。

***Typical usage example:***

```python
class MyPallet(FourDoFPallet):
    def __init__(self):
        M1 = Motor(can, 0x10)
        M2 = Motor(can, 0x11)
        M3 = Motor(can, 0x12)
        M4 = Motor(can, 0x13)
        
        super().__init__(M1, M2, M3, M4, 0.180, 0.190, 0.190, 0.010)

    def get_delta_parameter(self, parameter):
        """get robot arm joint motors parameter
        
        Args:
            parameter: mode(controller modes),
                       voltage(get voltage),
                       temperature(get temperature),
                       pos(get motors pos),
                       vel(get motors vel),
                       torque(get motors torque)

        Returns:
            motors parameter 
        """
        for motor in self.joints:
            if(parameter == 'mode'):
                mode = motor.get_mode(1)
                return(mode)
            if(parameter == 'voltage'):
                vbus = motor.get_voltage(1)
                return(vbus)
            if(parameter == 'temperature'):
                temperature = motor.get_temperature(1)
                return(temperature)
            if(parameter == 'pos'):
                pos = motor.get_pos(1)
                return(pos)
            if(parameter == 'vel'):
                vel = motor.get_vel(1)
                return(vel)
            if(parameter == 'torque'):
                torque = motor.get_torque(1)
                return(torque)  
```
##### 1.3.5 调用类的方法控制机械臂运动
- 创建好自己的类之后，我们就调用对应的方法来控制机械臂，具体类的方法可以参考`4-DoF-Pallet-API` 。

***Typical usage example:***

 定义对象。
```python
arm = MyPallet()
```

- 使能
```python
arm.enable()
```

- 获取关节位置
```python
arm.get_joints_poses()
```

- 初始化机械臂
```python
arm.init()
```

- 关节坐标系
```python
from ipywidgets import interact
from math import pi
@interact(a1=(-pi, pi, 0.01), a2=(-pi, pi, 0.01), a3=(-pi, pi, 0.01), a4=(-pi, pi, 0.01))
def calibrate(a1, a2, a3, a4):
    arm.set_joint_pos(0, a1)
    arm.set_joint_pos(1, a2)
    arm.set_joint_pos(2, a3)
    arm.set_joint_pos(3, a4)
```

- 笛卡尔坐标系
```python
from ipywidgets import interact
from math import pi
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.19, 0, 0.36, 0.0),duration = 2)
@interact(x = (-200, 200, 1), y = (-200, 200, 1), z = (330, 450, 1), yaw = (-pi, pi, 0.01))
def calibrate(x = 190, y = 0, z = 360, yaw = 0.0):
    arm.joint_space_interpolated_motion(arm.inverse_kinematics(x/1000, y/1000, z/1000, yaw))
```

- 正向运动学
```python
print(arm.forward_kinematics((arm.get_joints_poses())))
```
```python
print(arm.forward_kinematics((pi/4, pi/4, pi/4, pi/2)))
```

- 逆向运动学
```python
print(arm.inverse_kinematics(*arm.forward_kinematics((arm.get_joints_poses()))))
```
```python
print(arm.inverse_kinematics(0, 0.19, 0.36, 0.0))
```

- 关节空间运动
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.229, 0.229, 0.314, pi/2),duration = 5)
```
```python
arm.joint_space_interpolated_motion((pi/4, pi/4, pi/4, 0.0),speeds=[None,0.1,0.1],duration = 5)
```

- 笛卡尔空间运动
```python
arm.cartesian_space_interpolated_motion((0.229, 0.229, 0.314, 0.0), duration = 5)
```
```python
arm.cartesian_space_interpolated_motion(*arm.forward_kinematics((pi/4, pi/4, pi/4, 0.0)), y_speed=0.01, duration = 5)
```

- Pallet arm 参数查询
```python
arm.get_delta_parameter('mode') # 电机模式
arm.get_delta_parameter('voltage') # 电机总线电压
arm.get_delta_parameter('temperature') # 电机温度
arm.get_delta_parameter('pos') # 电机位置
arm.get_delta_parameter('vel') # 电机速度
arm.get_delta_parameter('torque') # 电机力矩
```

- 回原点
```python
arm.home(8)
```

- 失能
```python
arm.disable()
```

### 2. 算法介绍
- 我们约定机械臂算法遵循以下规则：
    - 机器人中的坐标轴使用x，y和z轴。正面是x轴的正方向，轴是红色（Red）。左边是y轴的正方向，轴用绿色（Green）表示。最后，上方是z轴的正方向，轴用蓝色（Blue）表示。遵循右手定则，可以将x轴视为食指，将y轴视为中指，将z轴视为拇指。顺序是x、y、z，且颜色是RGB颜色顺序。
    - 机器人的旋转轴使用RPY世界系欧拉角。RPY是roll（滚转），pitch（俯仰），yaw（偏航）的合写，分别代表了绕世界系 x，y，z三个轴的旋转。旋转的正方向也遵循右手定则，用右手卷住的方向是正（+）方向，如下图。
    ![U5zXuh.png](https://m1.im5i.com/2022/12/05/U5zXuh.png)
    <center>图2 机械臂的坐标系说明</center>
#### 2.1 运动学
- 码垛机械臂包含三个自由度,即三个坐标位置$(x, y, z)$。其连杆尺寸及坐标建立如下：

##### 2.1.1 正向运动学
已知码垛三个关节电机的角度值，求解末端位姿$(x, y, z)$。
- 建立码垛机械臂机构简化数学模型，如下图所示。以底座中心$O$建立坐标系$O-XYZ$,设底座中心点到第二个关节电机的高度为$l_{01}$，两连杆长度分别为$l_{23}$,$l_{34}$,到手爪长度为$l_{45}$。机械臂末端与连杆到地面的投影交于点$E$。

![U56MEW.png](https://m1.im5i.com/2022/12/13/U56MEW.png)
<center>图3 码垛机械臂的简化数学模型</center>

- 连接$AC$，设$∠BAC$为$\alpha$，$∠ZAC$为$\beta$。如下图。
![U56j7x.png](https://m1.im5i.com/2022/12/13/U56j7x.png)
<center>图4 码垛机械臂的几何法正解简化模型</center>

- 从图4中不难发现，$∠ABC$为
$$∠ABC = \theta_3 + \frac{\pi}{2} \tag {1}$$

- 设$|AC| = r$，使用余弦公式可得$AC$长度为
$${|AC|}^2 = {l_{23}}^2 +{l_{34}}^2 - 2 \times l_{23} \times l_{34} \times cos(\theta_3 + \frac{\pi}{2}) = r^2 \tag {2}$$

- 在$∆ABC$中，已知三边，可求得角$\alpha$为
$$cos\alpha = \frac{{l_{23}}^2 + r^2 - {l_{34}}^2}{2 \times l_{23} \times \sqrt{r^2}} \tag {3}$$

- 由内错角相等，可知
$$∠\beta = \theta_2 + \alpha = ∠ACE \tag {4}$$

- 则，$AC$的投影$OE$为
$$|OE| = |AC|sin\beta = r sin(\theta_2 + \alpha) \tag {5}$$

综上，可求得码垛机械臂正向运动学为
$$
\begin{cases}
 {x = |OE|cos\theta_1}  \\
 {y = |OE|sin\theta_1}  \\
 {z = r cos(\theta_2 + \alpha)+l_{01}-l_{45}}  \\
 {yaw = \theta_4 - \theta_1}
\end{cases}
 \tag {6}$$

##### 2.1.2 逆向运动学
已知码垛机械臂末端位姿$(x, y, z)$ ，求解三个关节电机的角度值。
- 与正向运动学类似，建立码垛机械臂机构简化数学模型，如下图所示。以底座中心$O$建立坐标系$O-XYZ$,设底座中心点到第二个关节电机的高度为$l_{01}$，两连杆长度分别为$l_{23}$,$l_{34}$,到手爪长度为$l_{45}$。机械臂末端的垂线与连杆到地面的投影交于点$E$。连接$AC$，过$A$点作$CE$的垂线，垂足为$F$。
![U560Ao.png](https://m1.im5i.com/2022/12/13/U560Ao.png)
<center>图5 码垛机械臂的几何法逆解解简化模型</center>

- 由图5可知$|FE| = l_{01}$, $|DE| = z$,因此   
$$|CF| = |DE| - |FE| + |CD| = z - l_{01} + l_{45} \tag {7}$$   

- 点$E$为末端到地面垂足，则
$$|OE| = |AF| = \sqrt{x^2 + y^2} \tag {8}$$

- 在$∆AFC$中，可求得$|AC|$
$$|AC| = \sqrt{{|AF|}^2 + {CF}^2} \tag {9}$$

- 因此，$∠ \beta$为
$$tan\beta = \frac{|AF|}{|CF|} \tag {10}$$

- 由式(3)，可求得$∠ \alpha$，在$∆ABC$中，$∠ABC$为
$$cos{ABC}=\frac{{l_{23}}^2 + {l_{34}}^2 - r^2}{2 \times l_{23} \times l_{34}} \tag {11}$$

综上，可求得码垛机械臂逆向运动学为
$$
\begin{cases}
 {\theta_1 = atan(\frac{y}{x})}  \\
 {\theta_2 = \beta - \alpha}  \\
 {\theta_3 = ∠ABC - \frac{\pi}{2}}  \\
 {\theta_4 = yaw + \theta_1}
\end{cases}
 \tag {12}$$


