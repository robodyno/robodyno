## 三自由度笛卡尔机械臂
`three_dof_cartesian_robot`

### 1. 使用方法
#### 1.1 查看库的使用方法
- 使用下面方法来打印库中对应的API说明注释:
```python
#  从robodyno中导入三自由度笛卡尔机械臂库
from robodyno_robots_python.three_dof_cartesian_robot import three_dof_cartesian_robot
```
```python
#  打印整体库的介绍及例子
print(three_dof_cartesian_robot.__doc__)
```
```
three_dof_cartesian_robot.py
Time    :   2023/05/10
Author  :   ryan 
Version :   1.0
Contact :   ryan@163.com
License :   (C)Copyright 2022, robottime / robodyno

3DoF Cartesian Robot Drive

  Typical usage example:

  from robodyno.robots.three_dof_cartesian_robot import ThreeDoFCartesian
  robot = ThreeDoFCartesian(
    j1 = cartesian_motor1,
    j2 = cartesian_motor2,
    j3 = cartesian_motor3,
    screw_lead_x = 0.102,
    screw_lead_y = 0.102,
    screw_lead_z = 0.102,
    end_effector = None
  )
```

#### 1.2 3-DoF-Cartesian-API
##### 1.2.0 导入三自由度笛卡尔机械臂的类
```python
from robodyno.robots.three_dof_cartesian_robot import ThreeDoFCartesian
```

##### 1.2.1 获取关节当前位置
`get_joints_poses()`
- 读取机械臂关节的位置
    - 返回值：
        - `poses`: 包含三个关节位置的列表

```python
#  查看使用方法
print(ThreeDoFCartesian.get_joints_pose.__doc__)
```
```
Read joints position to a list.
        
        Returns:
            a list of 3 joint positions
```

##### 1.2.2 机械臂使能
`enable()`
- 使能机械臂所有关节电机

```python
#  查看使用方法
print(ThreeDoFCartesian.enable.__doc__)
```
```
enable joints motors
```

##### 1.2.3 机械臂失能
`disable()`
- 失能机械臂所有关节电机

```python
#  查看使用方法
print(ThreeDoFCartesian.disable.__doc__)
```
```
disable joints motors
```

##### 1.2.4 机械臂初始化
`init(axes_poses)`
- 记录机械臂当前所有关节电机
    - 参数： 
        - `axes_poses`: 包含机械臂当前三个关节坐标位置的列表(rad),默认为0 。

```python
#  查看使用方法
print(ThreeDoFCartesian.init.__doc__)
```
```
Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses: list of 3 axes poses(rad)
```

##### 1.2.5 设置机械臂关节位置
`set_joint_pos(id, pos)`
- 设置机械臂对应关节电机位置
    - 参数： 
        - `id`: 机械臂对应位置关节（0-2）。
        - `pos`: 对应关节的目标位置(rad) 。

```python
#  查看使用方法
print(ThreeDoFCartesian.set_joint_pos.__doc__)
```
```
Set joint angle with joint and position.
        
        Args:
            id: joint id (0-2)
            pos: joint target position(red)
```

##### 1.2.6 关节空间的插值运动
`joint_space_interpolated_motion(target, speeds, duration)`
- 机械臂在关节空间中的插值运动
    - 参数： 
        - `target`: 机械臂运动的目标位置，包含三个关节角度的列表或元组(rad)。
        - `speeds`: 对应关节的插值运动速度列表(rad/s),默认为None,其优先级高于时间插值 。
        - `duration`: 关节空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(ThreeDoFCartesian.joint_space_interpolated_motion.__doc__)
```
```
Robot interpolated motion in joint space.
        
        Args:
            target: iterable of 3 joints target angle(rad)
            speeds: iterable of 3 joints motion speed(rad/s)
            duration: default motion duration(s)
```

##### 1.2.7 笛卡尔空间的插值运动
`cartesian_space_interpolated_motion(self, x, y, z, x_speed, y_speed, z_speed, duration)`
- 机械臂在笛卡尔空间中的插值运动
    - 参数： 
        - `x`: 机械臂运动的目标位置的x值(m) 。
        - `y`: 机械臂运动的目标位置的y值(m) 。 
        - `z`: 机械臂运动的目标位置的z值(m) 。 
        - `x_speed`: 机械臂x方向的插值运动速度(m/s),默认为None 。
        - `y_speed`: 机械臂y方向的插值运动速度(m/s),默认为None 。
        - `z_speed`: 机械臂z方向的插值运动速度(m/s),默认为None 。
        - `duration`: 笛卡尔空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(ThreeDoFCartesian.cartesian_space_interpolated_motion.__doc__)

```
```
Robot Interpolated motion in cartesian space.
        
        Args:
            x: target robot end x
            y: target robot end y
            z: target robot end z
            x_speed: speed alone X dimension(m/s)
            y_speed: speed alone Y dimension(m/s)
            z_speed: speed alone Z dimension(m/s)
            duration: default motion duration(s)
```

##### 1.2.8 机械臂回到初始位姿
`home(duration)`
- 机械臂回的初始位姿
    - 参数： 
        - `duration`: 机械臂运动持续时间(s),默认为5 。

```python
#  查看使用方法
print(ThreeDoFCartesian.home.__doc__)
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
        - `(x, y, z)`: 机械臂末端位姿的元组 。

```python
#  查看使用方法
print(ThreeDoFCartesian.forward_kinematics.__doc__)
```
```
Forward kinematics algorism
        
        Args:
            angles: list of 3 joint angles(rad)
        
        Returns:
            (x, y, z): tuples of 3 axis position
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
print(ThreeDoFCartesian.inverse_kinematics.__doc__)
```
```
inverse kinematics algorism
        
        Args:
            x: robot end x
            y: robot end y
            z: robot end z
        
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
##### 1.3.2 导入三自由度笛卡尔机械臂的类
```python
from robodyno_robots_python.three_dof_cartesian_robot import ThreeDoFCartesian
```

##### 1.3.3 查看ThreeDoFCartesian类需要的参数
- 使用下面方法可以查看`three_dof_cartesian_robot`使用实例
```python
print(three_dof_cartesian_robot.__doc__)
```
```
3DoF Cartesian Robot Drive

  Typical usage example:

  from robodyno.robots.three_dof_cartesian_robot import ThreeDoFCartesian
  robot = ThreeDoFCartesian(
    j1 = cartesian_motor1,
    j2 = cartesian_motor2,
    j3 = cartesian_motor3,
    screw_lead_x = 0.102,
    screw_lead_y = 0.102,
    screw_lead_z = 0.102,
    end_effector = None
  )
```
- 在上面实例中，可以了解到`ThreeDoFCartesian`类需要我们传入3个关节电机，以及运动学计算的具体尺寸(m)。

- 也可以使用下面的方法，查看类的具体属性
```python
print(ThreeDoFCartesian.__doc__)
```
```
3 DoF Cartesian robot driver
    
    Attributes:
        joints: list of 3 joint motors
        screw_lead_x: The lead screw on the x axis (m)
        screw_lead_y: The lead screw on the y axis (m)
        screw_lead_z: The lead screw on the z axis (m)
        end_effector: end effector object
```
- 三自由度笛卡尔机械臂详细的尺寸说明如下图：  
![1](../../robodyno_motor/)

##### 1.3.4 构建机械臂的类
- 构建一个自己的类，并继承ThreeDoFCartesian，使用Motor创建3个关节电机，并将关节电机及运动学计算尺寸传给父类。
- 也可以在类中编写自己的方法，如下面例子中的获取机械臂参数。

***Typical usage example:***

```python
class MyCartesian(ThreeDoFCartesian):
    def __init__(self):
        M1 = Motor(can, 0x10, 'ROBODYNO_PRO_44')
        M2 = Motor(can, 0x11, 'ROBODYNO_PRO_44')
        M3 = Motor(can, 0x12, 'ROBODYNO_PRO_44')
        
        super().__init__(M1, M2, M3, 0.0102, 0.0102, 0.0102)

    def circle(self, xo, yo, zo, r, num=50, sn=True):
        """robot arm draw circle
        
        Args:
            xo: X coordinates of the center of the circle(m)
            yo: Y coordinates of the center of the circle(m)
            zo: Z coordinates of the center of the circle(m)
            r: Radius of circle(m)
            num: Number of copies of a circle,fifty of copies of a circle by default
            sn: The direction of drawing a circle,Draw a circle clockwise by default

        Returns:
            list of joint angles
        """
        if sn:
            self.start_theta -= 2*pi / num
            if (self.start_theta < 0):
                self.start_theta += 2*pi 
        else:
            self.start_theta += 2*pi / num
            if (self.start_theta > 2*pi):
                self.start_theta -= 2*pi

        x = xo + r * cos(self.start_theta)
        y = yo + r * sin(self.start_theta)
        z = zo
        angles = self.inverse_kinematics(x, y, z)
        return angles 
```
##### 1.3.5 调用类的方法控制机械臂运动
- 创建好自己的类之后，我们就调用对应的方法来控制机械臂，具体类的方法可以参考`3-DoF-Cartesian-API` 。

***Typical usage example:***

 定义对象。
```python
arm = MyCartesian()
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
@interact(a1=(-15*pi, 15*pi, 0.1), a2=(-15*pi, 15*pi, 0.1), a3=(-15*pi, 15*pi, 0.1))
def calibrate(a1=0, a2=0, a3=0):
    arm.set_joint_pos(0, a1)
    arm.set_joint_pos(1, a2)
    arm.set_joint_pos(2, a3)
```

- 笛卡尔坐标系
```python
from ipywidgets import interact
from math import pi
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0, 0, 0),duration = 2)
@interact(x = (-105, 105, 1), y = (-105, 105, 1), z = (-105, 105, 1))
def calibrate(x = 0, y = 0, z = 0):
    arm.joint_space_interpolated_motion(arm.inverse_kinematics(x/1000, y/1000, z/1000))
```

- 正向运动学
```python
print(arm.forward_kinematics((arm.get_joints_poses())))
```
```python
print(arm.forward_kinematics((pi/4, pi/4, pi/4)))
```

- 逆向运动学
```python
print(arm.inverse_kinematics(*arm.forward_kinematics((arm.get_joints_poses()))))
```
```python
print(arm.inverse_kinematics(0, 0.19, 0.36))
```

- 关节空间运动
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.229, 0.229, 0.314),duration = 5)
```
```python
arm.joint_space_interpolated_motion((pi/4, pi/4, pi/4),speeds=[None,0.1,0.1],duration = 5)
```

- 笛卡尔空间运动
```python
arm.cartesian_space_interpolated_motion((0.229, 0.229, 0.314), duration = 5)
```
```python
arm.cartesian_space_interpolated_motion(*arm.forward_kinematics((pi/4, pi/4, pi/4)), y_speed=0.01, duration = 5)
```

- 画圆
```python
from IPython.display import clear_output
for motor in arm.motors:
    motor.position_filter_mode(10)
arm.delay(2)
arm.move_to_axis(arm.circle(0, 0, -0.005, 0.30, 500, False),2)
arm.delay(0.01)
arm.move_to_axis(arm.circle(0, 0, -0.005, 0.30, 500, False),2)
for i in range(1000):
    # clear_output(wait = True)
    arm.move_to_axis(arm.circle(0, 0, -0.005, 0.30, 500, False))
    arm.delay(0.1)
arm.delay(1)
arm.home(2)
arm.delay(1)
arm.move_to_axis(arm.circle(0, 0, -0.005, 0.20, 500, False),2)
for i in range(1000):
    # clear_output(wait = True)
    arm.move_to_axis(arm.circle(0, 0, -0.005, 0.20, 500, False))
    arm.delay(0.1)
arm.delay(1)
arm.home(2)
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
- 笛卡尔机械臂包含三个自由度,即三个坐标位置$(x, y, z)$。其连杆尺寸及坐标建立如下：

##### 2.1.1 正向运动学
已知笛卡尔三个关节电机的角度值，求解末端位姿$(x, y, z)$。
- 建立笛卡尔机械臂机构简化数学模型，如下图所示。。以底座中心$O$建立坐标系$O-XYZ$,由笛卡尔机器人可知，$X,Y,Z$三轴均为丝杆上下平移运动，分别设$X$轴丝杆螺距 $S_X$ ，设$Y$轴丝杆螺距 $S_Y$ ，设$Z$轴丝杆螺距 $S_Z$ 。  

![U56MEW.png](https://m1.im5i.com/2022/12/13/U56MEW.png)
<center>图3 笛卡尔机械臂的简化数学模型</center>

- 连接$AC$，设$∠BAC$为$\alpha$，$∠ZAC$为$\beta$。如下图。  
![U56j7x.png](https://m1.im5i.com/2022/12/13/U56j7x.png)
<center>图4 笛卡尔机械臂的几何法正解简化模型</center>

- 末端 $B$ 的 $x$ 坐标为：
$$x = \frac {\theta_1 S_X}{2\pi} \tag {1}$$

- 末端 $B$ 的 $y$ 坐标为：
$$y = \frac {\theta_2 S_Y}{2\pi} \tag {2}$$

- 末端 $B$ 的 $z$ 坐标为：
$$z = \frac {\theta_3 S_Z}{2\pi} \tag {3}$$

综上，可求得笛卡尔机械臂正向运动学为
$$
\begin{cases}
 {x = \frac {\theta_1 S_X}{2\pi}}  \\
 {y = \frac {\theta_2 S_Y}{2\pi}}  \\
 {z = \frac {\theta_3 S_Z}{2\pi}}  \\                
\end{cases}
 \tag {4}$$

##### 2.1.2 逆向运动学
已知笛卡尔机械臂末端位姿$(x, y, z)$ ，求解三个关节电机的角度值。
- 与正向运动学类似，建立笛卡尔机械臂机构简化数学模型，如下图所示。以底座中心$O$建立坐标系$O-XYZ$,由笛卡尔机器人可知，$X,Y,Z$三轴均为丝杆上下平移运动，分别设$X$轴丝杆螺距 $S_X$ ，设$Y$轴丝杆螺距 $S_Y$ ，设$Z$轴丝杆螺距 $S_Z$ 。  
![U560Ao.png](https://m1.im5i.com/2022/12/13/U560Ao.png)
<center>图5 笛卡尔机械臂的几何法逆解解简化模型</center>

- 末端 $B$ 的 $x$ 坐标为：
$$\theta_1 = \frac {{2\pi}{x}}{S_X}  \tag {5}$$

- 末端 $B$ 的 $y$ 坐标为：
$$\theta_2 = \frac {{2\pi}{y}}{S_Y}  \tag {6}$$

- 末端 $B$ 的 $z$ 坐标为：
$$\theta_3 = \frac {{2\pi}{z}}{S_Z}  \tag {7}$$


综上，可求得笛卡尔机械臂逆向运动学为
$$
\begin{cases}
 {\theta_1 = \frac {{2\pi}{x}}{S_X}}  \\
 {\theta_2 = \frac {{2\pi}{y}}{S_Y}}  \\
 {\theta_3 = \frac {{2\pi}{z}}{S_Z}}                  
\end{cases}
 \tag {8}$$


