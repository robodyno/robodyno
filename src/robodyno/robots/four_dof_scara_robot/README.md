## 四自由度Scara机械臂
`four_dof_scara_robot`

### 1. 使用方法
#### 1.1 查看库的使用方法
- 使用下面方法来打印库中对应的API说明注释:
```python
#  从robodyno中导入四自由度Scara机械臂库
from robodyno.robots.four_dof_scara_robot import four_dof_scara_robot
```
```python
#  打印整体库的介绍及例子
print(four_dof_scara_robot.__doc__)
```
```
four_dof_scara_robot.py
Time    :   2022/10/17
Author  :   ryan 
Version :   1.0
Contact :   ryanzhang@163.com
License :   (C)Copyright 2022, robottime / robodyno

4DoF Scara robot Driver

  Typical usage example:

  from robodyno.robots.four_dof_scara_robot import FourDoFScara
  robot = FourDoFScara(
    j1 = shoulder_motor,
    j2 = upperarm_motor,
    j3 = forearm_motor,
    j4 = hand_motor,
    l12 = 0.100,
    l23 = 0.185,
    l34 = 0.185,
    l04 = 0.255
    screw_lead = 0.0102,
    end_effector = None
  )
```

#### 1.2 4-DoF-Scara-API
##### 1.2.0 导入四自由度Scara机械臂的类
```python
from robodyno.robots.four_dof_scara_robot import FourDoFScara
```

##### 1.2.1 获取关节当前位置
`get_joints_poses()`
- 读取机械臂关节的位置
    - 返回值：
        - `poses`: 包含四个关节位置的列表

```python
#  查看使用方法
print(FourDoFScara.get_joints_poses.__doc__)
```
```
Read joints positions to a list.
        
        Returns:
            a list of 4 joint positions
```

##### 1.2.2 机械臂使能
`enable()`
- 使能机械臂所有关节电机

```python
#  查看使用方法
print(FourDoFScara.enable.__doc__)
```
```
enable joints motors
```

##### 1.2.3 机械臂失能
`disable()`
- 失能机械臂所有关节电机

```python
#  查看使用方法
print(FourDoFScara.disable.__doc__)
```
```
disable joints motors
```

##### 1.2.4 机械臂初始化
`init(axes_poses)`
- 记录机械臂当前所有关节电机
    - 参数： 
        - `axes_poses`: 包含机械臂当前四个关节坐标位置的列表(rad),默认为0 。

```python
#  查看使用方法
print(FourDoFScara.init.__doc__)
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
print(FourDoFScara.set_joint_pos.__doc__)
```
```
Set joint angle with joint id and position
        
        Args:
            id: joint id (0-3)
            pos: joint target position(rad)
```

##### 1.2.6 关节空间的插值运动
`joint_space_interpolated_motion(target, speeds, duration)`
- 机械臂在关节空间中的插值运动
    - 参数： 
        - `target`: 机械臂运动的目标位置，包含四个关节角度的列表或元组(rad)。
        - `speeds`: 对应关节的插值运动速度列表(rad/s),默认为None,其优先级高于时间插值 。
        - `duration`: 关节空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(FourDoFScara.joint_space_interpolated_motion.__doc__)
```
```
Robot interpolated motion in joint space.
        
        Args:
            target: iterable of 4 joints target angle(rad)
            speeds: iterable of 4 joints motion speed(rad/s)
            duration: default motion duration(s)
```

##### 1.2.7 笛卡尔空间的插值运动
`cartesian_space_interpolated_motion(self, x, y, z, yaw, x_speed, y_speed, z_speed, yaw_speed, hand_coordinate, duration)`
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
        - `hand_coordinate`: 机械臂运动学的手系(0 or 1),默认右手系(1) 。
        - `duration`: 笛卡尔空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(FourDoFScara.cartesian_space_interpolated_motion.__doc__)
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
            hand_coordinate: 0 is left hand coordinate system
                             1 is right hand coordinate system
            duration: default motion duration(s)
```

##### 1.2.8 机械臂回到初始位姿
`home(duration)`
- 机械臂回的初始位姿
    - 参数： 
        - `duration`: 机械臂运动持续时间(s),默认为5 。

```python
#  查看使用方法
print(FourDoFScara.home.__doc__)
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
        - `angles`: 包含机械臂4个关节角度的列表(rad) 。
    - 返回值：
        - `(x, y, z, yaw)`: 机械臂末端位姿的元组 。

```python
#  查看使用方法
print(FourDoFScara.forward_kinematics.__doc__)
```
```
Forward kinematics algorism
        
        Args:
            angles: list of 4 joint angles(rad)
        
        Returns:
            (x, y, z, yaw): tuples of 3 axis position and 1 axis posture
```

##### 1.2.10 机械臂逆向运动学
`inverse_kinematics(x, y, z, yaw, hand_coordinate)`
- 机械臂回的初始位姿
    - 参数： 
        - `x`: 机械臂的目标位置的x值(m) 。
        - `y`: 机械臂的目标位置的y值(m) 。 
        - `z`: 机械臂的目标位置的z值(m) 。 
        - `yaw`: 机械臂的目标姿态的yaw值(rad) 。
        - `hand_coordinate`: 机械臂运动学的手系(0 or 1),默认右手系(1) 。
    - 返回值：
        - `angle`: 包含机械臂4个关节角度的列表(rad) 。

```python
#  查看使用方法
print(FourDoFScara.inverse_kinematics.__doc__)
```
```
inverse kinematics algorism
        
        Args:
            x: robot end x
            y: robot end y
            z: robot end z
            yaw: robot end yaw
            hand_coordinate: 0 is left hand coordinate system
                             1 is right hand coordinate system
        
        Returns:
            list of 4 joint angles
```

#### 1.3 具体使用实例
*注：本实例基于Jupyter lab*

##### 1.3.1 导入电机库和can总线
```python
from robodyno.components import Motor
from robodyno.interfaces import CanBus
can = CanBus()
```
##### 1.3.2 导入四自由度Scara机械臂的类
```python
from robodyno_robots_python.four_dof_scara_robot import FourDoFScara
```

##### 1.3.3 查看FourDoFScara类需要的参数
- 使用下面方法可以查看`four_dof_scara_robot`使用实例
```python
print(four_dof_scara_robot.__doc__)
```
```
4DoF Scara robot Driver

  Typical usage example:

  from robodyno.robots.four_dof_scara_robot import FourDoFScara
  robot = FourDoFScara(
    j1 = shoulder_motor,
    j2 = upperarm_motor,
    j3 = forearm_motor,
    j4 = hand_motor,
    l12 = 0.100,
    l23 = 0.185,
    l34 = 0.185,
    l04 = 0.255
    screw_lead = 0.0102,
    end_effector = None
  )
```
- 在上面实例中，可以了解到`FourDoFScara`类需要我们传入4个关节电机，以及运动学计算的具体尺寸(m)。

- 也可以使用下面的方法，查看类的具体属性
```python
print(FourDoFScara.__doc__)
```
```
4 DoF scara robot driver
    
    Attributes:
        joints: list of 4 joint motors
        l12: link from joint 1 to joint 2  (m)
        l23: link from joint 2 to joint 3  (m)
        l34: link from joint 3 to joint 4  (m)
        l04: link from world to joint 4 longitudinal distance (m)
        screw_lead: screw lead (m)
        end_effector: end effector object
```
- 四自由度Scara机械臂详细的尺寸说明如下图：
![U5EFnh.png](https://m1.im5i.com/2022/11/25/U5EFnh.png)
<center>图1 Scara机械臂详细的尺寸</center>

##### 1.3.4 构建机械臂的类
- 构建一个自己的类，并继承FourDoFScara，使用Motor创建4个关节电机，并将关节电机及运动学计算尺寸传给父类。
- 也可以在类中编写自己的算法，如下面例子中的机械臂末端画圆。

***Typical usage example:***

```python
from math import pi, cos, sin
class MyScara(FourDoFScara):
    def __init__(self):
        M1 = Motor(can, 0x10)
        M2 = Motor(can, 0x11)
        M3 = Motor(can, 0x12)
        M4 = Motor(can, 0x13)
        
        super().__init__(M1, M2, M3, M4, 0.06, 0.185, 0.185, 0.255, 0.01)
        self.start_theta = pi

    def circle(self, xo, yo, zo, yaw, r, num=50, sn=True):
        """robot arm draw circle
        
        Args:
            xo: X coordinates of the center of the circle(m)
            yo: Y coordinates of the center of the circle(m)
            zo: Z coordinates of the center of the circle(m)
            r: Radius of circle(m)
            yaw: Robot end yaw(yaw)
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
        angles = self.inverse_kinematics(x, y, z, yaw)
        return angles
```
##### 1.3.5 调用类的方法控制机械臂运动
- 创建好自己的类之后，我们就调用对应的方法来控制机械臂，具体类的方法可以参考`4-DoF-Scara-API` 。

***Typical usage example:***

 定义对象。
```python
arm = MyScara()
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
@interact(a1=(-20*pi, 20*pi, 0.1), a2=(-pi, pi, 0.01), a3=(-pi, pi, 0.01), a4=(-pi, pi, 0.01))
def calibrate(a1=0, a2=0, a3=0, a4=0):
    arm.set_joint_pos(0, a1)
    arm.set_joint_pos(1, a2)
    arm.set_joint_pos(2, a3)
    arm.set_joint_pos(3, a4)
```

- 笛卡尔坐标系
```python
from ipywidgets import interact
from math import pi
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.43, 0, 0.255, 0),duration = 2)
@interact(x = (-125, 430, 1), y = (-430, 430, 1), z = (0, 400, 1), yaw = (-pi, pi, 0.01))
def calibrate(x = 430, y = 0, z = 255, yaw = 0.0):
    arm.joint_space_interpolated_motion(arm.inverse_kinematics(x/1000, y/1000, z/1000, yaw))
    print(" ")
```

- 正向运动学
```python
print(arm.forward_kinematics((arm.get_joints_poses())))
```
```python
print(arm.forward_kinematics((0, 0, 1.57, 0)))
```

- 逆向运动学
```python
print(arm.inverse_kinematics(*arm.forward_kinematics((arm.get_joints_poses()))))
```
```python
print(arm.inverse_kinematics(0.43, 0, 0.235, 0))
```

- 关节空间运动
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.245, 0.185, 0.275, -1.57),duration = 5)
```
```python
arm.joint_space_interpolated_motion((-12.57, 0, 1.57, -1.57),speeds=[None,None,0.1,0.1],duration = 5)
```

- 笛卡尔空间运动
```python
arm.cartesian_space_interpolated_motion((0.245, 0.185, 0.255, 1.57), duration = 5)
```
```python
arm.cartesian_space_interpolated_motion(*arm.forward_kinematics((-12.42, 0, 1.57, -1.57)), y_speed=0.01, hand_coordinate=0,duration = 5)
```

- 画圆
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.2, 0.0, 0.275, 0.0),duration = 5)
arm.joint_space_interpolated_motion(arm.circle(0.2, 0.0, 0.275, 0.0, 0.05, 50, False),duration = 2)
for i in range(100):
    arm.joint_space_interpolated_motion(arm.circle(0.2, 0.0, 0.275, 0.0, 0.05, 50, False))
    arm.delay(0.05)
for j in range(100):
    arm.joint_space_interpolated_motion(arm.circle(0.2, 0.0, 0.275, 0.0, 0.05, 50, True))
    arm.delay(0.05)
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.2, 0.0, 0.275, 0),duration = 2)
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
    ![U5MFfT.png](https://m1.im5i.com/2022/11/25/U5MFfT.png)

    <center>图2 机械臂的坐标系说明</center>

#### 2.1 运动学
- Scara机械臂包含四个自由度,即三个坐标位置和一个旋转姿势$(x, y, z, yaw)$。其连杆尺寸及坐标建立如下：  
![U50yaQ.png](https://m1.im5i.com/2022/11/25/U50yaQ.png)
<center>图3 Scara机械臂详细的尺寸及坐标系说明</center>

##### 2.1.1 正向运动学
已知Scara四个关节电机的角度值，求解末端位姿$(x, y, z, yaw)$。
- 以俯视角度构建下面坐标系求解正运动学  
![U5Ejdo.png](https://m1.im5i.com/2022/11/25/U5Ejdo.png)
<center>图4 Scara机械臂数学简化模型</center>

- 末端 $B$ 的 $x$ 坐标为向量 $OA$ 与向量 $AB$ 在 $x$ 轴上投影之和，末端 $B$ 的 $y$ 坐标亦然：
$$
\begin{cases}
 {x = L_{23}\cos\theta_2 + L_{34}\cos(\theta_2 + \theta_3)}  \\
 {y = L_{23}\sin\theta_2 + L_{34}\sin(\theta_2 + \theta_3)}
\end{cases}
 \tag {1}$$

- 第一轴为丝杆上下平移运动，设丝杆螺距 $s$ ，末端 $B$ 的 $z$ 坐标：
$$z = \frac {\theta_1s}{2\pi} \tag {2}$$

- 末端 $B$ 的姿态角 $yaw$ ：
$$yaw = \theta_2 + \theta_3 + (-\theta_4)  \tag {3}$$

- 综上，scara机器人的运动学正解为：
$$
\begin{cases}
 {x = L_{23}\cos\theta_2 + L_{34}\cos(\theta_2 + \theta_3)}  \\
 {y = L_{23}\sin\theta_2 + L_{34}\sin(\theta_2 + \theta_3)}  \\
 {z = \frac {\theta_1s}{2\pi}}                           \\
 {yaw = \theta_2 + \theta_3 + (-\theta_4) }
\end{cases}
 \tag {4}$$


##### 2.1.2 逆向运动学
已知Scara末端位姿$(x, y, z, yaw)$ ，求解四个关节电机的角度值。
- 以俯视角度构建下面坐标系求解逆运动学  
![U5EhXQ.png](https://m1.im5i.com/2022/11/25/U5EhXQ.png)
<center>图5 Scara机械臂逆解辅助线</center>

- 连接 $OB$ ，过 $B$ 作 $BC$ 垂直于 $OA$ 于 $C$ ，在$\Delta OAB$中，由余弦定理得：
$$\cos(\pi - \theta_3) = \frac {L_{23}^2 + L_{34}^2 - (x^2 + y^2)}{2L_{23}L_{34}}  \tag {5}$$

- 记$c_3=cos\theta_3$ ，上式可写成：
$$c_3 = \frac {x^2 + y^2 - L_{23}^2 - L_{34}^2}{2L_{23}L_{34}} \tag {6}$$

- 记$s_3=sin\theta_3$ ，则 $s_3$ 有两个解：
$$s_3 =± \sqrt {1−c_3^2} \tag {7}$$

取正数解，机器人处于右手系(right handcoor)；取负数解，机器人处于左手系(left handcoor)；特殊地，若$s_2=0$，机器人处于奇异位置(singular position)，此时$\theta_2=k\pi(k\in{Z})$，一般 $\theta_2\in{\{-2\pi,-\pi,0,\pi,2\pi}\}$。

- 至此，可求得$\theta_3$：
$$\theta_3 = atan2(s3, c3) \tag {8}$$

- 从上图中，我们可以求得：
$$\alpha = atan2(y, x) \tag {9}$$

- 在$\Delta OBC$中，记$r=\|OB\|$：
$$sin\beta = \frac {\|BC\|}{\|OB\|} = \frac {L_{34}s_3}{r} \tag {10}$$

$$cos\beta = \frac {\|OC\|}{\|OB\|} = \frac {\|OA\| + \|AC\|}{\|OB\|} = \frac {L_{23} + L_{34}c_3}{r}  \tag {11}$$

- 由于$r>0$，由上式得：
$$\beta = atan2(\sin\beta,\cos\beta) = atan2(L_{34}s_3 , L_{23} + L_{34}c_3) \tag {12}$$

- 至此，可求得$\theta_2$
$$\theta_2 = \alpha - \beta = atan2(y, x) - atan2(L_{34}s_3 , L_{23} + L_{34}c_3) \tag {13}$$

- 设丝杆螺距 $s$ ，可求得$\theta_1$：
$$\theta_1 = \frac {2{\pi}z}{s} \tag {14}$$

- 可求得末端$\theta_4$:
$$\theta_4 = \theta_2 + \theta_3 - yaw \tag {15}$$

- 综上，Scara机器人的运动学逆解为：
$$
\begin{cases}
 {\theta_1 = \frac {2{\pi}z}{s}}  \\
 {\theta_2 = atan2(y, x) - atan2(L_{34}s_3 , L_{23} + L_{34}c_3)}  \\
 {\theta_3 = atan2(s3, c3)}                           \\
 {\theta_4 = \theta_2 + \theta_3 - yaw }
\end{cases}
 \tag {16}$$

##### 2.1.3 参考文献
[1] [scara机器人运动学正逆解](https://blog.csdn.net/maple_2014/article/details/104596998)
