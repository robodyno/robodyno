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
```python
This module provides a class for controlling Robodyno Four DoF Scara Robot.

The FourDoFScara class provided by this module is used to control Robodyno Four DoF Scara Robot
through the CAN bus. It provides methods for setting Four DoF Scara Robot parameters, reading Four DoF Scara Robot
states, and controlling the Four DoF Scara Robot to run in different space.

Examples:

    >>> from robodyno.interfaces import CanBus
    >>> from robodyno.components import Motor, SliderModule
    >>> from robodyno.robots.four_dof_scara_robot import FourDoFScara
    >>> can = CanBus()
    >>> class MyScara(FourDoFScara):
    >>>     def __init__(self):
    >>>         M1 = SliderModule(can, 0x10)
    >>>         M2 = Motor(can, 0x11)
    >>>         M3 = Motor(can, 0x12)
    >>>         M4 = Motor(can, 0x13)
    >>>         super().__init__(M1, M2, M3, M4, 0.24, 0.06, 0.150, 0.150, 0.045)
    >>> arm = MyScara()
    >>> arm.init()
    >>> arm.get_joints_poses()
    [0.0, 0.0, 0.0, 0.0]
    >>> can.disconnect()
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
        - `poses`: 包含1个直线模块距离(m)和3个关节位置(rad)的列表

```python
#  查看使用方法
print(FourDoFScara.get_joints_poses.__doc__)
```
```
Read joints positions to a list.
        
        Returns:
            (list): list of 1 slider distance(m) and 3 joint angles(rad)
            
        Raises:
            RuntimeError: If the motor Joint is invalid.
```

##### 1.2.2 机械臂使能
`enable()`
- 使能机械臂所有关节电机

```python
#  查看使用方法
print(FourDoFScara.enable.__doc__)
```
```python
enable joints motors
```

##### 1.2.3 机械臂失能
`disable()`
- 失能机械臂所有关节电机

```python
#  查看使用方法
print(FourDoFScara.disable.__doc__)
```
```python
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
```python
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
```python
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
```python
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
        - `hand_coordinate`: 机械臂运动学的手系(False or True),默认右手系(True) 。
        - `duration`: 笛卡尔空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(FourDoFScara.cartesian_space_interpolated_motion.__doc__)
```
```python
Robot Interpolated motion in cartesian space.
        
        Args:
            x (float): target robot end x
            y (float): target robot end y
            z (float): target robot end z
            yaw (float): target robot end yaw
            x_speed (float): speed alone X dimension(m/s)
            y_speed (float): speed alone Y dimension(m/s)
            z_speed (float): speed alone Z dimension(m/s)
            yaw_speed (float): rotation speed on Z axis(rad/s)
            hand_coordinate (bool): False is left hand coordinate system
                                    True is right hand coordinate system
            duration (float): default motion duration(s)
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
```python
Move back to zero position.
        
        Args:
            duration: motion duration(s)
```

##### 1.2.9 机械臂正向运动学
`forward_kinematics(d, theta2, theta3, theta4)`
- 机械臂回的初始位姿
    - 参数： 
        - `d`: 直线模块的移动距离(m) 。
        - `theta2`: 机械臂关节2的角度(rad) 。
        - `theta3`: 机械臂关节3的角度(rad) 。
        - `theta4`: 机械臂关节4的角度(rad) 。
    - 返回值：
        - `(x, y, z, yaw)`: 机械臂末端位姿的元组，包含三个位置和一个姿态。

```python
#  查看使用方法
print(FourDoFScara.forward_kinematics.__doc__)
```
```
Forward kinematics algorism
        
        Args:
            d: slider distance(m)
            theta2: 2 joint angles(rad)
            theta3: 3 joint angles(rad)
            theta4: 4 joint angles(rad)
        
        Returns:
            (tuple): (x, y, z, yaw) tuples of 3 axis position and 1 axis posture
```

##### 1.2.10 机械臂逆向运动学
`inverse_kinematics(x, y, z, yaw, hand_coordinate=True)`
- 机械臂回的初始位姿
    - 参数： 
        - `x`: 机械臂的目标位置的x值(m) 。
        - `y`: 机械臂的目标位置的y值(m) 。 
        - `z`: 机械臂的目标位置的z值(m) 。 
        - `yaw`: 机械臂的目标姿态的yaw值(rad) 。
        - `hand_coordinate`: 机械臂运动学的手系(False or True),默认右手系(True) 。
    - 返回值：
        - `(d, theta2, theta3, theta4)`: 包含机械臂1个直线模块距离和3关节角度的列表(rad) 。

```python
#  查看使用方法
print(FourDoFScara.inverse_kinematics.__doc__)
```
```
inverse kinematics algorism
        
        Args:
            x (float): robot end x
            y (float): robot end y
            z (float): robot end z
            yaw (float): robot end yaw
            hand_coordinate (bool): False is left hand coordinate system
                                    True is right hand coordinate system
        
        Returns:
            (d, theta2, theta3, theta4): list of 1 slider distance(m) and 3 joint angles(rad)
            
        Raises:
            ValueError: If the Scara Pose not in range.
```

#### 1.3 具体使用实例
*注：本实例基于Jupyter lab*

##### 1.3.1 导入电机库和can总线
```python
from robodyno.interfaces import CanBus
from robodyno.components import Motor, SliderModule
can = CanBus()
```
##### 1.3.2 导入四自由度Scara机械臂的类
```python
from robodyno.robots.four_dof_scara_robot import FourDoFScara
```

##### 1.3.3 查看FourDoFScara类需要的参数
- 使用下面方法可以查看`four_dof_scara_robot`使用实例
```python
print(four_dof_scara_robot.__doc__)
```
```
    from robodyno.interfaces import CanBus
    from robodyno.components import Motor, SliderModule
    from robodyno.robots.four_dof_scara_robot import FourDoFScara
    can = CanBus()
    class MyScara(FourDoFScara):
        def __init__(self):
            M1 = SliderModule(can, 0x10)
            M2 = Motor(can, 0x11)
            M3 = Motor(can, 0x12)
            M4 = Motor(can, 0x13)
            super().__init__(M1, M2, M3, M4, 0.24, 0.06, 0.150, 0.150, 0.045)
    arm = MyScara()
    arm.init()
    arm.get_joints_poses()
    [0.0, 0.0, 0.0, 0.0]
    can.disconnect()
```
- 在上面实例中，可以了解到`FourDoFScara`类需要我们传入4个关节电机，以及运动学计算的具体尺寸(m)。

- 也可以使用下面的方法，查看类的具体属性
```python
print(FourDoFScara.__doc__)
```
```
Controls Robodyno Four DoF Scara Robot through the CAN bus.
    
    Attributes:
        joints (list): list of 4 joint motors
        d1 (float): DH parameter d1 from link 1 to link 2  (m)
        a1 (float): DH parameter a1 from link 1 to link 2  (m)
        a2 (float): DH parameter a2 from link 2 to link 3  (m)
        a3 (float): DH parameter a3 from link 3 to link 4  (m)
        d4 (float): DH parameter d4 from link 4 to eelink  (m)
        end_effector (object): end effector object
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
from robodyno.interfaces import CanBus
from robodyno.components import Motor, SliderModule
from robodyno.robots.four_dof_scara_robot import FourDoFScara
can = CanBus()

class MyScara(FourDoFScara):
    def __init__(self):
        M1 = SliderModule(can, 0x10)
        M2 = Motor(can, 0x11)
        M3 = Motor(can, 0x12)
        M4 = Motor(can, 0x13)
        
        super().__init__(M1, M2, M3, M4, 0.24, 0.06, 0.150, 0.150, 0.045)
        self.start_theta = pi

    def circle(self, xo, yo, zo, yaw, r, num=50, sn=True):
        """robot arm draw circle
        
        Args:
            xo: X coordinates of the center of the circle(m)
            yo: Y coordinates of the center of the circle(m)
            zo: Z coordinates of the center of the circle(m)
            yaw: Robot end yaw(rad)
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
from math import pi
from ipywidgets import FloatSlider, interact, Checkbox

scara_angle = arm.get_joints_poses()

@interact(J1 = FloatSlider(value=scara_angle[0], min=-0.2, max=0.2, step=0.001, readout_format='.3f'), 
          J2=(-pi, pi, 0.01), J3=(-pi, pi, 0.01), J4=(-pi, pi, 0.01))
def calibrate(J1, J2=scara_angle[1], J3=scara_angle[2], J4=scara_angle[3]):
    arm.joint_space_interpolated_motion((J1, J2, J3, J4))
```

- 笛卡尔坐标系
```python
from math import pi
from ipywidgets import FloatSlider, interact, Checkbox
scara_x, scara_y, scara_z, scara_yaw = arm.forward_kinematics(*arm.get_joints_poses())

@interact(
    hand_coordinate = Checkbox(value=True, description="Hand Coordinate"),
    x   = FloatSlider(value=scara_x  , min=-0.125, max=0.36, step=0.001, readout_format='.3f'),
    y   = FloatSlider(value=scara_y  , min=-0.36,  max=0.36, step=0.001, readout_format='.3f'),
    z   = FloatSlider(value=scara_z  , min=0,      max=0.4,  step=0.001, readout_format='.3f'),
    yaw = FloatSlider(value=scara_yaw, min=-2*pi,  max=2*pi, step=0.01 , readout_format='.3f'),
)
def calibrate(x = scara_x, y = scara_y, z = scara_z, yaw = scara_yaw, hand_coordinate=True):
    clear_output(wait=True)
    arm.joint_space_interpolated_motion(
        arm.inverse_kinematics(x, y, z, yaw, hand_coordinate), 
        speeds=[1.0, 0.5, 0.5, 1.0]
    )
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
print(arm.inverse_kinematics(0.36, 0, 0.195, 0))
```

- 关节空间运动
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.21, 0.15, 0.195, 1.57),duration = 5)
```
```python
arm.joint_space_interpolated_motion((-0.01, 0, 1.57, -1.57),speeds=[None,None,0.1,0.1],duration = 5)
```

- 笛卡尔空间运动
```python
arm.cartesian_space_interpolated_motion(0.21, 0.15, 0.195, 1.57, duration = 5)
```
```python
arm.cartesian_space_interpolated_motion(*arm.forward_kinematics((-0.01, 0, 1.57, -1.57)), y_speed=0.01, hand_coordinate=0,duration = 5)
```

- 画圆
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.25, 0.0, 0.195, 0.0),duration = 5)
arm.joint_space_interpolated_motion(arm.circle(0.25, 0.0, 0.195, 0.0, 0.05, 50, False),duration = 2)
for i in range(100):
    arm.joint_space_interpolated_motion(arm.circle(0.25, 0.0, 0.195, 0.0, 0.05, 50, False))
    time.sleep(0.01)
for j in range(100):
    arm.joint_space_interpolated_motion(arm.circle(0.25, 0.0, 0.195, 0.0, 0.05, 50, True))
    time.sleep(0.01)
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0.25, 0.0, 0.195, 0),duration = 2)
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
- 机械臂的正向运动学方程基于以下公式计算变换矩阵$^0_4T$。已知直线模块升降距离$d_1$和关节角度$θ_{2−4}$。变换矩阵定义为:
$$
\begin{align}
^0_6T(d_1, \theta_2, \theta_3, \theta_4) & = \begin{bmatrix}^0_4R & ^0P_{4} \\ 0 & 1 \\  \end{bmatrix} \\
 & = \begin{bmatrix}^0\hat{X}_4 & ^0\hat{Y}_4 & ^0\hat{Z}_4 & ^0P_{4} \\ 0 & 0 & 0 & 1 \\  \end{bmatrix} \\ 
 & =  \begin{bmatrix}^0\hat{X}_{4x} & ^0\hat{Y}_{4x} & ^0\hat{Z}_{4x} & ^0P_{4x} \\ ^0\hat{X}_{4y} & ^0\hat{Y}_{4y} & ^0\hat{Z}_{4y} & ^0P_{4y} \\ ^0\hat{X}_{4z} & ^0\hat{Y}_{4z} & ^0\hat{Z}_{4z} & ^0P_{4z} \\ 0 & 0 & 0 & 1 \\  \end{bmatrix}
\end{align} \tag 1
$$

- 在上式中，$^0\hat{X}_4$,$^0\hat{Y}_4$和$^0\hat{Z}_4$分别代表坐标系4相对于坐标系0在不同轴方向上的偏移旋转。
- 我们可以把变换矩阵分解成一个对于每个关节的变换链给出
$$^0_4T(d_1, \theta_2, \theta_3, \theta_4) = {^0_1T(d_1)}{^1_2T(\theta_2)}{^2_3T(\theta_3)}{^3_4T(\theta_4)} \tag 2$$

- 因此，只需求出每个关节的变换矩阵通过矩阵相乘即可求出变换矩阵 ${^0_4T}$。

- 以下面位姿来构建Scara机械臂的零位，并对每个关节建立坐标系。

- 下面通过标准的DH矩阵来构建机械臂，其DH参数表为
$$
\begin{array}{c|cccc}
i & \theta_i & d_i & \alpha_i & a_i \\
\hline
1 & 0 & d_1+\Delta d & \pi & a_1\\
2 & \theta_2 & 0   & \pi & a_2\\
3 & \theta_3 & 0   & \pi & a_3\\
4 & \theta_4 & d_4 & \pi & 0\\
\end{array}
$$

- 其中DH参数的含义为
    - $\theta_i$ 为绕$\hat Z_{i-1}$轴从$\hat X_{i-1}$到$\hat X_{i}$的转角，绕$\hat Z_{i-1}$正向转动方向为正。
    - $d_i$ 为沿$\hat Z_{i-1}$轴从$\hat X_{i-1}$到$\hat X_{i}$的距离，与$\hat Z_{i-1}$方向相同为正。
    - $\alpha_i$ 为绕$\hat X_{i}$轴从$\hat Z_{i-1}$到$\hat Z_{i}$的转角，绕$\hat X_{i}$正向转动方向为正。
    - $a_i$ 为沿$\hat X_{i}$轴从$\hat Z_{i-1}$到$\hat Z_{i}$的距离，与$\hat X_{i}$方向相同为正。

- 可以使用DH参数来编写相邻连杆坐标系之间的转换，其中$link_{i-1}$和$link_i$之间的转换可由下式求出
$$
\begin{align}
^{i-1}{_iT} & =  R_Z(\theta_i)D_Z(d_i)R_X(\alpha_i)D_X(a_i)\\
 & = \begin{bmatrix} 
 cos\theta_i & -sin\theta_icos\alpha_i & sin\theta_isin\alpha_i & a_icos\theta_i  \\
 sin\theta_i & cos\theta_icos\alpha_i & -cos\theta_isin\alpha_i & a_isin\theta_i  \\
 0 & sin\alpha_i & cos\alpha_i & d_i  \\
  0 & 0 & 0 & 1 \\  
  \end{bmatrix} 
 \end{align} \tag 3$$

- 综上，根据DH参数表及式(3)可求得相邻连杆间的变换矩阵，再由式(2)进一步求得$^0_4T$。
- 由式(1)再次分离可得
$$
\begin{cases}
 {x = {^0P_{4x}}}  \\
 {y = {^0P_{4y}}}  \\
 {z = {^0P_{4z}}}  \\
 {-,-,yaw = \begin{bmatrix}
 ^0\hat{X}_{4x} & ^0\hat{Y}_{4x} & ^0\hat{Z}_{4x}  \\ 
 ^0\hat{X}_{4y} & ^0\hat{Y}_{4y} & ^0\hat{Z}_{4y}  \\ 
 ^0\hat{X}_{4z} & ^0\hat{Y}_{4z} & ^0\hat{Z}_{4z}    \end{bmatrix}}  \\
\end{cases}
 \tag {4}$$

- 在式(4)中，旋转矩阵与欧拉角的转换可以使用下面的转角排列的公式计算
*X-Y-Z固定角*
$$
\begin{align}
^A_BR_{XYZ}(\gamma,\beta,\alpha) 
& =  R_Z(\alpha)R_Y(\beta)R_X(\gamma)\\
& = \begin{bmatrix}
 cos\alpha & -sin\alpha & 0   \\
 sin\alpha & cos\alpha & 0   \\
 0 & 0 & 1   \\
\end{bmatrix}
\begin{bmatrix}
 cos\beta & 0 & sin\beta   \\
  0 & 1 & 0   \\
 -sin\beta & 0 & cos\beta   \\
\end{bmatrix}
\begin{bmatrix}
 1 & 0 & 0   \\
 0 & cos\gamma & -sin\gamma   \\
 0 & sin\gamma & cos\gamma   \\
\end{bmatrix}
\\
& = \begin{bmatrix} 
 cos\alpha\cos\beta & \cos\alpha\sin\beta\sin\gamma - \sin\alpha\cos\gamma & \cos\alpha\sin\beta\cos\gamma + \sin\alpha\sin\gamma   \\
 sin\alpha\cos\beta & \sin\alpha\sin\beta\sin\gamma + \cos\alpha\cos\gamma & \sin\alpha\sin\beta\cos\gamma - \cos\alpha\sin\gamma   \\
 -sin\beta & \cos\beta\sin\gamma & \cos\beta\cos\gamma   \\ 
  \end{bmatrix} 
  \\
& = \begin{bmatrix}
 r_{11} & r_{12} & r_{13}   \\
 r_{21} & r_{22} & r_{23}   \\
 r_{31} & r_{32} & r_{33}   \\
\end{bmatrix}
 \end{align} \tag 5$$
- 则，$yaw$为
$$
\begin{cases}
yaw = \alpha & = atan2(\frac{sin\alpha\cos\beta}{\cos\beta},\frac{cos\alpha\cos\beta}{\cos\beta})\\
& = atan2(\frac{r_{21}}{\cos\beta},\frac{r_{11}}{\cos\beta})\\
& = atan2(\frac{ ^0\hat{X}_{6y}}{\cos\beta},\frac{^0\hat{X}_{6x}}{\cos\beta})
 \end{cases} 
\tag 6$$



##### 2.1.2 逆向运动学
已知Scara末端位姿$(x, y, z, yaw)$ ，求解四个关节电机的角度值。
- 以俯视角度构建下面坐标系求解逆运动学  
![U5EhXQ.png](https://m1.im5i.com/2022/11/25/U5EhXQ.png)
<center>图5 Scara机械臂逆解辅助线</center>

- 连接 $OB$ ，过 $B$ 作 $BC$ 垂直于 $OA$ 于 $C$ ，在$\Delta OAB$中，由余弦定理得：
$$\cos(\pi - \theta_3) = \frac {a_{2}^2 + a_{3}^2 - (x^2 + y^2)}{2a_{2}a_{3}}  \tag {5}$$

- 记$c_3=cos\theta_3$ ，上式可写成：
$$c_3 = \frac {x^2 + y^2 - a_{2}^2 - a_{3}^2}{2a_{2}a_{3}} \tag {6}$$

- 记$s_3=sin\theta_3$ ，则 $s_3$ 有两个解：
$$s_3 =± \sqrt {1−c_3^2} \tag {7}$$

取正数解，机器人处于右手系(right handcoor)；取负数解，机器人处于左手系(left handcoor)；特殊地，若$s_2=0$，机器人处于奇异位置(singular position)，此时$\theta_2=k\pi(k\in{Z})$，一般 $\theta_2\in{\{-2\pi,-\pi,0,\pi,2\pi}\}$。

- 至此，可求得$\theta_3$：
$$\theta_3 = atan2(s3, c3) \tag {8}$$

- 从上图中，我们可以求得：
$$\alpha = atan2(y, x) \tag {9}$$

- 在$\Delta OBC$中，记$r=\|OB\|$：
$$sin\beta = \frac {\|BC\|}{\|OB\|} = \frac {a_{3}s_3}{r} \tag {10}$$

$$cos\beta = \frac {\|OC\|}{\|OB\|} = \frac {\|OA\| + \|AC\|}{\|OB\|} = \frac {a_{2} + a_{3}c_3}{r}  \tag {11}$$

- 由于$r>0$，由上式得：
$$\beta = atan2(\sin\beta,\cos\beta) = atan2(a_{3}s_3 , a_{2} + a_{3}c_3) \tag {12}$$

- 至此，可求得$\theta_2$, 由右手定则可知$\theta_2$的旋转方向与实际相反，因此需要翻转为$-\theta_2$。
$$-\theta_2 = -(\alpha - \beta) = \beta -\alpha = atan2(a_{3}s_3 , a_{2} + a_{3}c_3) - atan2(y, x) \tag {13}$$

- 可求得末端$\theta_4$:
$$\theta_4 = \theta_3 - \theta_2 - yaw \tag {15}$$

- 直线模块的变化量$\Delta d$为：
$$\Delta d = z - d_1 + d_4 \tag {14}$$



- 综上，Scara机器人的运动学逆解为：
$$
\begin{cases}
 {\Delta d = z - d_1 + d_4}  \\
 {\theta_2 = atan2(a_{3}s_3 , a_{2} + a_{3}c_3) - atan2(y, x)}  \\
 {\theta_3 = atan2(s3, c3)}                           \\
 {\theta_4 = \theta_3 - \theta_2 - yaw }
\end{cases}
 \tag {16}$$

##### 2.1.3 参考文献
[1] [scara机器人运动学正逆解](https://blog.csdn.net/maple_2014/article/details/104596998)
