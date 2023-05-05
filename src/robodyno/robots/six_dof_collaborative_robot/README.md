## 六自由度协作机械臂
`six_dof_collab_robot`

### 1. 使用方法
#### 1.1 查看库的使用方法
- 使用下面方法来打印库中对应的API说明注释:
```python
#  从robodyno中导入六自由度协作机械臂库
from robodyno.robots.six_dof_collaborative_robot import six_dof_collab_robot
```
```python
#  打印整体库的介绍及例子
print(six_dof_collab_robot.__doc__)
```
```
six_dof_collab_robot.py
Time    :   2022/10/08
Author  :   ryan 
Version :   1.0
Contact :   ryanzhang@163.com
License :   (C)Copyright 2022, robottime / robodyno

6DoF Collaborative Robot Driver

  Typical usage example:

  from robodyno.robot.six_dof_collaborative_robot import SixDoFCollabRobot
  robot = SixDoFCollabRobot(
      j1 = base_motor,
      j2 = shoulder_motor,
      j3 = upperarm_motor,
      j4 = elbow_motor,
      j5 = forearm_motor,
      j6 = hand_motor,
      l01 = 0.18,
      l12 = 0.135,
      l23 = 0.135,
      l34 = 0.075,
      l45 = 0.075,
      l56 = 0.1,
      end_effector = None
  )
```

#### 1.2 6-DoF-Collab-API

##### 1.2.0 导入六自由度协作机械臂的类
```python
from robodyno.robot.six_dof_collaborative_robot import SixDoFCollabRobot
```

##### 1.2.1 获取关节当前位置
`get_joints_poses()`
- 读取机械臂关节的位置
    - 返回值：
        - `poses`: 包含六个关节位置的列表

```python
#  查看使用方法
print(SixDoFCollabRobot.get_joints_poses.__doc__)
```
```
Read joints positions to a list.
        
        Returns:
            a list of 6 joints positions
```

##### 1.2.2 机械臂使能
`enable()`
- 使能机械臂所有关节电机

```python
#  查看使用方法
print(SixDoFCollabRobot.enable.__doc__)
```
```
enable joints motors
```

##### 1.2.3 机械臂失能
`disable()`
- 失能机械臂所有关节电机

```python
#  查看使用方法
print(SixDoFCollabRobot.disable.__doc__)
```
```
disable joints motors
```

##### 1.2.4 机械臂初始化
`init(axes_poses)`
- 记录机械臂当前所有关节电机
    - 参数： 
        - `axes_poses`: 包含机械臂当前六个关节坐标位置的列表(rad),默认为0 。

```python
#  查看使用方法
print(SixDoFCollabRobot.init.__doc__)
```
```
Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses: list of 6 axes poses(rad)
```

##### 1.2.5 设置机械臂关节位置
`set_joint_pos(id, pos)`
- 设置机械臂对应关节电机位置
    - 参数： 
        - `id`: 机械臂对应位置关节（0-5）。
        - `pos`: 对应关节的目标位置(rad) 。

```python
#  查看使用方法
print(SixDoFCollabRobot.set_joint_pos.__doc__)
```
```
Set joint angle with joint id and position
        
        Args:
            id: joint id (0-5)
            pos: joint target position(rad)
```

##### 1.2.6 关节空间的插值运动
`joint_space_interpolated_motion(target, speeds, duration)`
- 机械臂在关节空间中的插值运动
    - 参数： 
        - `target`: 机械臂运动的目标位置，包含六个关节角度的列表或元组(rad)。
        - `speeds`: 对应关节的插值运动速度列表(rad/s),默认为None,其优先级高于时间插值 。
        - `duration`: 关节空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(SixDoFCollabRobot.joint_space_interpolated_motion.__doc__)
```
```
Robot interpolated motion in joint space.
        
        Args:
            target: iterable of 6 joints target angle(rad)
            speeds: iterable of 6 joints motion speed(rad/s)
            duration: default motion duration(s)
```

##### 1.2.7 笛卡尔空间的插值运动
`cartesian_space_interpolated_motion(self, x, y, z, roll, pitch, yaw, sol_id, x_speed, y_speed, z_speed, roll_speed, pitch_speed, yaw_speed, duration)`
- 机械臂在笛卡尔空间中的插值运动
    - 参数： 
        - `x`: 机械臂运动的目标位置的x值(m) 。
        - `y`: 机械臂运动的目标位置的y值(m) 。 
        - `z`: 机械臂运动的目标位置的z值(m) 。 
        - `roll`: 机械臂运动的目标姿态的roll值(rad) 。
        - `pitch`: 机械臂运动的目标姿态的pitch值(rad) 。
        - `yaw`: 机械臂运动的目标姿态的yaw值(rad) 。
        - `sol_id`: 机械臂逆解解算的解的编号(0-7)，默认为2 。
        - `x_speed`: 机械臂x方向的插值运动速度(m/s),默认为None 。
        - `y_speed`: 机械臂y方向的插值运动速度(m/s),默认为None 。
        - `z_speed`: 机械臂z方向的插值运动速度(m/s),默认为None 。
        - `roll_speed`: 机械臂x轴的插值旋转速度(rad/s),默认为None 。
        - `pitch_speed`: 机械臂y轴的插值旋转速度(rad/s),默认为None 。
        - `yaw_speed`: 机械臂z轴的插值旋转速度(rad/s),默认为None 。
        - `duration`: 笛卡尔空间的插值运动持续时间(s)，默认为0,其优先级低于速度插值。

```python
#  查看使用方法
print(SixDoFCollabRobot.cartesian_space_interpolated_motion.__doc__)
```
```
Robot Interpolated motion in Cartesian space.
        
        Args:
            x: target robot end x
            y: target robot end y
            z: target robot end z
            roll: target robot end roll
            pitch: target robot end pitch
            yaw: target robot end yaw
            sol_id: inverse kinematics solve id (0-7)
            x_speed: speed alone X dimension(m/s)
            y_speed: speed alone Y dimension(m/s)
            z_speed: speed alone Z dimension(m/s)
            roll_speed: rotation speed on X axis(rad/s)
            pitch_speed: rotation speed on Y axis(rad/s)
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
print(SixDoFCollabRobot.home.__doc__)
```
```
Move back to zero position.
        
        Args:
            duration: motion duration(s)
```

##### 1.2.9 机械臂正向运动学
`forward_kinematics(axes)`
- 机械臂回的初始位姿
    - 参数： 
        - `axes`: 包含机械臂6个关节角度的列表(rad) 。
    - 返回值：
        - `(x, y, z, roll, pitch, yaw)`: 机械臂末端位姿的元组 。

```python
#  查看使用方法
print(SixDoFCollabRobot.forward_kinematics.__doc__)
```
```
Forward kinematics algorism
        
        Args:
            axes: list of 6 joint angles
        
        Returns:
            tuples of robot end transform
            
            example:
                (x, y, z, roll, pitch, yaw)
```

##### 1.2.10 机械臂逆向运动学
`inverse_kinematics(x, y, z, roll, pitch, yaw, sol_id)`
- 机械臂回的初始位姿
    - 参数： 
        - `x`: 机械臂的目标位置的x值(m) 。
        - `y`: 机械臂的目标位置的y值(m) 。 
        - `z`: 机械臂的目标位置的z值(m) 。 
        - `roll`: 机械臂的目标姿态的roll值(rad) 。
        - `pitch`: 机械臂的目标姿态的pitch值(rad) 。
        - `yaw`: 机械臂的目标姿态的yaw值(rad) 。
        - `sol_id`: 机械臂逆解解算的解的编号(0-7)，默认为2 。
    - 返回值：
        - `axes`: 包含机械臂6个关节角度的列表(rad) 。

```python
#  查看使用方法
print(SixDoFCollabRobot.inverse_kinematics.__doc__)
```
```
Inverse kinematics algorism
        
        Args:
            x: robot end x
            y: robot end y
            z: robot end z
            roll: robot end roll
            pitch: robot end pitch
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
##### 1.3.2 导入六自由度机械臂的类
```python
from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot
```

##### 1.3.3 查看SixDoFCollabRobot类需要的参数
- 使用下面方法可以查看`six_dof_collab_robot`使用实例
```python
print(six_dof_collab_robot.__doc__)
```
```
6DoF Collaborative Robot Driver

  Typical usage example:

  from robodyno.robot.six_dof_collaborative_robot import SixDoFCollabRobot
  robot = SixDoFCollabRobot(
      j1 = base_motor,
      j2 = shoulder_motor,
      j3 = upperarm_motor,
      j4 = elbow_motor,
      j5 = forearm_motor,
      j6 = hand_motor,
      l01 = 0.18,
      l12 = 0.135,
      l23 = 0.135,
      l34 = 0.075,
      l45 = 0.075,
      l56 = 0.1,
      end_effector = None
  )
```
- 在上面实例中，可以了解到`SixDoFCollabRobot`类需要我们传入六个关节电机，以及DH参数的具体尺寸(m)。

- 也可以使用下面的方法，查看类的具体属性
```python
print(SixDoFCollabRobot.__doc__)
```
```
6 DoF collaborarive robot driver
    
    Attributes:
        joints: list of 6 joint motors
        l01: link from world to joint 1 (m) 
        l12: link from joint 1 to joint 2 (m) 
        l23: link from joint 2 to joint 3 (m) 
        l34: link from joint 3 to joint 4 (m) 
        l45: link from joint 4 to joint 5 (m) 
        l56: link from joint 5 to joint 6 (m) 
        end_effector: end effector object 
```
- 六自由度协作机械臂详细的尺寸说明如下图：
![1](../../robodyno_motor/1.png)

##### 1.3.4 构建机械臂的类
- 构建一个自己的类，并继承SixDoFCollabRobot，使用Motor创建6个关节电机，并将关节电机及DH参数尺寸传给父类。
- 也可以在类中编写自己的方法，如下面例子中的清除机械臂关节电机错误。

***Typical usage example:***

```python
class MySixDoFArm(SixDoFCollabRobot):
    def __init__(self):
        M1 = Motor(can, 0x10, 'ROBODYNO_PRO_44')
        M2 = Motor(can, 0x11, 'ROBODYNO_PRO_44')
        M3 = Motor(can, 0x12, 'ROBODYNO_PRO_44')
        M4 = Motor(can, 0x13, 'ROBODYNO_PRO_44')
        M5 = Motor(can, 0x14, 'ROBODYNO_PRO_12')
        M6 = Motor(can, 0x15, 'ROBODYNO_PRO_12')
        
        super().__init__(M1, M2, M3, M4, M5, M6, 0.18, 0.135, 0.135, 0.075, 0.075, 0.1)

    def clear_joints_errors(self):
        """clear joint motors errors"""
        for joint in self.joints:
            joint.clear_errors()

```

##### 1.3.5 调用类的方法控制机械臂运动
- 创建好自己的类之后，我们就调用对应的方法来控制机械臂，具体类的方法可以参考`6-DoF-Collab-API` 。

***Typical usage example:***
- 定义对象。
```python
arm = MySixDoFArm()
```

- 使能
```python
arm.enable()
```

- 清除错误
```python
arm.clear_joints_errors()
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
@interact(a1=(-pi, pi, 0.01), a2=(-pi, pi, 0.01), a3=(-pi, pi, 0.01), a4=(-pi, pi, 0.01), a5=(-pi, pi, 0.01), a6=(-pi, pi, 0.01))
def calibrate(a1, a2, a3, a4, a5, a6):
    arm.set_joint_pos(0, a1)
    arm.set_joint_pos(1, a2)
    arm.set_joint_pos(2, a3)
    arm.set_joint_pos(3, a4)
    arm.set_joint_pos(4, a5)
    arm.set_joint_pos(5, a6)
```

- 笛卡尔坐标系
```python
from ipywidgets import interact
from math import pi
arm.joint_space_interpolated_motion(arm.inverse_kinematics(-0.075,0.21,0.215,0,pi,pi,1),duration = 5)
@interact(x = (-175, 25, 1), y = (110, 310, 1), z = (115, 315, 1))
def calibrate(x = -75, y = 210, z = 215):
    arm.joint_space_interpolated_motion(arm.inverse_kinematics(x/1000, y/1000, z/1000,0,pi,pi,1))
```

- 正向运动学
```python
print(arm.forward_kinematics((arm.get_joints_poses())))
```
```python
print(arm.forward_kinematics((4.7,0,-pi/2,0,-pi/2,0)))
```

- 逆向运动学
```python
print(arm.inverse_kinematics(*arm.forward_kinematics((4.7,0,-pi/2,0,-pi/2,0),2)))
```
```python
print(arm.inverse_kinematics(-0.075,0.21,0.215,0,pi,pi,1))
```

- 关节空间运动
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(-0.075,0.21,0.215,0,pi,pi,1),duration = 5)
```
```python
arm.joint_space_interpolated_motion((4.7,0,-pi/2,0,-pi/2,0),speeds=(1, None, 0.5, None, 0.5, None))
```

- 笛卡尔空间运动
```python
arm.cartesian_space_interpolated_motion(*arm.forward_kinematics((4.7,0,-pi/2,0,-pi/2,0)),duration = 5)
```
```python
arm.cartesian_space_interpolated_motion(-0.075,0.21,0.215,0,pi,pi, x_speed=0.1, y_speed=0.1, z_speed=0.1,duration = 5)
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
    ![U5EP3D.png](https://m1.im5i.com/2022/11/25/U5EP3D.png)
    <center>图2 机械臂的坐标系说明</center>
#### 2.1 运动学
- Delta机械臂包含三个自由度,即三个坐标位置$(x, y, z)$。其连杆尺寸及坐标建立如下：

<center>图3 三自由度Delta机械臂详细的尺寸及坐标系说明</center>

- 运动学解算中的符号描述
    - $^0P_6 = \begin{bmatrix}^0P_{6x} \\ ^0P_{6y} \\ ^0P_{6z} \\ \end{bmatrix}$，指从坐标0到坐标6的原点位置描述。
    - $^0\hat{Y}_6 = \begin{bmatrix}^0\hat{Y}_{6x} \\ ^0\hat{Y}_{6y} \\ ^0\hat{Y}_{6z} \\ \end{bmatrix}$，指坐标系6相对于坐标系0在y轴方向上的偏移旋转。
    - ${^0_6}T$，是从坐标系0到坐标系6的坐标变换矩阵。这意味着${^0P} = {^0_6}T \cdot {^6P} $。


##### 2.1.1 正向运动学
已知六轴机械臂关节电机的角度值，求解末端位姿$(x, y, z, roll, pitch, yaw)$。
- 机械臂的正向运动学方程基于以下公式计算变换矩阵$^0_6T$。已知关节角度$θ_{1−6}$。变换矩阵定义为:
$$
\begin{align}
^0_6T(\theta_1, \theta_2, \theta_3, \theta_4, \theta_5, \theta_6) & = \begin{bmatrix}^0_6R & ^0P_{6} \\ 0 & 1 \\  \end{bmatrix} \\
 & = \begin{bmatrix}^0\hat{X}_6 & ^0\hat{Y}_6 & ^0\hat{Z}_6 & ^0P_{6} \\ 0 & 0 & 0 & 1 \\  \end{bmatrix} \\ 
 & =  \begin{bmatrix}^0\hat{X}_{6x} & ^0\hat{Y}_{6x} & ^0\hat{Z}_{6x} & ^0P_{6x} \\ ^0\hat{X}_{6y} & ^0\hat{Y}_{6y} & ^0\hat{Z}_{6y} & ^0P_{6y} \\ ^0\hat{X}_{6z} & ^0\hat{Y}_{6z} & ^0\hat{Z}_{6z} & ^0P_{6z} \\ 0 & 0 & 0 & 1 \\  \end{bmatrix}
\end{align} \tag 1
$$

- 在上式中，$^0\hat{X}_6$,$^0\hat{Y}_6$和$^0\hat{Z}_6$分别代表坐标系6相对于坐标系0在不同轴方向上的偏移旋转。
- 我们可以把变换矩阵分解成一个对于每个关节的变换链给出
$$^0_6T(\theta_1, \theta_2, \theta_3, \theta_4, \theta_5, \theta_6) = {^0_1T(\theta_1)}{^1_2T(\theta_2)}{^2_3T(\theta_3)}{^3_4T(\theta_4)}{^4_5T(\theta_5)}{^5_6T(\theta_6)} \tag 2$$

- 因此，只需求出每个关节的变换矩阵通过矩阵相乘即可求出变换矩阵 ${^0_6T}$。

- 以下面位姿来构建六自由度协作机械臂的零位，并对每个关节建立坐标系。
![U5OFr3.png](https://m1.im5i.com/2022/12/14/U5OFr3.png)

- 下面通过标准的DH矩阵来构建机械臂，其DH参数表为
$$
\begin{array}{c|cccc}
i & \theta_i & d_i & \alpha_i & a_i \\
\hline
1 & \theta_0 & d_1 & \frac{\pi}{2} & 0\\
2 & \theta_1 & 0   & 0 & -a_2\\
3 & \theta_2 & 0   & 0 & -a_3\\
4 & \theta_3 & d_4 & \frac{\pi}{2} & 0\\
5 & \theta_4 & d_5 & \frac{-\pi}{2} & 0\\
6 & \theta_5 & d_6 & 0 & 0\\
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

- 综上，根据DH参数表及式(3)可求得相邻连杆间的变换矩阵，再由式(2)进一步求得$^0_6T$。
- 由式(1)再次分离可得
$$
\begin{cases}
 {x = {^0P_{6x}}}  \\
 {y = {^0P_{6y}}}  \\
 {z = {^0P_{6z}}}  \\
 {roll,pitch,yaw = \begin{bmatrix}
 ^0\hat{X}_{6x} & ^0\hat{Y}_{6x} & ^0\hat{Z}_{6x}  \\ 
 ^0\hat{X}_{6y} & ^0\hat{Y}_{6y} & ^0\hat{Z}_{6y}  \\ 
 ^0\hat{X}_{6z} & ^0\hat{Y}_{6z} & ^0\hat{Z}_{6z}    \end{bmatrix}}  \\
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
- 则，$roll, pitch, yaw$为
$$
\begin{cases}
pitch = \beta & =  atan2(-(-sin\beta), \sqrt{(cos\alpha\cos\beta)^2 + (sin\alpha\cos\beta)^2})\\
& =  atan2(-(r_{31}), \sqrt{(r_{11})^2 + (r_{21})^2})\\
& =  atan2(-( ^0\hat{X}_{6z}), \sqrt{(^0\hat{X}_{6x})^2 + (^0\hat{X}_{6y})^2})\\
yaw = \alpha & = atan2(\frac{sin\alpha\cos\beta}{\cos\beta},\frac{cos\alpha\cos\beta}{\cos\beta})\\
& = atan2(\frac{r_{21}}{\cos\beta},\frac{r_{11}}{\cos\beta})\\
& = atan2(\frac{ ^0\hat{X}_{6y}}{\cos\beta},\frac{^0\hat{X}_{6x}}{\cos\beta})\\
roll = \gamma & = atan2(\frac{\cos\beta\sin\gamma}{\cos\beta},\frac{\cos\beta\cos\gamma}{\cos\beta})   \\  
& = atan2(\frac{r_{32}}{\cos\beta},\frac{r_{33}}{\cos\beta})   \\
& = atan2(\frac{^0\hat{Y}_{6z}}{\cos\beta},\frac{^0\hat{Z}_{6z}}{\cos\beta})   \\
 \end{cases} \tag 6$$
