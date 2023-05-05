## 三自由度Delta机械臂
`three_dof_delta_robot`

### 1. 使用方法
#### 1.1 查看库的使用方法
- 使用下面方法来打印库中对应的API说明注释:
```python
#  从robodyno中导入三自由度Delta机械臂库
from robodyno.robots.three_dof_delta_robot import three_dof_delta_robot
```
```python
#  打印整体库的介绍及例子
print(three_dof_delta_robot.__doc__)
```
```
three_dof_delta_robot.py
Time    :   2022/10/09
Author  :   ryan 
Version :   1.0
Contact :   ryanzhang@163.com
License :   (C)Copyright 2022, robottime / robodyno

3DoF Delta Robot Drive

  Typical usage example:

  from robodyno.robots.three_dof_delta_robot import ThreeDoFDelta
  robot = ThreeDoFDelta(
    j1 = delta_motor1,
    j2 = delta_motor2,
    j3 = delta_motor3,
    l1 = 0.12864,
    l2 = 0.3,
    r1 = 0.13856,
    r2 = 0.025,
    end_effector = None
  )
```

#### 1.2 3-DoF-Delta-API
##### 1.2.0 导入四自由度Delta机械臂的类
```python
from robodyno.robots.three_dof_delta_robot import ThreeDoFDelta
```

##### 1.2.1 获取关节当前位置
`get_joints_poses()`
- 读取机械臂关节的位置
    - 返回值：
        - `poses`: 包含三个关节位置的列表

```python
#  查看使用方法
print(ThreeDoFDelta.get_joints_poses.__doc__)
```
```
Read joints positions to a list.

        Returns:
            a list of 3 joint positions
```

##### 1.2.2 机械臂使能
`enable()`
- 使能机械臂所有关节电机

```python
#  查看使用方法
print(ThreeDoFDelta.enable.__doc__)
```
```
enable joints motors
```

##### 1.2.3 机械臂失能
`disable()`
- 失能机械臂所有关节电机

```python
#  查看使用方法
print(ThreeDoFDelta.disable.__doc__)
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
print(ThreeDoFDelta.init.__doc__)
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
print(ThreeDoFDelta.set_joint_pos.__doc__)
```
```
Set joint angle with joint id and position
        
        Args:
            id: joint id (0-2)
            pos: joint target position(rad)
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
print(ThreeDoFDelta.joint_space_interpolated_motion.__doc__)
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
print(ThreeDoFDelta.cartesian_space_interpolated_motion.__doc__)
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
print(ThreeDoFDelta.home.__doc__)
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
        - `(x, y, z)`: 机械臂末端位姿的元组 。

```python
#  查看使用方法
print(ThreeDoFDelta.forward_kinematics.__doc__)
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
        - `angles`: 包含机械臂3个关节角度的列表(rad) 。

```python
#  查看使用方法
print(ThreeDoFDelta.inverse_kinematics.__doc__)
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
##### 1.3.2 导入三自由度Delta机械臂的类
```python
from robodyno.robots.three_dof_delta_robot import ThreeDoFDelta
```

##### 1.3.3 查看ThreeDoFDelta类需要的参数
- 使用下面方法可以查看`three_dof_delta_robot`使用实例
```python
print(three_dof_delta_robot.__doc__)
```
```
3DoF Delta Robot Drive

  Typical usage example:

  from robodyno.robots.three_dof_delta_robot import ThreeDoFDelta
  robot = ThreeDoFDelta(
    j1 = delta_motor1,
    j2 = delta_motor2,
    j3 = delta_motor3,
    l1 = 0.12864,
    l2 = 0.3,
    r1 = 0.13856,
    r2 = 0.025,
    end_effector = None
  )
```
- 在上面实例中，可以了解到`ThreeDoFDelta`类需要我们传入3个关节电机，以及运动学计算的具体尺寸(m)。

- 也可以使用下面的方法，查看类的具体属性
```python
print(ThreeDoFDelta.__doc__)
```
```
3 DoF delta robot driver
    
    Attributes:
        joints: list of 3 joint motors
        l1: link is the active arm (m)
        l2: link is the slave arm (m) 
        r1: fixed platform radius (m)
        r2: motion platform radius (m)
        end_effector: end effector object
```
- 三自由度Delta机械臂详细的尺寸说明如下图：  
![U5EmTQ.png](https://m1.im5i.com/2022/11/25/U5EmTQ.png)
<center>图1 三自由度Delta机械臂详细的尺寸说明</center>
##### 1.3.4 构建机械臂的类
- 构建一个自己的类，并继承ThreeDoFDelta，使用Motor创建3个关节电机，并将关节电机及运动学计算尺寸传给父类。
- 也可以在类中编写自己的方法，如下面例子中的控制机械臂两点搬运的方法。

***Typical usage example:***

```python
class MyDelta(ThreeDoFDelta):
    def __init__(self):
        M1 = Motor(can, 0x10, 'ROBODYNO_PRO_44')
        M2 = Motor(can, 0x11, 'ROBODYNO_PRO_44')
        M3 = Motor(can, 0x12, 'ROBODYNO_PRO_44')
        
        super().__init__(M1, M2, M3, 0.12864, 0.3, 0.13856, 0.025)

    def two_carry(self):
        """delta robot arm two dot carry"""
        self.joint_space_interpolated_motion(self.inverse_kinematics(0,0,-0.3), duration=0.1)
        time.sleep(0.3)
        self.joint_space_interpolated_motion(self.inverse_kinematics(-0.09,0,-0.35), duration=0.1)
        time.sleep(0.3)
        self.joint_space_interpolated_motion(self.inverse_kinematics(0,0,-0.3), duration=0.1)
        time.sleep(0.3)
        self.joint_space_interpolated_motion(self.inverse_kinematics(0.09,0,-0.35), duration=0.1)
        time.sleep(0.3)
```
##### 1.3.5 调用类的方法控制机械臂运动
- 创建好自己的类之后，我们就调用对应的方法来控制机械臂，具体类的方法可以参考`3-DoF-Delta-API` 。

***Typical usage example:***

 定义对象。
```python
arm = MyDelta()
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
@interact(a1=(-pi, pi, 0.01), a2=(-pi, pi, 0.01), a3=(-pi, pi, 0.01))
def calibrate(a1, a2, a3):
    arm.set_joint_pos(0, a1)
    arm.set_joint_pos(1, a2)
    arm.set_joint_pos(2, a3)
```

- 笛卡尔坐标系
```python
from ipywidgets import interact
from math import pi
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0,0,-0.3), duration = 2)
@interact(x = (-150, 150, 1), y = (-150, 150, 1), z = (-450, -190, 1))
def calibrate(x = 0, y = 0, z = -300):
    arm.joint_space_interpolated_motion(arm.inverse_kinematics(x/1000, y/1000, z/1000))
    print(" ")
```

- 正向运动学
```python
print(arm.forward_kinematics((arm.get_joints_poses())))
```
```python
print(arm.forward_kinematics((-0.92,-0.92,-0.92)))
```

- 逆向运动学
```python
print(arm.inverse_kinematics(*arm.forward_kinematics((arm.get_joints_poses()))))
```
```python
print(arm.inverse_kinematics(0,0,-0.3))
```

- 关节空间运动
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0,0,-0.3),duration = 5)
```
```python
arm.joint_space_interpolated_motion((-0.92,-0.92,-0.92),speeds=(0.1,0.05,None))
```

- 笛卡尔空间运动
```python
arm.cartesian_space_interpolated_motion(0.05, 0.1, -0.3, x_speed=0.01, y_speed=0.02, z_speed=0.06 ,duration = 5)
```
```python
arm.cartesian_space_interpolated_motion(*arm.forward_kinematics((-0.92,-0.92,-0.92)),duration = 5)
```

- 两点搬运
```python
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0,0,-0.3),duration = 5)
for i in range(2):
    arm.two_carry()
arm.joint_space_interpolated_motion(arm.inverse_kinematics(0,0,-0.3),duration = 2)
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
![U5EHeq.png](https://m1.im5i.com/2022/11/25/U5EHeq.png)
<center>图3 三自由度Delta机械臂详细的尺寸及坐标系说明</center>

##### 2.1.1 正向运动学
已知Delta三个关节电机的角度值，求解末端位姿$(x, y, z)$。
- 建立Delta机构简化数学模型，如下图所示，其中圆$O$所在平面为定平台，圆$p$所在平面为动平台，$∆C_1C_2C_3$和$∆A_1A_2A_3$为等边三角形，点$C_1$、$C_2$、$C_3$、$A_1$、$A_2$、$A_3$分别为三个主动臂和三个从动臂与上下两个平台的连接点。
![U5EBNz.png](https://m1.im5i.com/2022/11/25/U5EBNz.png)
<center>图4 三自由度Delta机械臂简化数学模型</center>

- 以定平台中心$O$建立坐标系$Ο-XYZ$，以动平台中心$p$建立坐标系$p-xyz$。由Delta机构的设计原理可知，三条支链完全对称，因此不妨设第$i（i=1,2,3）$条支链的主动臂$ \left|B_iC_i\right|$长度为$l_1$，从动臂$ \left|A_iB_i\right|$长度为$l_2$，主动臂与定平台夹角为$\theta_i$，三条支链与X轴的夹角为$ \varphi_i=\frac {2(i-1)\pi}{3}，i=1,2,3，$，定平台半径为$r_1$，动平台半径为$r_2$。

- 将$A_iB_i$分别沿$A_ip$平移使其交于点$p$得到$D_ip$，连接$D_1D_2$、$D_2D_3$、$D_3D_1$得到四棱锥$ p{-D}_1D_2D_3$,如下图所示。   
![U5E3xs.png](https://m1.im5i.com/2022/11/25/U5E3xs.png)   
<center>图5 三自由度Delta机械臂几何法正解简化模型</center>

- 根据图5不难得到，定平台三个铰接点$C_1、C_2、C_3$的坐标为
$$\begin{bmatrix}x_i \\ y_i\\ z_i \\ \end{bmatrix} = \begin{bmatrix}r_1\cosφ_i \\ r_1\sinφ_i\\ 0 \\ \end{bmatrix} , i=1,2,3\tag 1$$

- 向量$ \overrightarrow{OB_i} $可表示为
$$\overrightarrow{OB_i} = \overrightarrow{OC_i} + \overrightarrow{C_iB_i} \ , i=1,2,3 \tag 2$$

- 其中$ \overrightarrow{C_iB_i} $又可表示为
$$\begin{bmatrix}x_i \\ y_i\\ z_i \\ \end{bmatrix} = \begin{bmatrix}-l_1\sin\theta_i\cosφ_i \\ -l_1\sin\theta_i\sinφ_i\\ -l_1cos\theta_i \\ \end{bmatrix} , i=1,2,3 \tag 3$$

- $\overrightarrow{A_ip}$可表示为
$$\begin{bmatrix}x_i \\ y_i\\ z_i \\ \end{bmatrix} = \begin{bmatrix}-r_2\cosφ_i \\ -r_2\sinφ_i\\ 0 \\ \end{bmatrix} , i=1,2,3 \tag 4$$

- 由图5可知，$\overrightarrow{OD_i}$可以表示为
$$\overrightarrow{OD_i} = \overrightarrow{OB_i} + \overrightarrow{B_iD_i} \tag 5$$

- 由Delta几何性质可知
$$\overrightarrow{B_iD_i} = \overrightarrow{A_ip} \tag 6$$

- 因此，$\overrightarrow{OD_i}$又可以表示为
$$\overrightarrow{OD_i} = \overrightarrow{OC_i} + \overrightarrow{C_iB_i} + \overrightarrow{A_ip} \tag 7$$

- 综合式$(1) - (7)$可得，在坐标系$O-XYZ$中，$D_i$的坐标为
$$\begin{bmatrix}x_i \\ y_i\\ z_i \\ \end{bmatrix} = \begin{bmatrix}(r_1 - r_2 - l_1\sin\theta_i)\cosφ_i \\ (r_1 - r_2 - l_1\sin\theta_i)\sinφ_i\\ -l_1cos\theta_i \\ \end{bmatrix} , i=1,2,3 \tag 8$$

- 此时不难发现，Delta机构的正运动学解算已经转化为已知三个顶点坐标和各棱的长度求解另外一个顶点坐标的问题。解决问题的思路为先求得三棱锥的顶点$p$到底面的垂足，再求得垂线矢量，从而求得顶点坐标。
- 将图5中的四棱锥$p-D_1D_2D_3$取出单独分析，作垂线$pE$垂直于平面$D_1D_2D_3$于点$E$，取$D_2D_3$中点$F$，连接$EF$、$ED_2$，如下图所示。   
![U5E1GB.png](https://m1.im5i.com/2022/11/25/U5E1GB.png)
<center>图6 等效四棱锥</center>

- 首先，证明下底面垂足$E$为底面$\triangle D_1D_2D_3$的外心。如图6所示，设$E$为$\triangle D_1D_2D_3$的外接圆的圆心，则$FE\perp D_2D_3$,又$\triangle pD_2D_3$为等腰三角形，已知$pF\perp D_2D_3$,由立体几何三垂线定理得，$pE\perp D_2D_3$，同理$pE\perp D_1D_2$。所以,$pE$垂直于底面，为底面的垂线。
- 因此，$|\overrightarrow{D_2E}|$为三角形$D_1D_2D_3$的外接圆半径,设
$$
\begin{cases}
 {a = |\overrightarrow{D_1D_2}|}  \\
 {b = |\overrightarrow{D_2D_3}|}  \\
 {c = |\overrightarrow{D_1D_3}|}  \\
\end{cases}
 \tag 9$$

- 则$|\overrightarrow{D_2E}|$为
$$|\overrightarrow{D_2E}| = \frac {abc}{4S} \tag {10}$$

- 其中，$S$由海伦公式可得

$$
\begin{cases}
 {S = \sqrt{p(p-a)(p-b)(p-c)}}  \\
 {p=\frac {(a+b+c)}{2}}  \\
\end{cases}
 \tag {11}$$

- 对于向量$\overrightarrow{FE}$,其模长为
$$
\begin{aligned}
|\overrightarrow{FE}|
& = \sqrt{|\overrightarrow{D_2E}|^2 - |\overrightarrow{D_2F}|^2} \\
& = \sqrt{(\frac{abc}{4S})^2 - (\frac{b}{2})^2}
\end{aligned}
 \tag {12}$$


- 向量$\overrightarrow{FE}$的单位方向向量为
$$\hat{n}_{\overrightarrow{FE}} = \frac{\overrightarrow{D_2D_1}\times\overrightarrow{D_2D_3}\times\overrightarrow{D_3D_2}}{|\overrightarrow{D_2D_1}|\times|\overrightarrow{D_2D_3}|\times|\overrightarrow{D_3D_2}|} \tag {13}$$

- 对于向量$\overrightarrow{Ep}$,其模长为
$$
\begin{aligned}
|\overrightarrow{Ep}|
& = \sqrt{|\overrightarrow{D_1p}|^2 - |\overrightarrow{D_1E}|^2} \\
& = \sqrt{(l_2)^2 - (\frac{abc}{4S})^2}
\end{aligned}
 \tag {14}$$

- 向量$\overrightarrow{Ep}$的单位方向向量为
$$\hat{n}_{\overrightarrow{Ep}} = \frac{\overrightarrow{D_2D_1}\times\overrightarrow{D_2D_3}}{|\overrightarrow{D_2D_1}\times\overrightarrow{D_2D_3}|} \tag {15}$$

- 由图5可知, Delta的正向运动学只需求解向量$\overrightarrow{Op}$即可，因此
$$
\begin{cases}
 \overrightarrow{Op} 
 & = \overrightarrow{OE} + \overrightarrow{Ep}  \\
 \overrightarrow{OE} 
 & = \overrightarrow{OF} + \overrightarrow{FE}  \\
 \overrightarrow{OF} 
 & = \frac{(\overrightarrow{OD_2} + \overrightarrow{OD_3})}{2}  \\
 \overrightarrow{FE} 
 & = |\overrightarrow{FE}|\hat{n}_{\overrightarrow{FE}}  \\
 \overrightarrow{Ep} 
 & = |\overrightarrow{Ep}|\hat{n}_{\overrightarrow{Ep}}  \\
\end{cases}
 \tag {16}$$

##### 2.1.2 逆向运动学
已知Delta末端位姿$(x, y, z)$ ，求解三个关节电机的角度值。
- 根据Delta的结构，可以将其转化为求解图7中的单支链的$\theta_i$,进而得到机械臂三个主动臂的关节电机的角度值。
![U5EIHx.png](https://m1.im5i.com/2022/11/25/U5EIHx.png)
<center>图7 单支链求解示意图</center>

- 设$p$在、静平台坐标系下的坐标为$(x, y, z)$,则$A_i$在静平台坐标系中的位置矢量为
$$|\overrightarrow{OA_i}| = 
\begin{bmatrix}-r_2\cosφ_i \\ -r_2\sinφ_i\\ 0 \\ \end{bmatrix} + \begin{bmatrix}x \\ y\\ z \\ \end{bmatrix}, i=1,2,3 \tag {17}$$

- 联立$(1)-(3)$式，可得$B_i$在坐标系$O-XYZ$中的位置矢量为
$$|\overrightarrow{OB_i}| = r_1\begin{bmatrix}\cosφ_i \\ \sinφ_i\\ 0 \\ \end{bmatrix} -l_1 \begin{bmatrix}\sin\theta_i\cosφ_i \\ \sin\theta_i\sinφ_i\\ cos\theta_i \\ \end{bmatrix} \\
= \begin{bmatrix}(r_1-l_1sin\theta_i)\cosφ_i \\
(r_1-l_1sin\theta_i)\sinφ_i\\ -l_1cos\theta_i \\ \end{bmatrix} \tag {18}$$

- 由图7可知，向量$\overrightarrow{OA_i}$为
$$\overrightarrow{OA_i} = \overrightarrow{OB_i} + \overrightarrow{B_iA_i} \tag {19}$$

- 根据机构杆长约束$|\overrightarrow{B_iA_i}| = l_2$, 建立机构约束方程为


$$
\begin{aligned}
|\overrightarrow{B_iA_i}|^2
=  & [(r_1+r_2-l_1\sin\theta_i)cosφ_i - x]^2 + \\ 
& [(r_1+r_2-l_1\sin\theta_i)sinφ_i - y]^2 + [-(l_1cos\theta + z)]^2 \\
= & {l_2}^2
\end{aligned}
 \tag {20}$$

- 运用三角函数万能公式$sin\theta_i=\frac{2t}{1+t^2}, cos\theta_i=\frac{1-t^2}{1+t^2}$,对式$(20)$进行简化并整理为
$$A_i{t_i}^2 + B_i{t_i}+C_i =0,(i=1,2,3)  \tag {21}$$

- 式$(21)$是关于$t_i$的一元二次方程，可解得
$$t_i = \frac{-B_i \pm \sqrt{{B_i}^2 - 4A_iC_i} }{2A_i}= tan{\frac{\theta_i}{2}} ,(i=1,2,3) \tag {22}$$

- 分析机械手空间位置，结构方程最终取一个解
$$t_i = \frac{-B_i - \sqrt{{B_i}^2 - 4A_iC_i} }{2A_i}= tan{\frac{\theta_i}{2}} ,(i=1,2,3) \tag {23}$$

- 由$\theta_i = 2arctan(t_i)$,可得
$$\theta_i = 2arctan(\frac{-B_i - \sqrt{{B_i}^2 - 4A_iC_i} }{2A_i}),(i=1,2,3) \tag {24}$$

- 其中
$$
\begin{aligned}
 A_1 
 & = \frac{x^2+y^2+z^2+(r1-r2)^2+{l_1}^2-{l_2}^2-2x(r_1-r_2)}{2l_1} -(r_1-r_2-x) \\
 B_1 
 & = 2z  \\
 C_1 
 & = \frac{x^2+y^2+z^2+(r1-r2)^2+{l_1}^2-{l_2}^2-2x(r_1-r_2)}{2l_1} +(r_1-r_2-x)   \\
 A_2 
 & = \frac{x^2+y^2+z^2+(r1-r2)^2+{l_1}^2-{l_2}^2+(x-\sqrt{3}y)(r_1-r_2)}{l_1} - 2(r_1-r_2) - (x-\sqrt{3}y)\\
 B_2 
 & = 4z  \\
 C_2 
 & = \frac{x^2+y^2+z^2+(r1-r2)^2+{l_1}^2-{l_2}^2+(x-\sqrt{3}y)(r_1-r_2)}{l_1} + 2(r_1-r_2) + (x-\sqrt{3}y)   \\
 A_3 
 & = \frac{x^2+y^2+z^2+(r1-r2)^2+{l_1}^2-{l_2}^2+(x+\sqrt{3}y)(r_1-r_2)}{l_1} - 2(r_1-r_2) - (x+\sqrt{3}y)\\
 B_3 
 & = 4z  \\
 C_3 
 & = \frac{x^2+y^2+z^2+(r1-r2)^2+{l_1}^2-{l_2}^2+(x+\sqrt{3}y)(r_1-r_2)}{l_1} + 2(r_1-r_2) + (x+\sqrt{3}y)   \\
\end{aligned}
$$

- 综上，可求得Delta机械臂的关节电机角度$\theta_1,\theta_2,\theta_3$.

##### 2.1.3 参考文献
[1] [Delta机器人：运动学正反解分析](https://blog.csdn.net/qq413886183/article/details/106993725)    
[2] [赵杰,朱延河,蔡鹤皋.Delta型并联机器人运动学正解几何解法[J].哈尔滨工业大学学报,2003(01):25-27.](https://max.book118.com/html/2017/0530/110429743.shtm)      
[3] [伍经纹,徐世许,王鹏,宋婷婷.基于Adams的三自由度Delta机械手的运动学仿真分析[J].软件,2017,38(06):108-112.](https://www.docin.com/p-2435890504.html)       
[4] [Delta并联机构运动学分析及轨迹规划](https://www.docin.com/p-1059914591.html)      

