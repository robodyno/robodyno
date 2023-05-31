# -*-coding:utf-8 -*-
#
# The MIT License (MIT)
#
# Copyright (c) 2023 Robottime(Beijing) Technology Co., Ltd
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""This module provides a class for controlling Robodyno Three DoF Palletizing Robot.

The ThreeDoFPallet class provided by this module is used to control Robodyno Three DoF Palletizing Robot
through the CAN bus. It provides methods for setting Three DoF Palletizing Robot parameters, reading Three DoF Palletizing Robot
states, and controlling the Three DoF Palletizing Robot to run in different space.

Examples:

    >>> from robodyno.components import Motor
    >>> from robodyno.interfaces import CanBus
    >>> from robodyno.robots.three_dof_pallet_robot import ThreeDoFPallet
    >>> can = CanBus()
    >>> class MyPallet(ThreeDoFPallet):
    >>>     def __init__(self):
    >>>         M1 = Motor(can, 0x10, 'ROBODYNO_PRO_44')
    >>>         M2 = Motor(can, 0x11, 'ROBODYNO_PRO_44')
    >>>         M3 = Motor(can, 0x12, 'ROBODYNO_PRO_12')
    >>>     super().__init__((M1, M2, M3, 0.180, 0.190, 0.190, 0.010)
    >>> arm = MyPallet()
    >>> arm.init()
    >>> arm.get_joints_poses()
    [0.0, 0.0, 0.0]
    >>> can.disconnect()
"""

import time
from math import pi, cos, sin, sqrt, atan2, acos
from ..utils.interpolations import linear_interpolation

class ThreeDoFPallet(object):
    """Controls Robodyno Three DoF Palletizing Robot through the CAN bus.
    
    Attributes:
        joints (list): list of 3 joint motors
        l01 (float): link from world to joint 1 (m)
        l23 (float): link from joint 2 to joint 3 transverse distance (m)
        l34 (float): link from joint 3 to joint 4 longitudinal distance (m)
        l45 (float): link from joint 4 to joint 5 transverse distance (m)
        end_effector (object): end effector object
    """
    def __init__(self, j1, j2, j3, l01: float, l23: float, 
                 l34: float, l45: float, end_effector: object = None):
        """Initializes robot with joints and links
        
        Args:
            j1 (Motor): base_motor.
            j2 (Motor): upperarm_motor.
            j3 (Motor): forearm_motor.
            l01 (float): link from world to joint 1 (m)
            l23 (float): link from joint 2 to joint 3 transverse distance (m)
            l34 (float): link from joint 3 to joint 4 longitudinal distance (m)
            l45 (float): link from joint 4 to joint 5 transverse distance (m)
            end_effector (object): end effector object
        """
        self.joints = [j1, j2, j3]
        self.l01 = l01
        self.l23 = l23
        self.l34 = l34
        self.l45 = l45
        self.end_effector = end_effector

        for m in self.joints:
          m.position_filter_mode(8)
        self._axes_poses = [0 for i in range(3)]
        self._axes_zeros = [0 for i in range(3)]

    def get_joints_pose(self) -> list:
        """Read joints position to a list.
        
        Returns:
            (list): a list of 3 joint positions
        """
        poses = []
        for i in range(3):
            pos = None
            count = 0
            while pos is None:
                if count > 3:
                    raise RuntimeError('Filed to get position of Joint {}'.format(i+1))
                count += 1
                pos = self.joints[i].get_pos(0.2)
            poses.append(pos - self._axes_zeros[i])
        return poses

    def enable(self) -> None:
        """enable joints motors"""
        for i in range(3):
            self.joints[i].enable()

    def disable(self) -> None:
        """disable joints motors"""
        for i in range(3):
            self.joints[i].disable()

    def init (self, axes_poses: list = [0 for i in range(3)]) -> None:
        """Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses (list): list of 3 axes poses(rad)
        """
        self._axes_poses = axes_poses.copy()
        self._axes_zeros = [0 for i in range(3)]
        cur_poses = self.get_joints_pose()
        for i in range(3):
            self._axes_zeros[i] = cur_poses[i] - self._axes_poses[i]
            self._axes_poses[i] = 0

    def set_joint_pos(self, id: int, pos: float) -> None:
        """Set joint angle with joint and position.
        
        Args:
            id (int): joint id (0-2)
            pos (float): joint target position(red)
        """
        self.joints[id].set_pos(pos + self._axes_zeros[id])
        self._axes_poses[id] = pos        
        
    def joint_space_interpolated_motion(self, target: list, 
                                        speeds: list = [0 for i in range(3)], 
                                        duration: float = 0) -> None:
        """Robot interpolated motion in joint space.
        
        Args:
            target (list): iterable of 3 joints target angle(rad)
            speeds (list): iterable of 3 joints motion speed(rad/s)
            duration (float): default motion duration(s)
        """
        interpolation = []
        joint_poses = self._axes_poses.copy()
        for i in range(3):
            interpolation.append(linear_interpolation(joint_poses[i], target[i], speed=speeds[i], duration=duration))
        update_flag = True
        while update_flag:
            update_flag = False
            for i in range(3):
                pos = next(interpolation[i], None)
                if pos is not None:
                    self.set_joint_pos(i, pos)
                    update_flag = True
            time.sleep(0.05)

    def home(self, duration: float = 5) -> None:
        """Move back to zero position.
        
        Args:
            duration (float): motion duration(s)
        """
        self.joint_space_interpolated_motion((0,0,0), duration = duration)

    def cartesian_space_interpolated_motion(self, x: float, y: float, z: float, x_speed: float = None, 
                                            y_speed: float = None, z_speed: float = None, duration: float = 0) -> None:
        """Robot Interpolated motion in cartesian space.
        
        Args:
            x (float): target robot end x
            y (float): target robot end y
            z (float): target robot end z
            x_speed (float): speed alone X dimension(m/s)
            y_speed (float): speed alone Y dimension(m/s)
            z_speed (float): speed alone Z dimension(m/s)
            duration (float): default motion duration(s)
        """
        current_pose = self.forward_kinematics(self._axes_poses.copy())
        current_x = current_pose[0]
        current_y = current_pose[1]
        current_z = current_pose[2]

        x_interpolation = linear_interpolation(current_x, x, x_speed, duration)
        y_interpolation = linear_interpolation(current_y, y, y_speed, duration)
        z_interpolation = linear_interpolation(current_z, z, z_speed, duration)
        
        while True:
            temp_x = next(x_interpolation, x)
            temp_y = next(y_interpolation, y)
            temp_z = next(z_interpolation, z)

            axes = self.inverse_kinematics(temp_x, temp_y, temp_z)
            for i in range(4):
                self.set_joint_pos(i, axes[i])
            if temp_x == x and temp_y == y and temp_z == z:
                break
            time.sleep(0.05)

    def forward_kinematics(self, angles: list) -> tuple:
        """Forward kinematics algorism
        
        Args:
            angles (list): list of 3 joint angles(rad)
        
        Returns:
            (tuple): (x, y, z) tuples of 3 axis position
        """
        a_2 = self.l23 * self.l23 + self.l34 * self.l34 - 2 * self.l23 * self.l34 * cos(pi/2 + angles[2])
        a = sqrt(a_2)
        # alpha = (pi-pi/2-theta3) / 2
        cos_alpha = (self.l23 * self.l23 + a_2 - self.l34 * self.l34) / (2 * a * self.l23)
        alpha = acos(cos_alpha)
        # print(alpha)
        
        b = a * sin(alpha + angles[1])
        
        x = b * cos(angles[0])
        y = b * sin(angles[0])
        z = a * cos(alpha + angles[1]) + self.l01 - self.l45
        return (x, y, z)
        

    def inverse_kinematics(self, x: float, y: float, z: float) -> list:
        """inverse kinematics algorism
        
        Args:
            x (float): robot end x
            y (float): robot end y
            z (float): robot end z
        
        Returns:
            (list): list of joint angles
            
        Raises:
            ValueError: If the palletizing robot Pose not in range.
        """
        cf = z - self.l01 + self.l45
        angle = [0, 0, 0]
        
        a = sqrt(x * x + y * y)
        # beta = atan2(z, a)
        r = sqrt(a * a + cf * cf)
        cos_alpha = (r * r + self.l23 * self.l23 - self.l34 * self.l34) / (2 * r * self.l23)
        
        cos_abc = (self.l23 * self.l23 + self.l34 * self.l34 - r * r) / (2 * self.l34 * self.l23)
        
        try:
            alpha = acos(cos_alpha)
            abc = acos(cos_abc)
        except ValueError:
            print("Pose not in range! Choose other Pose that is not a singularity")
            return self.get_joints_poses()

        angle[0] = atan2(y, x)
        angle[1] = atan2(a,z) -alpha
        angle[2] = abc - pi/2

        return angle
        
