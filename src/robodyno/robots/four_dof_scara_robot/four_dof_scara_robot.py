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

"""This module provides a class for controlling Robodyno Four DoF Scara Robot.

The FourDoFScara class provided by this module is used to control Robodyno Four DoF Scara Robot
through the CAN bus. It provides methods for setting Four DoF Scara Robot parameters, reading Four DoF Scara Robot
states, and controlling the Four DoF Scara Robot to run in different space.

Examples:

    >>> from robodyno.components import Motor
    >>> from robodyno.interfaces import CanBus
    >>> from robodyno.robots.four_dof_scara_robot import FourDoFScara
    >>> can = CanBus()
    >>> class MyScara(FourDoFScara):
    >>>     def __init__(self):
    >>>         M1 = Motor(can, 0x10, 'ROBODYNO_PRO_44')
    >>>         M2 = Motor(can, 0x11, 'ROBODYNO_PRO_44')
    >>>         M3 = Motor(can, 0x12, 'ROBODYNO_PRO_12')
    >>>         M4 = Motor(can, 0x13, 'ROBODYNO_PRO_12')
    >>>     super().__init__(M1, M2, M3, M4, 0.06, 0.185, 0.185, 0.255, 0.01)
    >>> arm = MyScara()
    >>> arm.init()
    >>> arm.get_joints_poses()
    [0.0, 0.0, 0.0, 0.0]
    >>> can.disconnect()
"""

import time
from math import pi, cos, sin, sqrt, atan2
from ..utils.interpolations import linear_interpolation

class FourDoFScara(object):
    """Controls Robodyno Four DoF Scara Robot through the CAN bus.
    
    Attributes:
        joints (list): list of 4 joint motors
        l12 (float): link from joint 1 to joint 2  (m)
        l23 (float): link from joint 2 to joint 3  (m)
        l34 (float): link from joint 3 to joint 4  (m)
        l04 (float): link from world to joint 4 longitudinal distance (m)
        screw_lead (float): screw lead (m)
        end_effector (object): end effector object

    """
    def __init__(self, j1, j2, j3, j4, l12: float, l23: float, l34: float, 
                 l04: float, screw_lead: float, end_effector: object = None):
        """Initializes robot with joints and links
        
        Args:
            j1 (Motor): shoulder_motor.
            j2 (Motor): upperarm_motor.
            j3 (Motor): forearm_motor.
            j4 (Motor): hand_motor.
            l12 (float): link from joint 1 to joint 2  (m)
            l23 (float): link from joint 2 to joint 3  (m)
            l34 (float): link from joint 3 to joint 4  (m)
            l04 (float): link from world to joint 4 longitudinal distance (m)
            screw_lead (float): screw lead (m)
            end_effector (object): end effector object
        """
        self.joints = [j1, j2, j3, j4]
        self.l12 = l12
        self.l23 = l23
        self.l34 = l34
        self.l04 = l04
        self.screw_lead = screw_lead
        self.end_effector = end_effector

        for m in self.joints:
            m.position_filter_mode(8)
        # self.motors[0].position_traj_mode(10,5,5)
        self._axes_poses = [0 for i in range(4)]
        self._axes_zeros = [0 for i in range(4)]

    def get_joints_poses(self) -> list:
        """Read joints positions to a list.
        
        Returns:
            (list): a list of 4 joint positions
            
        Raises:
            RuntimeError: If the motor Joint is invalid.
        """
        poses = []
        for i in range(4):
            pos = None
            count = 0
            while pos is None:
                if count > 4:
                    raise RuntimeError('Filed to get position of Joint {}'.format(i+1))
                count += 1
                pos = self.joints[i].get_pos(0.2)
            poses.append(pos - self._axes_zeros[i])
        return poses

    def enable(self) -> None:
        """enable joints motors"""
        for i in range(4):
            self.joints[i].enable()

    def disable(self) -> None:
        """disable joints motors"""
        for i in range(4):
            self.joints[i].disable()

    def init(self, axes_poses: list = [0 for i in range(4)]) -> None:
        """Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses (list): list of 4 axes poses(rad)
        """
        self._axes_poses = axes_poses.copy()
        self._axes_zeros = [0 for i in range(4)]
        cur_poses = self.get_joints_poses()
        for i in range(4):
            self._axes_zeros[i] = cur_poses[i] - self._axes_poses[i]
            self._axes_poses[i] = 0

    def set_joint_pos(self, id: int, pos: float) -> None:
        """Set joint angle with joint id and position
        
        Args:
            id (int): joint id (0-3)
            pos (float): joint target position(rad)
        """
        self.joints[id].set_pos(pos + self._axes_zeros[id])
        self._axes_poses[id] = pos

    def joint_space_interpolated_motion(self, target: list, 
                                        speeds: list = [None for i in range(4)], 
                                        duration: float = 0) -> None:
        """Robot interpolated motion in joint space.
        
        Args:
            target (list): iterable of 4 joints target angle(rad)
            speeds (list): iterable of 4 joints motion speed(rad/s)
            duration (float): default motion duration(s)
        """
        interpolation = []
        joint_poses = self._axes_poses.copy()
        for i in range(4):
            interpolation.append(linear_interpolation(joint_poses[i], target[i], speed=speeds[i], duration=duration))
        update_flag = True
        while update_flag:
            update_flag = False
            for i in range(4):
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
        self.joint_space_interpolated_motion((0,0,0,0), duration = duration)

    def cartesian_space_interpolated_motion(self, x: float, y: float, z: float, yaw: float, x_speed: float = None, 
                                            y_speed: float = None, z_speed: float = None, yaw_speed: float = None, 
                                            hand_coordinate: bool = 1, duration: float = 0) -> None:
        """Robot Interpolated motion in cartesian space.
        
        Args:
            x (float): target robot end x
            y (float): target robot end y
            z (float): target robot end z
            yaw (float): target robot end yaw
            x_speed (float): speed alone X dimension(m/s)
            y_speed (float): speed alone Y dimension(m/s)
            z_speed (float): speed alone Z dimension(m/s)
            yaw_speed (float): rotation speed on Z axis(rad/s)
            hand_coordinate (bool): 0 is left hand coordinate system
                             1 is right hand coordinate system
            duration (float): default motion duration(s)
        """
        current_pose = self.forward_kinematics(self._axes_poses.copy())
        current_x = current_pose[0]
        current_y = current_pose[1]
        current_z = current_pose[2]
        current_yaw = current_pose[3]

        x_interpolation = linear_interpolation(current_x, x, x_speed, duration)
        y_interpolation = linear_interpolation(current_y, y, y_speed, duration)
        z_interpolation = linear_interpolation(current_z, z, z_speed, duration)
        yaw_interpolation = linear_interpolation(current_yaw, yaw, yaw_speed, duration)
        
        while True:
            temp_x = next(x_interpolation, x)
            temp_y = next(y_interpolation, y)
            temp_z = next(z_interpolation, z)
            temp_yaw = next(yaw_interpolation, yaw)

            axes = self.inverse_kinematics(temp_x, temp_y, temp_z, temp_yaw , hand_coordinate)
            for i in range(4):
                self.set_joint_pos(i, axes[i])
            if temp_x == x and temp_y == y and temp_z == z and temp_yaw == yaw:
                break
            time.sleep(0.05)

    def forward_kinematics(self, angles: list) -> tuple:
        """Forward kinematics algorism
        
        Args:
            angles (list): list of 4 joint angles(rad)
        
        Returns:
            (tuple): (x, y, z, yaw) tuples of 3 axis position and 1 axis posture
        """
        x = self.l23 * cos(angles[1]) + self.l34 * cos(angles[1] + angles[2]) + self.l12
        y = self.l23 * sin(angles[1]) + self.l34 * sin(angles[1] + angles[2])
        z = angles[0] * self.screw_lead / (2 * pi) + self.l04
        yaw = -angles[3] + angles[1] + angles[2]

        return (x, y, z, yaw)
    
    def inverse_kinematics(self, x: float, y: float, z: float, 
                           yaw: float, hand_coordinate: bool = 1) -> list:
        """inverse kinematics algorism
        
        Args:
            x (float): robot end x
            y (float): robot end y
            z (float): robot end z
            yaw (float): robot end yaw
            hand_coordinate (bool): 0 is left hand coordinate system
                             1 is right hand coordinate system
        
        Returns:
            (list): list of 4 joint angles
            
        Raises:
            ValueError: If the Scara Pose not in range.
        """
        angle = [0, 0, 0, 0]
        x -= self.l12
        z -= self.l04
        
        c3 = (x*x + y*y - self.l23*self.l23 - self.l34*self.l34) / (2.0 * self.l23 * self.l34)
        temp = 1 - c3 * c3
        try:
            if(hand_coordinate == 1):
                s3 = sqrt(temp)  # right
            elif(hand_coordinate == 0):
                s3 = -sqrt(temp) # left
        except ValueError:
            print("Pose not in range! Choose other Pose that is not a singularity")
            return self.get_joints_poses()
         
        angle[2] = atan2(s3, c3)
         
        alpha = atan2(y, x)
        r = sqrt(x*x + y*y)
        sin_beta = self.l34 * sin(angle[2]) / r
        cos_beta = (self.l23 + self.l34 * cos(angle[2])) / r
        beta = atan2(sin_beta, cos_beta)
        angle[1] = alpha - beta
        
        angle[0] = (2*pi * z) / self.screw_lead
        angle[3] = angle[1] + angle[2] - yaw
         
        return angle
        
