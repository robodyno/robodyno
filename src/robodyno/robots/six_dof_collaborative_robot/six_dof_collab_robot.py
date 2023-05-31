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

"""This module provides a class for controlling Robodyno Six Dof Collaborative Robot.

The SixDoFCollabRobot class provided by this module is used to control Robodyno Six Dof Collaborative Robot
through the CAN bus. It provides methods for setting Six Dof Collaborative Robot parameters, reading Six Dof Collaborative Robot
states, and controlling the Six Dof Collaborative Robot to run in different space.

Examples:

    >>> from robodyno.components import Motor
    >>> from robodyno.interfaces import CanBus
    >>> from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot
    >>> can = CanBus()
    >>> class MySixDoFArm(SixDoFCollabRobot):
    >>>     def __init__(self):
    >>>         M1 = Motor(can, 0x10, 'ROBODYNO_PRO_44')
    >>>         M2 = Motor(can, 0x11, 'ROBODYNO_PRO_44')
    >>>         M3 = Motor(can, 0x12, 'ROBODYNO_PRO_44')
    >>>         M4 = Motor(can, 0x13, 'ROBODYNO_PRO_44')
    >>>         M5 = Motor(can, 0x14, 'ROBODYNO_PRO_12')
    >>>         M6 = Motor(can, 0x15, 'ROBODYNO_PRO_12')
    >>>     super().__init__(M1, M2, M3, M4, M5, M6, 0.18, 0.135, 0.135, 0.075, 0.075, 0.1)
    >>> arm = MySixDoFArm()
    >>> arm.init()
    >>> arm.get_joints_poses()
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    >>> can.disconnect()
"""

import time
import numpy as np
from cmath import acos, asin
from math import pi, sqrt, sin, cos, atan2
from ..utils.transformations import dh_matrix, euler_from_matrix, translation_from_matrix, translation_matrix, euler_matrix
from ..utils.interpolations import linear_interpolation

class SixDoFCollabRobot(object):
    """Controls Robodyno Six Dof Collaborative Robot through the CAN bus.
    
    Attributes:
        joints (list): list of 6 joint motors
        l01 (float): link from world to joint 1 (m) 
        l12 (float): link from joint 1 to joint 2 (m) 
        l23 (float): link from joint 2 to joint 3 (m) 
        l34 (float): link from joint 3 to joint 4 (m) 
        l45 (float): link from joint 4 to joint 5 (m) 
        l56 (float): link from joint 5 to joint 6 (m) 
        end_effector (object): end effector object 
    """
    
    def __init__(self, j1, j2, j3, j4, j5, j6, 
                 l01: float, l12: float, l23: float, l34: float, 
                 l45: float, l56: float, end_effector: object = None):
        """Initializes robot with joints and links
        
        Args:
            j1 (Motor): base_motor.
            j2 (Motor): shoulder_motor.
            j3 (Motor): upperarm_motor.
            j4 (Motor): elbow_motor.
            j5 (Motor): forearm_motor.
            j6 (Motor): hand_motor.
            l01 (float): link from world to joint 1 (m)
            l12 (float): link from joint 1 to joint 2 (m)
            l23 (float): link from joint 2 to joint 3 (m)
            l34 (float): link from joint 3 to joint 4 (m)
            l45 (float): link from joint 4 to joint 5 (m)
            l56 (float): link from joint 5 to joint 6 (m)
            end_effector (object): end effector object.
        """
        self.joints = [j1, j2, j3, j4, j5, j6]
        self.l01 = l01
        self.l12 = l12
        self.l23 = l23
        self.l34 = l34
        self.l45 = l45
        self.l56 = l56
        self.end_effector = end_effector

        for m in self.joints:
            m.position_filter_mode(8)
        self._axes_poses = [0 for i in range(6)]
        self._axes_zeros = [0 for i in range(6)]

    def get_joints_poses(self) -> list:
        """Read joints positions to a list.
        
        Returns:
            (list): a list of 6 joints positions
            
        Raises:
            RuntimeError: If the motor Joint is invalid.
        """
        poses = []
        for i in range(6):
            pos = None
            count = 0
            while pos is None:
                if count > 5:
                    raise RuntimeError('Filed to get position of Joint {}.'.format(i+1))
                count += 1
                pos = self.joints[i].get_pos(0.2)
            poses.append(pos - self._axes_zeros[i])
        return poses

    def enable(self) -> None:
        """enable joints motors"""
        for i in range(6):
            self.joints[i].enable()
    
    def disable(self) -> None:
        """disable joints motors"""
        for i in range(6):
            self.joints[i].disable()
    
    def init(self, axes_poses: list = [0 for i in range(6)]) -> None:
        """Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses (list): list of 6 axes poses(rad)
        """
        self._axes_poses = axes_poses.copy()
        self._axes_zeros = [0 for i in range(6)]
        cur_poses = self.get_joints_poses()
        for i in range(6):
            self._axes_zeros[i] = cur_poses[i] - self._axes_poses[i]
            self._axes_poses[i] = 0

    def set_joint_pos(self, id: int, pos: float) -> None:
        """Set joint angle with joint id and position
        
        Args:
            id (int): joint id (0-5)
            pos (float): joint target position(rad)
        """
        self.joints[id].set_pos(pos + self._axes_zeros[id])
        self._axes_poses[id] = pos
    
    def joint_space_interpolated_motion(self, target: list, 
                                        speeds: list = [None for i in range(6)], 
                                        duration:float = 0) -> None:
        """Robot interpolated motion in joint space.
        
        Args:
            target (list): iterable of 6 joints target angle(rad)
            speeds (list): iterable of 6 joints motion speed(rad/s)
            duration (float): default motion duration(s)
        """
        interpolations = []
        joint_poses = self._axes_poses.copy()
        for i in range(6):
            interpolations.append(linear_interpolation(joint_poses[i], target[i], speed = speeds[i], duration=duration))
        update_flag = True
        while update_flag:
            update_flag = False
            for i in range(6):
                pos = next(interpolations[i], None)
                if pos is not None:
                    self.set_joint_pos(i, pos)
                    update_flag = True
            time.sleep(0.05)

    def home(self, duration: float = 5) -> None:
        """Move back to zero position.
        
        Args:
            duration (float): motion duration(s)
        """
        self.joint_space_interpolated_motion((0,0,0,0,0,0), duration = duration)

    def cartesian_space_interpolated_motion(self, x: float, y: float, z, roll: float, pitch: float, yaw: float, 
                                            sol_id: int = 2, x_speed: float = None, y_speed: float = None, z_speed: float = None, 
                                            roll_speed: float = None, pitch_speed: float = None, yaw_speed: float = None, duration: float = 0) -> None:
        """Robot Interpolated motion in Cartesian space.
        
        Args:
            x (float): target robot end x
            y (float): target robot end y
            z (float): target robot end z
            roll (float): target robot end roll
            pitch (float): target robot end pitch
            yaw (float): target robot end yaw
            sol_id (int): inverse kinematics solve id (0-7)
            x_speed (float): speed alone X dimension(m/s)
            y_speed (float): speed alone Y dimension(m/s)
            z_speed (float): speed alone Z dimension(m/s)
            roll_speed (float): rotation speed on X axis(rad/s)
            pitch_speed (float): rotation speed on Y axis(rad/s)
            yaw_speed (float): rotation speed on Z axis(rad/s)
            duration (float): default motion duration(s)
        """
        current_x, current_y, current_z, current_roll, current_pitch, current_yaw =  self.forward_kinematics(self._axes_poses.copy())

        x_interpolation = linear_interpolation(current_x, x, x_speed, duration)
        y_interpolation = linear_interpolation(current_y, y, y_speed, duration)
        z_interpolation = linear_interpolation(current_z, z, z_speed, duration)
        roll_interpolation = linear_interpolation(current_roll, roll, roll_speed, duration)
        pitch_interpolation = linear_interpolation(current_pitch, pitch, pitch_speed, duration)
        yaw_interpolation = linear_interpolation(current_yaw, yaw, yaw_speed, duration)
        while True:
            temp_x = next(x_interpolation, x)
            temp_y = next(y_interpolation, y)
            temp_z = next(z_interpolation, z)
            temp_roll = next(roll_interpolation, roll)
            temp_pitch = next(pitch_interpolation, pitch)
            temp_yaw = next(yaw_interpolation, yaw)
            axes = self.inverse_kinematics(temp_x, temp_y, temp_z, temp_roll, temp_pitch, temp_yaw, sol_id)
            for i in range(6):
                self.set_joint_pos(i, axes[i])
            if temp_x == x and temp_y == y and temp_z == z and temp_roll == roll and temp_pitch == pitch and temp_yaw == yaw:
                break
            time.sleep(0.05)

    def forward_kinematics(self, axes: list) -> tuple:
        """Forward kinematics algorism
        
        Args:
            axes (list): list of 6 joint angles(rad)
        
        Returns:
            (tuple): (x, y, z, roll, pitch, yaw) tuples of robot end transform
            
        """
        T01 = dh_matrix(pi/2 , 0 , self.l01, axes[0])
        T12 = dh_matrix(0    , -self.l12, 0 , axes[1]-pi/2)
        T23 = dh_matrix(0    , -self.l23, 0 , -axes[2])
        T34 = dh_matrix(pi/2 , 0 , self.l34, axes[3]-pi/2)
        T45 = dh_matrix(-pi/2, 0 , self.l45, axes[4])
        T56 = dh_matrix(0    , 0 , self.l56, axes[5])

        T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
        x, y, z = list(translation_from_matrix(T06))
        roll, pitch, yaw = euler_from_matrix(T06)

        return (x, y, z, roll, pitch, yaw)

    def inverse_kinematics(self, x: float, y: float, z: float, roll: float, 
                           pitch: float, yaw: float, sol_id: int =2) -> list:
        """Inverse kinematics algorism
        
        Args:
            x (float): robot end x
            y (float): robot end y
            z (float): robot end z
            roll (float): robot end roll
            pitch (float): robot end pitch
            yaw (float): robot end yaw
            sol_id (int): inverse kinematics solve id (0-7)
        
        Returns:
            (list): list of joint angles
        """
        sol = np.zeros((6, 8))
        T06 = translation_matrix([x,y,z]) @ euler_matrix(roll, pitch, yaw)
        P05 = T06 @ np.array([[0, 0, -self.l56, 1]]).T
        
        # solve theta1
        phi1 = atan2(P05[1, 0], P05[0, 0])
        phi2 = acos(self.l34 / sqrt(P05[1, 0]*P05[1, 0] + P05[0, 0] * P05[0, 0])).real
        sol[0, 0:4] = pi / 2 + phi1 - phi2
        sol[0, 4:8] = pi / 2 + phi1 + phi2
        
        for i in range(8):
            # solve theta5
            sol[4, i] = acos((T06[0, 3] * sin(sol[0, i]) - T06[1, 3] * cos(sol[0, i]) - self.l34) / self.l56).real 
            if i % 2 == 1:
                sol[4, i] = -sol[4, i]
            
            # solve theta6
            T01 = dh_matrix(pi / 2, 0, self.l01, sol[0, i])
            T10 = np.linalg.inv(T01)
            T61 = np.linalg.inv(T10 @ T06)
            sol[5, i] = atan2(-T61[1, 2] / sin(sol[4, i]), T61[0, 2] / sin(sol[4, i])) if sol[4, i] else 0

            # solve theta3
            T45 = dh_matrix(-pi / 2, 0, self.l45, sol[4, i])
            T56 = dh_matrix(0, 0, self.l56, sol[5, i])
            T14 = T10 @ T06 @ np.linalg.inv(T56) @ np.linalg.inv(T45)
            P13 = T14 @ np.array([[0, -self.l34, 0, 1]]).T - np.array([[0, 0, 0, 1]]).T
            P13_norm = np.linalg.norm(P13)
            sol[2, i] = acos((P13_norm * P13_norm - self.l12 * self.l12 - self.l23 * self.l23) / (2 * self.l12 * self.l23)).real
            if i == 2 or i == 3 or i == 6 or i == 7:
                sol[2, i] = -sol[2, i]
            sol[1, i] = atan2(-P13[1, 0], -P13[0, 0]) - asin((self.l23 * sin(sol[2, i])) / P13_norm).real 

            # solve theta 4
            T12 = dh_matrix(0, -self.l12, 0, sol[1, i]) 
            T23 = dh_matrix(0, -self.l23, 0, sol[2, i])
            T34 = np.linalg.inv(T23) @ np.linalg.inv(T12) @ T14
            sol[3, i] = atan2(T34[1, 0], T34[0, 0]) 
            
            sol[1,i] = sol[1,i] + pi/2
            sol[2, i] = -sol[2, i]
            sol[3,i] = sol[3,i] + pi/2
        axes = []
        for i in range(6):
            axes.append(sol[i, sol_id])
        return axes