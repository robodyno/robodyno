#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""three_dof_delta_robot.py
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
  
"""
import time
import numpy as np
from math import pi, cos, sin, sqrt, atan
from ..utils.interpolations import linear_interpolation


class ThreeDoFDelta(object):
    """3 DoF delta robot driver
    
    Attributes:
        joints: list of 3 joint motors
        l1: link is the active arm (m)
        l2: link is the slave arm (m) 
        r1: fixed platform radius (m)
        r2: motion platform radius (m)
        end_effector: end effector object
    """
    def __init__(self, j1, j2, j3, l1, l2, r1, r2, end_effector = None):
        """init robot with joints and links"""
        self.joints = [j1, j2, j3]
        self.l1 = l1
        self.l2 = l2
        self.r1 = r1
        self.r2 = r2
        self.end_effector = end_effector

        for m in self.joints:
            m.position_filter_mode(8)
        self._axes_poses = [0 for i in range(3)]
        self._axes_zeros = [0 for i in range(3)]
        self._link_deviation = 0.08 # 4.63°

    def get_joints_poses(self):
        """Read joints positions to a list.

        Returns:
            a list of 3 joint positions
        """
        poses = []
        for i in range(3):
            pos = None
            count = 0
            while not pos:
                if count > 5:
                    raise RuntimeError('Filed to get position of Joint {}'.format(i+1))
                count += 1
                pos = self.joints[i].get_pos(0.2)
            poses.append(pos - self._axes_zeros[i])
        return poses

    def enable(self):
        """enable joints motors"""
        for i in range(3):
            self.joints[i].enable()

    def disable(self):
        """disable joints motors"""
        for i in range(3):
            self.joints[i].disable()

    def init(self, axes_poses = [0 for i in range(3)]):
        """Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses: list of 3 axes poses(rad)
        """
        self._axes_poses = axes_poses.copy()
        self._axes_zeros = [0 for i in range(3)]
        cur_poses = self.get_joints_poses()
        for i in range(3):
            self._axes_zeros[i] = cur_poses[i] - self._axes_poses[i]
            self._axes_poses[i] = 0

    def set_joint_pos(self, id, pos):
        """Set joint angle with joint id and position
        
        Args:
            id: joint id (0-2)
            pos: joint target position(rad)
        """
        self.joints[id].set_pos(pos + self._axes_zeros[id])
        self._axes_poses[id] = pos

    def joint_space_interpolated_motion(self, target, speeds = [None for i in range(3)], duration = 0):
        """Robot interpolated motion in joint space.
        
        Args:
            target: iterable of 3 joints target angle(rad)
            speeds: iterable of 3 joints motion speed(rad/s)
            duration: default motion duration(s)
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
    
    def home(self, duration = 5):
        """Move back to zero position.
        
        Args:
            duration: motion duration(s)
        """
        self.joint_space_interpolated_motion((0,0,0), duration = duration)

    def cartesian_space_interpolated_motion(self, x, y, z, x_speed = None, y_speed = None, z_speed = None, duration = 0):
        """Robot Interpolated motion in cartesian space.
        
        Args:
            x: target robot end x
            y: target robot end y
            z: target robot end z
            x_speed: speed alone X dimension(m/s)
            y_speed: speed alone Y dimension(m/s)
            z_speed: speed alone Z dimension(m/s)
            duration: default motion duration(s)
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
            for i in range(3):
                self.set_joint_pos(i, axes[i])
            if temp_x == x and temp_y == y and temp_z == z:
                break
            time.sleep(0.05)

    def forward_kinematics(self, angles):
        """Forward kinematics algorism
        
        Args:
            angles: list of 3 joint angles(rad)
        
        Returns:
            (x, y, z): tuples of 3 axis position
        """
        cos_0 = cos(0) # 0°
        sin_0 = sin(0)
        cos_120 = cos(2 * pi / 3) # 120°
        sin_120 = sin(2 * pi / 3)
        cos_240 = cos(4 * pi / 3) # 240°
        sin_240 = sin(4 * pi / 3)
        
        theta1 = angles[0] + self._link_deviation  
        theta2 = angles[1] + self._link_deviation 
        theta3 = angles[2] + self._link_deviation 

        O_C1 = np.array([self.r1 * cos_0  , self.r1 * sin_0  , 0])
        O_C2 = np.array([self.r1 * cos_120, self.r1 * sin_120, 0])
        O_C3 = np.array([self.r1 * cos_240, self.r1 * sin_240, 0])

        C1_B1 = np.array([-self.l1*sin(theta1)*cos_0  , -self.l1*sin(theta1)*sin_0  , -self.l1*cos(theta1)])
        C2_B2 = np.array([-self.l1*sin(theta2)*cos_120, -self.l1*sin(theta2)*sin_120, -self.l1*cos(theta2)])
        C3_B3 = np.array([-self.l1*sin(theta3)*cos_240, -self.l1*sin(theta3)*sin_240, -self.l1*cos(theta3)])

        A1_P = np.array([-self.r2*cos_0  , -self.r2*sin_0  , 0])
        A2_P = np.array([-self.r2*cos_120, -self.r2*sin_120, 0])
        A3_P = np.array([-self.r2*cos_240, -self.r2*sin_240, 0])

        O_D1 = O_C1 + C1_B1 + A1_P
        O_D2 = O_C2 + C2_B2 + A2_P
        O_D3 = O_C3 + C3_B3 + A3_P
        
        D2_D1 = O_D1 - O_D2
        D2_D3 = O_D3 - O_D2
        D3_D1 = O_D1 - O_D3
        D3_D2 = O_D2 - O_D3

        a = np.linalg.norm(D2_D1)
        b = np.linalg.norm(D2_D3)
        c = np.linalg.norm(D3_D1)

        p = (a + b + c) / 2
        S = sqrt(p * (p-a) * (p-b) * (p-c))
        D2_E_norm = a * b * c / (4 * S)

        D2_F_norm = b / 2
        FE_norm = np.linalg.norm(sqrt(D2_E_norm * D2_E_norm - D2_F_norm * D2_F_norm))

        nF_E1 = np.cross(np.cross(D2_D1, D2_D3), D3_D2) / (a * b * b)
        nF_E = nF_E1 / np.linalg.norm(nF_E1)

        nE_P1 = np.cross(D2_D1, D2_D3) / (a * b)
        nE_P = nE_P1 / np.linalg.norm(nE_P1)

        D1_P_norm = self.l2
        D1_E_norm = a * b * c / (4 * S)

        EP_norm = sqrt(D1_P_norm * D1_P_norm - D1_E_norm * D1_E_norm)
        EP = EP_norm * nE_P

        OF = (O_D2 + O_D3) / 2
        FE = FE_norm *nF_E
        OE = OF + FE

        OP = OE + EP

        x = OP[0]
        y = OP[1]
        z = OP[2]

        return (x, y, z)

    def inverse_kinematics(self, x, y, z):
        """inverse kinematics algorism
        
        Args:
            x: robot end x
            y: robot end y
            z: robot end z
        
        Returns:
            list of joint angles
        """
        angles = [0, 0, 0]
        m = x*x + y*y + z*z + (self.r1-self.r2)*(self.r1-self.r2) + self.l1*self.l1 - self.l2*self.l2 
        A = [(m - 2 * x * (self.r1 - self.r2)) / (2 * self.l1) - (self.r1 - self.r2 - x),
             (m + (self.r1 - self.r2) * (x - sqrt(3) * y)) / (self.l1) - 2 * (self.r1 - self.r2) - (x - sqrt(3) * y),
             (m + (self.r1 - self.r2) * (x + sqrt(3) * y)) / (self.l1) - 2 * (self.r1 - self.r2) - (x + sqrt(3) * y)]

        B = [2 * z,
             4 * z,
             4 * z]
        C = [(m - 2 * x * (self.r1 - self.r2)) / (2 * self.l1) + (self.r1 - self.r2 - x),
             (m + (self.r1 - self.r2) * (x - sqrt(3) * y)) / (self.l1) + 2 * (self.r1 - self.r2) + (x - sqrt(3) * y),
             (m + (self.r1 - self.r2) * (x + sqrt(3) * y)) / (self.l1) + 2 * (self.r1 - self.r2) + (x + sqrt(3) * y)]
        
        delta = [0,0,0]
        for i in range(3):
            delta[i] = B[i] * B[i] - 4 * A[i] * C[i]
            try:
                t = (-B[i] - sqrt(delta[i])) / (2 * A[i])
                angles[i] = 2 * atan(t) - (pi/2 + self._link_deviation)
            except ValueError:
                print('Pose not in range! Choose other Pose that is not a singularity')
                return self.get_joints_poses()

        return angles

