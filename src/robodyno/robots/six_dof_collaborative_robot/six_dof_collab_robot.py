#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""six_dof_collab_robot.py
Time    :   2022/10/08
Author  :   ryan 
Version :   1.0
Contact :   ryanzhang@163.com
License :   (C)Copyright 2022, robottime / robodyno

6DoF Collaborative Robot Driver

  Typical usage example:

  from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot
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
"""

import time
from math import pi, sqrt, sin, cos, atan2
from cmath import acos, asin
import numpy as np
from ..utils.transformations import dh_matrix, euler_from_matrix, translation_from_matrix, translation_matrix, euler_matrix
from ..utils.interpolations import linear_interpolation

class SixDoFCollabRobot(object):
    """6 DoF collaborarive robot driver
    
    Attributes:
        joints: list of 6 joint motors
        l01: link from world to joint 1 (m) 
        l12: link from joint 1 to joint 2 (m) 
        l23: link from joint 2 to joint 3 (m) 
        l34: link from joint 3 to joint 4 (m) 
        l45: link from joint 4 to joint 5 (m) 
        l56: link from joint 5 to joint 6 (m) 
        end_effector: end effector object 
    """
    
    def __init__(self, j1, j2, j3, j4, j5, j6, l01, l12, l23, l34, l45, l56, end_effector = None):
        """init robot with joints and links"""
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

    def get_joints_poses(self):
        """Read joints positions to a list.
        
        Returns:
            a list of 6 joints positions
        """
        poses = []
        for i in range(6):
            pos = None
            count = 0
            while not pos:
                if count > 5:
                    raise RuntimeError('Filed to get position of Joint {}.'.format(i+1))
                count += 1
                pos = self.joints[i].get_pos(0.2)
            poses.append(pos - self._axes_zeros[i])
        return poses

    def enable(self):
        """enable joints motors"""
        for i in range(6):
            self.joints[i].enable()
    
    def disable(self):
        """disable joints motors"""
        for i in range(6):
            self.joints[i].disable()
    
    def init(self, axes_poses = [0 for i in range(6)]):
        """Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses: list of 6 axes poses(rad)
        """
        self._axes_poses = axes_poses.copy()
        self._axes_zeros = [0 for i in range(6)]
        cur_poses = self.get_joints_poses()
        for i in range(6):
            self._axes_zeros[i] = cur_poses[i] - self._axes_poses[i]
            self._axes_poses[i] = 0

    def set_joint_pos(self, id, pos):
        """Set joint angle with joint id and position
        
        Args:
            id: joint id (0-5)
            pos: joint target position(rad)
        """
        self.joints[id].set_pos(pos + self._axes_zeros[id])
        self._axes_poses[id] = pos
    
    def joint_space_interpolated_motion(self, target, speeds = [None for i in range(6)], duration = 0):
        """Robot interpolated motion in joint space.
        
        Args:
            target: iterable of 6 joints target angle(rad)
            speeds: iterable of 6 joints motion speed(rad/s)
            duration: default motion duration(s)
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

    def home(self, duration = 5):
        """Move back to zero position.
        
        Args:
            duration: motion duration(s)
        """
        self.joint_space_interpolated_motion((0,0,0,0,0,0), duration = duration)

    def cartesian_space_interpolated_motion(self, x, y, z, roll, pitch, yaw, sol_id = 2, 
        x_speed = None, y_speed = None, z_speed = None, roll_speed = None, 
        pitch_speed = None, yaw_speed = None, duration = 0):
        """Robot Interpolated motion in Cartesian space.
        
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

    def forward_kinematics(self, axes):
        """Forward kinematics algorism
        
        Args:
            axes: list of 6 joint angles(rad)
        
        Returns:
            tuples of robot end transform
            
            example:
                (x, y, z, roll, pitch, yaw)
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

    def inverse_kinematics(self, x, y, z, roll, pitch, yaw, sol_id=2):
        """Inverse kinematics algorism
        
        Args:
            x: robot end x
            y: robot end y
            z: robot end z
            roll: robot end roll
            pitch: robot end pitch
            yaw: robot end yaw
            sol_id: inverse kinematics solve id (0-7)
        
        Returns:
            list of joint angles
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