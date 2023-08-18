#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""four_dof_pallet_robot.py
Time    :   2022/10/17
Author  :   ryan 
Version :   1.0
Contact :   ryanzhang@163.com
License :   (C)Copyright 2022, robottime / robodyno

4DoF Pallet Robot Drive

  Typical usage example:

  from robodyno.robots.four_dof_pallet_robot import FourDoFPallet
  robot = FourDoFPallet(
    j1 = base_motor,
    j2 = upperarm_motor,
    j3 = forearm_motor,
    j4 = hand_motor,
    l01 = 0.0794,
    l23 = 0.190,
    l34 = 0.190,
    l45 = 0.010,
    end_effector = None
  )
"""
import time
from math import pi, cos, sin, sqrt, atan2, acos
from ..utils.interpolations import linear_interpolation

class FourDoFPallet(object):
    """4 DoF pallet robot driver
    
    Attributes:
        joints: list of 4 joint motors
        l01: link from world to joint 1 (m)
        l23: link from joint 2 to joint 3 transverse distance (m)
        l34: link from joint 3 to joint 4 longitudinal distance (m)
        l45: link from joint 4 to joint 5 transverse distance (m)
        end_effector: end effector object
    """
    def __init__(self, j1, j2, j3, j4, l01, l23, l34, l45, end_effector = None):
        self.joints = [j1, j2, j3, j4]
        self.l01 = l01
        self.l23 = l23
        self.l34 = l34
        self.l45 = l45
        self.end_effector = end_effector

        for m in self.joints:
          m.position_filter_mode(8)
        self._axes_poses = [0 for i in range(4)]
        self._axes_zeros = [0 for i in range(4)]

    def get_joints_pose(self):
        """Read joints position to a list.
        
        Returns:
            a list of 4 joint positions
        """
        poses = []
        for i in range(4):
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
        for i in range(4):
            self.joints[i].enable()

    def disable(self):
        """disable joints motors"""
        for i in range(4):
            self.joints[i].disable()

    def init (self, axes_poses = [0 for i in range(4)]):
        """Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses: list of 4 axes poses(rad)
        """
        self._axes_poses = axes_poses.copy()
        self._axes_zeros = [0 for i in range(4)]
        cur_poses = self.get_joints_pose()
        for i in range(4):
            self._axes_zeros[i] = cur_poses[i] - self._axes_poses[i]
            self._axes_poses[i] = 0

    def set_joint_pos(self, id, pos):
        """Set joint angle with joint and position.
        
        Args:
            id: joint id (0-3)
            pos: joint target position(red)
        """
        self.joints[id].set_pos(pos + self._axes_zeros[id])
        self._axes_poses[id] = pos        
        
    def joint_space_interpolated_motion(self, target, speeds = [0 for i in range(4)], duration = 0):
        """Robot interpolated motion in joint space.
        
        Args:
            target: iterable of 4 joints target angle(rad)
            speeds: iterable of 4 joints motion speed(rad/s)
            duration: default motion duration(s)
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

    def home(self, duration = 5):
        """Move back to zero position.
        
        Args:
            duration: motion duration(s)
        """
        self.joint_space_interpolated_motion((0,0,0,0), duration = duration)

    def cartesian_space_interpolated_motion(self, x, y, z, yaw, x_speed = None, y_speed = None, z_speed = None, yaw_speed = None, duration = 0):
        """Robot Interpolated motion in cartesian space.
        
        Args:
            x: target robot end x
            y: target robot end y
            z: target robot end z
            yaw: target robot end yaw
            x_speed: speed alone X dimension(m/s)
            y_speed: speed alone Y dimension(m/s)
            z_speed: speed alone Z dimension(m/s)
            yaw_speed: rotation speed on Z axis(rad/s)
            duration: default motion duration(s)
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

            axes = self.inverse_kinematics(temp_x, temp_y, temp_z, temp_yaw)
            for i in range(4):
                self.set_joint_pos(i, axes[i])
            if temp_x == x and temp_y == y and temp_z == z and temp_yaw == yaw:
                break
            time.sleep(0.05)

    def forward_kinematics(self, angles):
        """Forward kinematics algorism
        
        Args:
            angles: list of 4 joint angles(rad)
        
        Returns:
            (x, y, z, yaw): tuples of 3 axis position and 1 axis posture
        """
        a_2 = self.l23 * self.l23 + self.l34 * self.l34 - 2 * self.l23 * self.l34 * cos(pi/2 + angles[2])
        a = sqrt(a_2)
        cos_alpha = (self.l23 * self.l23 + a_2 - self.l34 * self.l34) / (2 * a * self.l23)
        alpha = acos(cos_alpha)
        
        b = a * sin(alpha + angles[1])
        
        x = b * cos(angles[0])
        y = b * sin(angles[0])
        z = a * cos(alpha + angles[1]) + self.l01 - self.l45
        yaw = -angles[0] + angles[3]

        return (x, y, z, yaw)
        

    def inverse_kinematics(self, x, y, z, yaw):
        """inverse kinematics algorism
        
        Args:
            x: robot end x
            y: robot end y
            z: robot end z
            yaw: robot end yaw
        
        Returns:
            list of joint angles
        """
        z -= self.l01 - self.l45
        angle = [0, 0, 0, 0]
        
        a = sqrt(x * x + y * y)
        # beta = atan2(z, a)
        b = sqrt(a * a + z * z)
        cos_alpha = (b * b + self.l23 * self.l23 - self.l34 * self.l34) / (2 * b * self.l23)
        
        # cos_c = (self.l23 * self.l23 + self.l34 * self.l34 - b * b) / (2 * self.l34 * self.l23)
        # theta_31 = acos(cos_c) - pi/2
        
        try:
            alpha = acos(cos_alpha)
        except ValueError:
            print("Pose not in range! Choose other Pose that is not a singularity")
            return self.get_joints_poses()

        angle[0] = atan2(y, x)
        angle[1] = atan2(a,z) -alpha
        angle[2] = pi - pi/2 - alpha*2
        angle[3] = yaw + angle[0]

        return angle
