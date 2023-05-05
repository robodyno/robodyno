#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""four_dof_scara_robot.py
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
"""
import time
from math import pi, cos, sin, sqrt, atan2
from ..utils.interpolations import linear_interpolation

class FourDoFScara(object):
    """4 DoF scara robot driver
    
    Attributes:
        joints: list of 4 joint motors
        l12: link from joint 1 to joint 2  (m)
        l23: link from joint 2 to joint 3  (m)
        l34: link from joint 3 to joint 4  (m)
        l04: link from world to joint 4 longitudinal distance (m)
        screw_lead: screw lead (m)
        end_effector: end effector object

    """
    def __init__(self, j1, j2, j3, j4, l12, l23, l34, l04, screw_lead, end_effector = None):
        """init robot with joints and links"""
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

    def get_joints_poses(self):
        """Read joints positions to a list.
        
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

    def init(self, axes_poses = [0 for i in range(4)]):
        """Calibrate robot motors with given axes poses.
        
        Args:
            axes_poses: list of 4 axes poses(rad)
        """
        self._axes_poses = axes_poses.copy()
        self._axes_zeros = [0 for i in range(4)]
        cur_poses = self.get_joints_poses()
        for i in range(4):
            self._axes_zeros[i] = cur_poses[i] - self._axes_poses[i]
            self._axes_poses[i] = 0

    def set_joint_pos(self, id, pos):
        """Set joint angle with joint id and position
        
        Args:
            id: joint id (0-3)
            pos: joint target position(rad)
        """
        self.joints[id].set_pos(pos + self._axes_zeros[id])
        self._axes_poses[id] = pos

    def joint_space_interpolated_motion(self, target, speeds = [None for i in range(4)], duration = 0):
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

    def cartesian_space_interpolated_motion(self, x, y, z, yaw, x_speed = None, y_speed = None, z_speed = None, yaw_speed = None, hand_coordinate = 1, duration = 0):
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
            hand_coordinate: 0 is left hand coordinate system
                             1 is right hand coordinate system
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

            axes = self.inverse_kinematics(temp_x, temp_y, temp_z, temp_yaw , hand_coordinate)
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
        x = self.l23 * cos(angles[1]) + self.l34 * cos(angles[1] + angles[2]) + self.l12
        y = self.l23 * sin(angles[1]) + self.l34 * sin(angles[1] + angles[2])
        z = angles[0] * self.screw_lead / (2 * pi) + self.l04
        yaw = -angles[3] + angles[1] + angles[2]

        return (x, y, z, yaw)
    
    def inverse_kinematics(self, x, y, z, yaw, hand_coordinate = 1):
        """inverse kinematics algorism
        
        Args:
            x: robot end x
            y: robot end y
            z: robot end z
            yaw: robot end yaw
            hand_coordinate: 0 is left hand coordinate system
                             1 is right hand coordinate system
        
        Returns:
            list of 4 joint angles
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
        
