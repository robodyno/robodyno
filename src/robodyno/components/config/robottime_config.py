# -*-coding:utf-8 -*-
#
# Apache License, Version 2.0
#
# Copyright (c) 2023 Robottime(Beijing) Technology Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Hardware parameters of robottime devices."""

from robodyno.components.config.model import Model

ROBOTTIME_PARAMS = {
    'motor': {
        Model.ROBODYNO_PRO_P44: {
            'reduction': -44,
            'available_velocity': 27,
            'available_torque': 13,
            'available_current': 15,
            'torque_constant': 0.7742,
            'with_brake': False,
            'default_vel_limit': 5.71,
            'default_current_limit': 13.0,
            'default_pos_kp': 100.0,
            'default_vel_kp': 0.02,
            'default_vel_ki': 0.1,
        },
        Model.ROBODYNO_PRO_P12: {
            'reduction': -12.45,
            'available_velocity': 95,
            'available_torque': 4,
            'available_current': 15,
            'torque_constant': 0.2191,
            'with_brake': False,
            'default_vel_limit': 20.19,
            'default_current_limit': 13.0,
            'default_pos_kp': 100.0,
            'default_vel_kp': 0.02,
            'default_vel_ki': 0.1,
        },
        Model.ROBODYNO_PRO_H50: {
            'reduction': 50,
            'available_velocity': 23.6,
            'available_torque': 15,
            'available_current': 15,
            'torque_constant': 0.88,
            'with_brake': False,
            'default_vel_limit': 5.03,
            'default_current_limit': 13.0,
            'default_pos_kp': 100.0,
            'default_vel_kp': 0.02,
            'default_vel_ki': 0.1,
        },
        Model.ROBODYNO_PRO_H100: {
            'reduction': 100,
            'available_velocity': 11.8,
            'available_torque': 30,
            'available_current': 15,
            'torque_constant': 1.76,
            'with_brake': False,
            'default_vel_limit': 2.51,
            'default_current_limit': 13.0,
            'default_pos_kp': 100.0,
            'default_vel_kp': 0.02,
            'default_vel_ki': 0.1,
        },
        Model.ROBODYNO_PRO_DIRECT: {
            'reduction': 1,
            'available_velocity': 1180,
            'available_torque': 0.3,
            'available_current': 15,
            'torque_constant': 0.0176,
            'with_brake': False,
            'default_vel_limit': 251.33,
            'default_current_limit': 13.0,
            'default_pos_kp': 100.0,
            'default_vel_kp': 0.02,
            'default_vel_ki': 0.1,
        },
        Model.ROBODYNO_PLUS_H50: {
            'reduction': 50,
            'available_velocity': 24,
            'available_torque': 48,
            'available_current': 25,
            'torque_constant': 1.7229,
            'with_brake': True,
            'default_vel_limit': 3.77,
            'default_current_limit': 20.0,
            'default_pos_kp': 30.0,
            'default_vel_kp': 0.37,
            'default_vel_ki': 1.85,
        },
        Model.ROBODYNO_PLUS_H100: {
            'reduction': 100,
            'available_velocity': 12,
            'available_torque': 96,
            'available_current': 25,
            'torque_constant': 3.4458,
            'with_brake': True,
            'default_vel_limit': 1.88,
            'default_current_limit': 20.0,
            'default_pos_kp': 30.0,
            'default_vel_kp': 0.37,
            'default_vel_ki': 1.85,
        },
        Model.ROBODYNO_PLUS_P12: {
            'reduction': -12.45,
            'available_velocity': 96,
            'available_torque': 12,
            'available_current': 25,
            'torque_constant': 0.429,
            'with_brake': True,
            'default_vel_limit': 15.14,
            'default_current_limit': 20.0,
            'default_pos_kp': 30.0,
            'default_vel_kp': 0.37,
            'default_vel_ki': 1.85,
        },
        Model.ROBODYNO_PLUS_DIRECT: {
            'reduction': 1,
            'available_velocity': 1200,
            'available_torque': 0.96,
            'available_current': 25,
            'torque_constant': 0.034458,
            'with_brake': True,
            'default_vel_limit': 188.50,
            'default_current_limit': 20.0,
            'default_pos_kp': 30.0,
            'default_vel_kp': 0.37,
            'default_vel_ki': 1.85,
        },
        Model.ROBODYNO_NANO_P100: {
            'reduction': 100,
            'available_velocity': 500, # for develop 
            'available_torque': 100, # for develop
            'available_current': 100, # for develop
            'torque_constant': 0.25,
            'with_brake': False,
            'default_vel_limit': 10.0,
            'default_current_limit': 0.5,
            'default_pos_kp': 25.0,
            'default_vel_kp': 200.0,
            'default_vel_ki': 5.0,
        }
    }
}
