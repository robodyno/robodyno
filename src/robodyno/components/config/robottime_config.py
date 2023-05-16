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

"""Hardware parameters of robottime devices."""

from .model import Model

ROBOTTIME_PARAMS = {
    'motor': {
        Model.ROBODYNO_PRO_P44: {
            'reduction': -44,
            'available_velocity': 27,
            'available_torque': 13,
            'available_current': 15,
            'torque_constant': 0.7742,
            'with_brake': False,
        },
        Model.ROBODYNO_PRO_P12: {
            'reduction': -12.45,
            'available_velocity': 95,
            'available_torque': 4,
            'available_current': 15,
            'torque_constant': 0.2191,
            'with_brake': False,
        },
        Model.ROBODYNO_PRO_H50: {
            'reduction': 50,
            'available_velocity': 23.6,
            'available_torque': 15,
            'available_current': 15,
            'torque_constant': 0.88,
            'with_brake': False,
        },
        Model.ROBODYNO_PRO_H100: {
            'reduction': 100,
            'available_velocity': 11.8,
            'available_torque': 30,
            'available_current': 15,
            'torque_constant': 1.76,
            'with_brake': False,
        },
        Model.ROBODYNO_PRO_DIRECT: {
            'reduction': 1,
            'available_velocity': 1180,
            'available_torque': 0.3,
            'available_current': 15,
            'torque_constant': 0.0176,
            'with_brake': False,
        },
        Model.ROBODYNO_PLUS_H50: {
            'reduction': 50,
            'available_velocity': 24,
            'available_torque': 48,
            'available_current': 25,
            'torque_constant': 1.7229,
            'with_brake': True,
        },
        Model.ROBODYNO_PLUS_H100: {
            'reduction': 100,
            'available_velocity': 12,
            'available_torque': 96,
            'available_current': 25,
            'torque_constant': 3.4458,
            'with_brake': True,
        },
        Model.ROBODYNO_PLUS_P12: {
            'reduction': -12.45,
            'available_velocity': 96,
            'available_torque': 12,
            'available_current': 25,
            'torque_constant': 0.429,
            'with_brake': True,
        },
        Model.ROBODYNO_PLUS_DIRECT: {
            'reduction': 1,
            'available_velocity': 1200,
            'available_torque': 0.96,
            'available_current': 25,
            'torque_constant': 0.034458,
            'with_brake': True,
        },
        Model.ROBODYNO_NANO_P100: {
            'reduction': 100,
            'available_velocity': 16.23,
            'available_torque': 0.15,
            'available_current': 0.6,
            'torque_constant': 0.25,
            'with_brake': False,
        }
    }
}
