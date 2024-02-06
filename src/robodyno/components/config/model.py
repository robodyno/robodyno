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

"""Models of Robodyno devices."""

from enum import Enum


class Model(Enum):
    """Enumerates the supported Robodyno device models."""

    ROBODYNO_PRO_P44 = 0x00
    ROBODYNO_PRO_P12 = 0x01
    ROBODYNO_PRO_P44A = 0x02
    ROBODYNO_PRO_P12A = 0x03
    ROBODYNO_PRO_H100 = 0x04
    ROBODYNO_PRO_H50 = 0x05
    ROBODYNO_PRO_B100 = 0x06
    ROBODYNO_PRO_DIRECT = 0x0F
    ROBODYNO_PLUS_H50 = 0x14
    ROBODYNO_PLUS_H100 = 0x15
    ROBODYNO_PLUS_P12 = 0x10
    ROBODYNO_PLUS_P12A = 0x12
    ROBODYNO_PLUS_DIRECT = 0x1F

    ROBODYNO_D_VAC01 = 0x61
    ROBODYNO_D_CNV02 = 0x80
    ROBODYNO_D_ADG03 = 0x20

    ROBODYNO_NANO_P100 = 0x20

    ROBODYNO_BATTERY = 0x40

    ROBODYNO_PWM_DRIVER = 0x61
    ROBODYNO_STEPPER_DRIVER = 0x62
    ROBODYNO_VACUUM_GRIPPER = 0x63
    ROBODYNO_ADAPTIVE_GRIPPER = 0x64
    ROBODYNO_FLEXIBLE_GRIPPER = 0x65
    ROBODYNO_EXB_FCTY = 0x80
    ROBODYNO_IMU_SENSOR = 0xA1
    ROBODYNO_GPS_SENSOR = 0xA2
    ROBODYNO_LED_DRIVER = 0xA3
    ROBODYNO_ULTRASONIC_SENSOR = 0xA4
    ROBODYNO_CLIFF_SENSOR = 0xA0
    ROBODYNO_IMPACT_SENSOR = 0xA5

    THIRD_PARTY = 0xFF

    @classmethod
    def _missing_(cls, value):
        return cls.THIRD_PARTY

    @classmethod
    def is_pro(cls, model):
        """Check if the model is a pro model."""
        return model in [
            cls.ROBODYNO_PRO_P44,
            cls.ROBODYNO_PRO_P12,
            cls.ROBODYNO_PRO_P44A,
            cls.ROBODYNO_PRO_P12A,
            cls.ROBODYNO_PRO_H100,
            cls.ROBODYNO_PRO_H50,
            cls.ROBODYNO_PRO_B100,
            cls.ROBODYNO_PRO_DIRECT,
        ]

    @classmethod
    def is_plus(cls, model):
        """Check if the model is a plus model."""
        return model in [
            cls.ROBODYNO_PLUS_H50,
            cls.ROBODYNO_PLUS_H100,
            cls.ROBODYNO_PLUS_P12,
            cls.ROBODYNO_PLUS_P12A,
            cls.ROBODYNO_PLUS_DIRECT,
        ]
