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
    ROBODYNO_PRO_H50 = 0x02
    ROBODYNO_PRO_H100 = 0x03
    ROBODYNO_PRO_DIRECT = 0x0F
    ROBODYNO_PLUS_H50 = 0x10
    ROBODYNO_PLUS_H100 = 0x11
    ROBODYNO_PLUS_P12 = 0x12
    ROBODYNO_PLUS_DIRECT = 0x1F

    ROBODYNO_D_VAC01 = 0x61
    ROBODYNO_D_CNV02 = 0x80
    ROBODYNO_D_ADG03 = 0x20

    ROBODYNO_NANO_P100 = 0x20
    ROBODYNO_PWM_DRIVER = 0x61
    ROBODYNO_STEPPER_DRIVER = 0x62
    ROBODYNO_VACUUM_GRIPPER = 0x63
    ROBODYNO_ADAPTIVE_GRIPPER = 0x64
    ROBODYNO_FLEXIBLE_GRIPPER = 0x65
    ROBODYNO_EXB_FCTY = 0x80
    ROBODYNO_IMU_SENSOR = 0xA1
    ROBODYNO_GPS_SENSOR = 0xA2
    ROBODYNO_LED_DRIVER = 0xA3

    THIRD_PARTY = 0xFF

    @classmethod
    def _missing_(cls, value):
        return cls.THIRD_PARTY
