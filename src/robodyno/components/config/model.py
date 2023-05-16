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
    ROBODYNO_D_CNV02 = 0x62
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
