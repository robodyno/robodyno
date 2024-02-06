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

"""Setup script for robodyno."""

import os
from setuptools import setup

from src.robodyno import __version__

with open(
    os.path.join(os.path.dirname(__file__), 'README.md'),
    mode='r',
    encoding='utf-8',
) as f:
    long_description = f.read()

setup(
    name='robodyno',
    version=__version__,
    maintainer='robottime',
    maintainer_email='lab@robottime.cn',
    author='song',
    author_email='zhaosongy@126.com',
    description='The Robodyno Robot SDK for Python 3',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='http://101.42.250.169/',
    keywords=['robodyno', 'robot', 'robot module'],
    license='Apache License, Version 2.0',
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: POSIX :: Linux',
        'Framework :: Robot Framework',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'Programming Language :: Python :: 3 :: Only',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
    ],
    package_dir={
        '': 'src',
    },
    packages=[
        'robodyno',
        'robodyno.components',
        'robodyno.components.config',
        'robodyno.components.can_bus',
        'robodyno.components.webots',
        'robodyno.interfaces',
        'robodyno.tools',
        'robodyno.robots',
        'robodyno.robots.four_dof_palletizing_robot',
        'robodyno.robots.four_dof_scara_robot',
        'robodyno.robots.six_dof_collaborative_robot',
        'robodyno.robots.three_dof_delta_robot',
        'robodyno.robots.three_dof_palletizing_robot',
        'robodyno.robots.three_dof_cartesian_robot',
        'robodyno.robots.utils',
    ],
    python_requires='>=3.6',
    install_requires=[
        'numpy>=1.10.0',
        'rich >= 12.6.0',
        'click >= 7.1.2',
        'python-can>=3.2.0, <4.0',
        'importlib-metadata',
    ],
    extras_require={
        ':sys_platform == "win32"': ['candle-bus'],
    },
    entry_points={
        'robodyno.components.can_bus': [
            'Motor = robodyno.components.can_bus.motor:Motor',
            'PwmDriver = robodyno.components.can_bus.pwm_driver:PwmDriver',
            'StepperDriver = robodyno.components.can_bus.stepper_driver:StepperDriver',
            'SliderModule = robodyno.components.can_bus.slider_module:SliderModule',
            'ImuSensor = robodyno.components.can_bus.imu_sensor:ImuSensor',
            'GpsSensor = robodyno.components.can_bus.gps_sensor:GpsSensor',
            'LedDriver = robodyno.components.can_bus.led_driver:LedDriver',
            'Battery = robodyno.components.can_bus.battery:Battery',
            'UltrasonicSensor = robodyno.components.can_bus.ultrasonic_sensor:UltrasonicSensor',
            'CliffSensor = robodyno.components.can_bus.cliff_sensor:CliffSensor',
            'ImpactSensor = robodyno.components.can_bus.impact_sensor:ImpactSensor',
            'MagneticSensor = robodyno.components.can_bus.magnetic_sensor:MagneticSensor',
        ],
        'robodyno.components.webots': [
            'Motor = robodyno.components.webots.motor:Motor',
            'SliderModule = robodyno.components.webots.slider_module:SliderModule',
        ],
        'robodyno.robots': [
            'FourDoFPallet = robodyno.robots.four_dof_palletizing_robot.four_dof_pallet_robot:FourDoFPallet',
            'FourDoFScara = robodyno.robots.four_dof_scara_robot.four_dof_scara_robot:FourDoFScara',
            'SixDoFCollabRobot = robodyno.robots.six_dof_collaborative_robot.six_dof_collab_robot:SixDoFCollabRobot',
            'ThreeDoFDelta = robodyno.robots.three_dof_delta_robot.three_dof_delta_robot:ThreeDoFDelta',
            'ThreeDoFPallet = robodyno.robots.three_dof_palletizing_robot.three_dof_pallet_robot:ThreeDoFPallet',
            'ThreeDoFCartesian = robodyno.robots.three_dof_cartesian_robot.three_dof_cartesian_robot:ThreeDoFCartesian',
        ],
        'console_scripts': [
            'robodyno = robodyno.tools.cli:robodyno',
        ],
    },
)
