import os
import re
from setuptools import setup

with open(os.path.join(os.path.dirname(__file__), 'src', 'robodyno', 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# 从changelog文件中获取最新版本的版本号
with open(os.path.join(os.path.dirname(__file__), 'CHANGELOG.md'), 'r') as f:
    for line in f:
        if line.startswith('##'):
            match = re.search(r'\d+\.\d+\.\d+', line)
            if match:
                version = match.group(0)
                break

setup(
    name='robodyno',
    version=version,
    maintainer='robottime',
    maintainer_email='lab@robottime.cn',
    author='song',
    author_email='zhaosongy@126.com',
    description='The Robodyno Robot SDK for Python 3',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='http://101.42.250.169/',
    keywords=['robodyno', 'robot', 'robot module'],
    license='MIT License',
    classifiers=[
        'License :: OSI Approved :: MIT License',
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
    ],
    package_dir={
        '': 'src',
    },
    packages=[
        'robodyno',
        'robodyno.components',
        'robodyno.components.brands',
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
        'robodyno.robots.utils',
    ],
    python_requires='>=3.6',
    install_requires=[
        'numpy>=1.10.0', 
        'colorama>=0.4.5',
        'python-can>=3.2.0, <4.0',
        'importlib-metadata',
    ],
    extras_require={
        ':sys_platform == "win32"': [
            'candle-bus'
        ],
    },
    entry_points={
        'robodyno.components.can_bus': [
            'Motor = robodyno.components.can_bus.motor:Motor',
            'PwmDriver = robodyno.components.can_bus.pwm_driver:PwmDriver',
            'StepperDriver = robodyno.components.can_bus.stepper_driver:StepperDriver',
            'VGripper = robodyno.components.can_bus.vacuum_gripper:VGripper',
            'SliderModule = robodyno.components.can_bus.slider_module:SliderModule',
            'ImuSensor = robodyno.components.can_bus.imu_sensor:ImuSensor',
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
        ],
        'console_scripts': [
            'robodyno = robodyno:robodyno',
            'robodyno-motor = robodyno:robodyno_motor'
        ],
    }
)
