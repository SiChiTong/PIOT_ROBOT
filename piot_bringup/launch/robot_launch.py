#!/usr/bin/env python3
#
# Copyright 2021 NTREX CO., LTD.
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
#
# Authors: Gyuha Kang

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    
    can_control_pkg_dir = LaunchConfiguration(
        'can_control_pkg_dir',
        default=os.path.join(get_package_share_directory('piot_can_control'), 'launch'))

    converter_pkg_dir = LaunchConfiguration(
        'converter_pkg_dir',
        default=os.path.join(get_package_share_directory('piot_converter')))

        
    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('rplidar_ros2'), 'launch'))

    imu_pkg_dir = LaunchConfiguration(
        'imu_pkg_dir',
        default=os.path.join(get_package_share_directory('witmotion_ros'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/piot_state_publisher_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([can_control_pkg_dir, '/piot_can_control_launch.py']),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([converter_pkg_dir, '/piot_converter_launch.py']),
        ),       

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/rplidar_s2_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([imu_pkg_dir, '/witmotion.py']),
        ),
    ])
