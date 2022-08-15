# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    lifecycle_nodes = ['mcr_uwb']

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='mcr_uwb',
            executable='mcr_uwb',
            name='mcr_uwb',
            # prefix=['xterm -e gdb --args']
            ),
        Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           parameters=[{'use_sim_time': True}],
           arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'uwb']),        
        Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           parameters=[{'use_sim_time': True}],
           arguments=['0.2185', '0', '-0.00495', '0', '0', '0', 'uwb', 'head_uwb']),

        Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           parameters=[{'use_sim_time': True}],
           arguments=['0.17', '0', '0.164', '3.14159', '0', '0', 'uwb', 'head_tof']),

        Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           parameters=[{'use_sim_time': True}],
           arguments=['-0.023', '0.0845', '-0.00325', '1.5708', '0', '0', 'uwb', 'rear_uwb']),

        Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           parameters=[{'use_sim_time': True}],
           arguments=['-0.0235', '-0.0845', '-0.00325', '-1.5708', '0', '0', 'uwb', 'rear_tof']),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_simulator',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])
    ])

