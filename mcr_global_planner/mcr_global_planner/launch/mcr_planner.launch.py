#!/usr/bin/env python3

# Copyright (c) 2020 Samsung Research Russia
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
from launch import LaunchDescription
import launch_ros.actions
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['mcr_planner']
    use_sim_time = True
    autostart = True
    package_dir = get_package_share_directory("mcr_global_planner")
    planner_param_file = os.path.join(package_dir, "params/planner_params.yaml")

    configured_params = RewrittenYaml(
        source_file=planner_param_file,
        param_rewrites={}
    )

    # Nodes launching commands
    planner_cmd = launch_ros.actions.Node(
            package='mcr_global_planner',
            executable='global_planner',
            name='mcr_planner',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params],
            prefix=['xterm -e gdb --args']
            )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    ld = LaunchDescription()

    ld.add_action(planner_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
