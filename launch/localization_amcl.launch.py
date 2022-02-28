# Copyright (c) 2022 Neobotix GmbH
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

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from pathlib import Path
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    parameters =  LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')    
    lifecycle_nodes = ['map_server', 'amcl']
    use_multi_robots = LaunchConfiguration('use_multi_robots', default='False')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time' : use_sim_time,
        'yaml_filename': map_file}

    configured_params = RewrittenYaml(
        source_file=parameters,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_multi_robots])),
        actions=[
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
        ]
    )

    load_nodes_multi_robot = GroupAction(
        condition=IfCondition(use_multi_robots),
        actions=[
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(load_nodes)
    ld.add_action(load_nodes_multi_robot)
    
    return ld
