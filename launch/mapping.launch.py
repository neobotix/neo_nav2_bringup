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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable 
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments

    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    param_file_arg = LaunchConfiguration('param_file')

    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description="Use simulation (Gazebo) clock if true"
        )

    declare_param_file_arg = DeclareLaunchArgument(
            'param_file',
            default_value=os.path.join(
                get_package_share_directory('neo_nav2_bringup'),
                'config',
                'mapping.yaml'),
            description='Full path to param file to load'
        )

    # Node for starting the slam_toolbox
    start_sync_slam_toolbox_node = Node(
        parameters=[
          param_file_arg,
          {'use_sim_time': use_sim_time_arg}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_param_file_arg)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
