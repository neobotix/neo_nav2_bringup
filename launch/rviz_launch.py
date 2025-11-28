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

# Contributor: Adarsh Karan K P

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

def execution_stage(
        context: LaunchContext,
        namespace, 
        use_namespace,
        rviz_config_file,
        rviz_output):
    
    rviz_output = str(rviz_output.perform(context))
    
    # When the rviz_output is log, stdout and stderr will not be shown in the terminal 
    if rviz_output == 'log':
        rviz_output_dict = {'both': 'log'}
    else:
        rviz_output_dict = {'both': 'screen'}

    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': ('/', namespace)})

    no_namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': ('', namespace)})

    start_namespaced_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(use_namespace),
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config_file],
        output=rviz_output_dict,
        )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(PythonExpression(['not ', use_namespace])),
        arguments=['-d', no_namespaced_rviz_config_file],
        output=rviz_output_dict,
        )

    exit_event_handler_namespaced = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    launch_actions = [
        start_namespaced_rviz_cmd,
        start_rviz_cmd,
        exit_event_handler_namespaced
    ]

    return launch_actions

def generate_launch_description():
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('neo_nav2_bringup'),
            'rviz', 
            'single_robot.rviz'),
        description='Full path to the RVIZ config file to use')
    
    declare_rviz_output_cmd = DeclareLaunchArgument(
        'rviz_output',
        default_value='screen',
        description='Output type for RViz node (screen or log)')

    opq_function = OpaqueFunction(function=execution_stage,
                              args=[
                                  LaunchConfiguration('namespace'),
                                  LaunchConfiguration('use_namespace'),
                                  LaunchConfiguration('rviz_config'),
                                  LaunchConfiguration('rviz_output')
                              ])
    
    ld = LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_rviz_config_file_cmd,
        declare_rviz_output_cmd,
        opq_function
    ])
    return ld

