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
from launch.actions import DeclareLaunchArgument, GroupAction, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('neo_nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_multi_robots = LaunchConfiguration('use_multi_robots')
    use_rviz = LaunchConfiguration('use_rviz')
    graph_filepath = LaunchConfiguration('graph_filepath')
    use_route = LaunchConfiguration('use_route')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']
    lifecycle_nodes_with_route = lifecycle_nodes + ['smoother_server',
                                                    'route_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'navigation.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_multi_robots_cmd =  DeclareLaunchArgument(
        'use_multi_robots', default_value='False',
        description='A flag to remove the remappings')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Launch RViz for visualization'
        )

    declare_graph_filepath_cmd = DeclareLaunchArgument(
        'graph_filepath', default_value='',
        description='Full path to the graph file for route planning'
        )

    declare_use_route_cmd = DeclareLaunchArgument(
        'use_route', default_value='False',
        description='Launch route-specific nodes like smoother and route server'
        )

    route_rviz_config = os.path.join(bringup_dir, 'rviz', 'route_server.rviz')
    single_robot_rviz_config = os.path.join(bringup_dir, 'rviz', 'single_robot.rviz')
    rviz_config = PythonExpression([
            '"', route_rviz_config, '" if "',
            use_route, '".lower() == "true" else "',
            single_robot_rviz_config, '"'
        ])

    load_nodes = GroupAction(
        condition=UnlessCondition(use_multi_robots),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
                condition=IfCondition(use_route)),
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                parameters=[configured_params, 
                            {'graph_filepath': graph_filepath}],
                remappings=remappings,
                condition=IfCondition(use_route)),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_with_route}],
                condition=IfCondition(use_route)),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}],
                condition=UnlessCondition(use_route)),
        ]
    )

    load_nodes_multi_robot = GroupAction(
        condition=IfCondition(use_multi_robots),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params]),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params]),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params]),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                condition=IfCondition(use_route)),
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                parameters=[configured_params, 
                            {'graph_filepath': graph_filepath}],
                condition=IfCondition(use_route)),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_with_route}],
                condition=IfCondition(use_route)),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}],
                condition=UnlessCondition(use_route)),
        ]
    )

    # Start RViz with the chosen config
    start_rviz = IncludeLaunchDescription(
        condition=IfCondition(use_rviz),
        launch_description_source=PythonLaunchDescriptionSource(
            [bringup_dir, '/launch/rviz_launch.py']),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_multi_robots,
            'use_sim_time': use_sim_time,
            'rviz_output': 'log',
            'rviz_config': rviz_config,
        }.items(),
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_multi_robots_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_route_cmd)
    ld.add_action(declare_graph_filepath_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_nodes_multi_robot)
    ld.add_action(start_rviz)

    return ld
