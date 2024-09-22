# Copyright (c) 2018 Intel Corporation
# Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node

GAZEBO_WORLD = "sonoma_raceway_world"

def generate_launch_description():
    # Get the directories for packages
    botanbot_bringup_dir = get_package_share_directory('botanbot_bringup')
    vox_nav_bringup_dir = get_package_share_directory('vox_nav_bringup')
    botanbot_gazebo_dir = get_package_share_directory('botanbot_gazebo')

    # Declare launch arguments inline
    declare_arguments = [
        DeclareLaunchArgument(
            'params',
            default_value=os.path.join(botanbot_bringup_dir, 'params', 'vox_nav_uneven_world_params.yaml'),
            description='Path to vox_nav parameters file.'
        ),
        DeclareLaunchArgument(
            'localization_params',
            default_value=os.path.join(botanbot_bringup_dir, 'params', 'robot_localization_params.yaml'),
            description='Path to localization parameters file.'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(botanbot_bringup_dir, 'rviz', 'vox_nav_default_view.rviz'),
            description='Path to RViz configuration file.'
        ),
        DeclareLaunchArgument(
            'namespace', default_value='', description='Top-level namespace.'
        ),
        DeclareLaunchArgument(
            'use_namespace', default_value='false', description='Use namespace for vox_nav.'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true.'
        ),
        DeclareLaunchArgument(
            'use_simulator', default_value='True', description='Start simulator if true.'
        ),
        DeclareLaunchArgument(
            'use_robot_state_pub', default_value='True', description='Start robot state publisher if true.'
        ),
        DeclareLaunchArgument(
            'headless', default_value='False', description='Execute Gazebo in headless mode.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(botanbot_gazebo_dir, 'worlds', GAZEBO_WORLD, GAZEBO_WORLD + '.world'),
            description='Path to Gazebo world file.'
        ),
        DeclareLaunchArgument(
            'joy_config_filepath',
            default_value=os.path.join(botanbot_bringup_dir, 'params', 'joystick_xbox.yaml'),
            description='Path to joystick configuration file.'
        )
    ]

    # Start Gazebo server and client with inline LaunchConfiguration
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(LC('use_simulator')),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', LC('world')],
        cwd=[os.path.join(botanbot_bringup_dir, 'launch')],
        output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([LC('use_simulator'), ' and not ', LC('headless')])),
        cmd=['gzclient'],
        cwd=[os.path.join(botanbot_bringup_dir, 'launch')],
        output='screen'
    )

    # Start robot state publisher with inline LaunchConfiguration
    urdf = os.path.join(get_package_share_directory('botanbot_description'), 'urdf/botanbot.urdf')
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(LC('use_robot_state_pub')),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LC('namespace'),
        output='screen',
        parameters=[{'use_sim_time': LC('use_sim_time')}],
        arguments=[urdf],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Include vox_nav launch with inline LaunchConfiguration
    bringup_vox_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vox_nav_bringup_dir, 'launch', 'bringup_vox_nav.launch.py')),
        launch_arguments={
            'params': LC('params'),
            'localization_params': LC('localization_params'),
            'rviz_config': LC('rviz_config'),
            'namespace': LC('namespace'),
            'use_namespace': LC('use_namespace'),
            'use_sim_time': LC('use_sim_time')
        }.items()
    )

    # Include joystick teleop with inline LaunchConfiguration
    joy_config_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')),
        launch_arguments={'config_filepath': LC('joy_config_filepath')}.items()
    )

    # Include twist_mux with inline LaunchConfiguration
    twist_mux_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('twist_mux'), 'launch', 'twist_mux_launch.py'))
    )

    # Create launch description and populate
    ld = LaunchDescription()

    # Add declared arguments
    for declare_argument in declare_arguments:
        ld.add_action(declare_argument)

    # Add commands to launch description
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_vox_nav_cmd)
    ld.add_action(joy_config_cmd)
    ld.add_action(twist_mux_cmd)

    return ld
