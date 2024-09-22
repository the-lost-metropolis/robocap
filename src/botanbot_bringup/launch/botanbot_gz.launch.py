# Import necessary modules
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC, PythonExpression

GAZEBO_WORLD = "uneven_world"

def generate_launch_description():
    # Get the directories for packages
    botanbot_bringup_dir = get_package_share_directory('botanbot_bringup')
    botanbot_gazebo_dir = get_package_share_directory('botanbot_gazebo')

    # Declare launch arguments inline
    declare_arguments = [
        DeclareLaunchArgument(
            'use_simulator', default_value='True', description='Start simulator if true.'
        ),
        DeclareLaunchArgument(
            'headless', default_value='False', description='Execute Gazebo in headless mode.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(botanbot_gazebo_dir, 'worlds', GAZEBO_WORLD, GAZEBO_WORLD + '.world'),
            description='Path to Gazebo world file.'
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

    # Create launch description and populate
    ld = LaunchDescription()

    # Add declared arguments
    for declare_argument in declare_arguments:
        ld.add_action(declare_argument)

    # Add commands to launch description
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld