# Import necessary modules
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC, PythonExpression

def generate_launch_description():
    # Get the directories for packages
    botanbot_bringup_dir = get_package_share_directory('botanbot_bringup')

    # Declare launch arguments inline
    declare_arguments = [
        DeclareLaunchArgument(
            'use_simulator', default_value='True', description='Start simulator if true.'
        ),
        DeclareLaunchArgument(
            'headless', default_value='False', description='Execute Gazebo in headless mode.'
        ),
    ]

    # Start Gazebo Sim server with inline LaunchConfiguration
    start_gazebo_sim_server_cmd = ExecuteProcess(
        condition=IfCondition(LC('use_simulator')),
        cmd=['gz', 'sim', '-s', 'ros_gz_sim', '-r'],  # `-r` to reset the simulation
        cwd=[os.path.join(botanbot_bringup_dir, 'launch')],
        output='screen'
    )

    # Start Gazebo Sim client with inline LaunchConfiguration (optional GUI)
    start_gazebo_sim_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([LC('use_simulator'), ' and not ', LC('headless')])),
        cmd=['gz', 'gui'],
        cwd=[os.path.join(botanbot_bringup_dir, 'launch')],
        output='screen'
    )

    # Create launch description and populate
    ld = LaunchDescription()

    # Add declared arguments
    for declare_argument in declare_arguments:
        ld.add_action(declare_argument)

    # Add commands to launch description
    ld.add_action(start_gazebo_sim_server_cmd)
    ld.add_action(start_gazebo_sim_client_cmd)

    return ld
