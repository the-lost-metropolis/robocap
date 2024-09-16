from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the path to the Xacro file in the source directory
    xacro_file = '/home/developer/repo/src/robocap_sim/urdf/robot.urdf.xacro'

    # Convert Xacro to URDF
    urdf_file = '/tmp/robot.urdf'  # Temporary file location for the converted URDF

    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )

    # Start Ignition Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '--verbose'}.items(),  # Use --verbose for more logging information
    )

    # Spawn the ground plane
    spawn_ground_plane = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ground_plane', '-topic', 'world/default/create', '-file', 'https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Ground%20Plane'],
        output='screen'
    )

    # Spawn robot in Ignition Gazebo with a z-offset of 0.1 meter
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_file, '-name', 'robot', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        xacro_to_urdf,
        gz_sim,
        spawn_ground_plane,
        spawn_robot,
    ])
