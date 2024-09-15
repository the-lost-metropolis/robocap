import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

# Define the starting position and orientation of Robot1
ROBOT1_START_POSITION = [0.0, 0.0, 0.0]
ROBOT1_START_YAW = 0.0

def generate_launch_description():
    # Define the launch argument for using simulation time (default: True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Include the Gazebo simulation launch file from the 'ros_gz_sim' package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
    )

    # Get the path to the robot description (xacro) file
    sample_robot_description_path = os.path.join(
        get_package_share_directory('sample_robot_description'))

    xacro_file = os.path.join(sample_robot_description_path,
                              'robots',
                              'sample_robot.urdf.xacro')

    # Load and process the xacro file, generate URDF with mappings (e.g., 'use_sim' set to 'true')
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')  # Convert the xacro to pretty XML (URDF)

    # Set parameters for the robot state publisher (providing the robot description)
    params = {'robot_description': robot_desc}

    # Get the RViz configuration file path
    rviz_config_file = os.path.join(sample_robot_description_path, 'config', 'sample_robot_description.rviz')

    # Define the Robot State Publisher Node (publishes the state of the robot to the ROS ecosystem)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Define the Node to spawn the robot entity in the Gazebo simulation
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'sample_robot',
                   '-x', str(ROBOT1_START_POSITION[0]),
                   '-y', str(ROBOT1_START_POSITION[1]),
                   '-z', str(ROBOT1_START_POSITION[2]),
                   '-R', str(0.0),
                   '-P', str(0.0),
                   '-Y', str(ROBOT1_START_YAW),
                   '-allow_renaming', 'false'],
    )

    # Load the joint state controller, making the joint states of the robot available
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load the omni-wheel controller for controlling the robot's movements
    load_omni_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'omni_wheel_controller'],
        output='screen'
    )

    # Set up the ROS-Gazebo bridge to relay topics between ROS and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
        ],
        output='screen'
    )

    # Define the Node for converting velocities for the omni-wheel controller
    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
        remappings=[
            ('/cmd_vel_stamped', '/omni_wheel_controller/cmd_vel'),
        ],
    )

    # Define the RViz node to visualize the robot in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    # Return the launch description with all actions and event handlers
    return LaunchDescription([
        # Register event handlers to ensure controllers are loaded after the robot is spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_omni_wheel_controller],
            )
        ),
        # Start the Gazebo simulation, Robot State Publisher, and other processes
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        velocity_converter,
        rviz,
    ])
