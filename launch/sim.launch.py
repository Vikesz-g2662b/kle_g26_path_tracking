"""Gazebo szimulacio inditasa a vehicle_blue robottal es a ROS 2 - Gazebo hid."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('path_tracking_compare')
    world = os.path.join(pkg_share, 'worlds', 'vehicle_blue.sdf')

    # Gazebo inditasa a vilaggal (-r: rogton elinditja a szimulaciot)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    # Hid a ROS 2 es a Gazebo kozott
    # ]  ROS -> Gazebo,  [  Gazebo -> ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    return LaunchDescription([gazebo, bridge])
