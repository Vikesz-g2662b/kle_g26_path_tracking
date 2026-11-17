"""Gazebo szimulacio, palya generalas, szabalyozo es RViz inditasa."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('path_tracking_compare')
    world = os.path.join(pkg_share, 'worlds', 'vehicle_blue.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'sim.rviz')

    path_type_arg = DeclareLaunchArgument(
        'path_type', default_value='circle', description='circle, figure8 vagy track')
    method_arg = DeclareLaunchArgument(
        'method', default_value='pure_pursuit', description='szabalyozo tipusa')
    speed_arg = DeclareLaunchArgument(
        'speed', default_value='1.0', description='sebesseg [m/s]')
    lookahead_arg = DeclareLaunchArgument(
        'lookahead', default_value='2.0', description='elorenezesi tavolsag [m]')

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

    path_generator = Node(
        package='path_tracking_compare',
        executable='path_generator',
        parameters=[{'path_type': LaunchConfiguration('path_type'),
                     'use_sim_time': True}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # ParameterValue kell, kulonben a "3" int lenne es hibat dob
    controller = Node(
        package='path_tracking_compare',
        executable='controller',
        parameters=[{
            'method': LaunchConfiguration('method'),
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'lookahead': ParameterValue(LaunchConfiguration('lookahead'), value_type=float),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # par masodperc, hogy a gazebo es az rviz elinduljon
    controller_delayed = TimerAction(period=5.0, actions=[controller])

    return LaunchDescription([
        path_type_arg, method_arg, speed_arg, lookahead_arg,
        gazebo, bridge, path_generator, rviz, controller_delayed,
    ])
