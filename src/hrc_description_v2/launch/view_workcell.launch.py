"""Visual smoke test for hrc_description_v2.

Launches robot_state_publisher + joint_state_publisher_gui + rviz2 so
you can interactively drive every joint of both UR5e arms and the
Robotiq fingers and confirm the URDF parses cleanly.

    ros2 launch hrc_description_v2 view_workcell.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('hrc_description_v2')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'workcell.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'view_workcell.rviz'])

    ur_type_arg = DeclareLaunchArgument(
        'ur_type', default_value='ur5e',
        description='UR arm variant (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ...)')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                xacro_path, ' ',
                'ur_type:=', LaunchConfiguration('ur_type'),
            ]),
            value_type=str),
    }

    initial_positions = PathJoinSubstitution([
        pkg_share, 'config', 'initial_positions.yaml'])

    return LaunchDescription([
        ur_type_arg,
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[robot_description]),
        Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            output='screen', parameters=[initial_positions]),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'),
    ])
