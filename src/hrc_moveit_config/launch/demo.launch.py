"""MoveIt demo for HRC-ARMS.

Launches:
  - robot_state_publisher (from hrc_description's xacro)
  - joint_state_publisher (with the 'ready' initial pose)
  - move_group (MoveIt planner)
  - RViz with MotionPlanning panel preconfigured for left_arm

Plan-only — no real or sim controllers, no trajectory execution.
Use the RViz "Planning" panel to query goal states and call 'Plan'.

    ros2 launch hrc_moveit_config demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_description = get_package_share_directory("hrc_description_v2")
    pkg_moveit      = get_package_share_directory('hrc_moveit_config')

    xacro_file       = os.path.join(pkg_description, 'urdf',   'workcell.urdf.xacro')
    srdf_file        = os.path.join(pkg_moveit,      'config', 'hrc.srdf')
    kinematics_file  = os.path.join(pkg_moveit,      'config', 'kinematics.yaml')
    joint_limits_yml = os.path.join(pkg_moveit,      'config', 'joint_limits.yaml')
    ompl_yml         = os.path.join(pkg_moveit,      'config', 'ompl_planning.yaml')
    pilz_cart_yml    = os.path.join(pkg_moveit,      'config', 'pilz_cartesian_limits.yaml')
    initial_pose_yml = os.path.join(pkg_description, 'config', 'initial_positions.yaml')
    rviz_config      = os.path.join(pkg_moveit,      'rviz',   'moveit.rviz')

    # Plan-only: skip .trajectory_execution() entirely. Loading an
    # empty controller_names list crashes the launch_ros parameter
    # parser ('got () of type tuple' — ROS2 cannot infer the array
    # type for an empty list).
    moveit_config = (
        MoveItConfigsBuilder('hrc_workcell', package_name='hrc_moveit_config')
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_yml)
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .pilz_cartesian_limits(file_path=pilz_cart_yml)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[moveit_config.robot_description])

    jsp = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        output='screen', parameters=[initial_pose_yml])

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        output='screen', parameters=[moveit_config.to_dict()])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ])

    return LaunchDescription([rsp, jsp, move_group, rviz])
