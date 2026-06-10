"""MoveIt-driven handoff demo launch.

Launches:
  - robot_state_publisher (workcell URDF)
  - move_group with hrc_moveit_config (planner + planning scene)
  - mujoco_runner (hrc_physics sidecar — owns the cube body, publishes
    /scene/cube_pose, /scene/contacts, and the /scene/cube_marker
    visualization)
  - handoff_moveit_demo (state machine, MoveIt action client,
    publishes /joint_states with the replayed trajectory; consumes
    cube state from the physics sidecar)
  - RViz with the workcell view

NO joint_state_publisher — the handoff_moveit_demo IS the
/joint_states source. NO trajectory_execution — handoff_moveit_demo
replays the planned trajectory manually since this is plan-only.

    ros2 launch hrc_handoff_demo handoff.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_description = get_package_share_directory("hrc_description_v2")
    pkg_moveit      = get_package_share_directory('hrc_moveit_config')
    pkg_handoff     = get_package_share_directory('hrc_handoff_demo')

    xacro_file       = os.path.join(pkg_description, 'urdf',   'workcell.urdf.xacro')
    srdf_file        = os.path.join(pkg_moveit,      'config', 'hrc.srdf')
    kinematics_file  = os.path.join(pkg_moveit,      'config', 'kinematics.yaml')
    joint_limits_yml = os.path.join(pkg_moveit,      'config', 'joint_limits.yaml')
    ompl_yml         = os.path.join(pkg_moveit,      'config', 'ompl_planning.yaml')
    pilz_cart_yml    = os.path.join(pkg_moveit,      'config', 'pilz_cartesian_limits.yaml')
    rviz_config      = os.path.join(pkg_handoff,     'rviz',   'handoff.rviz')

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
        output='screen',
        parameters=[
            moveit_config.robot_description,
            # Boost dynamic-TF publish rate so the URDF gripper renders
            # smoothly with joint_state at 100 Hz. Keep use_tf_static
            # at default (static transforms on /tf_static, latched).
            {'publish_frequency': 200.0},
        ])

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        output='screen', parameters=[moveit_config.to_dict()])

    # No MuJoCo physics — cube tracked via marker-reparenting in the
    # demo node (zero-lag, simple, what worked before we added physics).

    handoff = Node(
        package='hrc_handoff_demo', executable='handoff_demo',
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ])

    return LaunchDescription([rsp, move_group, handoff, rviz])
