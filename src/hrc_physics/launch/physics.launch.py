"""Launch the MuJoCo physics sidecar by itself.

Standalone use: bring up the workcell with hrc_handoff_demo's launch
in one terminal, this one in another, and the cube will follow real
physics in the existing RViz view (visualize the
visualization_msgs/Marker on topic /scene/cube_marker).

    ros2 launch hrc_physics physics.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hrc_physics',
            executable='mujoco_runner',
            name='mujoco_runner',
            output='screen',
        )
    ])
