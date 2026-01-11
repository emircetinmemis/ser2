"""
Launch SMACH safety system only

source install/setup.bash && cls && ros2 launch robile_safety_system smach_only.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get config file
    smach_config = os.path.join(get_package_share_directory("robile_safety_smach"), "config", "config.yaml")

    return LaunchDescription(
        [
            # SMACH Safety Node
            Node(
                package="robile_safety_smach",
                executable="safety_smach_node",
                name="safety_smach_node",
                output="screen",
                parameters=[smach_config],
                emulate_tty=True,
            ),
        ]
    )
