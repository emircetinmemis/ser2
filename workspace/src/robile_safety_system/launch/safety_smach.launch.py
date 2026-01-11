"""
Launch battery + SMACH safety system

source install/setup.bash && cls && ros2 launch robile_safety_system safety_smach.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get config files
    battery_config = os.path.join(get_package_share_directory("robile_battery_simulator"), "config", "config.yaml")

    smach_config = os.path.join(get_package_share_directory("robile_safety_smach"), "config", "config.yaml")

    return LaunchDescription(
        [
            # Battery Simulator
            Node(
                package="robile_battery_simulator",
                executable="battery_simulator",
                name="battery_simulator_node",
                output="screen",
                parameters=[battery_config],
                emulate_tty=True,
            ),
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
