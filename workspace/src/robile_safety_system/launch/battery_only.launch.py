"""
Launch battery simulator only

source install/setup.bash && cls && ros2 launch robile_safety_system battery_only.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get config file
    battery_config = os.path.join(get_package_share_directory("robile_battery_simulator"), "config", "config.yaml")

    return LaunchDescription(
        [
            Node(
                package="robile_battery_simulator",
                executable="battery_simulator",
                name="battery_simulator_node",
                output="screen",
                parameters=[battery_config],
                emulate_tty=True,
            ),
        ]
    )
