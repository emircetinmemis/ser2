"""
Launch battery + Behavior Tree safety system

source install/setup.bash && cls && ros2 launch robile_safety_system safety_bt.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get config files
    battery_config = os.path.join(get_package_share_directory("robile_battery_simulator"), "config", "config.yaml")

    bt_config = os.path.join(get_package_share_directory("robile_safety_bt"), "config", "config.yaml")

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
            # Behavior Tree Safety Node
            Node(
                package="robile_safety_bt",
                executable="safety_bt_node",
                name="safety_bt_node",
                output="screen",
                parameters=[bt_config],
                emulate_tty=True,
            ),
        ]
    )
