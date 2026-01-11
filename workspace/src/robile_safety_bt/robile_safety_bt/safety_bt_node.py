"""
Robot Safety using Behavior Trees (py_trees)
Entry point for the behavior tree safety system
"""

import sys
from dataclasses import dataclass

import py_trees.console as console
import py_trees_ros as ptr
import rclpy
from rclpy.node import Node

from robile_safety_bt.tree import create_root


@dataclass
class SafetyConfig:
    """Centralized configuration - Single source of truth!"""

    # Battery thresholds
    low_battery_threshold: float = 20.0
    battery_ok_threshold: float = 85.0

    # Collision thresholds
    collision_distance: float = 0.5
    safe_distance: float = 0.75

    # Robot motion
    rotation_speed: float = 5.0
    monitor_rate: float = 10.0

    @classmethod
    def from_node(cls, node: Node):
        """Load configuration from ROS2 parameters"""
        node.declare_parameter("low_battery_threshold", cls.low_battery_threshold)
        node.declare_parameter("battery_ok_threshold", cls.battery_ok_threshold)
        node.declare_parameter("collision_distance", cls.collision_distance)
        node.declare_parameter("safe_distance", cls.safe_distance)
        node.declare_parameter("rotation_speed", cls.rotation_speed)
        node.declare_parameter("monitor_rate", cls.monitor_rate)

        return cls(
            low_battery_threshold=node.get_parameter("low_battery_threshold").value,
            battery_ok_threshold=node.get_parameter("battery_ok_threshold").value,
            collision_distance=node.get_parameter("collision_distance").value,
            safe_distance=node.get_parameter("safe_distance").value,
            rotation_speed=node.get_parameter("rotation_speed").value,
            monitor_rate=node.get_parameter("monitor_rate").value,
        )

    def log_config(self, logger):
        """Pretty print configuration"""
        logger.info("=" * 60)
        logger.info("🌳 Behavior Tree Safety Configuration:")
        logger.info(f"  Battery Low:      {self.low_battery_threshold:.1f}%")
        logger.info(f"  Battery OK:       {self.battery_ok_threshold:.1f}%")
        logger.info(f"  Collision Dist:   {self.collision_distance:.2f}m")
        logger.info(f"  Safe Dist:        {self.safe_distance:.2f}m")
        logger.info(f"  Rotation Speed:   {self.rotation_speed:.1f} rad/s")
        logger.info("=" * 60)


def main():
    """Initializes and executes the behavior tree"""

    rclpy.init(args=None)

    # Create a temporary node to load config
    temp_node = Node("safety_bt_config_loader")
    config = SafetyConfig.from_node(temp_node)
    config.log_config(temp_node.get_logger())
    temp_node.destroy_node()

    # Create behavior tree root
    root = create_root(config)

    # Create behavior tree with py_trees_ros
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
        tree.node.get_logger().info("✅ Behavior tree setup complete")
        tree.node.get_logger().info("🚀 Starting execution...")
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + f"❌ Failed to setup tree: {str(e)}" + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("⏹️  Tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    # Tick frequency (100ms = 10 Hz)
    tick_period_ms = int(1000 / config.monitor_rate)
    tree.tick_tock(period_ms=tick_period_ms)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.node.get_logger().info("⏹️  Shutting down behavior tree")
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
