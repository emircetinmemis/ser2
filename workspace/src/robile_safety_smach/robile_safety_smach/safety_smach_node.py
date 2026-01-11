"""
Robot Safety State Machine using SMACH
Implements battery monitoring, collision avoidance, and emergency stop
"""

import threading
import time
from dataclasses import dataclass

import rclpy
import smach
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


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
        # Declare all parameters with defaults from this class
        node.declare_parameter("low_battery_threshold", cls.low_battery_threshold)
        node.declare_parameter("battery_ok_threshold", cls.battery_ok_threshold)
        node.declare_parameter("collision_distance", cls.collision_distance)
        node.declare_parameter("safe_distance", cls.safe_distance)
        node.declare_parameter("rotation_speed", cls.rotation_speed)
        node.declare_parameter("monitor_rate", cls.monitor_rate)

        # Read and return config
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
        logger.info("🔧 SMACH Safety Configuration:")
        logger.info(f"  Battery Low:      {self.low_battery_threshold:.1f}%")
        logger.info(f"  Battery OK:       {self.battery_ok_threshold:.1f}%")
        logger.info(f"  Collision Dist:   {self.collision_distance:.2f}m")
        logger.info(f"  Safe Dist:        {self.safe_distance:.2f}m")
        logger.info(f"  Rotation Speed:   {self.rotation_speed:.1f} rad/s")
        logger.info("=" * 60)


class MonitorBatteryAndCollision(smach.State):
    """State to monitor battery level, collisions, and emergency stop"""

    def __init__(self, node: Node, config: SafetyConfig):
        smach.State.__init__(
            self, outcomes=["battery_low", "collision", "emergency", "all_good"]
        )
        self.node = node
        self.config = config

        # State variables
        self.battery_level = 100.0
        self.min_scan_distance = float("inf")
        self.emergency_stop = False

        # Subscribers
        self.battery_sub = node.create_subscription(
            Float32, "/battery_level", self.battery_callback, 10
        )
        self.scan_sub = node.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.emergency_sub = node.create_subscription(
            Bool, "/emergency_stop", self.emergency_callback, 10
        )

        node.get_logger().info("✅ MonitorBatteryAndCollision state initialized")

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if 0.1 < r < msg.range_max]
        self.min_scan_distance = min(valid_ranges) if valid_ranges else float("inf")

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data

    def execute(self, userdata):
        self.node.get_logger().info(
            f"👁️  Monitoring - Battery: {self.battery_level:.1f}%, "
            f"Min dist: {self.min_scan_distance:.2f}m, "
            f"Emergency: {self.emergency_stop}"
        )

        # Check emergency first (highest priority)
        if self.emergency_stop:
            self.node.get_logger().error("🚨 EMERGENCY STOP TRIGGERED!")
            return "emergency"

        # Check collision (high priority)
        if self.min_scan_distance < self.config.collision_distance:
            self.node.get_logger().warn(
                f"⚠️  COLLISION! Obstacle at {self.min_scan_distance:.2f}m"
            )
            return "collision"

        # Check battery
        if self.battery_level < self.config.low_battery_threshold:
            self.node.get_logger().warn(
                f"🔋 LOW BATTERY! Level: {self.battery_level:.1f}%"
            )
            return "battery_low"

        time.sleep(0.1)
        return "all_good"


class RotateBase(smach.State):
    """State to rotate robot for recharging"""

    def __init__(self, node: Node, config: SafetyConfig):
        smach.State.__init__(self, outcomes=["battery_ok"])
        self.node = node
        self.config = config

        # State variables
        self.battery_level = 0.0

        # Subscriber
        self.battery_sub = node.create_subscription(Float32, "/battery_level", self.battery_callback, 10)

        # Publisher
        self.cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)

        node.get_logger().info("✅ RotateBase state initialized")

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def execute(self, userdata):
        self.node.get_logger().info(f"🔄 ROTATING to recharge - Battery: {self.battery_level:.1f}%")

        # Keep rotating until recharged
        rate = self.node.create_rate(10)
        while self.battery_level < self.config.battery_ok_threshold:
            # Publish rotation command
            twist = Twist()
            twist.angular.z = self.config.rotation_speed
            self.cmd_vel_pub.publish(twist)

            try:
                rate.sleep()
            except Exception:
                pass

        # Stop rotating
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        self.node.get_logger().info(f"✅ Battery recharged to {self.battery_level:.1f}%")

        return "battery_ok"


class StopMotion(smach.State):
    """State to stop robot when collision detected"""

    def __init__(self, node: Node, config: SafetyConfig):
        smach.State.__init__(self, outcomes=["safe_distance"])
        self.node = node
        self.config = config

        # State variables
        self.min_scan_distance = 0.0

        # Subscriber
        self.scan_sub = node.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        # Publisher
        self.cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)

        node.get_logger().info("✅ StopMotion state initialized")

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if 0.1 < r < msg.range_max]
        self.min_scan_distance = min(valid_ranges) if valid_ranges else float("inf")

    def execute(self, userdata):
        self.node.get_logger().info(f"🛑 STOPPED - Obstacle at {self.min_scan_distance:.2f}m")

        # Keep stopping until obstacle cleared
        rate = self.node.create_rate(10)
        while self.min_scan_distance <= self.config.safe_distance:
            # Publish stop command
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            try:
                rate.sleep()
            except Exception:
                pass

        self.node.get_logger().info(f"✅ Obstacle cleared! Distance: {self.min_scan_distance:.2f}m")

        return "safe_distance"


class EmergencyStop(smach.State):
    """State for emergency stop - highest priority, stops all robot motion"""

    def __init__(self, node: Node, config: SafetyConfig):
        smach.State.__init__(self, outcomes=["cleared"])
        self.node = node
        self.config = config

        # State variables
        self.emergency_stop = True

        # Subscriber
        self.emergency_sub = node.create_subscription(
            Bool, "/emergency_stop", self.emergency_callback, 10
        )

        # Publisher
        self.cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)

        node.get_logger().info("✅ EmergencyStop state initialized")

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data

    def execute(self, userdata):
        self.node.get_logger().error("🚨 EMERGENCY STOP ACTIVE - Robot halted!")

        # Keep stopping until emergency is cleared
        rate = self.node.create_rate(10)
        while self.emergency_stop:
            # Publish stop command
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            try:
                rate.sleep()
            except Exception:
                pass

        self.node.get_logger().info("✅ Emergency stop cleared!")

        return "cleared"


def main(args=None):
    """Main function to initialize and execute the state machine"""

    rclpy.init(args=args)

    # Create ROS2 node
    node = Node("safety_smach_node")

    # Load configuration
    config = SafetyConfig.from_node(node)
    config.log_config(node.get_logger())

    node.get_logger().info("🏗️  Building state machine...")

    # Create state machine
    sm = smach.StateMachine(outcomes=["shutdown"])

    # Add states
    with sm:
        smach.StateMachine.add(
            "MONITOR_BATTERY_AND_COLLISION",
            MonitorBatteryAndCollision(node, config),
            transitions={
                "emergency": "EMERGENCY_STOP",
                "battery_low": "ROTATE_BASE",
                "collision": "STOP_MOTION",
                "all_good": "MONITOR_BATTERY_AND_COLLISION",
            },
        )

        smach.StateMachine.add(
            "EMERGENCY_STOP",
            EmergencyStop(node, config),
            transitions={"cleared": "MONITOR_BATTERY_AND_COLLISION"},
        )

        smach.StateMachine.add(
            "ROTATE_BASE",
            RotateBase(node, config),
            transitions={"battery_ok": "MONITOR_BATTERY_AND_COLLISION"},
        )

        smach.StateMachine.add(
            "STOP_MOTION",
            StopMotion(node, config),
            transitions={"safe_distance": "MONITOR_BATTERY_AND_COLLISION"},
        )

    node.get_logger().info("✅ State machine configured")
    node.get_logger().info("🚀 Starting execution...")

    # Execute state machine in separate thread
    sm_thread = threading.Thread(target=sm.execute)
    sm_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️  Shutting down...")
    finally:
        # Stop the robot
        cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        stop_twist = Twist()
        cmd_vel_pub.publish(stop_twist)

        node.destroy_node()
        rclpy.shutdown()
        sm_thread.join()


if __name__ == "__main__":
    main()
