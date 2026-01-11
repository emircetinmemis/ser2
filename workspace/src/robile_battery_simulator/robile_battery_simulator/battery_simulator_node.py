"""
Battery Simulator Node - Pure Physics Simulation
No decision logic - just simulates battery drain/charge based on robot motion
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class BatterySimulator(Node):
    """
    Simulates battery behavior:
    - Drains when robot moves
    - Charges when robot is stationary + rotating (on charging pad)
    - Can be manually set for testing
    """

    def __init__(self):
        super().__init__("battery_simulator")

        # Declare parameters
        self.declare_parameter("initial_battery", 100.0)
        self.declare_parameter("max_battery_level", 94.0)
        self.declare_parameter("idle_drain_rate", 0.5)
        self.declare_parameter("moving_drain_rate", 5.0)
        self.declare_parameter("rotating_drain_rate", 1.0)
        self.declare_parameter("charge_rate", 3.0)
        self.declare_parameter("update_rate", 10.0)

        # Get parameters
        self.battery_level = self.get_parameter("initial_battery").value
        self.max_battery_level = self.get_parameter("max_battery_level").value
        self.idle_drain_rate = self.get_parameter("idle_drain_rate").value
        self.moving_drain_rate = self.get_parameter("moving_drain_rate").value
        self.rotating_drain_rate = self.get_parameter("rotating_drain_rate").value
        self.charge_rate = self.get_parameter("charge_rate").value
        update_rate = self.get_parameter("update_rate").value

        # Robot state (from cmd_vel)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Publishers
        self.battery_pub = self.create_publisher(Float32, "/battery_level", 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.battery_set_sub = self.create_subscription(Float32, "/battery_set", self.battery_set_callback, 10)

        # Timer for battery updates
        self.dt = 1.0 / update_rate
        self.timer = self.create_timer(self.dt, self.update_battery)

        self.get_logger().info("=" * 60)
        self.get_logger().info("🔋 Battery Simulator Node Started")
        self.get_logger().info(f"Initial battery: {self.battery_level:.1f}%")
        self.get_logger().info(f"Max battery level: {self.max_battery_level:.1f}%")
        self.get_logger().info(f"Idle drain: {self.idle_drain_rate}%/s")
        self.get_logger().info(f"Moving drain: {self.moving_drain_rate}%/s")
        self.get_logger().info(f"Charge rate: {self.charge_rate}%/s")
        self.get_logger().info("Manual control: publish to /battery_set")
        self.get_logger().info("=" * 60)

    def cmd_vel_callback(self, msg):
        """Track robot velocities"""
        self.linear_velocity = abs(msg.linear.x)
        self.angular_velocity = abs(msg.angular.z)

    def battery_set_callback(self, msg):
        """Manual battery override for testing"""
        old_level = self.battery_level
        self.battery_level = max(0.0, min(100.0, msg.data))
        self.get_logger().info(f"🔧 MANUAL: Battery {old_level:.1f}% → {self.battery_level:.1f}%")

    def update_battery(self):
        """Update battery based ONLY on robot motion state"""

        is_moving = self.linear_velocity > 0.01
        is_rotating = self.angular_velocity > 0.01
        is_stationary = not is_moving

        # Determine rate based on physics only
        if is_stationary and is_rotating:
            # Stationary + rotating = on charging pad
            if self.battery_level >= self.max_battery_level:
                rate = 0.0
                state = "FULL (charge limit)"
            else:
                rate = self.charge_rate
                state = "CHARGING"
        elif is_moving:
            rate = -self.moving_drain_rate
            state = "MOVING"
        else:
            rate = -self.idle_drain_rate
            state = "IDLE"

        # Update battery
        self.battery_level += rate * self.dt
        self.battery_level = max(0.0, min(100.0, self.battery_level))

        # Publish
        msg = Float32()
        msg.data = self.battery_level
        self.battery_pub.publish(msg)

        # Log every 2 seconds
        current_time = self.get_clock().now().nanoseconds / 1e9
        if int(current_time) % 2 == 0 and int(current_time * 10) % 10 == 0:
            sign = "+" if rate > 0 else ""
            self.get_logger().info(f"🔋 {self.battery_level:5.1f}% | {state:15s} | {sign}{rate:.1f}%/s")


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
