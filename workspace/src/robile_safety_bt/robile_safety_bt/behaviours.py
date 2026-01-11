"""
Behavior Tree Behaviors for Robot Safety System
"""

import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis for battery recharging"""

    def __init__(self, name="Rotate", topic_name="/cmd_vel", ang_vel=5.0):
        super(Rotate, self).__init__(name)
        self.topic_name = topic_name
        self.ang_vel = ang_vel
        self.cmd_vel_pub = None
        self.blackboard = None

    def setup(self, **kwargs):
        """Setup publisher and blackboard client"""
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_message) from e

        # Create publisher
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.topic_name, 10)

        # Attach to blackboard
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="battery_low_warning", access=pt.common.Access.READ)

        self.node.get_logger().info(f"[{self.name}] ✅ Setup complete")
        return True

    def update(self):
        """Rotates the robot while battery is low"""
        # Read battery warning from blackboard
        battery_low = self.blackboard.get("battery_low_warning")

        if battery_low:
            # Battery is low - rotate for recharge
            twist = Twist()
            twist.angular.z = self.ang_vel
            self.cmd_vel_pub.publish(twist)
            return pt.common.Status.RUNNING  # Keep this behavior active
        else:
            # Battery is OK - this behavior not needed
            return pt.common.Status.FAILURE  # Let selector try next behavior

    def terminate(self, new_status):
        """Stop rotation when behavior terminates"""
        # Only stop if we were actually running
        if new_status == pt.common.Status.INVALID:
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        return super().terminate(new_status)


class StopMotion(pt.behaviour.Behaviour):
    """Stops the robot when collision is detected"""

    def __init__(self, name="StopMotion", topic_name="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        self.topic_name = topic_name
        self.cmd_vel_pub = None
        self.blackboard = None

    def setup(self, **kwargs):
        """Setup publisher and blackboard client"""
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_message) from e

        # Create publisher
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.topic_name, 10)

        # Attach to blackboard
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="collision_warning", access=pt.common.Access.READ)

        self.node.get_logger().info(f"[{self.name}] ✅ Setup complete")
        return True

    def update(self):
        """Stops the robot if collision warning is active"""
        collision_warning = self.blackboard.get("collision_warning")

        if collision_warning:
            # Collision detected - stop the robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return pt.common.Status.RUNNING  # Keep this behavior active
        else:
            # No collision - this behavior not needed
            return pt.common.Status.FAILURE  # Let selector try next behavior

    def terminate(self, new_status):
        """Cleanup when behavior terminates"""
        return super().terminate(new_status)


class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Subscribes to battery level and updates blackboard with low battery warning"""

    def __init__(self, battery_topic_name: str = "/battery_level", name: str = "Battery2BB", threshold_low: float = 20.0, threshold_ok: float = 85.0):
        super().__init__(
            name=name,
            topic_name=battery_topic_name,
            topic_type=Float32,
            blackboard_variables={"battery": "data"},
            initialise_variables={"battery": 100.0},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
            qos_profile=ptr.utilities.qos_profile_unlatched(),
        )

        self.threshold_low = threshold_low
        self.threshold_ok = threshold_ok
        self.last_logged_state = None  # Track state to reduce logging

        # Register blackboard keys for writing
        self.blackboard.register_key(key="battery_low_warning", access=pt.common.Access.WRITE)

        # Initialize warning flag
        self.blackboard.battery_low_warning = False

    def update(self):
        """Check battery level and update warning flag with hysteresis"""
        # First call parent to update blackboard with raw battery data
        status = super().update()

        # Check if battery data exists on blackboard yet
        try:
            battery_level = self.blackboard.get("battery")
        except KeyError:
            return pt.common.Status.SUCCESS

        # Hysteresis logic to prevent oscillation
        new_warning = self.blackboard.battery_low_warning

        if battery_level < self.threshold_low:
            # Battery low - set warning
            new_warning = True
        elif battery_level > self.threshold_ok:
            # Battery recharged - clear warning
            new_warning = False
        # else: maintain current state (hysteresis)

        # Only log on state changes
        if new_warning != self.last_logged_state:
            if new_warning:
                self.logger.info(f"[{self.name}] 🔋 Battery LOW: {battery_level:.1f}%")
            else:
                self.logger.info(f"[{self.name}] ✅ Battery OK: {battery_level:.1f}%")
            self.last_logged_state = new_warning

        self.blackboard.battery_low_warning = new_warning
        return pt.common.Status.SUCCESS


class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Subscribes to laser scan and updates blackboard with collision warning"""

    def __init__(self, topic_name: str = "/scan", name: str = "Scan2BB", collision_distance: float = 0.5, safe_distance: float = 0.75):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=LaserScan,
            blackboard_variables={"laser_scan": "ranges"},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
            qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10),
        )

        self.collision_distance = collision_distance
        self.safe_distance = safe_distance
        self.last_logged_state = None  # Track state to reduce logging

        # Register blackboard key for collision warning
        self.blackboard.register_key(key="collision_warning", access=pt.common.Access.WRITE)

        # Initialize warning flag
        self.blackboard.collision_warning = False

    def update(self):
        """Check laser scan for obstacles and update collision warning with hysteresis"""
        # Call parent to update blackboard with raw scan data
        status = super().update()

        # Check if laser_scan data exists on blackboard yet
        try:
            ranges = self.blackboard.get("laser_scan")
        except KeyError:
            return pt.common.Status.SUCCESS

        if ranges:
            # Find minimum valid distance
            valid_ranges = [r for r in ranges if 0.1 < r < 100.0]

            if valid_ranges:
                min_distance = min(valid_ranges)
                new_warning = self.blackboard.collision_warning

                # Hysteresis logic
                if min_distance < self.collision_distance:
                    # Obstacle too close - set warning
                    new_warning = True
                elif min_distance > self.safe_distance:
                    # Safe distance - clear warning
                    new_warning = False
                # else: maintain current state (hysteresis)

                # Only log on state changes
                if new_warning != self.last_logged_state:
                    if new_warning:
                        self.logger.info(f"[{self.name}] ⚠️  COLLISION: {min_distance:.2f}m")
                    else:
                        self.logger.info(f"[{self.name}] ✅ Safe: {min_distance:.2f}m")
                    self.last_logged_state = new_warning

                self.blackboard.collision_warning = new_warning

        return pt.common.Status.SUCCESS


class EmergencyStop2bb(ptr.subscribers.ToBlackboard):
    """Subscribes to emergency stop topic and updates blackboard"""

    def __init__(
        self, topic_name: str = "/emergency_stop", name: str = "Emergency2BB"
    ):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=Bool,
            blackboard_variables={"emergency_stop_raw": "data"},
            initialise_variables={"emergency_stop_raw": False},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
            qos_profile=ptr.utilities.qos_profile_unlatched(),
        )

        # Register blackboard key for emergency stop
        self.blackboard.register_key(
            key="emergency_stop", access=pt.common.Access.WRITE
        )

        # Initialize emergency stop flag
        self.blackboard.emergency_stop = False

    def update(self):
        """Update emergency stop status on blackboard"""
        status = super().update()

        try:
            emergency = self.blackboard.get("emergency_stop_raw")
            if emergency != self.blackboard.emergency_stop:
                if emergency:
                    self.logger.error(f"[{self.name}] 🚨 EMERGENCY STOP ACTIVATED!")
                else:
                    self.logger.info(f"[{self.name}] ✅ Emergency stop cleared")
            self.blackboard.emergency_stop = emergency
        except KeyError:
            pass

        return pt.common.Status.SUCCESS


class EmergencyStopBehavior(pt.behaviour.Behaviour):
    """Emergency stop behavior - highest priority, stops all robot motion"""

    def __init__(self, name="EmergencyStop", topic_name="/cmd_vel"):
        super(EmergencyStopBehavior, self).__init__(name)
        self.topic_name = topic_name
        self.cmd_vel_pub = None
        self.blackboard = None

    def setup(self, **kwargs):
        """Setup publisher and blackboard client"""
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = (
                f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            )
            raise KeyError(error_message) from e

        # Create publisher
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.topic_name, 10)

        # Attach to blackboard
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="emergency_stop", access=pt.common.Access.READ
        )

        self.node.get_logger().info(f"[{self.name}] ✅ Setup complete")
        return True

    def update(self):
        """Stop the robot if emergency stop is active"""
        emergency = self.blackboard.get("emergency_stop")

        if emergency:
            # Emergency active - stop everything
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.node.get_logger().error(f"[{self.name}] 🚨 EMERGENCY STOP!")
            return pt.common.Status.RUNNING
        else:
            # No emergency - this behavior not needed
            return pt.common.Status.FAILURE

    def terminate(self, new_status):
        """Cleanup when behavior terminates"""
        return super().terminate(new_status)
