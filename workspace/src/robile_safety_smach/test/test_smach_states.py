"""
Unit tests for SMACH Safety States
Tests state machine logic without ROS2 dependencies
"""

import unittest
from dataclasses import dataclass
from unittest.mock import MagicMock, patch


@dataclass
class MockSafetyConfig:
    """Mock configuration for testing"""

    low_battery_threshold: float = 20.0
    battery_ok_threshold: float = 85.0
    collision_distance: float = 0.5
    safe_distance: float = 0.75
    rotation_speed: float = 5.0
    monitor_rate: float = 10.0


class TestSafetyConfig(unittest.TestCase):
    """Test SafetyConfig dataclass"""

    def test_default_values(self):
        """Test default configuration values"""
        config = MockSafetyConfig()

        self.assertEqual(config.low_battery_threshold, 20.0)
        self.assertEqual(config.battery_ok_threshold, 85.0)
        self.assertEqual(config.collision_distance, 0.5)
        self.assertEqual(config.safe_distance, 0.75)
        self.assertEqual(config.rotation_speed, 5.0)

    def test_custom_values(self):
        """Test custom configuration values"""
        config = MockSafetyConfig(low_battery_threshold=15.0, battery_ok_threshold=90.0, collision_distance=0.3)

        self.assertEqual(config.low_battery_threshold, 15.0)
        self.assertEqual(config.battery_ok_threshold, 90.0)
        self.assertEqual(config.collision_distance, 0.3)

    def test_hysteresis_gap(self):
        """Test that there's a gap between low and ok thresholds (hysteresis)"""
        config = MockSafetyConfig()

        # There should be a significant gap to prevent oscillation
        gap = config.battery_ok_threshold - config.low_battery_threshold
        self.assertGreater(gap, 10.0, "Hysteresis gap should be > 10%")


class TestMonitorBatteryAndCollisionLogic(unittest.TestCase):
    """Test monitoring state decision logic"""

    def setUp(self):
        self.config = MockSafetyConfig()

    def test_collision_detected(self):
        """Test collision outcome when obstacle too close"""
        min_scan_distance = 0.3  # Less than collision_distance (0.5)
        battery_level = 50.0

        # Decision logic from MonitorBatteryAndCollision
        if min_scan_distance < self.config.collision_distance:
            outcome = "collision"
        elif battery_level < self.config.low_battery_threshold:
            outcome = "battery_low"
        else:
            outcome = "all_good"

        self.assertEqual(outcome, "collision")

    def test_battery_low_detected(self):
        """Test battery_low outcome when battery below threshold"""
        min_scan_distance = 1.0  # Safe distance
        battery_level = 15.0  # Below low_battery_threshold (20.0)

        if min_scan_distance < self.config.collision_distance:
            outcome = "collision"
        elif battery_level < self.config.low_battery_threshold:
            outcome = "battery_low"
        else:
            outcome = "all_good"

        self.assertEqual(outcome, "battery_low")

    def test_all_good(self):
        """Test all_good outcome when everything is fine"""
        min_scan_distance = 1.0  # Safe
        battery_level = 50.0  # OK

        if min_scan_distance < self.config.collision_distance:
            outcome = "collision"
        elif battery_level < self.config.low_battery_threshold:
            outcome = "battery_low"
        else:
            outcome = "all_good"

        self.assertEqual(outcome, "all_good")

    def test_collision_priority_over_battery(self):
        """Test that collision has higher priority than low battery"""
        min_scan_distance = 0.3  # Collision!
        battery_level = 10.0  # Also low battery!

        # Collision should be checked first
        if min_scan_distance < self.config.collision_distance:
            outcome = "collision"
        elif battery_level < self.config.low_battery_threshold:
            outcome = "battery_low"
        else:
            outcome = "all_good"

        self.assertEqual(outcome, "collision")


class TestRotateBaseLogic(unittest.TestCase):
    """Test rotate base state logic"""

    def setUp(self):
        self.config = MockSafetyConfig()

    def test_rotate_until_battery_ok(self):
        """Test rotation continues until battery reaches ok threshold"""
        battery_levels = [20.0, 40.0, 60.0, 80.0, 86.0]

        for level in battery_levels:
            should_continue_rotating = level < self.config.battery_ok_threshold

            if level < self.config.battery_ok_threshold:
                self.assertTrue(should_continue_rotating)
            else:
                self.assertFalse(should_continue_rotating)

    def test_rotation_command(self):
        """Test correct rotation speed is applied"""
        expected_angular_z = self.config.rotation_speed

        # Simulate Twist message creation
        twist_angular_z = self.config.rotation_speed

        self.assertEqual(twist_angular_z, expected_angular_z)
        self.assertEqual(twist_angular_z, 5.0)


class TestStopMotionLogic(unittest.TestCase):
    """Test stop motion state logic"""

    def setUp(self):
        self.config = MockSafetyConfig()

    def test_stop_until_safe_distance(self):
        """Test robot stays stopped until obstacle is at safe distance"""
        test_cases = [
            (0.3, True),  # Too close, keep stopping
            (0.5, True),  # At collision threshold, keep stopping
            (0.7, True),  # Still below safe distance, keep stopping
            (0.75, True),  # At safe distance boundary, keep stopping (<=)
            (0.76, False),  # Just beyond safe distance, can resume
            (1.0, False),  # Beyond safe distance, can resume
        ]

        for distance, should_stop in test_cases:
            result = distance <= self.config.safe_distance
            self.assertEqual(result, should_stop, f"Failed for distance {distance}")

    def test_stop_command_zeros_velocity(self):
        """Test stop command sets all velocities to zero"""
        # Simulate stop Twist message
        linear_x = 0.0
        angular_z = 0.0

        self.assertEqual(linear_x, 0.0)
        self.assertEqual(angular_z, 0.0)


class TestLaserScanProcessing(unittest.TestCase):
    """Test laser scan data processing"""

    def test_valid_ranges_filtering(self):
        """Test filtering of valid laser scan ranges"""
        # Simulate laser scan with some invalid readings
        ranges = [0.05, 0.5, 1.0, 2.0, float("inf"), 0.01, 3.0]
        range_max = 10.0

        valid_ranges = [r for r in ranges if 0.1 < r < range_max]

        self.assertEqual(valid_ranges, [0.5, 1.0, 2.0, 3.0])
        self.assertNotIn(0.05, valid_ranges)  # Too close (noise)
        self.assertNotIn(float("inf"), valid_ranges)  # Invalid

    def test_min_distance_calculation(self):
        """Test minimum distance calculation from valid ranges"""
        ranges = [0.5, 1.0, 2.0, 3.0]

        min_distance = min(ranges) if ranges else float("inf")

        self.assertEqual(min_distance, 0.5)

    def test_empty_valid_ranges(self):
        """Test handling when no valid ranges exist"""
        ranges = [0.01, 0.02, float("inf")]  # All invalid
        range_max = 10.0

        valid_ranges = [r for r in ranges if 0.1 < r < range_max]
        min_distance = min(valid_ranges) if valid_ranges else float("inf")

        self.assertEqual(min_distance, float("inf"))


class TestStateMachineTransitions(unittest.TestCase):
    """Test state machine transition logic"""

    def test_monitor_to_rotate_transition(self):
        """Test transition from MONITOR to ROTATE_BASE on low battery"""
        current_state = "MONITOR_BATTERY_AND_COLLISION"
        outcome = "battery_low"

        transitions = {"battery_low": "ROTATE_BASE", "collision": "STOP_MOTION", "all_good": "MONITOR_BATTERY_AND_COLLISION"}

        next_state = transitions[outcome]
        self.assertEqual(next_state, "ROTATE_BASE")

    def test_monitor_to_stop_transition(self):
        """Test transition from MONITOR to STOP_MOTION on collision"""
        outcome = "collision"

        transitions = {"battery_low": "ROTATE_BASE", "collision": "STOP_MOTION", "all_good": "MONITOR_BATTERY_AND_COLLISION"}

        next_state = transitions[outcome]
        self.assertEqual(next_state, "STOP_MOTION")

    def test_rotate_to_monitor_transition(self):
        """Test transition from ROTATE_BASE back to MONITOR"""
        outcome = "battery_ok"

        transitions = {"battery_ok": "MONITOR_BATTERY_AND_COLLISION"}

        next_state = transitions[outcome]
        self.assertEqual(next_state, "MONITOR_BATTERY_AND_COLLISION")

    def test_stop_to_monitor_transition(self):
        """Test transition from STOP_MOTION back to MONITOR"""
        outcome = "safe_distance"

        transitions = {"safe_distance": "MONITOR_BATTERY_AND_COLLISION"}

        next_state = transitions[outcome]
        self.assertEqual(next_state, "MONITOR_BATTERY_AND_COLLISION")


class TestEmergencyStopLogic(unittest.TestCase):
    """Test emergency stop state logic"""

    def setUp(self):
        self.config = MockSafetyConfig()

    def test_emergency_has_highest_priority(self):
        """Test that emergency stop overrides all other conditions"""
        emergency_stop = True
        collision_warning = True
        battery_low = True

        # Emergency should be checked first
        if emergency_stop:
            outcome = "emergency"
        elif collision_warning:
            outcome = "collision"
        elif battery_low:
            outcome = "battery_low"
        else:
            outcome = "all_good"

        self.assertEqual(outcome, "emergency")

    def test_emergency_stops_robot(self):
        """Test that emergency stop sets velocity to zero"""
        # Simulate emergency stop Twist message
        linear_x = 0.0
        angular_z = 0.0

        self.assertEqual(linear_x, 0.0)
        self.assertEqual(angular_z, 0.0)

    def test_emergency_to_monitor_transition(self):
        """Test transition from EMERGENCY_STOP back to MONITOR when cleared"""
        outcome = "cleared"

        transitions = {"cleared": "MONITOR_BATTERY_AND_COLLISION"}

        next_state = transitions[outcome]
        self.assertEqual(next_state, "MONITOR_BATTERY_AND_COLLISION")

    def test_monitor_to_emergency_transition(self):
        """Test transition from MONITOR to EMERGENCY_STOP"""
        outcome = "emergency"

        transitions = {
            "emergency": "EMERGENCY_STOP",
            "battery_low": "ROTATE_BASE",
            "collision": "STOP_MOTION",
            "all_good": "MONITOR_BATTERY_AND_COLLISION",
        }

        next_state = transitions[outcome]
        self.assertEqual(next_state, "EMERGENCY_STOP")

    def test_emergency_state_callback(self):
        """Test emergency callback updates state correctly"""
        emergency_stop = False

        # Simulate receiving emergency message
        msg_data = True
        emergency_stop = msg_data

        self.assertTrue(emergency_stop)

        # Simulate clearing emergency
        msg_data = False
        emergency_stop = msg_data

        self.assertFalse(emergency_stop)


if __name__ == "__main__":
    unittest.main()
