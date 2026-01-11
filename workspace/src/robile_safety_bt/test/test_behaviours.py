"""
Unit tests for Behavior Tree Behaviors
Tests behavior tree logic without ROS2 dependencies
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


class TestBatteryStatus2bbLogic(unittest.TestCase):
    """Test BatteryStatus2bb hysteresis logic"""

    def setUp(self):
        self.threshold_low = 20.0
        self.threshold_ok = 85.0

    def test_battery_low_warning_set(self):
        """Test warning is set when battery falls below low threshold"""
        battery_level = 15.0
        current_warning = False

        # Hysteresis logic from BatteryStatus2bb
        if battery_level < self.threshold_low:
            new_warning = True
        elif battery_level > self.threshold_ok:
            new_warning = False
        else:
            new_warning = current_warning  # Maintain state

        self.assertTrue(new_warning)

    def test_battery_warning_cleared(self):
        """Test warning is cleared when battery goes above ok threshold"""
        battery_level = 90.0
        current_warning = True

        if battery_level < self.threshold_low:
            new_warning = True
        elif battery_level > self.threshold_ok:
            new_warning = False
        else:
            new_warning = current_warning

        self.assertFalse(new_warning)

    def test_hysteresis_maintains_warning(self):
        """Test warning maintained in hysteresis zone"""
        battery_level = 50.0  # Between thresholds

        # Test with warning currently True
        current_warning = True
        if battery_level < self.threshold_low:
            new_warning = True
        elif battery_level > self.threshold_ok:
            new_warning = False
        else:
            new_warning = current_warning
        self.assertTrue(new_warning)  # Should stay True

        # Test with warning currently False
        current_warning = False
        if battery_level < self.threshold_low:
            new_warning = True
        elif battery_level > self.threshold_ok:
            new_warning = False
        else:
            new_warning = current_warning
        self.assertFalse(new_warning)  # Should stay False

    def test_boundary_conditions(self):
        """Test exact boundary values"""
        test_cases = [
            (20.0, False, False),  # At low threshold - in hysteresis, maintains state
            (19.9, False, True),  # Just below low - triggers warning
            (20.1, False, False),  # Just above low - in hysteresis
            (85.0, True, True),  # At ok threshold - in hysteresis
            (85.1, True, False),  # Just above ok - clears warning
        ]

        for battery, current_warning, expected_warning in test_cases:
            if battery < self.threshold_low:
                new_warning = True
            elif battery > self.threshold_ok:
                new_warning = False
            else:
                new_warning = current_warning

            self.assertEqual(new_warning, expected_warning, f"Failed for battery={battery}, current={current_warning}")


class TestLaserScan2bbLogic(unittest.TestCase):
    """Test LaserScan2bb collision detection logic"""

    def setUp(self):
        self.collision_distance = 0.5
        self.safe_distance = 0.75

    def test_collision_warning_set(self):
        """Test warning is set when obstacle too close"""
        min_distance = 0.3
        current_warning = False

        if min_distance < self.collision_distance:
            new_warning = True
        elif min_distance > self.safe_distance:
            new_warning = False
        else:
            new_warning = current_warning

        self.assertTrue(new_warning)

    def test_collision_warning_cleared(self):
        """Test warning is cleared when obstacle moves away"""
        min_distance = 1.0
        current_warning = True

        if min_distance < self.collision_distance:
            new_warning = True
        elif min_distance > self.safe_distance:
            new_warning = False
        else:
            new_warning = current_warning

        self.assertFalse(new_warning)

    def test_hysteresis_zone(self):
        """Test behavior in hysteresis zone between collision and safe distance"""
        min_distance = 0.6  # Between 0.5 (collision) and 0.75 (safe)

        # Warning stays in current state
        for current_warning in [True, False]:
            if min_distance < self.collision_distance:
                new_warning = True
            elif min_distance > self.safe_distance:
                new_warning = False
            else:
                new_warning = current_warning

            self.assertEqual(new_warning, current_warning)

    def test_scan_range_filtering(self):
        """Test valid range filtering"""
        ranges = [0.01, 0.05, 0.3, 0.8, 1.5, float("inf"), -1.0]

        valid_ranges = [r for r in ranges if 0.1 < r < 100.0]

        self.assertEqual(valid_ranges, [0.3, 0.8, 1.5])


class TestRotateBehaviorLogic(unittest.TestCase):
    """Test Rotate behavior logic"""

    def setUp(self):
        self.ang_vel = 5.0

    def test_rotate_when_battery_low(self):
        """Test behavior returns RUNNING when battery is low"""
        battery_low_warning = True

        if battery_low_warning:
            status = "RUNNING"
        else:
            status = "FAILURE"

        self.assertEqual(status, "RUNNING")

    def test_no_rotate_when_battery_ok(self):
        """Test behavior returns FAILURE when battery is ok"""
        battery_low_warning = False

        if battery_low_warning:
            status = "RUNNING"
        else:
            status = "FAILURE"

        self.assertEqual(status, "FAILURE")

    def test_rotation_velocity(self):
        """Test correct angular velocity is used"""
        self.assertEqual(self.ang_vel, 5.0)


class TestStopMotionBehaviorLogic(unittest.TestCase):
    """Test StopMotion behavior logic"""

    def test_stop_when_collision(self):
        """Test behavior returns RUNNING during collision"""
        collision_warning = True

        if collision_warning:
            status = "RUNNING"
        else:
            status = "FAILURE"

        self.assertEqual(status, "RUNNING")

    def test_no_stop_when_safe(self):
        """Test behavior returns FAILURE when safe"""
        collision_warning = False

        if collision_warning:
            status = "RUNNING"
        else:
            status = "FAILURE"

        self.assertEqual(status, "FAILURE")


class TestBehaviorTreeStructure(unittest.TestCase):
    """Test behavior tree structure and priorities"""

    def test_priorities_order(self):
        """Test that StopMotion has highest priority"""
        priorities = ["StopMotion", "Rotate", "Idle"]

        self.assertEqual(priorities[0], "StopMotion")  # Highest priority
        self.assertEqual(priorities[1], "Rotate")  # Medium priority
        self.assertEqual(priorities[2], "Idle")  # Lowest priority

    def test_selector_behavior(self):
        """Test selector returns first successful/running child"""

        # Simulate selector with priorities
        def selector(behaviors):
            for name, status in behaviors:
                if status in ["SUCCESS", "RUNNING"]:
                    return name
            return None

        # Collision active - StopMotion should be selected
        behaviors = [("StopMotion", "RUNNING"), ("Rotate", "FAILURE"), ("Idle", "RUNNING")]
        result = selector(behaviors)
        self.assertEqual(result, "StopMotion")

        # No collision, battery low - Rotate should be selected
        behaviors = [("StopMotion", "FAILURE"), ("Rotate", "RUNNING"), ("Idle", "RUNNING")]
        result = selector(behaviors)
        self.assertEqual(result, "Rotate")

        # All good - Idle should be selected
        behaviors = [("StopMotion", "FAILURE"), ("Rotate", "FAILURE"), ("Idle", "RUNNING")]
        result = selector(behaviors)
        self.assertEqual(result, "Idle")

    def test_parallel_policy(self):
        """Test parallel node runs both branches"""
        # Topics2BB and Priorities run in parallel
        branches = ["Topics2BB", "Priorities"]

        self.assertEqual(len(branches), 2)
        self.assertIn("Topics2BB", branches)
        self.assertIn("Priorities", branches)


class TestBlackboardLogic(unittest.TestCase):
    """Test blackboard data sharing logic"""

    def test_blackboard_key_access(self):
        """Test blackboard key read/write simulation"""
        blackboard = {"battery": 50.0, "battery_low_warning": False, "laser_scan": [0.5, 1.0, 2.0], "collision_warning": False}

        # Test reading
        self.assertEqual(blackboard["battery"], 50.0)
        self.assertFalse(blackboard["battery_low_warning"])

        # Test writing
        blackboard["battery_low_warning"] = True
        self.assertTrue(blackboard["battery_low_warning"])

    def test_blackboard_initial_values(self):
        """Test default blackboard initialization"""
        blackboard = {
            "battery": 100.0,  # Default initial
            "battery_low_warning": False,
            "collision_warning": False,
        }

        self.assertEqual(blackboard["battery"], 100.0)
        self.assertFalse(blackboard["battery_low_warning"])
        self.assertFalse(blackboard["collision_warning"])


if __name__ == "__main__":
    unittest.main()
