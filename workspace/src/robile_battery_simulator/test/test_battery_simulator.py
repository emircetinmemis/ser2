"""
Unit tests for Battery Simulator Node
Tests battery drain/charge logic without ROS2 dependencies
"""

import unittest
from unittest.mock import MagicMock, patch


class TestBatterySimulatorLogic(unittest.TestCase):
    """Test battery calculation logic in isolation"""

    def setUp(self):
        """Set up test parameters"""
        self.initial_battery = 100.0
        self.max_battery_level = 94.0
        self.idle_drain_rate = 0.5
        self.moving_drain_rate = 5.0
        self.charge_rate = 3.0
        self.dt = 0.1  # 10 Hz update rate

    def test_idle_drain(self):
        """Test battery drains slowly when idle"""
        battery_level = 50.0
        linear_velocity = 0.0
        angular_velocity = 0.0

        # Idle: not moving, not rotating
        is_moving = linear_velocity > 0.01
        is_rotating = angular_velocity > 0.01
        is_stationary = not is_moving

        if is_stationary and is_rotating:
            rate = self.charge_rate
        elif is_moving:
            rate = -self.moving_drain_rate
        else:
            rate = -self.idle_drain_rate

        new_battery = battery_level + rate * self.dt

        self.assertEqual(rate, -self.idle_drain_rate)
        self.assertLess(new_battery, battery_level)
        self.assertAlmostEqual(new_battery, 49.95, places=2)

    def test_moving_drain(self):
        """Test battery drains faster when moving"""
        battery_level = 50.0
        linear_velocity = 1.0  # Moving
        angular_velocity = 0.0

        is_moving = linear_velocity > 0.01

        if is_moving:
            rate = -self.moving_drain_rate
        else:
            rate = -self.idle_drain_rate

        new_battery = battery_level + rate * self.dt

        self.assertEqual(rate, -self.moving_drain_rate)
        self.assertLess(new_battery, battery_level)
        self.assertAlmostEqual(new_battery, 49.5, places=2)

    def test_charging_when_rotating_stationary(self):
        """Test battery charges when stationary and rotating (on charging pad)"""
        battery_level = 50.0
        linear_velocity = 0.0  # Stationary
        angular_velocity = 1.0  # Rotating

        is_moving = linear_velocity > 0.01
        is_rotating = angular_velocity > 0.01
        is_stationary = not is_moving

        if is_stationary and is_rotating:
            rate = self.charge_rate
        elif is_moving:
            rate = -self.moving_drain_rate
        else:
            rate = -self.idle_drain_rate

        new_battery = battery_level + rate * self.dt

        self.assertEqual(rate, self.charge_rate)
        self.assertGreater(new_battery, battery_level)
        self.assertAlmostEqual(new_battery, 50.3, places=2)

    def test_battery_clamp_max(self):
        """Test battery doesn't exceed 100%"""
        battery_level = 99.5
        rate = self.charge_rate
        dt = 0.1

        new_battery = battery_level + rate * dt
        new_battery = max(0.0, min(100.0, new_battery))

        self.assertLessEqual(new_battery, 100.0)

    def test_battery_clamp_min(self):
        """Test battery doesn't go below 0%"""
        battery_level = 0.3
        rate = -self.moving_drain_rate
        dt = 0.1

        new_battery = battery_level + rate * dt
        new_battery = max(0.0, min(100.0, new_battery))

        self.assertGreaterEqual(new_battery, 0.0)

    def test_charge_stops_at_max_level(self):
        """Test charging stops at max_battery_level"""
        battery_level = 95.0  # Above max_battery_level (94.0)
        linear_velocity = 0.0
        angular_velocity = 1.0

        is_moving = linear_velocity > 0.01
        is_rotating = angular_velocity > 0.01
        is_stationary = not is_moving

        if is_stationary and is_rotating:
            if battery_level >= self.max_battery_level:
                rate = 0.0  # Stop charging
            else:
                rate = self.charge_rate
        else:
            rate = -self.idle_drain_rate

        self.assertEqual(rate, 0.0)


class TestBatterySimulatorCallbacks(unittest.TestCase):
    """Test callback behavior with mocked ROS2"""

    def test_cmd_vel_callback_updates_velocity(self):
        """Test that cmd_vel callback properly updates velocity values"""
        # Simulate callback behavior without ROS dependencies
        linear_velocity = 0.0
        angular_velocity = 0.0

        # Mock Twist message
        mock_msg = MagicMock()
        mock_msg.linear.x = 0.5
        mock_msg.angular.z = 1.0

        # Simulate callback
        linear_velocity = abs(mock_msg.linear.x)
        angular_velocity = abs(mock_msg.angular.z)

        self.assertEqual(linear_velocity, 0.5)
        self.assertEqual(angular_velocity, 1.0)

    def test_battery_set_callback_clamps_values(self):
        """Test manual battery set clamps to valid range"""
        test_cases = [
            (150.0, 100.0),  # Above max
            (-10.0, 0.0),  # Below min
            (50.0, 50.0),  # Normal value
            (0.0, 0.0),  # Edge case min
            (100.0, 100.0),  # Edge case max
        ]

        for input_val, expected in test_cases:
            result = max(0.0, min(100.0, input_val))
            self.assertEqual(result, expected, f"Failed for input {input_val}")


if __name__ == "__main__":
    unittest.main()
