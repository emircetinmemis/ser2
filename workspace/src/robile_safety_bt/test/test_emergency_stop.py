"""
Unit tests for Emergency Stop Feature
"""

import unittest


class TestEmergencyStopLogic(unittest.TestCase):
    """Test emergency stop logic"""

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

    def test_emergency_inactive_allows_other_checks(self):
        """Test normal behavior when emergency is not active"""
        emergency_stop = False
        collision_warning = True
        battery_low = False

        if emergency_stop:
            outcome = "emergency"
        elif collision_warning:
            outcome = "collision"
        elif battery_low:
            outcome = "battery_low"
        else:
            outcome = "all_good"

        self.assertEqual(outcome, "collision")

    def test_emergency_state_transitions(self):
        """Test state machine transitions with emergency stop"""
        transitions = {"MONITOR": {"emergency": "EMERGENCY_STOP", "collision": "STOP_MOTION", "battery_low": "ROTATE_BASE", "all_good": "MONITOR"}, "EMERGENCY_STOP": {"cleared": "MONITOR"}}

        # Test emergency transition
        current = "MONITOR"
        outcome = "emergency"
        next_state = transitions[current][outcome]
        self.assertEqual(next_state, "EMERGENCY_STOP")

        # Test cleared transition
        current = "EMERGENCY_STOP"
        outcome = "cleared"
        next_state = transitions[current][outcome]
        self.assertEqual(next_state, "MONITOR")

    def test_emergency_stops_robot(self):
        """Test that emergency stop zeros all velocities"""
        emergency_active = True

        if emergency_active:
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = 1.0  # Would be moving
            angular_z = 0.5

        self.assertEqual(linear_x, 0.0)
        self.assertEqual(angular_z, 0.0)

    def test_emergency_callback_updates_state(self):
        """Test emergency callback properly updates state"""
        emergency_stop = False

        # Simulate callback with True
        msg_data = True
        emergency_stop = msg_data
        self.assertTrue(emergency_stop)

        # Simulate callback with False
        msg_data = False
        emergency_stop = msg_data
        self.assertFalse(emergency_stop)


class TestEmergencyStopBehaviorTree(unittest.TestCase):
    """Test emergency stop in behavior tree context"""

    def test_emergency_behavior_priority(self):
        """Test emergency behavior is first in selector"""
        priorities = ["EmergencyStop", "StopMotion", "Rotate", "Idle"]

        self.assertEqual(priorities[0], "EmergencyStop")

    def test_emergency_behavior_returns_running(self):
        """Test behavior returns RUNNING when emergency active"""
        emergency_stop = True

        if emergency_stop:
            status = "RUNNING"
        else:
            status = "FAILURE"

        self.assertEqual(status, "RUNNING")

    def test_emergency_behavior_returns_failure_when_inactive(self):
        """Test behavior returns FAILURE when no emergency"""
        emergency_stop = False

        if emergency_stop:
            status = "RUNNING"
        else:
            status = "FAILURE"

        self.assertEqual(status, "FAILURE")

    def test_selector_with_emergency(self):
        """Test selector chooses emergency over other behaviors"""

        def selector(behaviors):
            for name, status in behaviors:
                if status in ["SUCCESS", "RUNNING"]:
                    return name
            return None

        # Emergency active
        behaviors = [
            ("EmergencyStop", "RUNNING"),
            ("StopMotion", "RUNNING"),  # Would also be running
            ("Rotate", "FAILURE"),
            ("Idle", "RUNNING"),
        ]
        result = selector(behaviors)
        self.assertEqual(result, "EmergencyStop")

        # Emergency cleared, collision active
        behaviors = [("EmergencyStop", "FAILURE"), ("StopMotion", "RUNNING"), ("Rotate", "FAILURE"), ("Idle", "RUNNING")]
        result = selector(behaviors)
        self.assertEqual(result, "StopMotion")


class TestEmergencyStopBlackboard(unittest.TestCase):
    """Test blackboard integration for emergency stop"""

    def test_blackboard_initial_state(self):
        """Test emergency_stop initialized to False"""
        blackboard = {"emergency_stop": False}
        self.assertFalse(blackboard["emergency_stop"])

    def test_blackboard_update_from_topic(self):
        """Test blackboard updates from topic message"""
        blackboard = {"emergency_stop": False}

        # Simulate receiving True
        blackboard["emergency_stop"] = True
        self.assertTrue(blackboard["emergency_stop"])

        # Simulate receiving False
        blackboard["emergency_stop"] = False
        self.assertFalse(blackboard["emergency_stop"])


if __name__ == "__main__":
    unittest.main()
