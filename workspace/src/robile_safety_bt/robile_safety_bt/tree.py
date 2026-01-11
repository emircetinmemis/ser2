"""
Behavior Tree Structure for Robot Safety System
"""

import py_trees as pt

from robile_safety_bt.behaviours import (
    BatteryStatus2bb,
    EmergencyStop2bb,
    EmergencyStopBehavior,
    LaserScan2bb,
    Rotate,
    StopMotion,
)


def create_root(config) -> pt.behaviour.Behaviour:
    """
    Creates behavior tree for robot safety system.

    Tree Structure:
        Root (Parallel)
        ├── Topics2BB (Sequence) - Update blackboard with sensor data
        │   ├── EmergencyStop2bb
        │   ├── BatteryStatus2bb
        │   └── LaserScan2bb
        └── Priorities (Selector) - Execute behaviors based on priorities
            ├── EmergencyStop (highest priority - emergency override!)
            ├── StopMotion (high priority - collision avoidance)
            ├── Rotate (medium priority - battery management)
            └── Idle (lowest priority - default behavior)

    Args:
        config: SafetyConfig object with all thresholds

    Returns:
        Root node of the behavior tree
    """

    # Root - Parallel node runs both branches simultaneously
    root = pt.composites.Parallel(
        name="Root", policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    # Branch 1: Topics2BB - Update blackboard with sensor data
    topics2BB = pt.composites.Sequence(name="Topics2BB", memory=False)

    # Branch 2: Priorities - Execute behaviors based on conditions
    priorities = pt.composites.Selector(name="Priorities", memory=False)

    # Behaviors for Topics2BB
    emergency2bb = EmergencyStop2bb(name="Emergency2BB", topic_name="/emergency_stop")

    battery2bb = BatteryStatus2bb(
        name="Battery2BB",
        battery_topic_name="/battery_level",
        threshold_low=config.low_battery_threshold,
        threshold_ok=config.battery_ok_threshold,
    )

    scan2bb = LaserScan2bb(
        name="Scan2BB",
        topic_name="/scan",
        collision_distance=config.collision_distance,
        safe_distance=config.safe_distance,
    )

    # Behaviors for Priorities (in order of priority)
    emergency_stop = EmergencyStopBehavior(name="EmergencyStop", topic_name="/cmd_vel")

    stop_motion = StopMotion(name="StopMotion", topic_name="/cmd_vel")

    rotate = Rotate(name="Rotate", topic_name="/cmd_vel", ang_vel=config.rotation_speed)

    idle = pt.behaviours.Running(name="Idle")

    # Build tree structure
    root.add_children([topics2BB, priorities])
    topics2BB.add_children([emergency2bb, battery2bb, scan2bb])
    priorities.add_children([emergency_stop, stop_motion, rotate, idle])

    return root
