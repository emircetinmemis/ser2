#!/usr/bin/env python3
"""Generate FSM and BT diagrams"""

from graphviz import Digraph

# FSM Diagram
fsm = Digraph("FSM", format="png")
fsm.attr(rankdir="TB")

# States
fsm.node("MONITOR", "MONITOR_BATTERY_AND_COLLISION\n(Check sensors)", shape="box", style="rounded,filled", fillcolor="lightblue")
fsm.node("ROTATE", "ROTATE_BASE\n(Recharge battery)", shape="box", style="rounded,filled", fillcolor="lightcoral")
fsm.node("STOP", "STOP_MOTION\n(Avoid collision)", shape="box", style="rounded,filled", fillcolor="lightcoral")

# Transitions
fsm.edge("MONITOR", "ROTATE", label="battery < 20%")
fsm.edge("ROTATE", "MONITOR", label="battery > 85%")
fsm.edge("MONITOR", "STOP", label="obstacle < 0.5m")
fsm.edge("STOP", "MONITOR", label="obstacle > 0.75m")
fsm.edge("MONITOR", "MONITOR", label="all OK")

fsm.render("fsm_diagram", cleanup=True)
print("FSM diagram saved as fsm_diagram.png")

# BT Diagram
bt = Digraph("BT", format="png")
bt.attr(rankdir="TB")

# Nodes
bt.node("Root", "Root\n(Parallel)", shape="box", style="filled", fillcolor="lightblue")
bt.node("Topics", "Topics2BB\n(Sequence)", shape="box", style="filled", fillcolor="lightyellow")
bt.node("Priorities", "Priorities\n(Selector)", shape="box", style="filled", fillcolor="lightyellow")
bt.node("Battery", "Battery2BB\n(Subscriber)", shape="ellipse", style="filled", fillcolor="plum")
bt.node("Scan", "Scan2BB\n(Subscriber)", shape="ellipse", style="filled", fillcolor="plum")
bt.node("Stop", "StopMotion\n(Action)", shape="ellipse", style="filled", fillcolor="lightcoral")
bt.node("Rotate", "Rotate\n(Action)", shape="ellipse", style="filled", fillcolor="lightcoral")
bt.node("Idle", "Idle\n(Running)", shape="ellipse", style="filled", fillcolor="lightgreen")

# Structure
bt.edge("Root", "Topics")
bt.edge("Root", "Priorities")
bt.edge("Topics", "Battery")
bt.edge("Topics", "Scan")
bt.edge("Priorities", "Stop", label="1 (highest)")
bt.edge("Priorities", "Rotate", label="2")
bt.edge("Priorities", "Idle", label="3 (default)")

bt.render("bt_diagram", cleanup=True)
print("BT diagram saved as bt_diagram.png")
