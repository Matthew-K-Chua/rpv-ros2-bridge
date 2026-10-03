"""The TurtleBot 4 needs no wrapper on the Pi.

It publishes /cmd_vel and odometry itself through its own bringup, which runs on
the robot rather than on the companion Pi. This file exists so that
`robot:=turtlebot4` resolves to something rather than to a special case in
launch/robot_onboard.launch.py -- one shape for every robot, with the difference
confined to these includes.
"""


def actions(context):
    """No driver to start. The camera and scan chain come from the caller."""
    del context
    print("[turtlebot4.include] no robot wrapper needed; the TurtleBot 4's own "
          "bringup publishes /cmd_vel and odometry.")
    return []
