"""Collaborators of the Spot bridge node.

The node in ceai/rpv_bridge.py owns the ROS wiring -- parameters,
subscriptions, publishers, the action client -- and delegates the work here:

    config      every ROS parameter, declared and read in one place
    sensors     the frame cache, the stamped buffers, the motion gate
    frames      the pinned episodic frame and the pose-at-stamp lookup
    occupancy   the Nav2 grid and the hash that keeps it off the wire
    curtain     2D scan -> one depth value per image column
    depth       a depth strategy per depth_source
    payload     assembling the policy payload, and refusing to build a bad one
    policy      the HTTP round-trip
    goals       publishing to Nav2
    spin        rotation via the Nav2 Spin behaviour, and the waits around it
    loop        one step, and the control loop that repeats it
"""
