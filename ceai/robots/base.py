"""What differs between one robot and the next, in one object.

A profile is FROZEN. It is read at start-up and never mutated; deployment
choices (which target, which policy port) are ROS parameters layered on top,
not fields here.

Adding a robot means adding one module beside this one and registering it in
`ceai/robots/__init__.py`. Nothing else in the tree should learn its name.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from ..mount import MountPose

# The D435-on-a-Pi topic layout produced by launch/rgbd_camera.launch.py, which
# runs the node as namespace "camera", name "camera". realsense2_camera 4.55+
# nests the node name, so the segment really is doubled. Same on every robot,
# hence defaults rather than per-robot fields -- a robot that mounts its camera
# differently overrides them.
RGB_TOPIC = "/camera/camera/color/image_raw/compressed"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
ALIGNED_DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"

Point = Tuple[float, float]


@dataclass(frozen=True)
class RobotProfile:
    """Frames, topics and geometry for one robot.

    Field groups, and who reads each:

      frames    the bridge's TF lookups, slam_toolbox, both Nav2 costmaps
      topics    the bridge's subscriptions
      geometry  the policy server (agent_radius, camera_height), the bridge's
                standoff, and Nav2's footprint polygon
      wiring    which launch include brings the robot up, and which Nav2
                parameter file describes it
    """

    name: str

    # -- frames -------------------------------------------------------------
    # The rigid body Nav2 drives and TF is queried against: Spot's is `body`,
    # not `base_link`.
    base_frame: str
    # The continuously-connected parent of base_frame, which SLAM publishes
    # map-> onto. Spot roots its tree at `vision`, not `odom`.
    odom_frame: str
    # frame_id of /scan. With the scan derived from the D435 this is whatever
    # pointcloud_to_laserscan is told to target, which is base_frame.
    scan_frame: str
    camera_optical_frame: str = "camera_color_optical_frame"

    # -- topics -------------------------------------------------------------
    odom_topic: str = "/odom"
    scan_topic: str = "/scan"
    rgb_topic: str = RGB_TOPIC
    camera_info_topic: str = CAMERA_INFO_TOPIC
    depth_topic: str = ALIGNED_DEPTH_TOPIC
    cmd_vel_topic: str = "/cmd_vel"

    # -- geometry -----------------------------------------------------------
    # Circle the policy uses to decide which frontiers are reachable. Must match
    # the policy server's own agent_radius.
    agent_radius_m: float = 0.18
    # How far in front of a detected object Nav2 is asked to stop, measured from
    # base_frame's origin -- so a robot whose nose is far ahead of its centre
    # needs a bigger number for the same physical clearance.
    stop_standoff_m: float = 0.4
    # Camera lens height above the FLOOR. The policy places every detection
    # vertically from this, so it is live.
    camera_height_m: float = 0.25
    # base_frame's own origin above the floor, standing. NOT the same as
    # camera_height_m, and not zero on every robot -- Spot's body origin sits
    # mid-torso. Kept so camera_height_m can be checked against the mount
    # rather than maintained beside it.
    body_height_m: float = 0.0
    # The MEASURED base_frame -> camera_link transform, when this robot has a
    # recorded one. None means "nobody has measured this robot's camera", and
    # the Pi launch then refuses to build /scan rather than guessing -- see
    # ceai/mount.py.
    camera_mount: Optional[MountPose] = None
    # Nav2's footprint polygon in base_frame, counter-clockwise. An empty tuple
    # means the robot is described by a radius instead.
    footprint: Tuple[Point, ...] = ()
    # The slab of base_frame z that pointcloud_to_laserscan flattens into /scan.
    # base_frame is NOT floor-referenced on every robot -- Spot's origin sits
    # mid-torso -- so this band is per-robot and cannot be shared.
    scan_height_band: Tuple[float, float] = (0.0, 1.0)

    # -- wiring -------------------------------------------------------------
    # Route on the policy server. The unified RPV policy server exposes a single
    # robot-agnostic route, so every robot uses it; kept as a field only so a
    # future deployment could point one robot at a different endpoint.
    policy_route: str = "rpv_policy"
    # Depth strategy this robot runs unless told otherwise.
    depth_source: str = "realsense"
    # Basename (no directory) of the launch include that starts the robot's own
    # driver on the Pi, or "" when the robot needs no wrapper.
    driver_include: str = ""
    # Basename of this robot's Nav2 parameter file in config/.
    nav2_params: str = ""
    # Basename of this robot's RViz layout in config/, or "" to fall back to a
    # generic one. The layout (map, costmaps, /scan, /goal_pose, footprint) is
    # largely robot-independent, so a robot without its own may reuse another's.
    rviz: str = ""

    # -- derived ------------------------------------------------------------
    @property
    def policy_url(self) -> str:
        """Default policy endpoint. The deployment overrides host and port."""
        return f"http://localhost:13000/{self.policy_route}"

    @property
    def footprint_yaml(self) -> str:
        """The polygon as Nav2's `footprint` parameter spells it.

        Nav2 takes the polygon as a STRING of nested lists, not a yaml list, and
        a mismatch between this and config/nav2_<robot>.yaml is what the profile
        test compares.
        """
        points = ", ".join(f"[{x}, {y}]" for x, y in self.footprint)
        return f"[{points}]"

    def __post_init__(self):
        if self.agent_radius_m <= 0.0:
            raise ValueError(f"{self.name}: agent_radius_m must be positive")
        if self.footprint and len(self.footprint) < 3:
            raise ValueError(
                f"{self.name}: a footprint polygon needs at least 3 points")
