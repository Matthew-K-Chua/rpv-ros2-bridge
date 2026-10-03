"""Clearpath Robotics TurtleBot 4, in the RGBD topology.

REBUILT 2026-09-26 for the ethernet-cable arrangement: the same Pi, the same
D435 and the same scan chain as Spot, with the TurtleBot 4 as the mobile base
on the Pi's ethernet cable. The Create 3 supplies /odom, odom -> base_link and
takes /cmd_vel; the Pi supplies everything else. Nothing of the TurtleBot 4's
own sensor suite (RPLIDAR, OAK-D) is on the path -- turtlebot4.service is
STOPPED for a run, so those topics never exist.

The previous profile transcribed the RPLIDAR-and-OAK-D rig that ran in 2026-08
(`rplidar_link`, the `/oakd/...` topics, `depth_source: lidar`). Those values
are in git history (this file before 2026-09-26) if that rig is ever rebuilt.
"""
from ..mount import MountPose
from .base import RobotProfile

# base_link's height above the floor. DERIVED 2026-09-26 from the Create 3 URDF
# installed on the laptop (/opt/ros/humble/share/irobot_create_description/
# urdf/create3.urdf.xacro and wheel.urdf.xacro):
#   wheel axle z in base_link = wheel_height + wheel_drop_offset_z + base_link_z_offset
#                             = -0.0275 + 0.0035 + 0.0642 = 0.0402 m
#   wheel radius              = 0.03575 m
#   floor z in base_link      = 0.0402 - 0.03575 = +0.0045 m
# i.e. base_link sits ~4 mm BELOW the wheel contact plane: floor-referenced to
# within the wheel-drop travel. Carried as 0.0. UNVERIFIED against the robot
# (a tape from the floor to the top of the Create 3's shell, 0.0942 m per the
# same URDF, would confirm it).
BODY_HEIGHT_M = 0.0

# The D435's mount, base_link -> camera_link. MEASURED BY TAPE 2026-09-26 by the
# operator, camera on a bracket above the top plate:
#
#   x  0.110   lens forward of the Create 3's centre. base_link is on the axle
#              line, centred between the wheels (Create 3 URDF), so the centre
#              IS the origin in x and y.
#   y  0.0     mounted on the centreline.
#   z  0.820   lens above the FLOOR (0.820) minus BODY_HEIGHT_M (0.0). The
#              operator also measured the lens 0.475 above the top plate, which
#              puts the plate at 0.345 above the floor; the standard TurtleBot 4
#              URDF puts the tower sensor plate at 0.0942 + 0.25257 = 0.3468
#              above base_link. The two agree to 2 mm, which is the tape and
#              the URDF checking each other.
#   roll  0.0  level.
#   pitch 0.0  LEVEL, by the operator. NOT yet fitted against the floor. With
#              the floor at z ~ 0 and scan_height_band.min at 0.08, the floor
#              enters /scan where r * sin(pitch_error) > 0.08: 1.5 deg at
#              3 m (obstacle_range_m), 1.15 deg at 4 m. Do the RViz floor check
#              (see the README) before the first
#              run: an arc at constant range across the front of /scan is the
#              floor. spot.py's comment block has the RANSAC method to re-fit
#              if it appears.
#   yaw   0.0  looks straight ahead.
#
# Pass D435_X=... etc. (all six) to scripts/launch_onboard.sh to override; the
# launch refuses a partial set. If the camera is REMOUNTED, change it here.
CAMERA_MOUNT = MountPose(x=0.110, y=0.0, z=0.820, roll=0.0, pitch=0.0, yaw=0.0)

TURTLEBOT4 = RobotProfile(
    name="turtlebot4",
    base_frame="base_link",
    odom_frame="odom",
    # /scan is the flattener's; pointcloud_to_laserscan's target_frame is
    # base_frame, as on Spot.
    scan_frame="base_link",
    # camera_optical_frame, rgb_topic, camera_info_topic and depth_topic are
    # the RobotProfile defaults: the D435 on the Pi.
    odom_topic="/odom",
    # The Create 3 is 0.35 m across; Nav2 describes it by robot_radius 0.175
    # (config/nav2_turtlebot4.yaml, the stock turtlebot4_navigation value).
    # 0.18 is what the 2026-08 rig ran with and what the old TurtleBot 4 policy
    # server used (0.19); the Spot server this rig runs against uses 0.25 --
    # The README lists the effect.
    agent_radius_m=0.18,
    # Carried over from the 2026-08 rig, where the robot halted 0.4 m short of
    # the object. The Create 3's nose is only ~0.17 m ahead of base_link.
    stop_standoff_m=0.4,
    # Lens above the floor: the policy places every detection vertically from
    # this. Computed from the two measured parts and asserted against them in
    # the two must agree, as on Spot.
    camera_height_m=BODY_HEIGHT_M + CAMERA_MOUNT.z,
    body_height_m=BODY_HEIGHT_M,
    camera_mount=CAMERA_MOUNT,
    # Described by a radius, never a polygon (config/nav2_turtlebot4.yaml
    # robot_radius). An empty tuple is how RobotProfile spells that.
    footprint=(),
    # base_link z. With the floor at ~0 (BODY_HEIGHT_M), min_height IS the
    # whole margin against painting the floor into /scan. 0.08 m is PROPOSED,
    # sized against nothing yet: re-fit once the mount is measured (spot.py has
    # the arithmetic: a pitch error of a few degrees spends this margin by
    # 3-4 m). max 1.0 m is the default; the Create 3 is 0.35 m tall, so
    # anything it could drive under is also in the band.
    scan_height_band=(0.08, 1.0),
    depth_source="realsense",
    # The Create 3 publishes /odom and takes /cmd_vel itself, across the zenoh
    # pair on the TurtleBot 4 Pi and the ethernet cable; nothing to wrap.
    driver_include="",
    nav2_params="nav2_turtlebot4.yaml",
)
