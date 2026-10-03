"""Boston Dynamics Spot.

Every value here has a partner somewhere else that fails silently when it
drifts; the partner is named beside it and asserted in
the README's table of coupled constants.
"""
from ..mount import MountPose
from .base import RobotProfile

# Spot is ~1.1 x 0.5 m and its nose sits ~0.55 m ahead of the body origin, so
# the polygon is body-centred at +/-0.55 x, +/-0.25 y.
# TODO(measure): leg splay during a fast walk reaches wider than 0.5 m and
# nothing in this workspace quantifies it. Confirm clearance before trusting
# corridors narrower than ~1 m.
FOOTPRINT = ((0.55, 0.25), (0.55, -0.25), (-0.55, -0.25), (-0.55, 0.25))

# The body origin above the floor, Spot standing.
#
# MEASURED LIVE 2026-09-16: Spot's own `gpe` (ground plane estimate) and
# `feet_center` frames BOTH put the body origin at 0.5157 m. That replaces
# 0.5800, a tape measure taken 2026-09-12 which the robot's own kinematics
# disagree with by 64 mm. It does not match the operator's 610 mm either --
# 610 is most likely the TOP of the body rather than the origin.
#
# This does NOT affect /scan: the TF carries only the mount's own z, so the
# flattener never reads this number. It DOES affect camera_height_m below, from
# which the policy places every detection vertically -- at 0.5800 the policy was
# putting every detection ~64 mm high, silently.
#
# CAVEAT, unchanged and now the more important half: Spot's body height is NOT a
# constant. It bobs while walking and its stand height is commandable, so any
# constant here is ONE POSTURE.
# TODO(measure): read gpe / feet_center live rather than carrying a constant.
BODY_HEIGHT_M = 0.5157

# The D435's mount, body -> camera_link. Every component is sourced:
#
#   x  0.278   = 0.223 + 0.055
#                0.223 is the body -> front_rail translation from the Spot URDF
#                (spot_description/urdf/spot_macro.xacro:52). Boston Dynamics'
#                payload documentation puts the payload mount reference frame
#                origin at zero x displacement from the FRONTMOST RAIL BOLT, so
#                front_rail IS that bolt (BD docs, relayed by the operator
#                2026-09-16 -- not read from BD directly by this repo).
#                0.055 is the lens forward of that bolt, MEASURED by the
#                operator 2026-09-16.
#   y  0.0     the camera is mounted square to the body.
#   z  0.5405  = 0.0805 + 0.460
#                0.0805 is the same front_rail translation's z (xacro:52);
#                0.460 is the lens above the rails.
#   roll  0.0     mounted square to the body -- but the RANSAC fits below also
#                 measured +1.3 to +1.6 deg of roll against the floor. Left at
#                 0.0 DELIBERATELY: it is small next to the pitch error (0.07 m
#                 of arc tilt at 3 m), it was not what caused the phantom wall,
#                 and moving two mount components in one round would make the
#                 pitch correction unfalsifiable.
#                 TODO(measure): re-fit once the bracket has been checked, and
#                 fold this in if it survives.
#   pitch 0.0     LEVEL. OPERATOR-SET 2026-09-16 after the camera was REMOUNTED
#                 parallel to the ground; the bracket was levelled by eye, NOT
#                 re-fitted against the floor. Confirmed as what the Pi
#                 publishes (`tf2_echo body camera_link` -> RPY 0, 0, 0).
#                 TODO(measure): RANSAC-fit the floor plane again with the new
#                 bracket. A 3 deg residual puts the floor into scan_height_band
#                 at 4 m; 1.2 deg does it at 10 m (max_range_m). Harmless to
#                 the MAP past obstacle_range_m (3 m): a far floor return is
#                 published in /scan but traced free, never marked.
#                 The block below is the HISTORY of the previous, pitched mount
#                 and the measurement method -- keep it, it is how to re-fit.
#
#   (superseded) pitch 0.460700 = 26.397 deg, lens DOWN. POSITIVE PITCHES THE LENS DOWN.
#                     MEASURED 2026-09-16 by RANSAC on the floor plane in the
#                     depth optical frame, three captures: 26.23 / 26.25 /
#                     26.62 deg, plane residual 8.6 mm over ~10k inliers -- so
#                     the floor really is planar and the only free variable is
#                     the camera's orientation relative to it. Spot's body was
#                     level throughout (vision -> body pitch 0.11 deg, roll
#                     0.05 deg), so this is the MOUNT, not body attitude.
#
#                     The same fits put the lens 1.046 / 1.049 / 1.052 m above
#                     the floor against the 1.0562 m this profile predicts, so z
#                     below is right to ~10 mm and the error is PURELY pitch.
#
#                     It replaces 0.343010 (19.653 deg), which was SPECIFIED by
#                     the operator on 2026-09-16 and never measured against the
#                     floor. That 6.6 deg gap is what put a phantom wall in
#                     /scan: an under-pitched transform ramps the floor UPWARD
#                     with distance in body frame instead of holding it flat, and
#                     where the ramp crosses scan_height_band's min_height every
#                     bearing returns at the same range -- a circular arc that
#                     reads in rviz as a flat wall across the front. Predicted
#                     crossing 1.86 m; live /scan minimum 1.808 m.
#
#                     OPEN: 6.6 deg is a lot to be off by if 19.653 was the
#                     intended DESIGN angle. CHECK THE BRACKET PHYSICALLY. If it
#                     is remounted to 19.653, this value must come back with it.
#   yaw   0.0     the camera looks straight ahead.
#
# Pass d435_x:=... etc. to override all six at once; the launch refuses a
# partial set either way. If the camera is ever REMOUNTED, change it here --
# this is the one copy, and gui/settings.json only overrides it.
CAMERA_MOUNT = MountPose(
    x=0.278, y=0.0, z=0.5405, roll=0.0, pitch=0.0, yaw=0.0)

SPOT = RobotProfile(
    name="spot",
    # spot_ros2 builds "<frame_prefix>body"; frame_prefix is pinned "" in
    # config/spot_driver.yaml, so the frames are plain.
    base_frame="body",
    # `vision`, not `odom`: config/spot_driver.yaml sets both
    # preferred_odom_frame and tf_root to vision, so vision is the continuous
    # frame slam_toolbox must publish map-> onto. Setting this to `odom` gives
    # vision two parents and breaks TF.
    odom_frame="vision",
    # /scan is pointcloud_to_laserscan's target_frame, which is base_frame.
    scan_frame="body",
    # spot_driver publishes "odometry", not "odom"
    # (state_middleware_handle.cpp:26), sourced from preferred_odom_frame.
    odom_topic="/odometry",
    # Body origin above the floor plus the mount's z: the lens height the
    # policy places every detection vertically from. Computed here, and
    # must equal the sum of the two parts.
    camera_height_m=BODY_HEIGHT_M + CAMERA_MOUNT.z,
    body_height_m=BODY_HEIGHT_M,
    camera_mount=CAMERA_MOUNT,
    # Half-WIDTH (2026-09-19; was 0.55, the half-length). The policy dilates
    # every obstacle in its map by a square of side 2 * agent_radius
    # (vlfm/mapping/obstacle_map.py) and only offers frontiers in what is left,
    # so this number is the narrowest gap the policy will ever send Spot
    # through. At 0.55 that was 1.15 m: no doorway (0.7-0.9 m) ever became a
    # frontier, and the robot could not leave the room it started in. Nav2
    # checks the true polygon, whose inscribed radius is this same 0.25 m, so
    # the policy's navigable map and Nav2's non-lethal region now agree. Must
    # equal the polygon's inscribed radius (asserted) and the policy server's
    # own agent_radius (other repo, not asserted).
    # UNVERIFIED: leg splay past the 0.5 m body (the TODO above) and Spot's own
    # obstacle padding may still refuse a 0.7 m door; that is measured, not
    # configured here.
    agent_radius_m=0.25,
    # JUDGEMENT: the TurtleBot 4 halted 0.4 m short, measured from a base whose
    # nose is barely ahead of centre. Spot's nose is ~0.55 m ahead of the body
    # origin, so 0.4 m here would park the nose effectively touching the object.
    # 1.0 m keeps ~0.45 m of nose clearance.
    stop_standoff_m=1.0,
    footprint=FOOTPRINT,
    # body-frame z, where z=0 is the body origin -- roughly mid-torso standing,
    # MEASURED LIVE 0.5157 m above the floor on 2026-09-16. So this band is
    # 0.216 m to 1.016 m above the floor, and the floor sits 0.216 m BELOW
    # min_height.
    #
    # That 0.216 m is the entire margin protecting /scan from the floor, and it
    # is spent by any error in CAMERA_MOUNT.pitch: at 3 m a 5.4 deg pitch error
    # lifts the floor into the band, at 4 m only 4.0 deg is needed. The band was
    # NOT what caused the 2026-09-16 phantom wall -- the pitch was -- and it is
    # correct as it stands.
    #
    # It was sized to keep Spot's own thighs out of /scan, and that constraint is
    # GONE: the D435 looks outward from the front of the body and cannot see the
    # legs. The remaining constraint is the floor at z = -0.5157, and Spot's body
    # height is not a constant -- it bobs while walking. The band is carried over
    # unchanged rather than widened in the same change that replaced the sensor.
    # TODO(measure): how far the body origin really moves while walking, then
    # lower min_height towards the floor to see obstacles under 0.216 m. Do this
    # only AFTER the mount pitch is confirmed against the bracket: lowering
    # min_height eats exactly the margin that keeps the floor out of /scan.
    scan_height_band=(-0.30, 0.50),
    driver_include="spot_driver.include.py",
    nav2_params="nav2_spot.yaml",
    rviz="spot.rviz",
)
