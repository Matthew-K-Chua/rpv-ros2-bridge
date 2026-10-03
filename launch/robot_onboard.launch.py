"""Everything that runs on the Pi, for any robot.

    robot wrapper (spot_driver | none)  ->  /cmd_vel, odometry
    D435 RGBD                           ->  colour + aligned depth (cross zenoh)
    raw depth -> cloud -> flattener     ->  /scan            (crosses zenoh)

RAW DEPTH NEVER LEAVES THIS MACHINE. The link carries ~50 Mbit/s and depth at
848x480x15 is ~98 Mbit/s on its own. The cloud and the flattener run here so
that only a 720-float /scan crosses; config/zenoh_onboard.json5's deny list enforces
it, and the two must agree.

    ros2 launch ceai robot_onboard.launch.py robot:=spot \\
        d435_x:=.. d435_y:=.. d435_z:=.. d435_roll:=.. d435_pitch:=.. d435_yaw:=..

A MOUNT POSE IS REQUIRED. pointcloud_to_laserscan cannot reach the robot's base
frame without base -> camera_link, so without one /scan never appears at all --
with no error anywhere. It was demoted to policy-only while /scan came from
Spot's body cameras; it is load-bearing for NAVIGATION again.

It normally comes from the robot profile, where it is version-controlled with
its provenance. The six d435_* arguments override it for a camera that has been
remounted and not yet written down, and are still all-or-nothing.
"""
import importlib.util
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _repo import CONFIG_DIR, LAUNCH_DIR, REPO_ROOT, ensure_importable, which_ceai  # noqa: E402

ensure_importable()

from launch import LaunchDescription                                    # noqa: E402
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,  # noqa: E402
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource   # noqa: E402
from launch.substitutions import LaunchConfiguration                    # noqa: E402
from launch_ros.actions import ComposableNodeContainer, Node            # noqa: E402
from launch_ros.descriptions import ComposableNode                      # noqa: E402

from ceai import cameras, robots                                        # noqa: E402
from ceai.mount import DESCRIPTIONS as MOUNT_DESCRIPTIONS, MountPose    # noqa: E402

# The angular grid /scan is published on: 0.5 deg bins over the camera's own
# wedge ONLY. Until 2026-09-17 this was the full circle with unreached bins
# left +inf ("no return"). Now unreached bins are filled with a "clear out to
# here" reading (see flattener_parameters), and that must never be said about a
# bearing the camera did not look at -- so the grid is the wedge, nothing more.
SCAN_ANGLE_INCREMENT = math.pi / 360.0

# Karto and pointcloud_to_laserscan disagree about how many readings a scan of
# span S and increment r holds. The flattener publishes ceil(S / r) bins
# (pointcloud_to_laserscan_node.cpp, humble). Karto expects
# round(S / r) + 1 for anything that is not a full circle
# (/opt/ros/humble/include/karto_sdk/Karto.h ~4307, residual 1) and REJECTS
# every scan whose count differs -- LaserRangeFinder::Validate prints
# "LaserRangeScan contains N range readings, expected M" to stdout and the
# map never appears. The old full-circle grid only worked because slam_toolbox
# flags a 2*pi span as a 360 laser, where the residual is 0 and 720 == 720.
#
# ceil(S/r) == round(S/r) + 1 exactly when S/r has a fractional part in
# (0, 0.5), so the wedge is padded by a quarter of a bin -- split evenly so it
# stays centred on the mount yaw. See ceai/cameras.py
# with both formulas written out. Measured 2026-09-17: the unpadded 87 deg
# wedge gave 174 against an expected 175, and /map stopped.
KARTO_PAD_BINS = 0.25


def scan_wedge(camera, mount_yaw):
    """angle_min/max in the base frame: the depth FOV centred on the mount yaw.

    pointcloud_to_laserscan bins each point by atan2(y, x) in target_frame and
    drops anything outside [angle_min, angle_max], so this is the camera's
    horizontal FOV rotated by the recorded (or overridden) mount yaw, plus the
    quarter-bin pad that makes Karto accept the scan (KARTO_PAD_BINS).
    """
    half = (math.radians(camera.depth_fov_deg) + KARTO_PAD_BINS * SCAN_ANGLE_INCREMENT) / 2.0
    return mount_yaw - half, mount_yaw + half


def published_bin_count(params):
    """How many ranges pointcloud_to_laserscan will publish for these params."""
    return math.ceil((params["angle_max"] - params["angle_min"]) / params["angle_increment"])


def karto_expected_bin_count(params):
    """How many readings Karto's LaserRangeFinder will demand for these params
    (non-360 laser: Round(span / increment) + 1, Round being floor(x + 0.5))."""
    span = params["angle_max"] - params["angle_min"]
    return int(math.floor(span / params["angle_increment"] + 0.5)) + 1

# The raw depth stream, NOT aligned_depth_to_color: aligning crops depth to the
# narrower colour frustum, and 18 deg of an 87 deg wedge is not spare.
# See launch/rgbd_camera.launch.py's header.
DEPTH_IMAGE_TOPIC = "depth/image_rect_raw"
DEPTH_INFO_TOPIC = "depth/camera_info"
# Produced here, consumed here, denied at the zenoh bridge.
CLOUD_TOPIC = "depth/points"


def _truthy(text) -> bool:
    return str(text).strip().lower() in ("true", "1", "yes", "on")


def _load_include(basename):
    """Import launch/includes/<basename> and hand back its `actions`."""
    path = os.path.join(LAUNCH_DIR, "includes", basename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"robot include not found: {path}")
    spec = importlib.util.spec_from_file_location(
        f"ceai_include_{basename.replace('.', '_')}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.actions


def mount_pose(context, profile):
    """The base -> camera_link transform to publish, and where it came from.

    Returns (MountPose or None, source). Resolution order:

      1. the six d435_* arguments, if given -- still ALL SIX OR NONE, because a
         defaulted component is a guess and a guessed pitch tilts everything
         both Nav2 and the policy see;
      2. the robot profile's recorded mount, which is version-controlled and
         carries its provenance beside it;
      3. nothing -- and then the caller refuses to build /scan.

    The override path exists for a camera that has been remounted and not yet
    written down. If you find yourself passing the same six numbers every run,
    put them in ceai/robots/<name>.py instead.
    """
    given = MountPose.from_strings({
        name: LaunchConfiguration(f"d435_{name}").perform(context)
        for name in MOUNT_DESCRIPTIONS
    })
    if given is not None:
        return given, "the d435_* launch arguments"
    if profile.camera_mount is not None:
        # The REAL path, not the relative spelling: a Pi that has run `colcon
        # build` has two copies of this file, and until 2026-09-16 the stale
        # installed one silently won. This line is how you tell which was read.
        return profile.camera_mount, os.path.join(
            which_ceai(), "robots", f"{profile.name}.py")
    return None, "nowhere"


def camera_topic(namespace, leaf):
    """Absolute topic under the camera's namespace.

    realsense2_camera 4.55+ runs as namespace/name, both "camera", so the
    segment really is doubled: /camera/camera/depth/image_rect_raw.
    """
    prefix = f"/{namespace}/{namespace}" if namespace else ""
    return f"{prefix}/{leaf}"


def cloud_remappings(namespace):
    """PointCloudXyzNode's three topics.

    `image_rect` is the RAW depth stream, NOT aligned_depth_to_color: aligning
    crops depth to the narrower colour frustum, and 18 of 87 degrees is not
    spare on a robot with one wedge.
    """
    return [
        ("image_rect", camera_topic(namespace, DEPTH_IMAGE_TOPIC)),
        ("camera_info", camera_topic(namespace, DEPTH_INFO_TOPIC)),
        ("points", camera_topic(namespace, CLOUD_TOPIC)),
    ]


def flattener_parameters(profile, camera, depth_fps, mount_yaw):
    """pointcloud_to_laserscan's parameters. Pure, so the couplings are testable.

    The range pair is the most coupled in the tree: its partners are listed in
    ceai/cameras.py RANGE_PARTNERS and asserted in
    ceai/cameras.py RANGE_PARTNERS. Points beyond range_max are DROPPED, not
    clamped, so a range_max below what the room contains leaves slam_toolbox
    almost nothing to match and the map comes out as radial free-space spokes.
    """
    min_height, max_height = profile.scan_height_band
    angle_min, angle_max = scan_wedge(camera, mount_yaw)
    return {
        # What makes the scan usable by Nav2 AND what keeps the bridge's
        # lidar_frame correct: /scan's frame_id IS this.
        "target_frame": profile.base_frame,
        "transform_tolerance": 0.05,
        "min_height": min_height,
        "max_height": max_height,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": SCAN_ANGLE_INCREMENT,
        # One "revolution" is one depth frame.
        "scan_time": 1.0 / float(depth_fps),
        "range_min": camera.min_range_m,
        "range_max": camera.max_range_m,
        # A bin with no in-band return inside the wedge is published as
        # no_return_range_m (range_max + inf_epsilon; the node adds them with
        # no sign check, read in the humble source 2026-09-17), NOT +inf. That
        # value sits in slam's [max_laser_range, range_max) window, so Karto
        # traces the bearing free out to max_laser_range without marking a
        # hit, and above Nav2's obstacle_max_range, so the costmap clears along
        # it (out to raytrace_max_range) without marking. Since 2026-09-19
        # range_max is the sensor's reach (10 m) and the obstacle boundary is
        # the separate ceai/cameras.py obstacle_range_m (3 m).
        # +inf was "no return" and left open floor UNKNOWN: the floor is below
        # scan_height_band, so a bearing across an empty room had no in-band
        # point and never became free space -- the policy had no frontier.
        #
        # KNOWN LIE: "saw only floor" and "saw nothing" (stereo dropout on a
        # blank wall, glass, or anything nearer than range_min) are the same
        # empty bin here. The distinction exists only in the depth image; if a
        # free fan appears through a blank wall, this is why.
        "use_inf": False,
        "inf_epsilon": camera.no_return_range_m - camera.max_range_m,
    }


def scan_chain(profile, camera, depth_fps, namespace, mount_yaw):
    """depth image -> cloud -> /scan, in C++, on this machine.

    PointCloudXyzNode and not Xyzrgb: this path wants geometry only, and Xyzrgb
    would make a missing colour frame a reason for /scan to stop.

    pointcloud_to_laserscan and NOT depthimage_to_laserscan, which is installed
    and looks simpler: that node takes a band of pixel ROWS and merely relabels
    frame_id -- it does no TF transform. The D435 is pitched DOWN, so
    relabelling its rows as a horizontal scan in the robot's base frame would be
    meaningless. This node genuinely transforms into target_frame and filters on
    real base-frame height, which is the only thing that makes a tilted camera
    usable as a laser.
    """
    return [
        ComposableNodeContainer(
            name="depth_cloud_container",
            namespace="",
            package="rclcpp_components",
            # Multi-threaded even with one node: the container is where a second
            # camera would land, and single-threaded is a silent serialisation.
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzNode",
                    name="depth_points",
                    remappings=cloud_remappings(namespace),
                )
            ],
            output="screen",
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="scan_from_depth",
            remappings=[
                ("cloud_in", camera_topic(namespace, CLOUD_TOPIC)),
                ("scan", profile.scan_topic),
            ],
            parameters=[flattener_parameters(profile, camera, depth_fps, mount_yaw)],
            output="screen",
        ),
    ]


def _setup(context, *args, **kwargs):
    profile = robots.load(LaunchConfiguration("robot").perform(context))
    camera = cameras.load(LaunchConfiguration("camera_model").perform(context))
    depth_profile = camera.parse_profile(
        LaunchConfiguration("depth_profile").perform(context), "depth")
    namespace = LaunchConfiguration("camera_namespace").perform(context)

    camera_only = _truthy(LaunchConfiguration("camera_only").perform(context))
    skip_camera = _truthy(LaunchConfiguration("no_camera").perform(context))
    if camera_only and skip_camera:
        raise RuntimeError("camera_only and no_camera are mutually exclusive")

    pose, pose_source = mount_pose(context, profile)
    actions = []

    if not skip_camera:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(LAUNCH_DIR, "rgbd_camera.launch.py")),
                launch_arguments={
                    "camera_model": LaunchConfiguration("camera_model"),
                    "camera_namespace": LaunchConfiguration("camera_namespace"),
                    "color_profile": LaunchConfiguration("color_profile"),
                    "depth_profile": LaunchConfiguration("depth_profile"),
                    "enable_sync": LaunchConfiguration("enable_sync"),
                    "initial_reset": LaunchConfiguration("initial_reset"),
                }.items(),
            )
        )

    if camera_only:
        # Colour and depth on the wire, nothing else. For proving the camera
        # without the robot in the room.
        if pose is not None:
            actions.append(_static_tf(profile, pose, pose_source))
        print("[robot_onboard.launch] camera_only: no robot wrapper, no scan chain.")
        return actions

    if skip_camera:
        # The robot, without the USB camera in the picture. There is no depth
        # stream, so there is nothing for the scan chain to consume and no
        # reason to demand a mount pose.
        print("[robot_onboard.launch] no_camera: robot wrapper only, no scan chain.")
        if profile.driver_include:
            actions.extend(_load_include(profile.driver_include)(context))
            actions.append(_finish_listener(context))
        return actions

    if pose is None:
        raise RuntimeError(
            f"no camera mount pose for robot '{profile.name}'. /scan is built "
            f"from the depth camera, so without base -> camera_link "
            f"pointcloud_to_laserscan cannot reach target_frame "
            f"'{profile.base_frame}' and /scan NEVER APPEARS -- with no error "
            f"anywhere. Either record it as `camera_mount` in "
            f"ceai/robots/{profile.name}.py, or pass all six:\n  " +
            "\n  ".join(f"d435_{name}:=<{text}>"
                        for name, text in MOUNT_DESCRIPTIONS.items()) +
            "\nMeasure them; do not guess. camera_only:=true skips the scan "
            "chain if you only want the streams.")

    actions.append(_static_tf(profile, pose, pose_source))
    actions.extend(scan_chain(profile, camera, depth_profile.fps, namespace,
                              pose.yaw))

    if profile.driver_include:
        actions.extend(_load_include(profile.driver_include)(context))
        actions.append(_finish_listener(context))
    else:
        print(f"[robot_onboard.launch] {profile.name} needs no driver wrapper.")

    return actions


def _finish_listener(context):
    """ceai/finish_listener.py: sits the robot when the bridge announces the
    finish on /ceai/finish. It runs HERE because /sit is a service of the
    driver wrapper and services do not reliably cross zenoh; the topic does.
    Module form, cwd = this checkout, so no colcon build is needed and the
    stale-install trap in _repo.py cannot bite.

    sit_retry_s / sit_confirm_timeout_s: /sit is non-blocking and a late
    cmd_vel can override it, so the listener re-calls it until the driver's
    status/feedback reports sitting. `sit_retry_s:=0` restores the single,
    unconfirmed call."""
    retry_s = LaunchConfiguration("sit_retry_s").perform(context)
    confirm_s = LaunchConfiguration("sit_confirm_timeout_s").perform(context)
    return ExecuteProcess(
        cmd=[sys.executable, "-m", "ceai.finish_listener", "--ros-args",
             "-p", "namespace:=''",
             "-p", f"sit_retry_s:={float(retry_s)}",
             "-p", f"sit_confirm_timeout_s:={float(confirm_s)}"],
        cwd=REPO_ROOT,
        name="finish_listener",
        output="screen",
    )


def _static_tf(profile, pose, source):
    """base -> camera_link, published HERE and not on the workstation.

    pointcloud_to_laserscan needs it locally, and making a Pi-side TF lookup
    depend on the tunnel is an avoidable failure mode.

    The source is printed because a stale recorded mount and a fresh measured
    one look identical once the TF is up, and the difference is where every
    detection lands.
    """
    print(f"[robot_onboard.launch] {profile.base_frame} -> camera_link: {pose}")
    print(f"[robot_onboard.launch]   mount from {source}")
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera",
        arguments=pose.static_transform_args(profile.base_frame),
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    d435 = cameras.D435
    arguments = [
        DeclareLaunchArgument(
            "robot", default_value="",
            description=("REQUIRED. Which robot: " + ", ".join(robots.names())
                         + ". There is deliberately no default -- the wrong "
                         "robot's frames do not error, they just never resolve."),
        ),
        DeclareLaunchArgument(
            "camera_model", default_value="d435",
            description="Camera model: " + ", ".join(sorted(cameras.CAMERAS)),
        ),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("color_profile", default_value=str(d435.default_color)),
        DeclareLaunchArgument(
            "depth_profile", default_value=str(d435.default_depth),
            description="Depth stream WxHxFPS. This is /scan's rate.",
        ),
        DeclareLaunchArgument("enable_sync", default_value="true"),
        DeclareLaunchArgument("initial_reset", default_value="false"),
        DeclareLaunchArgument(
            "camera_only", default_value="false",
            description="Streams only: no robot wrapper, no scan chain.",
        ),
        DeclareLaunchArgument(
            "no_camera", default_value="false",
            description="Robot wrapper only -- isolates the USB camera from the rest.",
        ),
        DeclareLaunchArgument(
            "sit_retry_s", default_value="1.0",
            description=("finish_listener: seconds between /sit re-calls until "
                         "status/feedback reports sitting. 0 = call once, unconfirmed."),
        ),
        DeclareLaunchArgument(
            "sit_confirm_timeout_s", default_value="30.0",
            description="finish_listener: give up re-calling /sit after this long.",
        ),
        DeclareLaunchArgument(
            "spot_config", default_value=os.path.join(CONFIG_DIR, "spot_driver.yaml"),
            description=("spot_driver parameters. Carries start_estop: False -- "
                         "do not launch without it."),
        ),
        DeclareLaunchArgument(
            "spot_login", default_value=os.environ.get(
                "SPOT_LOGIN_YAML", os.path.expanduser("~/spot_files/config.yaml")),
            description="Out-of-tree yaml with username/password/hostname. Env vars win.",
        ),
    ]
    arguments += [
        DeclareLaunchArgument(
            f"d435_{name}", default_value="",
            description=(f"MEASURED -- {text}. All six or none. Empty uses the "
                         f"robot profile's recorded mount."),
        )
        for name, text in MOUNT_DESCRIPTIONS.items()
    ]
    arguments.append(OpaqueFunction(function=_setup))
    return LaunchDescription(arguments)
