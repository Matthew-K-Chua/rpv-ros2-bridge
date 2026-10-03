"""The workstation's half: SLAM and Nav2. No robot code runs here.

    /scan  ->  slam_toolbox  ->  /map + map->odom-frame TF
    /map, /scan, TF  ->  Nav2  ->  /cmd_vel  ->  (zenoh) ->  the robot walks

Everything robot-specific comes from `robot:=`, which selects a
ceai.robots.RobotProfile: the base frame, the continuous odometry frame, and
which Nav2 parameter file describes the robot's footprint and limits.

    ros2 launch ceai nav_stack.launch.py robot:=spot

`robot:=` is REQUIRED and there is no default. Without it this launch refuses to
start.

"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _repo import CONFIG_DIR, ensure_importable  # noqa: E402

ensure_importable()

from launch import LaunchDescription                                    # noqa: E402
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,  # noqa: E402
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource   # noqa: E402
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution     # noqa: E402
from launch_ros.actions import Node                                     # noqa: E402
from launch_ros.substitutions import FindPackageShare                   # noqa: E402
from ament_index_python.packages import get_package_share_directory     # noqa: E402

from ceai import cameras, robots                                        # noqa: E402


def slam_defaults_file() -> str:
    """slam_toolbox's stock async parameter file. Overrides sit on top of it;
    anything not listed in slam_parameters() keeps that file's value."""
    return os.path.join(get_package_share_directory("slam_toolbox"), "config",
                        "mapper_params_online_async.yaml")


def slam_parameters(profile, camera):
    """slam_toolbox overrides for one robot and one sensor.

    The laser range pair and the processing gates are the two groups worth
    reading. Both were measured into their current values on 2026-09-12, and
    both are sized against the SCAN, so they move when the sensor does.
    """
    return {
        "use_sim_time": False,
        "base_frame": profile.base_frame,
        # The CONTINUOUS parent of base_frame. Spot roots its tree at `vision`,
        # so map->vision is the edge slam must publish; map->odom would give
        # vision two parents and break TF.
        "odom_frame": profile.odom_frame,
        "map_frame": "map",
        "scan_topic": profile.scan_topic,
        "mode": "mapping",

        "use_scan_matching": False,
        # The stock file assumes a 20 m lidar and logs a warning on every start
        # when the real sensor is shorter. min tracks the flattener's range_min.
        # max is the camera's obstacle_range_m, DELIBERATELY BELOW the
        # flattener's range_max: Karto marks a HIT only below max_laser_range,
        # traces a reading in [max_laser_range, range_max) free up to
        # max_laser_range with no hit, and the flattener fills its no-return
        # bins with a value inside that window (ceai/cameras.py
        # no_return_range_m). Equal to range_max, as it was until 2026-09-17,
        # the window is empty and open floor never becomes free space. Since
        # 2026-09-19 the two differ by design (3 m against 10 m): far returns
        # reach /scan but never the map, and the matcher works on the near 3 m.
        "min_laser_range": camera.min_range_m,
        "max_laser_range": camera.slam_range_threshold_m,
        # -- how often slam is ALLOWED to process a scan --------------------
        # The stock gates are sized for a lidar robot mapping at walking pace,
        # and both caused measured failures on 2026-09-12:
        #
        # minimum_time_interval 0.5 caps slam at 2 Hz, so map->odom-frame is
        # 0.25 s old on average and 0.50 s at worst -- while controller_server
        # queries at 20 Hz with transform_tolerance 0.2 s. "Transform data too
        # old" was therefore STRUCTURAL: 254 of them in a 265 s run, every one
        # while walking, none while parked. slam sat at 6.2 % CPU throughout --
        # throttled, not starved.
        #
        # minimum_travel_distance/heading 0.5 mean slam processes NOTHING once
        # the robot arrives and stops. The frontier it just reached stays a
        # frontier, so the policy re-picks the same goal forever.
        #
        # 0.1 s = 10 Hz, comfortably inside the tolerance, and above the scan
        # rate, so every scan is processed rather than three in four dropped.
        # Raise them back if slam's CPU climbs enough to matter.
        # 2026-09-16 (3b19561): raised to 0.2, which at the D435's 6 Hz scan
        # rate (0.167 s spacing) admits every OTHER scan -- slam now runs at
        # ~3 Hz, not the 10 Hz the paragraph above was sized for.
        "minimum_time_interval": 0.2,
        "minimum_travel_distance": 0.0, # Changed so slam map updates on spin
        "minimum_travel_heading": 0.15,
        # -- how far into the future map->vision is stamped -----------------
        # slam_toolbox stamps map->odom-frame at SCAN STAMP + transform_timeout
        # and republishes that same stamp until the next scan is processed.
        # The stock 0.2 s assumes a lidar whose scans are near-current. Here a
        # scan is ~0.2-0.45 s old on arrival (bridge "frame age" log, same
        # link) and the 0.2 s gate above adds up to 0.33 s more, so map->vision
        # sat 5-210 ms BEHIND Spot's odometry stamp about half the time.
        # controller_server then failed "Unable to transform robot pose into
        # global plan's frame" (Lookup would require extrapolation into the
        # future, vision->map), waited its 0.2 s transform_tolerance each
        # time, and after 0.3 s without a valid command aborted every
        # FollowPath with "Controller patience exceeded" -- MEASURED
        # 2026-09-18 19:50, no_navigation.webm, every goal of the run.
        # 1.0 s covers the worst observed lag (~0.6 s) with margin. Cost: the
        # map->vision CORRECTION reaches Nav2 up to 1 s late; it is a slow
        # drift term, so that is harmless. Parameter name verified in
        # /opt/ros/humble/lib/libtoolbox_common.so (2.6.10).
        "transform_timeout": 1.0,

        # To remove noisy depth readings due to a shaky camera.
        "occupancy_threshold": 0.2,
        "min_pass_through": 5,
    }


def _setup(context, *args, **kwargs):
    profile = robots.load(LaunchConfiguration("robot").perform(context))
    camera = cameras.load(LaunchConfiguration("camera_model").perform(context))

    params_file = LaunchConfiguration("nav2_params").perform(context).strip()
    if not params_file:
        if not profile.nav2_params:
            raise RuntimeError(
                f"no Nav2 parameter file is registered for robot "
                f"'{profile.name}'. Either add one as config/nav2_"
                f"{profile.name}.yaml and name it in ceai/robots/"
                f"{profile.name}.py, or pass nav2_params:=/path/to/params.yaml.")
        params_file = os.path.join(CONFIG_DIR, profile.nav2_params)
    if not os.path.isfile(params_file):
        raise FileNotFoundError(f"Nav2 parameter file not found: {params_file}")

    print(f"[nav_stack.launch] robot={profile.name} "
          f"base={profile.base_frame} odom={profile.odom_frame} "
          f"laser={camera.min_range_m}-{camera.max_range_m} m")

    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[slam_defaults_file(), slam_parameters(profile, camera)],
            output="screen",
        ),
        # navigation_launch.py starts controller/planner/behavior/bt_navigator/
        # waypoint_follower/velocity_smoother -- no AMCL and no map_server, since
        # slam_toolbox above supplies /map and the map->odom-frame transform.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch",
                                      "navigation_launch.py"])),
            launch_arguments={
                "params_file": params_file,
                "use_sim_time": "False",
                "autostart": "True",
            }.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="",
            description=("REQUIRED. Which robot: " + ", ".join(robots.names())
                         + ". There is deliberately no default -- the wrong "
                         "robot's frames do not error, they just never resolve."),
        ),
        DeclareLaunchArgument(
            "camera_model", default_value="d435",
            description=("Which camera feeds /scan -- it sets slam_toolbox's "
                         "laser range. Known: " + ", ".join(sorted(cameras.CAMERAS))),
        ),
        DeclareLaunchArgument(
            "nav2_params", default_value="",
            description="Override the robot profile's Nav2 parameter file.",
        ),
        OpaqueFunction(function=_setup),
    ])
