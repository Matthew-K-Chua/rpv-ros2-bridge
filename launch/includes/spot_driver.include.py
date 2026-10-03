"""Brings Spot up on the Pi: spot_driver, and nothing else.

Included by launch/robot_onboard.launch.py when `robot:=spot`. It is a separate file
so that the safety arguments below are read as a unit and cannot be lost in a
diff to a larger launch file.

SAFETY, AND IT IS NOT NEGOTIABLE. `start_estop` MUST stay False: the driver
taking the E-stop EVICTS THE TABLET and the physical stop button silently stops
working. `auto_claim` / `auto_power_on` / `auto_stand` stay False so nothing
moves without a human typing the lifecycle calls. Those values live in
config/spot_driver.yaml, which is why this file refuses to start without it.
"""
import os

import yaml
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Site credentials live OUTSIDE this repo -- config.yaml is git-ignored and
# config.yaml.example is the template. The path arrives as the `spot_login`
# launch argument, declared (with its default) in launch/robot_onboard.launch.py.

#: yaml key -> the environment variable bosdyn reads it from.
LOGIN_KEYS = (
    ("username", "BOSDYN_CLIENT_USERNAME"),
    ("password", "BOSDYN_CLIENT_PASSWORD"),
    ("hostname", "SPOT_IP"),
)

def load_login(path: str) -> None:
    """Export Spot's credentials from the out-of-tree login yaml.

    Skipped entirely when the environment already carries them, which is the
    normal case on the Pi: ~/.bashrc exports them, and that reaches an
    interactive tmux window. It does NOT reach `ssh pi '<command>'`, which is
    why the file path exists at all.
    """
    needed = [(key, env) for key, env in LOGIN_KEYS if not os.environ.get(env)]
    if not needed:
        print("[spot_driver.include] login already in the environment; "
              f"not reading {path}")
        return

    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Spot login file '{path}' does not exist. Point the launch at the "
            f"real one with spot_login:=/path/to/config.yaml, or export "
            f"{' / '.join(env for _, env in needed)} yourself.")

    with open(path, "r") as handle:
        doc = yaml.safe_load(handle)
    if not isinstance(doc, dict):
        raise ValueError(f"Spot login file '{path}' did not parse to a mapping.")

    # Accept the ROS convention config.yaml.example uses ('/**:' then
    # 'ros__parameters:') and a flat top-level mapping, so a file written either
    # way works rather than failing with a yaml-shape error at the robot.
    params = doc
    scoped = doc.get("/**")
    if isinstance(scoped, dict) and isinstance(scoped.get("ros__parameters"), dict):
        params = scoped["ros__parameters"]

    missing = [key for key, _ in needed if not params.get(key)]
    if missing:
        raise ValueError(
            f"Spot login file '{path}' is missing {', '.join(missing)}. "
            f"Expected keys: {', '.join(key for key, _ in LOGIN_KEYS)}.")

    for key, env in needed:
        os.environ[env] = str(params[key])

    # Username and host only -- never print the password.
    print(f"[spot_driver.include] Spot login from {path}: user "
          f"'{os.environ['BOSDYN_CLIENT_USERNAME']}' at {os.environ['SPOT_IP']}")


def actions(context):
    """The driver include, with credentials already in the environment.

    Argument values are grounded in spot_ros2's own
    spot_driver/launch/spot_driver.launch.py and spot_launch_helpers.py:

      spot_name ""              nodes and topics unnamespaced (/odometry,
                                /cmd_vel), matching ceai/robots/spot.py.
      tf_prefix ""              plain body/odom/vision frames; config/
                                spot_driver.yaml pins frame_prefix "" to match.
      launch_image_publishers   FALSE since the D435 became the only scan
                                source. It is what asked Spot for the five body
                                depth streams over gRPC, which was most of what
                                the Pi spent itself on. It is ALSO what
                                broadcast the body cameras' static TF -- nothing
                                needs those frames now, but anything that did
                                would break silently.
      publish_point_clouds      False: the driver's own cloud path needs an RGB
                                stream we do not request, and turning it on
                                without one yields nothing, quietly.
    """
    load_login(LaunchConfiguration("spot_login").perform(context))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("spot_driver"), "launch",
                                      "spot_driver.launch.py"])
            ),
            launch_arguments={
                "config_file": LaunchConfiguration("spot_config"),
                "spot_name": "",
                "tf_prefix": "",
                "launch_rviz": "False",
                "launch_image_publishers": "False",
                "depth_registered_mode": "disable",
                "publish_point_clouds": "False",
                "uncompress_images": "True",
                "publish_compressed_images": "False",
                "stitch_front_images": "False",
            }.items(),
        )
    ]
