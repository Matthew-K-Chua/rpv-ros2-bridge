"""The D435 as an RGBD sensor: colour and aligned depth, together.

Until 2026-09-15 this file could not exist -- `enable_depth` was pinned False
under the belief that the Pi could not sustain both streams. That was two bugs
in realsense2_camera, not a capacity limit (measured 2026-09-15).
At the deployable pair, 424x240x6 + 424x240x6, the node costs 1.2 % of one core.

THREE STREAMS COME OUT AND THEY ARE NOT INTERCHANGEABLE:

  color/image_raw                  -> the policy's RGB, as /compressed
  aligned_depth_to_color/image_raw -> the policy's DEPTH. Registered to the
                                      colour frame by the camera, so the stamp
                                      and the shape match the RGB frame exactly.
  depth/image_rect_raw             -> NAVIGATION's depth, which becomes /scan on
                                      the Pi. The RAW stream, deliberately: the
                                      depth sensor sees 87 deg where colour sees
                                      69, and aligning crops depth to the colour
                                      frustum. With one wedge instead of five
                                      body cameras, 18 deg is not spare.

THE PATCHED DRIVER IS REQUIRED. An unpatched realsense2_camera fails to open the
second sensor about 28 % of the time and hardware-resets itself mid-stream; both
failures look like a broken camera. scripts/check_onboard.sh proves which library is
loaded -- run it before blaming the hardware.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _repo import ensure_importable  # noqa: E402

ensure_importable()

from launch import LaunchDescription                                    # noqa: E402
from launch.actions import DeclareLaunchArgument, OpaqueFunction        # noqa: E402
from launch.substitutions import LaunchConfiguration                    # noqa: E402
from launch_ros.actions import Node                                     # noqa: E402

from ceai import cameras                                               # noqa: E402

DEFAULT_CAMERA = "d435"


def _truthy(text) -> bool:
    return str(text).strip().lower() in ("true", "1", "yes", "on")


def camera_parameters(camera, color, depth, initial_reset, sync):
    """The realsense2_camera parameter dict for one RGBD pair.

    Profiles are written under TWO parameter names each because
    realsense2_camera renamed them between releases (`rgb_camera.profile` ->
    `rgb_camera.color_profile`, `depth_module.profile` ->
    `depth_module.depth_profile`). A ROS node silently ignores an override for a
    parameter it never declares, so setting both spellings works on either
    version and costs nothing on both.
    """
    return {
        # -- colour -------------------------------------------------------
        "enable_color": True,
        "rgb_camera.color_profile": color.as_parameter,
        "rgb_camera.profile": color.as_parameter,
        # -- depth --------------------------------------------------------
        "enable_depth": True,
        "depth_module.depth_profile": depth.as_parameter,
        "depth_module.profile": depth.as_parameter,
        # Registers depth onto the colour frame, giving the policy a raster
        # whose stamp and shape match the RGB frame by construction. Without
        # this the bridge's realsense depth source has nothing to subscribe to.
        "align_depth.enable": True,
        # Frame-syncs the two sensors so a depth frame and a colour frame
        # describe one instant. With differing frame rates the synced output
        # falls to the SLOWER of the two -- see the note in generate_launch_
        # description() about keeping the rates equal.
        "enable_sync": sync,
        # -- time base: HOST arrival, not librealsense's global clock -----
        # librealsense fits a line between the device and host clocks. On this
        # D435 the fit drifts and stamps frames in the FUTURE, which makes every
        # consumer ask TF for a pose at an instant that has not happened.
        # MEASURED on the Pi with `ros2 topic delay` (negative = stamp ahead of
        # the clock; the two machines agree to <0.2 ms, so this is the driver):
        #     global_time_enabled True   -0.054 s at 14:00, -0.65 s by 17:00
        #     global_time_enabled False  +0.008 s, steady
        # It DRIFTS 12x across a session, so a restart makes it look fine.
        # Slightly late and monotonic beats in-the-future and drifting.
        # Setting it on BOTH sensors also silences most of the XU polling noise.
        "rgb_camera.global_time_enabled": False,
        "depth_module.global_time_enabled": False,
        # -- start-up -----------------------------------------------------
        # DEFAULT FALSE since 2026-09-15. A hardware reset at every start is
        # precisely the operation this device handles worst, and it is the same
        # operation the driver's spurious-notification bug was firing. Leave it
        # false; if the camera is genuinely wedged, unplug it.
        "initial_reset": initial_reset,
        # -- everything else off ------------------------------------------
        # The cloud is built by depth_image_proc in the scan chain, in C++, from
        # the stream we choose; librealsense's own is a second code path over
        # the wrong stream.
        "pointcloud.enable": False,
        "enable_infra1": False,
        "enable_infra2": False,
        "enable_gyro": False,
        "enable_accel": False,
    }


def _setup(context, *args, **kwargs):
    camera = cameras.load(LaunchConfiguration("camera_model").perform(context))
    color = camera.parse_profile(
        LaunchConfiguration("color_profile").perform(context), "color")
    depth = camera.parse_profile(
        LaunchConfiguration("depth_profile").perform(context), "depth")
    sync = _truthy(LaunchConfiguration("enable_sync").perform(context))

    if sync and color.fps != depth.fps:
        print(f"[rgbd_camera.launch] NOTE: enable_sync is on and the rates "
              f"differ (color {color.fps} fps, depth {depth.fps} fps). The "
              f"synced output falls to {min(color.fps, depth.fps)} fps on BOTH "
              f"streams. Match them, or pass enable_sync:=false and accept "
              f"unpaired frames.")

    return [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace=LaunchConfiguration("camera_namespace"),
            parameters=[camera_parameters(
                camera, color, depth,
                _truthy(LaunchConfiguration("initial_reset").perform(context)),
                sync)],
            output="screen",
            emulate_tty=True,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    d435 = cameras.D435
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_model", default_value=DEFAULT_CAMERA,
            description=("Which camera. Known: "
                         + ", ".join(sorted(cameras.CAMERAS)) +
                         ". The model decides which frame rates are accepted."),
        ),
        DeclareLaunchArgument(
            # Namespace and node name are both 'camera', so topics nest as
            # /camera/camera/... on realsense2_camera 4.55+.
            "camera_namespace", default_value="camera",
            description="Namespace for the realsense node",
        ),
        DeclareLaunchArgument(
            "color_profile", default_value=str(d435.default_color),
            description=("Colour stream WxHxFPS. The policy does not need more "
                         "than a few frames a second; resolution matters more "
                         "than rate to SAM3."),
        ),
        DeclareLaunchArgument(
            "depth_profile", default_value=str(d435.default_depth),
            description=("Depth stream WxHxFPS -- this is /scan's rate. Raising "
                         "it costs Pi CPU (3.5 % of a core at 848x480x15, "
                         "measured) and nothing on the link, because raw depth "
                         "never crosses. Raise color_profile's rate WITH it "
                         "while enable_sync is on."),
        ),
        DeclareLaunchArgument(
            "enable_sync", default_value="true",
            description=("Pair depth and colour frames by timestamp. On is what "
                         "makes aligned depth safe to hand the policy beside "
                         "the RGB frame."),
        ),
        DeclareLaunchArgument(
            "initial_reset", default_value="false",
            description=("Hardware-reset the camera at start-up. FALSE by "
                         "default since 2026-09-15: the reset is the operation "
                         "this device handles worst and it was the mechanism "
                         "behind the mid-stream failures, not the cure."),
        ),
        OpaqueFunction(function=_setup),
    ])
