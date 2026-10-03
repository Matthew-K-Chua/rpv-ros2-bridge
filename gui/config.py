"""Console configuration, persisted to gui/settings.json next to this file.

Every value the two launcher scripts and the bridge need, in one place, so the
buttons reproduce exactly what the README tells you to type. The host-specific
values (the Pi's address, the workstation's SSH target, the Pi's checkout path)
have placeholder defaults and are set once in the Settings dialog.

Settings are read once at start-up. Editing settings.json by hand and
relaunching is always equivalent to using the Settings dialog.
"""
from __future__ import annotations

import json
import os
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List

REPO_ROOT = Path(__file__).resolve().parents[1]          # the rpv-ros2-bridge checkout
SETTINGS_PATH = Path(__file__).resolve().parent / "settings.json"


def _profile(robot: str):
    """The ceai/robots/<robot>.py profile. The console runs from the checkout,
    so the package is importable beside it whether or not the workspace is
    built."""
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from ceai.robots import load
    return load(robot)


def _profile_base_frame(robot: str) -> str:
    return _profile(robot).base_frame

# Used only when shortvis.yaml cannot be read at all. Deliberately the eight
# labels the Spot campaign actually runs with rather than a household set --
# see data/shortvis.yaml in rpv-sim2real-policy.
DEFAULT_TARGETS = [
    "chair", "tub", "monitor", "whiteboard", "cabinet", "bin", "door", "backpack",
]


@dataclass
class Config:
    # --- which robot ---------------------------------------------------------
    # Selects a ceai.robots.RobotProfile everywhere: the bridge's frames and
    # topics, slam_toolbox's frames, Nav2's parameter file, and which driver the
    # Pi launch brings up.
    robot: str = "spot"
    # Selects a ceai.cameras.DepthCamera: which stream rates are accepted, and
    # the depth band shared by /scan, slam_toolbox, Nav2 and the policy.
    camera_model: str = "d435"

    # --- tmux sessions -------------------------------------------------------
    offboard_session: str = "rpv_offboard"
    onboard_session: str = "rpv_onboard"

    # --- the workstation (this machine) ----------------------------------------
    ros_ws: str = str(Path.home() / "ros2_ws")
    ceai_dir: str = str(REPO_ROOT)
    # The rpv-sim2real-policy checkout. RPV_POLICY_REPO, if exported, wins; set
    # it here (Settings dialog) otherwise.
    policy_repo: str = os.environ.get(
        "RPV_POLICY_REPO", str(Path.home() / "rpv-sim2real-policy")
    )

    # --- the Pi ---------------------------------------------------------------
    # THE CONSOLE CANNOT START THE PI, and these are not an SSH client. This
    # machine has no route to the Pi: the tailnet ACL allows the Pi to connect
    # OUT and nothing in, MEASURED, and only port 22 between the laptop and the
    # Pi. They exist so the console can print the exact command to run from a
    # machine that does have a route -- see ProcessManager.onboard_start_command().
    onboard_ssh_user: str = "<pi-user>"
    onboard_ssh_host: str = "<pi-host>"
    onboard_ros_ws: str = "$HOME/ros_ws"             # ros_ws, NOT ros2_ws
    onboard_ceai_dir: str = "$HOME/ros_ws/src/rpv-ros2-bridge"
    # Where the Pi's autossh tunnel points, i.e. THIS machine.
    offboard_ssh: str = "<user>@<workstation-host>"
    tunnel_port: int = 7447
    onboard_launch_args: str = ""                    # e.g. "camera_only:=true"

    # --- policy / depth ------------------------------------------------------
    policy_host: str = "127.0.0.1"              # the policy binds loopback here
    policy_port: int = 13000
    policy_route: str = "rpv_policy"
    zoedepth_port: int = 12185
    # realsense = the D435's aligned depth, real metric depth. zoedepth =
    # monocular depth, for a robot with no depth sensor; it is the only setting
    # that starts the two ZoeDepth windows.
    depth_source: str = "realsense"             # realsense | zoedepth | lidar | stub
    #
    # RPV_MAX_DEPTH_M is NOT a setting here any more. It is both the [0,1]
    # normalisation divisor and the "invalid" sentinel in the policy, so a wrong
    # value silently discards detections -- and it used to be defaulted
    # differently in two files and duplicated here as a third. scripts/
    # launch_offboard.sh now derives it from ceai/cameras.py (realsense) or from
    # ZoeDepth's trust_max_m (zoedepth), so there is one source.

    # --- OVERRIDE the robot profile's recorded D435 mount pose. ALL SIX or none.
    # Normally BLANK: ceai/robots/<robot>.py carries the measured mount with its
    # provenance, and that is the copy that is version-controlled -- this file is
    # git-ignored, so values that live only here are lost on another machine.
    # Fill these in only for a camera that has been remounted and not yet
    # written down, then move them into the profile.
    d435_x: str = ""
    d435_y: str = ""
    d435_z: str = ""
    d435_roll: str = ""
    d435_pitch: str = ""
    d435_yaw: str = ""

    # --- target --------------------------------------------------------------
    default_target: str = "backpack"

    # --- ROS topics / frames -------------------------------------------------
    map_topic: str = "/map"
    goal_topic: str = "/goal_pose"
    scan_topic: str = "/scan"
    camera_topic: str = ""                      # blank -> the profile's rgb_topic
    map_frame: str = "map"
    # The base frame is `robot`'s, from ceai/robots/<robot>.py (body on Spot,
    # base_link on the TurtleBot 4) -- see the base_frame property. This
    # overrides it, and is normally blank.
    base_frame_override: str = ""
    show_scan: bool = True

    # --- rosbag --------------------------------------------------------------
    bag_dir: str = str(Path.home() / "ros2_ws" / "bags")
    # The Record button's other outputs, named after the bag: records/logs/
    # <name>.log (the policy's per-step decisions) and records/videos/
    # {value_map,policy}/<name>.mp4. git-ignored. gui/run_recorder.py.
    records_dir: str = str(REPO_ROOT / "records")
    # The robot-independent topics to record. The robot's own odom, scan and
    # camera topics are added from its profile (see bag_topics_resolved), so a
    # TurtleBot 4 records /odom and a Spot /odometry without editing this list.
    # The ALIGNED depth is the one the policy sees; the raw depth stream never
    # leaves the Pi and cannot be recorded here.
    bag_topics: List[str] = field(default_factory=lambda: [
        "/map", "/goal_pose", "/tf", "/tf_static",
    ])

    # --- RViz ----------------------------------------------------------------
    # Blank = use the robot profile's layout (config/<profile.rviz>). Set a path
    # to override. See the rviz_path property.
    rviz_config: str = ""

    # --- GUI -----------------------------------------------------------------
    status_poll_hz: float = 3.0
    # How long a PI/WS light may stay yellow ("starting") before it is called a
    # failure and turns red. Yellow used to have no expiry at all, so a launcher
    # that died in the first second looked identical to one still loading SAM3's
    # weights -- forever. Set to 0 to switch the expiry off and get that back.
    start_timeout_s: float = 90.0
    system_poll_s: float = 3.0

    # ---------------------------------------------------------------- #
    @property
    def base_frame(self) -> str:
        """The frame the map panel draws the robot at: the profile's, unless
        overridden. Not a stored field, so it cannot disagree with `robot`."""
        return self.base_frame_override.strip() or _profile_base_frame(self.robot)

    @property
    def live_camera_topic(self) -> str:
        """What the live camera panel subscribes to: `camera_topic` if set,
        else the colour topic the bridge itself reads from the robot profile
        (2026-09-17; before that a blank camera_topic meant no raw feed at
        all, and the live panel would have sat empty on the default settings)."""
        return self.camera_topic.strip() or _profile(self.robot).rgb_topic

    @property
    def policy_base_url(self) -> str:
        return f"http://{self.policy_host}:{self.policy_port}"

    @property
    def policy_url(self) -> str:
        return f"{self.policy_base_url}/{self.policy_route}"

    @property
    def status_url(self) -> str:
        return f"{self.policy_base_url}/status"

    @property
    def shortvis_path(self) -> Path:
        """The SAM3 prompt list. Lives in the OTHER repo, on this machine."""
        return Path(os.path.expanduser(self.policy_repo)) / "data" / "shortvis.yaml"

    @property
    def bridge_params_file(self) -> str:
        # config/<robot>_bridge.yaml: the deployment choices for the robot
        # `robot` selects, beside the profile that supplies its frames.
        return str(Path(self.ceai_dir) / "config" / f"{self.robot}_bridge.yaml")

    @property
    def rviz_path(self) -> str:
        """The RViz layout to open: the explicit override if set, else the robot
        profile's (config/<profile.rviz>), else config/spot.rviz as a generic
        fallback. Not a stored field, so it cannot disagree with `robot`."""
        override = self.rviz_config.strip()
        if override:
            return override
        basename = _profile(self.robot).rviz or "spot.rviz"
        return str(Path(self.ceai_dir) / "config" / basename)

    @property
    def bag_topics_resolved(self) -> List[str]:
        """Topics to record for THIS robot: the robot-independent list plus the
        selected profile's odom, scan and camera topics, de-duplicated with
        order preserved. Keeps /odom vs /odometry (and any per-robot camera
        namespace) correct without hand-editing bag_topics."""
        p = _profile(self.robot)
        ordered = list(self.bag_topics) + [
            p.odom_topic, p.scan_topic, p.cmd_vel_topic,
            p.rgb_topic, p.camera_info_topic, p.depth_topic,
        ]
        seen = set()
        out = []
        for t in ordered:
            if t and t not in seen:
                seen.add(t)
                out.append(t)
        return out

    def d435_values(self) -> List[str]:
        return [self.d435_x, self.d435_y, self.d435_z,
                self.d435_roll, self.d435_pitch, self.d435_yaw]

    def d435_complete(self) -> bool:
        """True only when all six are set. Partially specified is an error, not
        a default: a guessed pitch tilts everything the policy ranges."""
        vals = [v.strip() for v in self.d435_values()]
        return all(vals)

    def d435_partial(self) -> bool:
        vals = [v.strip() for v in self.d435_values()]
        return any(vals) and not all(vals)

    # ---------------------------------------------------------------- #
    @classmethod
    def load(cls) -> "Config":
        if SETTINGS_PATH.exists():
            try:
                data = json.loads(SETTINGS_PATH.read_text())
                known = {f for f in cls.__dataclass_fields__}   # type: ignore[attr-defined]
                unknown = sorted(set(data) - known)
                if unknown:
                    print(f"[config] ignoring unknown key(s) in {SETTINGS_PATH}: {', '.join(unknown)}")
                return cls(**{k: v for k, v in data.items() if k in known})
            except Exception as e:
                print(f"[config] failed to read {SETTINGS_PATH}: {e}; using defaults")
        return cls()

    def save(self) -> None:
        SETTINGS_PATH.write_text(json.dumps(asdict(self), indent=2) + "\n")
