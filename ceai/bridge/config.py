"""Every ROS parameter the bridge takes, declared and read in one place.

Parameter NAMES are a published contract: config/spot_bridge.yaml,
scripts/launch_offboard.sh and gui/process_manager.py all address this node by
name, and ROS ignores an override for an undeclared parameter silently. Renaming
one here fails quietly on the robot and nowhere else.

VALUES come in two layers. `robot` is declared first and selects a
`ceai.robots.RobotProfile`, which supplies the defaults for every frame, topic
and dimension that differs between robots. Anything in a parameter file then
overrides that, exactly as before. The layering is what makes `robot:=spot`
enough to retarget the node, while leaving every explicit override working.
"""
from dataclasses import dataclass, field
from typing import Any, Dict, Tuple

from ..robots import load as load_profile, names as robot_names

# Parameters that do not depend on which robot this is. Declaration order is
# this dict's order, and the robot-derived block below is spliced in after
# `robot` so a reader sees them grouped the way the node uses them.
COMMON_DEFAULTS: Dict[str, Any] = {
    # VLFM canvas: forwarded to the policy server, unused by this node.
    "size": 1000,
    "pixels_per_meter": 20,
    # frames
    "map_frame": "map",
    "frame_prefix": "",
    # topics: robot-side ones take the namespace, Nav2/SLAM-side ones do not
    "namespace": "",
    "map_topic": "/map",
    "goal_topic": "/goal_pose",
    "spin_action": "/spin",
    # policy-server geometry: read by the server, forwarded unchanged.
    # min_height/max_height are INERT on the Nav2-bridge deployment (the server
    # rebuilds its obstacle map from the occupancy grid rather than projecting
    # depth), and are kept only so the parameter surface still matches what the
    # launchers may pass.
    "min_height": 0.15,
    "max_height": 0.88,
    # policy round-trip
    "policy_timeout": 30.0,
    "target": "chair",
    # depth
    "depth_stub_m": 1.5,
    "min_depth": 0.5,
    "max_depth": 12.0,
    "lidar_col_pad": 3,
    # control loop
    "init_steps": 12,                 # must equal the server's INIT_SCAN_STEPS
    "step_period": 0.05,
    # Republish /goal_pose only once the goal has moved this far. 0.0 = off =
    # republish every step, which is the behaviour this has always had. See
    # GoalPublisher for why you might want it on.
    "goal_epsilon_m": 0.0,
    "goal_epsilon_yaw": 0.0,
    # Pull a goal that lands past the edge of /map back inside it by
    # goal_map_margin_m. Off by default; see GoalPublisher.clamp_to_map_bounds
    # for the failure it prevents. config/spot_bridge.yaml turns it on.
    "clamp_goal_to_map": False,
    "goal_map_margin_m": 0.3,
    # capture timing (sized from measured transport lag, 2026-09)
    "frame_max_age": 5.0,             # <=0 disables
    # 1.0 and not 1: the default's Python type is the parameter's declared ROS
    # type, so an integer here made `-p scan_max_dt:=0.5` (or the yaml) fatal
    # with InvalidParameterTypeException. Raised from 0.5 on 2026-09-16 to quiet
    # the zoedepth-scan warning; may need to come back down.
    "scan_max_dt": 1.0,               # <=0 disables
    "scan_buffer_len": 100,
    "tf_lookup_timeout": 0.2,
    "settle_time": 0.15,
    "frame_wait_timeout": 5.0,
    # motion gate
    "motion_lin_eps": 0.02,
    "motion_ang_eps": 0.05,
    # Seconds the base must stay still before the stop edge is believed.
    # 0.0 = off (the bare edge). See MotionTracker in ceai/bridge/sensors.py;
    # config/spot_bridge.yaml turns it on for Spot's stepping turns.
    "motion_still_hold": 0.0,
    "stationary_timeout": 10.0,
    # Policy steps (init scan + explore, each a POST the server answered) after
    # which the run ends as if the policy had called stop: the finish sequence
    # runs, the loop returns. 0 = off (run until called_stop). OPT-IN;
    # config/spot_bridge.yaml sets 500 for Spot; `-p max_steps:=0` disables.
    "max_steps": 0,
    # -- after the policy calls stop ----------------------------------------
    # Cancel Nav2, wait for the base to stop, then call the driver's /sit.
    # OFF by default; config/spot_bridge.yaml turns it on. ceai/bridge/finish.py
    # has the sequence and the caveat about services crossing zenoh.
    "sit_on_finish": False,
    # std_srvs/Trigger on the ROBOT side (takes `namespace`, like the topics).
    "sit_service": "/sit",
    # action_msgs/CancelGoal on the Nav2 side. An all-zero request cancels
    # every NavigateToPose goal, which is the one Nav2 is still driving.
    "nav_cancel_service": "/navigate_to_pose/_action/cancel_goal",
    # Per call: how long to wait for the service to exist, then for its reply.
    "finish_service_timeout": 10.0,
    # The finish is ALSO announced on this topic for ceai/finish_listener.py on
    # the Pi, which calls /sit locally -- topics cross zenoh, services may not.
    # Repeated for finish_announce_s seconds at 2 Hz (zenoh republishes as
    # VOLATILE; a single message can be missed). 0.0 = announce once.
    "finish_topic": "/ceai/finish",
    "finish_announce_s": 3.0,
}

# realsense  aligned D435 depth: real metric depth, registered to the colour
#            frame by the camera itself.
# zoedepth   monocular metric depth, for a robot with no depth sensor.
# lidar      the /scan ranging curtain (curtain.py).
# stub       a flat plane. Diagnostics only.
DEPTH_SOURCES = ("realsense", "zoedepth", "lidar", "stub")

# Depth sources that consume a 16UC1 millimetre raster off `depth_topic`.
RASTER_DEPTH_SOURCES = ("realsense", "zoedepth")


def defaults_for(profile) -> Dict[str, Any]:
    """The full name -> default map for one robot, in declaration order."""
    return {
        "robot": profile.name,
        # -- from the profile ---------------------------------------------
        "robot_frame": profile.base_frame,
        "lidar_frame": profile.scan_frame,
        "camera_optical_frame": profile.camera_optical_frame,
        "rgb_topic": profile.rgb_topic,
        "camera_info_topic": profile.camera_info_topic,
        "depth_topic": profile.depth_topic,
        "scan_topic": profile.scan_topic,
        "odom_topic": profile.odom_topic,
        "agent_radius": profile.agent_radius_m,
        "camera_height": profile.camera_height_m,
        "stop_standoff_m": profile.stop_standoff_m,
        "policy_url": profile.policy_url,
        "depth_source": profile.depth_source,
        # -- robot-independent --------------------------------------------
        **COMMON_DEFAULTS,
    }


# `robot` is the one parameter with no default -- see ceai/robots/__init__.py.
# An empty value is refused by declare() rather than resolved to anything.
NO_ROBOT = ""

# Parameters whose arithmetic assumes a float, so a yaml that writes `5` for a
# float cannot change behaviour.
_FLOAT_PARAMS = (
    "frame_max_age", "scan_max_dt", "tf_lookup_timeout", "settle_time",
    "frame_wait_timeout", "motion_lin_eps", "motion_ang_eps", "motion_still_hold",
    "stationary_timeout", "policy_timeout", "step_period", "stop_standoff_m",
    "min_depth", "max_depth", "depth_stub_m", "camera_height", "agent_radius",
    "goal_epsilon_m", "goal_epsilon_yaw", "goal_map_margin_m",
    "finish_service_timeout", "finish_announce_s",
)
_INT_PARAMS = ("lidar_col_pad", "init_steps", "scan_buffer_len", "max_steps")


@dataclass
class BridgeConfig:
    """Resolved settings. Built by `declare()`; plain data afterwards."""

    values: Dict[str, Any] = field(default_factory=dict)
    profile: Any = None

    # -- raw access ---------------------------------------------------------
    def __getattr__(self, name):
        try:
            return self.__dict__["values"][name]
        except KeyError:
            raise AttributeError(name) from None

    def __setattr__(self, name, value):
        if name != "values" and "values" in self.__dict__ and name in self.__dict__["values"]:
            self.__dict__["values"][name] = value
        else:
            super().__setattr__(name, value)

    # -- composed values ----------------------------------------------------
    # `namespace` prefixes the ROBOT-side topics, which are absolute
    # ('/odometry' -> '/spot/odometry'). It deliberately does not touch
    # map/goal/spin: those belong to the SLAM+Nav2 side, unnamespaced on
    # the workstation. `frame_prefix` prefixes robot-attached frames only, mirroring
    # spot_driver's <frame_prefix><frame>.
    @property
    def namespace(self) -> str:
        raw = str(self.values["namespace"]).strip("/")
        return f"/{raw}" if raw else ""

    @property
    def robot_frame(self) -> str:
        return self.values["frame_prefix"] + self.values["robot_frame"]

    @property
    def lidar_frame(self) -> str:
        return self.values["frame_prefix"] + self.values["lidar_frame"]

    @property
    def camera_optical_frame(self) -> str:
        return self.values["frame_prefix"] + self.values["camera_optical_frame"]

    def robot_topic(self, name: str) -> str:
        return self.namespace + self.values[name]

    def nav_topic(self, name: str) -> str:
        return self.values[name]

    @property
    def depth_band(self) -> Tuple[float, float]:
        return float(self.values["min_depth"]), float(self.values["max_depth"])

    @property
    def wants_depth_raster(self) -> bool:
        """Does this depth source subscribe to `depth_topic`?

        Rasters are ~300 kB each, so the node only carries them when they feed
        the policy.
        """
        return self.values["depth_source"] in RASTER_DEPTH_SOURCES

    @property
    def resolved_depth_topic(self) -> str:
        """`depth_topic`, namespaced only when it comes off the robot.

        The D435's aligned depth is published by the same camera as the colour
        frame, on the robot side, so it takes `namespace` exactly as rgb_topic
        does. ZoeDepth's raster is produced on the workstation next to the
        policy and never does.
        """
        if self.values["depth_source"] == "realsense":
            return self.robot_topic("depth_topic")
        return self.nav_topic("depth_topic")


def declare(node) -> BridgeConfig:
    """Declare every parameter on `node` and read it back.

    Two passes, because the second pass's defaults depend on the first's value:
    `robot` must be declared and read before the profile that supplies the rest
    can be chosen. ROS applies the same overrides either way, so a params file
    and `-p robot:=spot` behave identically.

    `robot` has NO default and an empty one is fatal here, at construction,
    rather than being resolved to some robot. See ceai/robots/__init__.py.
    """
    node.declare_parameter("robot", NO_ROBOT)
    robot = node.get_parameter("robot").value
    if not str(robot or "").strip():
        raise ValueError(
            f"this node needs to know which robot it is driving: pass "
            f"-p robot:=<name>, or set `robot:` in the parameter file. "
            f"Known robots: {', '.join(robot_names())}.")
    profile = load_profile(robot)

    defaults = defaults_for(profile)
    for name, default in defaults.items():
        if name != "robot":
            node.declare_parameter(name, default)
    values = {name: node.get_parameter(name).value for name in defaults}

    for name in _FLOAT_PARAMS:
        values[name] = float(values[name])
    for name in _INT_PARAMS:
        values[name] = int(values[name])

    config = BridgeConfig(values=values, profile=profile)
    if config.depth_source not in DEPTH_SOURCES:
        raise ValueError(
            f"depth_source={config.depth_source!r} is not one of {DEPTH_SOURCES}")
    return config
