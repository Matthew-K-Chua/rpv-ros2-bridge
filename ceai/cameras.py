"""The depth cameras this stack can run, and the stream profiles each offers.

One place owns what a camera model can physically do, because two different
files used to guess at it:

  * the old Pi launch file (deleted 2026-09-16) hard-coded the D435's colour
    frame rates, so a
    D455 request for 5 fps would have been rejected and a D455 request for 6 fps
    accepted -- librealsense would then fail to resolve it at start-up, which is
    the exact failure that validation exists to prevent.
  * the flattener's `range_min`/`range_max`, slam_toolbox's laser range and
    Nav2's raytrace/obstacle ranges are one decision written in four files. They
    are read from here now, and `RANGE_PARTNERS` below records the
    static yaml still agrees.

Provenance is marked per value. MEASURED means this rig, on the date given;
DATASHEET means Intel's published figure and has not been checked against this
unit; JUDGEMENT means someone chose it and said why.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, Tuple

# 'WxHxFPS' or 'W,H,FPS'. realsense2_camera wants the comma form; the 'x' form
# is accepted on input because it is what upstream's README uses.
_PROFILE_RE = re.compile(r"^\s*(\d+)\s*[xX,]\s*(\d+)\s*[xX,]\s*(\d+)\s*$")


@dataclass(frozen=True)
class StreamProfile:
    """One realsense stream request: width, height, frames per second."""

    width: int
    height: int
    fps: int

    @property
    def as_parameter(self) -> str:
        """The spelling realsense2_camera's `*_profile` parameters take."""
        return f"{self.width},{self.height},{self.fps}"

    def __str__(self) -> str:
        return f"{self.width}x{self.height}x{self.fps}"


@dataclass(frozen=True)
class DepthCamera:
    """What one camera model offers, and the depth band this stack trusts it in.

    `color_fps` / `depth_fps` are the rates the sensor is KNOWN to resolve.
    A rate outside them is refused here rather than at start-up inside
    librealsense, where it surfaces as an opaque "couldn't resolve requests".
    Widening either tuple is a measurement -- `rs-enumerate-devices -c` on the
    device -- not a guess.
    """

    model: str
    color_fps: Tuple[int, ...]
    depth_fps: Tuple[int, ...]
    default_color: StreamProfile
    default_depth: StreamProfile
    min_range_m: float
    # How far a point may be from the camera and still be PUBLISHED in /scan.
    # Points beyond it are dropped by the flattener, not clamped.
    max_range_m: float
    # How far a point may be and still be an OBSTACLE: a slam hit, a costmap
    # mark, a policy detection. Readings in [obstacle_range_m, max_range_m)
    # are "I saw something out there", traced free up to obstacle_range_m and
    # never marked. Split from max_range_m on 2026-09-19 so /scan can show the
    # room while only near, trustworthy returns shape the map.
    obstacle_range_m: float
    depth_fov_deg: float
    color_fov_deg: float

    # -- hits, "clear out to here", and ignored -----------------------------
    # A LaserScan bin can say three things to slam_toolbox (Karto AddScan,
    # /opt/ros/humble/include/karto_sdk/Karto.h ~6151, re-read 2026-09-19):
    # a reading below RangeThreshold is a HIT; one in [RangeThreshold,
    # MaximumRange) is traced FREE up to RangeThreshold ONLY (the endpoint is
    # scaled back to it, AddScan lines 6173-6179) with no hit; one >=
    # MaximumRange (or inf/nan) is IGNORED. slam sets MaximumRange from the
    # message's range_max and RangeThreshold from its max_laser_range
    # parameter, clamped to <= range_max (laser_utils.cpp, humble branch).
    # With both equal to max_range_m the "free, no hit" window is EMPTY --
    # there was no value a bin could carry that meant "clear", so open floor
    # never became free space and the policy had no frontier to walk to.
    #
    # Since 2026-09-19 the window is [obstacle_range_m, max_range_m): a wall
    # at 6 m shows in /scan and in RViz, and the bearing is traced free to
    # obstacle_range_m, but neither slam nor Nav2 marks it. Beyond
    # obstacle_range_m the D435's stereo error (z^2: ~30 cm at 8 m) and a
    # levelling residual of a degree or two (ceai/robots/spot.py, the
    # floor-in-band note) make a return unsafe to call an obstacle, which is
    # what 169aa79 measured at ~3.2 m. KNOWN COST: slam's scan matcher also
    # uses only hits, so the map is built and matched on the near 3 m.

    @property
    def slam_range_threshold_m(self) -> float:
        """slam_toolbox max_laser_range: the hit/free boundary. Strictly
        below max_range_m so that [this, max_range_m) is a non-empty window."""
        return self.obstacle_range_m

    @property
    def raytrace_range_m(self) -> float:
        """Nav2 raytrace_max_range for both costmaps: how far a reading past
        obstacle_range_m clears. Just above obstacle_max_range so marking never
        outruns clearing, and NOT the full max_range_m: a no-return bin cannot
        tell "saw only floor" from "stereo dropout on a blank wall", and clearing
        a 10 m bearing on that evidence would erase real walls."""
        return self.obstacle_range_m + 0.5

    @property
    def no_return_range_m(self) -> float:
        """What an in-wedge bin with no in-band return carries. Sits in
        [slam_range_threshold_m, max_range_m), so slam traces it free without
        marking a hit; above Nav2's obstacle_max_range, so Nav2 clears along it
        (out to raytrace_range_m) without marking either. Asserted in
        RANGE_PARTNERS below. This is the ARC of /scan dots seen
        in RViz around a robot in open space: the designed "nothing seen" value,
        not a return."""
        return self.max_range_m - 0.1

    # -- validation ---------------------------------------------------------
    def parse_profile(self, text: str, stream: str) -> StreamProfile:
        """'424x240x6' -> StreamProfile, or raise saying what this model offers.

        `stream` is "color" or "depth" -- deliberately the spelling of the
        LAUNCH ARGUMENT, so the error message names something the reader can
        type. The two rate tables differ, and inheriting one for the other is
        how a valid depth rate gets rejected.
        """
        rates = self._rates(stream)
        match = _PROFILE_RE.match(text)
        if not match:
            raise ValueError(
                f"{stream}_profile {text!r} is not WxHxFPS "
                f"(e.g. {self.default_for(stream)})")
        width, height, fps = (int(group) for group in match.groups())
        if fps not in rates:
            raise ValueError(
                f"{stream}_profile {text!r} asks for {fps} fps, which the "
                f"{self.model}'s {stream} stream is not known to offer "
                f"({'/'.join(str(rate) for rate in rates)}). librealsense would "
                f"fail to resolve the request and the node would abort at "
                f"start-up. Confirm with `rs-enumerate-devices -c` and widen "
                f"{self.model}'s table in ceai/cameras.py if the rate is real.")
        return StreamProfile(width, height, fps)

    def default_for(self, stream: str) -> StreamProfile:
        return self.default_color if stream == "color" else self.default_depth

    def _rates(self, stream: str) -> Tuple[int, ...]:
        if stream == "color":
            return self.color_fps
        if stream == "depth":
            return self.depth_fps
        raise ValueError(f"stream must be 'color' or 'depth', got {stream!r}")


# The one camera this stack has run. Everything here is either measured on the
# Pi or Intel's datasheet, marked as such.
D435 = DepthCamera(
    model="D435",
    # MEASURED in-tree since 2026-09-07 (the launch file's D435_COLOR_FPS).
    color_fps=(6, 15, 30, 60),
    # MEASURED 2026-09-15: the breakthrough soak ran depth at 6, 15 and 30 fps.
    # The sensor advertises more; this table lists only what has been run here,
    # because a rejected-but-valid rate is a loud error and an accepted-but-
    # invalid one is a start-up abort with an opaque message.
    depth_fps=(6, 15, 30),
    # MEASURED 2026-09-15: the deployable pair. 1.2 % of one Pi core, 74 MB RSS,
    # ~24 Mbit/s for both streams against a ~50 Mbit/s link.
    default_color=StreamProfile(424, 240, 6),
    default_depth=StreamProfile(424, 240, 6),
    # DATASHEET, NOT MEASURED ON THIS UNIT, but CONFIRMED AS CORRECT by the
    # operator on 2026-09-16. Intel puts the D435's Min-Z near 0.28 m at full
    # resolution and its ideal range at 0.3-3 m; 0.3 is that floor rounded up.
    #
    # It replaced 0.8, which slam_toolbox carried on 2026-09-14 "to match the pi
    # value" -- that was the BODY CAMERAS' flattener floor, and it was doing
    # double duty as a filter for Spot's own legs. The D435 looks outward from
    # the front of the body and cannot see the legs, so carrying 0.8 over would
    # have blinded the robot inside 0.8 m for a reason that no longer exists.
    min_range_m=0.3,
    # OPERATOR DECISION 2026-09-19: the band is now TWO numbers.
    #
    # max_range_m 10.0 -- DATASHEET, NOT MEASURED ON THIS UNIT: Intel gives the
    # D435 ~10 m at reduced accuracy. Everything the sensor returns inside it
    # is published in /scan, so RViz shows the room and a bearing with nothing
    # in it carries the no-return fill at 9.9 m (the arc), which slam traces
    # free to obstacle_range_m. History: 4.0 (body-camera era, never measured)
    # -> 8.0 (2026-09-16 experiment, to see far walls) -> 4.0 (169aa79,
    # 2026-09-18, because returns past ~3.2 m were marked occupied in visibly
    # free space) -> 10.0 with the split below, which keeps the far returns
    # and stops marking them.
    max_range_m=10.0,
    # obstacle_range_m 3.0 -- MEASURED 2026-09-18 (169aa79): the distance
    # inside which this unit's returns were trustworthy as obstacles; noise
    # from ~3.2 m out was being painted into the map. Drives slam's
    # max_laser_range, Nav2's obstacle_max_range (raytrace is this + 0.5), the
    # bridge's max_depth and the policy's RPV_MAX_DEPTH_M -- every partner in
    # RANGE_PARTNERS, except
    # config/nav2_spot.yaml's costmap ranges, which are moved by hand and
    # asserted there too.
    # Known cost: the 5x5 m local costmap clips anything past ~2.5 m anyway
    # (2026-09-16), so this reaches Nav2 only via the
    # global costmap.
    obstacle_range_m=3.0,
    # DATASHEET, horizontal. The depth sensor sees WIDER than the colour one,
    # which is why /scan is built from the raw depth stream and not from
    # aligned_depth_to_color: aligning crops depth to the colour frustum and
    # would throw away 18 deg of the only wedge this robot has.
    depth_fov_deg=87.0,
    color_fov_deg=69.0,
)

# Every place `max_range_m` / `obstacle_range_m` has a partner that fails
# SILENTLY when it drifts. Prose in a markdown table has not stopped one of
# these moving yet; the test module named below asserts them.
RANGE_PARTNERS = (
    # follow max_range_m
    "pointcloud_to_laserscan range_min/range_max (launch/robot_onboard.launch.py)",
    "pointcloud_to_laserscan inf_epsilon, i.e. no_return_range_m (launch/robot_onboard.launch.py)",
    # follow obstacle_range_m
    "slam_toolbox min_laser_range/max_laser_range (launch/nav_stack.launch.py)",
    "nav2 raytrace_max_range/obstacle_max_range (config/nav2_spot.yaml)",
    "the bridge's min_depth/max_depth (config/spot_bridge.yaml)",
    "RPV_MAX_DEPTH_M in the policy server's environment (scripts/launch_offboard.sh, scripts/check_offboard.sh)",
)

CAMERAS: Dict[str, DepthCamera] = {
    "d435": D435,
    # The D455 does not run on this Pi yet (breakthrough doc section 7: the
    # control endpoint dies ~1 s after enumeration, suspected current
    # starvation). It is deliberately ABSENT rather than guessed: it appears to
    # use 5 fps where the D435 uses 6, and a wrong table here is worse than no
    # table -- `load()` says so by name.
}


def names():
    """Known camera model names, sorted -- for help text and GUI dropdowns."""
    return sorted(CAMERAS)


def load(model: str) -> DepthCamera:
    """A camera by model name, or a ValueError naming the ones that exist."""
    key = str(model).strip().lower()
    try:
        return CAMERAS[key]
    except KeyError:
        known = ", ".join(sorted(CAMERAS)) or "<none>"
        raise ValueError(
            f"unknown camera model {model!r}; known models: {known}. Add one in "
            f"ceai/cameras.py from real `rs-enumerate-devices -c` output -- an "
            f"invented frame-rate table fails at start-up inside librealsense."
        ) from None
