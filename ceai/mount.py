"""The camera's measured mount pose, and the all-or-nothing rule around it.

The six numbers describing where the D435 sits on the robot are load-bearing for
NAVIGATION again: pointcloud_to_laserscan cannot reach the robot's base frame
without body -> camera_link, so a missing or wrong transform means `/scan` never
appears (missing) or appears tilted (wrong). A tilted scan maps a sloping floor
as a wall and there is no error anywhere.

"""
from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Dict, Optional

#: Argument name -> what it means. The order is the order a person measures in.
DESCRIPTIONS = {
    "x": "metres, base -> camera_link, forward positive",
    "y": "metres, base -> camera_link, left positive",
    "z": "metres, base -> camera_link, up positive",
    "roll": "radians",
    "pitch": "radians, POSITIVE PITCHES THE LENS DOWN",
    "yaw": "radians",
}

#: The launch-argument spelling of each field.
PREFIX = "d435_"


@dataclass(frozen=True)
class MountPose:
    """A rigid transform from the robot's base frame to `camera_link`.

    `camera_link` and not an optical frame: realsense2_camera publishes the rest
    of the camera tree itself (camera_link -> camera_depth_frame ->
    camera_*_optical_frame), and attaching to an optical frame would give it two
    parents and break TF.
    """

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    @classmethod
    def argument_names(cls):
        return [PREFIX + f.name for f in fields(cls)]

    @classmethod
    def from_strings(cls, values: Dict[str, str]) -> Optional["MountPose"]:
        """Build from launch-argument strings keyed WITHOUT the prefix.

        Returns None when every value is blank -- "no mount pose given", which
        the caller decides how to treat. Raises when some are given and some are
        not, and when one is not a number.
        """
        given = {name: text.strip() for name, text in values.items()}
        supplied = [name for name, text in given.items() if text]
        if not supplied:
            return None
        if len(supplied) != len(DESCRIPTIONS):
            missing = [name for name in DESCRIPTIONS if not given.get(name)]
            raise ValueError(
                "the camera mount pose is all-or-nothing: got "
                f"{', '.join(PREFIX + n for n in supplied)} but not "
                f"{', '.join(PREFIX + n for n in missing)}. Supply all six "
                "MEASURED values or none -- a defaulted component is a guess, "
                "and a guessed pitch tilts everything both Nav2 and the policy "
                "see.")
        try:
            return cls(**{name: float(text) for name, text in given.items()})
        except ValueError as exc:
            raise ValueError(f"mount pose values must be numbers: {exc}") from None

    def static_transform_args(self, parent_frame: str,
                              child_frame: str = "camera_link"):
        """argv for tf2_ros static_transform_publisher, long-flag form.

        The positional form of that executable changed between distros and
        silently reorders roll/pitch/yaw; the flags have not moved.
        """
        return [
            "--x", repr(self.x), "--y", repr(self.y), "--z", repr(self.z),
            "--roll", repr(self.roll), "--pitch", repr(self.pitch),
            "--yaw", repr(self.yaw),
            "--frame-id", parent_frame, "--child-frame-id", child_frame,
        ]

    def __str__(self) -> str:
        return (f"x={self.x} y={self.y} z={self.z} "
                f"roll={self.roll} pitch={self.pitch} yaw={self.yaw}")
