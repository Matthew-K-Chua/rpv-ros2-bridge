"""Assembling the policy payload, and refusing to build a bad one.

The field names here are read by the policy server in the other repo; the
contract is documented in the README.

The frame and the pose are pinned to the SAME instant. An image older than
frame_max_age is rejected outright, and the pose is looked up at the image's own
capture stamp rather than at "now" -- pairing a 100-300 ms old frame with a "now"
yaw is a ~10-15 deg bearing error on every object placed during a spin, and the
server's brg_err diagnostic cannot see it, because it compares the image column
against a point placed with that same wrong yaw.
"""
import base64


class PayloadBuilder:

    def __init__(self, config, clock, frames, scans, depth_source, occupancy,
                 intrinsics, episodic, pose_lookup, logger=None):
        self._config = config
        self._clock = clock
        self._frames = frames
        self._scans = scans
        self._depth = depth_source
        self._occupancy = occupancy
        self._intrinsics = intrinsics
        self._episodic = episodic
        self._pose_at = pose_lookup
        self._logger = logger

    def _warn(self, message, throttle=2.0):
        if self._logger:
            self._logger.warn(message, throttle_duration_sec=throttle)

    def sensors_ready(self) -> bool:
        """Cheap probe for the control loop's startup wait.

        build() is not a predicate: polling it would base64-encode the map and
        project a curtain every 200 ms just to learn "not yet".
        """
        return (self._frames.latest is not None
                and self._occupancy.ready
                and self._intrinsics.ready
                and self._episodic.pinned
                and (not self._depth.needs_scan or len(self._scans) > 0)
                and self._depth.ready())

    def build(self, is_first=False, finish_init=False, frame=None, full_occ=False):
        """The payload dict, or None to skip this step.

        `frame` pins the (jpeg, stamp) to reason about instead of whatever is
        newest: the init loop captures a frame while the base is still, then
        starts the next rotation, so "newest" would be a blurred one from the
        middle of that rotation.
        """
        if frame is None:
            frame = self._frames.latest
        if frame is None or not self._occupancy.ready:
            return None
        if not self._intrinsics.ready or not self._episodic.pinned:
            return None
        jpeg, stamp = frame

        if not self._frame_is_fresh(stamp):
            return None

        scan = self._pick_scan(stamp)
        if scan is False:
            return None

        pose = self._pose_at(stamp)
        if pose is None:
            return None

        shape = self._frames.shape(jpeg)
        if shape is None:
            self._warn("RGB frame did not decode; skipping this step.")
            return None
        height, width = shape

        if self._depth.needs_scan:
            self._depth.scan = scan
        depth_fields = self._depth.fields(stamp, height, width)
        if depth_fields is None:
            return None

        occupancy_fields = self._occupancy.fields(force=full_occ or is_first)
        robot_x, robot_y, robot_yaw = pose
        start_x, start_y, start_yaw = self._episodic.origin

        return {
            # The camera's own JPEG, untouched. OpenCV decodes it to BGR, so say
            # so rather than paying a decode + re-encode to hand over RGB.
            "rgb": base64.b64encode(jpeg).decode("utf-8"),
            "rgb_order": "bgr",
            **depth_fields,
            **occupancy_fields,
            "robot_map_x": robot_x,
            "robot_map_y": robot_y,
            "robot_map_yaw": robot_yaw,
            "start_x": start_x,
            "start_y": start_y,
            "start_yaw": start_yaw,
            "fx": self._intrinsics.fx,
            "fy": self._intrinsics.fy,
            "img_w": self._intrinsics.img_w,
            "min_depth": self._config.min_depth,
            "max_depth": self._config.max_depth,
            "camera_height": self._config.camera_height,
            "objectgoal": self._config.target,
            "is_first": is_first,
            "finish_init": finish_init,
        }

    def _frame_is_fresh(self, stamp) -> bool:
        """Liveness only.

        Checked before the TF lookup: no point resolving a pose at a stamp that
        is about to be thrown away.
        """
        max_age = self._config.frame_max_age
        age = (self._clock.now() - stamp).nanoseconds * 1e-9
        if 0.0 < max_age < age:
            self._warn(f"Stale RGB frame ({age:.2f}s > {max_age:.2f}s); skipping step. "
                       f"If this never clears, suspect PC<->robot clock skew, not "
                       f"latency.")
            return False
        if self._logger:
            self._logger.info(f"frame age {age:.3f}s", throttle_duration_sec=5.0)
        return True

    def _pick_scan(self, stamp):
        """The scan contemporary with the frame, None if not needed, False to skip.

        Not the NEWEST scan: the LiDAR runs ahead of the camera on this link, so
        the newest one ranges a different bearing.
        """
        if not self._depth.needs_scan:
            return None
        scan, dt = self._scans.nearest(stamp)
        if scan is None:
            self._warn("No LiDAR scan yet; skipping this step.")
            return False
        max_dt = self._config.scan_max_dt
        if 0.0 < max_dt < dt:
            # Usually the buffer is too short to reach back to the frame's stamp
            # -- compare scan_buffer_len against the camera delay.
            self._warn(f"Nearest LiDAR scan is {dt:.2f}s from the RGB frame "
                       f"(> {max_dt:.2f}s); skipping step.")
            return False
        return scan
