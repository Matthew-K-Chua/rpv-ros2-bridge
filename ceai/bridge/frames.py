"""TF: the pinned episodic frame, and the robot's pose at a given instant.

The episodic frame is fixed at the robot's start pose and all VLFM maths lives in
it. The server owns the map<->episodic conversion; the bridge only pins the
origin and forwards raw map-frame poses.
"""
import rclpy.time
import tf2_ros

from ..n2np_helpers import make_T_map_to_epi, yaw_from_quat


class EpisodicFrame:
    """Pinned once from the first available map->robot transform."""

    def __init__(self, tf_buffer, map_frame, robot_frame, logger=None):
        self._tf_buffer = tf_buffer
        self.map_frame = map_frame
        self.robot_frame = robot_frame
        self._logger = logger
        self.start_xy = None
        self.start_yaw = None
        self.map_to_epi = None
        self.epi_to_map = None

    @property
    def pinned(self) -> bool:
        return self.map_to_epi is not None

    @property
    def origin(self):
        """(x, y, yaw) of the episodic origin in the map frame."""
        return self.start_xy[0], self.start_xy[1], self.start_yaw

    def ensure(self) -> bool:
        if self.pinned:
            return True
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return False
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)
        self.start_xy = (x, y)
        self.start_yaw = yaw
        self.map_to_epi, self.epi_to_map = make_T_map_to_epi(x, y, yaw)
        if self._logger:
            self._logger.info(
                f"Episodic frame pinned to start pose ({x:.2f}, {y:.2f}, yaw={yaw:.2f}).")
        return self.pinned


class PoseLookup:
    """The robot's map-frame pose at a capture stamp."""

    def __init__(self, tf_buffer, map_frame, robot_frame, timeout, logger=None):
        self._tf_buffer = tf_buffer
        self.map_frame = map_frame
        self.robot_frame = robot_frame
        self._timeout = timeout
        self._logger = logger

    def __call__(self, stamp=None):
        """(x, y, yaw) at `stamp`, or None.

        The lookup waits: over zenoh, TF routinely lags the image stream, so the
        frame's stamp is briefly in the buffer's future and a zero-timeout lookup
        would raise and silently skip the step -- possibly every step.
        """
        if stamp is None:
            stamp = rclpy.time.Time()
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, stamp, timeout=self._timeout)
        except tf2_ros.ExtrapolationException as exc:
            # Distinct from "no TF at all": this is TF lag. Raise
            # tf_lookup_timeout or frame_max_age if it keeps firing.
            self._warn(f"TF lags the camera stamp: {exc}")
            return None
        except tf2_ros.TransformException as exc:
            self._warn(f"No {self.map_frame}->{self.robot_frame} TF: {exc}")
            return None
        return (tf.transform.translation.x,
                tf.transform.translation.y,
                yaw_from_quat(tf.transform.rotation))

    def _warn(self, message):
        if self._logger:
            self._logger.warn(message, throttle_duration_sec=2.0)
