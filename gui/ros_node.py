"""rclpy bridge between the live ROS graph and the Qt UI.

Runs an rclpy node on a background thread and forwards everything to the UI
through Qt signals, so ROS callbacks never touch widgets. Subscribes:

    /map        nav_msgs/OccupancyGrid   -> the map panel + click->world maths
    /scan       sensor_msgs/LaserScan    -> the 360-degree overlay
    /goal_pose  geometry_msgs/PoseStamped-> where the bridge is sending Spot
    <camera>    Image | CompressedImage  -> optional raw colour feed
    TF map->body                          -> the robot marker

Publishes /goal_pose for the manual click-to-go override.

FRAMES ARE SPOT'S: base frame `body`, not base_link; odometry is `vision`. The
`/scan` overlay is transformed with the LATEST map->body transform rather than
the transform at the scan's own stamp -- this is a picture for a human standing
next to the robot, not a measurement, and pinning it to the stamp would make it
disappear whenever TF lagged. The bridge's own scan/frame pairing is stamp-exact
and unaffected by anything here.

ROS is import-guarded: without rclpy the console still runs and the policy
/status panels still work.
"""
from __future__ import annotations

import math
import threading
from typing import Optional, Tuple

import numpy as np

from .config import Config
from .qtcompat import QtCore, Signal

ROS_AVAILABLE = True
ROS_IMPORT_ERROR = ""
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from sensor_msgs.msg import CompressedImage, Image, LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception as e:                                  # pragma: no cover
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = f"{type(e).__name__}: {e}"


# --------------------------------------------------------------------------- #
# pure helpers -- no ROS types, so they are testable without a ROS install
# --------------------------------------------------------------------------- #
def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def scan_to_points(ranges, angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, stride: int = 2):
    """LaserScan ranges -> sensor-frame (x, y), dropping invalid returns.

    720 bins at up to 15 Hz is more than a map overlay needs, so `stride` thins
    it; 2 keeps a 1-degree picture of a 0.5-degree scan. inf/NaN and anything
    outside [range_min, range_max] are dropped. Since 2026-09-17 the Spot
    flattener fills unreached bins with a value just inside range_max, so they
    plot as a faint arc at ~7.9 m; +inf is still dropped for any other source.
    """
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return np.empty((0, 2))
    idx = np.arange(0, r.size, max(1, int(stride)))
    r = r[idx]
    ang = angle_min + idx * angle_increment
    good = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, ang = r[good], ang[good]
    return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)


def transform_points(points, x: float, y: float, yaw: float):
    """Rotate by yaw then translate -- sensor frame to map frame."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.empty((0, 2))
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T + np.array([x, y])


def grid_from_msg_fields(data, width: int, height: int, resolution: float,
                         origin_x: float, origin_y: float) -> dict:
    """The dict shape the map panel consumes. Separated so it can be built in a
    test without constructing an OccupancyGrid message."""
    return {
        "data": np.asarray(data, dtype=np.int8).reshape(height, width),
        "resolution": float(resolution),
        "origin_x": float(origin_x),
        "origin_y": float(origin_y),
        "width": int(width),
        "height": int(height),
    }


# --------------------------------------------------------------------------- #
class RosBridge(QtCore.QObject):
    """Qt-facing facade. Owns the rclpy node and its spin thread."""

    map_received = Signal(object)                  # grid dict
    pose_received = Signal(float, float, float)    # x, y, yaw (map frame)
    goal_received = Signal(float, float, float)
    scan_received = Signal(object)                 # Nx2 map-frame points
    camera_received = Signal(object)
    available = Signal(bool)

    def __init__(self, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._goal_pub = None
        self._stopping = False
        self._pose: Optional[Tuple[float, float, float]] = None

    def start(self) -> None:
        if not ROS_AVAILABLE:
            self.available.emit(False)
            return
        self._thread = threading.Thread(target=self._spin, name="ros-bridge", daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node("rpv_console")
            self._setup(self._node)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._safe_emit(True)
            self._executor.spin()
        except Exception as e:                      # pragma: no cover
            if self._stopping:
                return
            print(f"[ros] spin stopped: {e}")
            self._safe_emit(False)

    def _safe_emit(self, ok: bool) -> None:
        # During teardown the underlying C++ object can be deleted out from
        # under the spin thread; swallow that rather than kill the thread.
        try:
            self.available.emit(ok)
        except RuntimeError:
            pass

    def _setup(self, node) -> None:
        # /map is latched (transient local, depth 1). Match it or the map only
        # appears on the next SLAM update, which can be a minute away.
        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(OccupancyGrid, self.cfg.map_topic, self._on_map, map_qos)

        # BEST_EFFORT: it matches a RELIABLE publisher too, so this works whether
        # /scan is built on the Pi and republished by zenoh. The
        # reverse (RELIABLE here) would silently receive nothing.
        scan_qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        if self.cfg.show_scan:
            node.create_subscription(LaserScan, self.cfg.scan_topic, self._on_scan, scan_qos)

        node.create_subscription(PoseStamped, self.cfg.goal_topic, self._on_goal, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        node.create_timer(0.2, self._on_tf_timer)

        # Same BEST_EFFORT reasoning as /scan above: the bridge reads this
        # topic with a sensor-data profile and gets frames, and a RELIABLE
        # subscriber against a best-effort publisher (realsense2_camera, or
        # zenoh's republication of it) matches nothing and shows a black panel
        # with no error (2026-09-19). The depth is shared with scan_qos.
        cam = self.cfg.live_camera_topic
        if cam.rstrip("/").endswith("compressed"):
            node.create_subscription(CompressedImage, cam, self._on_compressed, scan_qos)
        else:
            node.create_subscription(Image, cam, self._on_image, scan_qos)

        self._goal_pub = node.create_publisher(PoseStamped, self.cfg.goal_topic, 10)

    # --- subscriptions --------------------------------------------------- #
    def _on_map(self, msg) -> None:
        info = msg.info
        self.map_received.emit(grid_from_msg_fields(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
        ))

    def _on_tf_timer(self) -> None:
        try:
            t = self._tf_buffer.lookup_transform(
                self.cfg.map_frame, self.cfg.base_frame, rclpy.time.Time()
            )
        except TransformException:
            return
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._pose = (float(tr.x), float(tr.y), yaw)
        self.pose_received.emit(*self._pose)

    def _on_scan(self, msg) -> None:
        if self._pose is None:
            return                     # nothing to place it against yet
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max)
        self.scan_received.emit(transform_points(pts, *self._pose))

    def _on_goal(self, msg) -> None:
        q = msg.pose.orientation
        self.goal_received.emit(float(msg.pose.position.x), float(msg.pose.position.y),
                                quat_to_yaw(q.x, q.y, q.z, q.w))

    def _on_compressed(self, msg) -> None:
        self.camera_received.emit({"jpeg": bytes(msg.data)})

    def _on_image(self, msg) -> None:
        enc = (msg.encoding or "").lower()
        if enc not in ("rgb8", "bgr8"):
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if enc == "bgr8":
            arr = arr[:, :, ::-1]
        self.camera_received.emit({"rgb": np.ascontiguousarray(arr).tobytes(),
                                   "w": int(msg.width), "h": int(msg.height)})

    # --- publish ---------------------------------------------------------- #
    def publish_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        if self._goal_pub is None or self._node is None:
            return False
        msg = PoseStamped()
        msg.header.frame_id = self.cfg.map_frame
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(msg)
        return True

    def shutdown(self) -> None:
        self._stopping = True
        try:
            if self._executor is not None:
                self._executor.shutdown()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
