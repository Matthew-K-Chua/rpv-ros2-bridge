"""2D LiDAR scan -> one depth value per image column (the "ranging curtain").

Each return is transformed into the RGB optical frame and projected to a column,
which stores its forward distance (optical z -- what the policy's pinhole
back-projection expects). Every row of the resulting image would be identical,
because the value and object maps are 2D and only bearing plus range matter. That
is why only one row travels and the server tiles it.

Correct for a detection at the scan plane's height, wrong for anything above or
below it: a picture on a wall behind a chair gets the chair's range.

Columns with no usable return are left at max_depth, which the policy reads as
invalid.
"""
import numpy as np

from ..n2np_helpers import transform_to_matrix

# The TurtleBot 4 URDF's optical frame is rotated 180 deg about its viewing axis
# relative to the true optical convention, which mirrors bearings left/right.
# Re-derive the column geometry before reusing this for another camera.
_OPTICAL_Z_FIX = np.diag([-1.0, -1.0, 1.0, 1.0])


class CameraLidarTransform:
    """Caches the static lidar -> camera-optical matrix, looked up once."""

    def __init__(self, tf_buffer, lidar_frame, camera_optical_frame, logger=None):
        self._tf_buffer = tf_buffer
        self.lidar_frame = lidar_frame
        self.camera_optical_frame = camera_optical_frame
        self._logger = logger
        self.matrix = None

    def ensure(self) -> bool:
        if self.matrix is not None:
            return True
        import rclpy.time
        import tf2_ros
        try:
            tf = self._tf_buffer.lookup_transform(
                self.camera_optical_frame, self.lidar_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return False
        self.matrix = _OPTICAL_Z_FIX @ transform_to_matrix(tf.transform)
        if self._logger:
            self._logger.info(
                f"Cached {self.lidar_frame}->{self.camera_optical_frame} transform "
                f"(+180 deg optical-Z correction).")
        return True


class CurtainProjector:
    """Builds the depth row. Pure given a scan, intrinsics and the transform."""

    def __init__(self, config, intrinsics, transform, logger=None):
        self._config = config
        self._intrinsics = intrinsics
        self._transform = transform
        self._logger = logger

    def _warn(self, message):
        if self._logger:
            self._logger.warn(message, throttle_duration_sec=2.0)

    def row(self, width, scan):
        """(width,) float32 metres, or None if the curtain cannot be built.

        The funnel counts in the warnings make it obvious WHERE returns were
        lost: extrinsics, intrinsics/FOV, or the depth band.
        """
        if scan is None or not self._intrinsics.ready or not self._transform.ensure():
            return None

        min_depth, max_depth = self._config.depth_band
        fx, cx = self._intrinsics.fx, self._intrinsics.cx

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n_raw = ranges.shape[0]
        angles = scan.angle_min + np.arange(n_raw) * scan.angle_increment
        usable = (np.isfinite(ranges) & (ranges > scan.range_min)
                  & (ranges < scan.range_max))
        ranges, angles = ranges[usable], angles[usable]
        if ranges.size == 0:
            self._warn(f"LiDAR curtain EMPTY: 0/{n_raw} finite returns in "
                       f"[{scan.range_min:.2f},{scan.range_max:.2f}]m (scan all inf?).")
            return None
        n_good = ranges.size

        # Planar laser points (REP-103: x forward, y left) -> camera optical.
        points = np.column_stack([
            ranges * np.cos(angles),
            ranges * np.sin(angles),
            np.zeros_like(ranges),
            np.ones_like(ranges),
        ]) @ self._transform.matrix.T

        forward = points[:, 2]
        in_front = forward > 1e-3
        points, forward = points[in_front], forward[in_front]
        if forward.size == 0:
            self._warn(f"LiDAR curtain EMPTY: {n_good} returns but 0 in front of camera "
                       f"(z>0) -> check {self._transform.lidar_frame}->"
                       f"{self._transform.camera_optical_frame} extrinsics.")
            return None
        n_front = forward.size

        cols = np.round(cx + fx * points[:, 0] / forward).astype(int)
        on_image = (cols >= 0) & (cols < width)
        cols, forward = cols[on_image], forward[on_image]
        if forward.size == 0:
            self._warn(f"LiDAR curtain EMPTY: {n_front} returns in front but 0 project "
                       f"into the {width}px image -> check intrinsics "
                       f"(fx={fx:.1f}, cx={cx:.1f}) / FOV.")
            return None
        n_on_image = forward.size

        # Nearest return per column: the closer obstacle is the one that matters.
        col_depth = np.full(width, np.inf, dtype=np.float32)
        np.minimum.at(col_depth, cols, forward)

        col_depth = self._dilate(col_depth)

        # Outside the usable band the policy treats a reading as invalid anyway;
        # leaving it at max_depth places no phantom object.
        col_depth[col_depth <= min_depth] = np.inf
        col_depth[col_depth >= max_depth] = np.inf

        row = np.full(width, float(max_depth), dtype=np.float32)
        valid = np.isfinite(col_depth)
        if not valid.any():
            self._warn(f"LiDAR curtain EMPTY: {n_on_image} returns in image but all "
                       f"outside the [{min_depth:.2f},{max_depth:.2f}]m band.")
        elif self._logger:
            hits = col_depth[valid]
            self._logger.info(
                f"LiDAR curtain: {int(valid.sum())}/{width} cols valid "
                f"(raw {n_raw} -> band {n_good} -> front {n_front} -> img {n_on_image}); "
                f"depth min/med/max {hits.min():.2f}/"
                f"{float(np.median(hits)):.2f}/{hits.max():.2f}m.",
                throttle_duration_sec=2.0)
        row[valid] = col_depth[valid]
        return row

    def _dilate(self, col_depth):
        """Widen each hit horizontally to bridge LiDAR sparsity between columns."""
        pad = self._config.lidar_col_pad
        if pad <= 0:
            return col_depth
        base = col_depth.copy()
        for shift in range(1, pad + 1):
            col_depth[shift:] = np.minimum(col_depth[shift:], base[:-shift])
            col_depth[:-shift] = np.minimum(col_depth[:-shift], base[shift:])
        return col_depth

    def raster(self, height, width, scan):
        """The row broadcast down every line. For callers that want the image."""
        row = self.row(width, scan)
        if row is None:
            return None
        return np.broadcast_to(row, (height, width)).copy()
