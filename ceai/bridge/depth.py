"""Depth for the policy: one strategy per `depth_source`.

Each returns the payload fields for a step, or None to skip it. Exactly one of
depth_png16 / depth_row is ever set; the server prefers depth_png16 and needs no
depth_shape beside it, because a PNG carries its own.

  realsense  the D435's aligned depth. Real metric depth, registered to the
             colour frame BY THE CAMERA, so the stamp and the shape match by
             construction -- which is why a shape mismatch is treated as a
             wiring error here and merely as a warning for zoedepth.
  zoedepth   monocular metric depth, for a robot with no depth sensor. Needs the
             policy's RPV_MAX_DEPTH_M to match the ZoeDepth node's trust_max_m.
  lidar      the /scan ranging curtain (see curtain.py).
  stub       a flat plane. Diagnostics only.

RPV_MAX_DEPTH_M is BOTH the [0,1] normalisation divisor AND the value the
server reads as "invalid", for every source above. A mismatch silently discards
detections beyond it rather than erroring.
"""
import numpy as np

from ..n2np_helpers import array_to_b64
from .rpv import encode_png16_mm


class DepthSource:
    """Common interface: fields(frame_stamp, height, width) -> dict or None."""

    needs_scan = False

    def __init__(self, config, logger=None):
        self._config = config
        self._logger = logger

    def _warn(self, message):
        if self._logger:
            self._logger.warn(message, throttle_duration_sec=2.0)

    def _warn_rarely(self, message):
        if self._logger:
            self._logger.warn(message, throttle_duration_sec=10.0)

    def ready(self) -> bool:
        raise NotImplementedError

    def fields(self, frame_stamp, height, width):
        raise NotImplementedError


class RasterDepthSource(DepthSource):
    """A dense 16UC1 millimetre raster, sent as a 16-bit PNG of millimetres.

    The encoder is imported from the rpv-sim2real-policy checkout, next to the decoder the
    policy server uses, so the two ends cannot drift apart.

    Subclasses differ only in how they describe themselves when something is
    missing, and in whether a raster whose shape disagrees with the colour frame
    is survivable.
    """

    #: Name used in log messages.
    label = "depth"
    #: What to tell the operator when no raster has arrived.
    missing_hint = ""
    #: True when a shape mismatch means the two streams are different cameras
    #: rather than merely a different resolution the server can resize.
    shape_must_match = False
    #: Whether the raster's stamp equals the colour frame's exactly (a node
    #: that copies the stamp) or is merely close (two hardware clocks). See
    #: `_stamp_agrees` before tightening `scan_max_dt` on the strength of it.
    stamp_matches_exactly = False

    def __init__(self, config, buffer, logger=None):
        super().__init__(config, logger)
        self._buffer = buffer

    def ready(self) -> bool:
        return bool(self._buffer)

    def fields(self, frame_stamp, height, width):
        mm, dt = self._buffer.nearest(frame_stamp)
        if mm is None:
            self._warn(f"No {self.label} raster yet; skipping this step. "
                       f"{self.missing_hint}")
            return None
        if not self._stamp_agrees(dt):
            return None
        if mm.shape != (height, width) and not self._shape_agrees(mm.shape, height, width):
            return None
        return {"depth_png16": encode_png16_mm(mm)}

    def _stamp_agrees(self, dt) -> bool:
        """The raster must describe the same instant as the colour frame.

        A step built from two instants places its detections at a pose the robot
        has already left, and nothing downstream can tell.

        HOW EXACT "the same instant" IS DIFFERS BY SOURCE, and the distinction
        matters if you ever tighten `scan_max_dt`:

          zoedepth   the depth node copies the colour frame's stamp verbatim, so
                     a match is dt == 0 EXACTLY and anything else is a dropped
                     frame.
          realsense  each stream carries its OWN hardware timestamp. With
                     enable_sync the two come from one frameset and the gap is
                     sub-frame, but it is NOT zero. UNVERIFIED how large it
                     actually is on this device -- measure before assuming a
                     tight bound. The default 0.5 s gate is generous on purpose.
        """
        max_dt = self._config.scan_max_dt
        if 0.0 < max_dt < dt:
            self._warn(f"Nearest {self.label} raster is {dt:.2f}s from the RGB frame "
                       f"(> {max_dt:.2f}s) -- a frame was dropped. Skipping step.")
            return False
        return True

    def _shape_agrees(self, shape, height, width) -> bool:
        """Whether a differing raster shape is survivable. Logs either way."""
        if self.shape_must_match:
            self._warn(
                f"{self.label} raster is {shape} but the RGB frame is "
                f"{(height, width)}. With hardware alignment these are the same "
                f"camera and must agree -- this is a wiring error, not a "
                f"resolution choice. Check that depth_topic is the ALIGNED "
                f"depth and that both streams run one profile. Skipping step.")
            return False
        # Survivable: the server nearest-resizes. Still worth saying, because it
        # means the depth node and this bridge are reading different streams.
        self._warn_rarely(
            f"{self.label} raster is {shape} but the RGB frame is "
            f"{(height, width)}; the policy will resize. Check that both read "
            f"the same camera.")
        return True


class RealSenseDepthSource(RasterDepthSource):
    """The D435's `aligned_depth_to_color` stream, straight off the wire."""

    label = "D435 aligned depth"
    missing_hint = ("Is the camera up with align_depth.enable:=true, and is "
                    "depth_topic the aligned topic rather than "
                    "depth/image_rect_raw?")
    # Hardware alignment makes the two streams one camera, so a disagreement is
    # a wiring error rather than a resolution the server can resize around.
    shape_must_match = True
    # Sub-frame, not zero -- see _stamp_agrees.
    stamp_matches_exactly = False


class ZoeDepthSource(RasterDepthSource):
    """Monocular metric depth, produced by the ZoeDepth chain in the AI repo."""

    label = "ZoeDepth"
    missing_hint = ("Is ros_nodes/zoedepth_chain.launch.py running, and the "
                    "model server behind it?")
    # The model genuinely can run at another resolution; the server resizes.
    shape_must_match = False
    stamp_matches_exactly = True


class LidarCurtainSource(DepthSource):

    needs_scan = True

    def __init__(self, config, projector, logger=None):
        super().__init__(config, logger)
        self._projector = projector
        self.scan = None            # set per step by the payload builder

    def ready(self) -> bool:
        return True

    def fields(self, frame_stamp, height, width):
        row = self._projector.row(width, self.scan)
        if row is None:
            # Wait rather than run on a fabricated flat plane -- that was the old
            # "robot heads off in a weird direction" failure.
            self._warn("LiDAR depth unavailable (scan/TF/intrinsics not ready); "
                       "skipping this step.")
            return None
        return {"depth_row": array_to_b64(row), "depth_shape": [height, width]}


class StubDepthSource(DepthSource):

    def ready(self) -> bool:
        return True

    def fields(self, frame_stamp, height, width):
        row = np.full(width, float(self._config.depth_stub_m), dtype=np.float32)
        return {"depth_row": array_to_b64(row), "depth_shape": [height, width]}


def make_depth_source(config, depth_buffer, projector, logger=None):
    if config.depth_source == "realsense":
        return RealSenseDepthSource(config, depth_buffer, logger)
    if config.depth_source == "zoedepth":
        return ZoeDepthSource(config, depth_buffer, logger)
    if config.depth_source == "lidar":
        return LidarCurtainSource(config, projector, logger)
    return StubDepthSource(config, logger)
