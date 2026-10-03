"""Sensor intake: the frame cache, the stamped buffers, and the motion gate.

Every stamp handled here is the ROBOT's clock, so comparisons between them are
free of PC<->robot skew. The one place that compares against the PC clock is the
staleness gate in PayloadBuilder.

Why the buffers keep a history rather than just the newest message, and why the
depth buffer is much shorter than the scan buffer.
"""
import math
import threading
from collections import deque

import cv2
import numpy as np


class Intrinsics:
    """The RGB pinhole model, from CameraInfo K = [fx 0 cx; 0 fy cy; 0 0 1]."""

    def __init__(self):
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_w = None

    def update(self, msg):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.img_w = int(msg.width)

    @property
    def ready(self) -> bool:
        return self.fx is not None and self.cx is not None


class FrameCache:
    """The newest compressed RGB frame, with the stamp it was captured at.

    Image and stamp are stored as ONE tuple: the staleness gate and the
    pose-at-stamp lookup are only sound if they refer to the same capture, and a
    reader on the control thread must never see a new image against an old stamp.

    The bytes are cached undecoded and forwarded to the policy verbatim. Only
    about one frame in 30 is used, so decoding every one wasted ~29 decodes per
    step, and re-encoding put the frame through a second lossy JPEG generation
    before SAM3 ever saw it.
    """

    def __init__(self, logger=None):
        self.latest = None          # (jpeg bytes, rclpy.time.Time)
        self._hw = None
        self._logger = logger

    def update(self, msg, to_time):
        # bytes() copies out of the message buffer; the message is not ours to
        # keep once the callback returns.
        self.latest = (bytes(msg.data), to_time(msg.header.stamp))

    def shape(self, jpeg):
        """(h, w) of the stream, decoding at most once per process.

        The payload needs the shape but not the pixels, and the stream's
        resolution is fixed for the life of a run.
        """
        if self._hw is None:
            img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return None
            self._hw = img.shape[:2]
            if self._logger:
                self._logger.info(f"RGB stream is {self._hw[1]}x{self._hw[0]}.")
        return self._hw


class StampedBuffer:
    """Recent (stamp, item) pairs, searchable by capture time.

    The camera lags the other streams, so by the time a frame arrives the NEWEST
    scan or raster describes a different instant. Steps must be built from the
    one taken at the frame's own stamp, which is why old ones are kept.
    """

    def __init__(self, maxlen):
        self._items = deque(maxlen=int(maxlen))
        self._lock = threading.Lock()

    def append(self, stamp, item):
        with self._lock:
            self._items.append((stamp, item))

    def nearest(self, when):
        """Item closest in capture time to `when` -> (item, dt_seconds).

        (None, None) when the buffer is empty.
        """
        with self._lock:
            snapshot = list(self._items)
        best, best_dt = None, None
        for stamp, item in snapshot:
            dt = abs((stamp - when).nanoseconds) * 1e-9
            if best_dt is None or dt < best_dt:
                best, best_dt = item, dt
        return best, best_dt

    def __len__(self):
        return len(self._items)

    def __getitem__(self, index):
        return self._items[index]

    def __bool__(self):
        return bool(self._items)

    @property
    def maxlen(self):
        return self._items.maxlen


class MotionTracker:
    """Is the base moving, and when exactly did it last stop?

    The stop is dated by the odom message's own stamp, recorded as the
    moving->still edge happens. Polling for it instead would date the stop to
    whenever the control thread got round to looking -- wrong once a policy step
    can run in between, because the frame wait would then restart from "now" and
    pay another settle for a base that stopped seconds ago.
    """

    def __init__(self, lin_eps, ang_eps, still_hold=0.0):
        self.lin_eps = float(lin_eps)
        self.ang_eps = float(ang_eps)
        # OPT-IN (0.0 = off, the pre-2026-09-19 behaviour). With a hold, the
        # moving->still edge is only a CANDIDATE stop: it is dated as the stop
        # once the base has stayed still for this long by the odom stamps, and
        # discarded if motion resumes first. Spot turns in place by stepping,
        # and its reported yaw rate dips below ang_eps between steps, so the
        # bare edge fired MID-SPIN and the "frame after the stop" wait was
        # satisfied by a frame captured while still rotating (2026-09-19,
        # no_nav_again.webm: 12 ms between Nav2's "spin completed" and the
        # next spin, i.e. no wait at all). Disable with -p motion_still_hold:=0.0
        # or by deleting the line from config/spot_bridge.yaml.
        self.still_hold = float(still_hold)
        self.still = None           # None until the first sample
        self.stop_stamp = None
        self.latest = None
        self._still_since = None    # stamp of the edge awaiting its hold

    def update(self, msg, to_time):
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        still = (math.hypot(v.x, v.y) < self.lin_eps and abs(w.z) < self.ang_eps)
        if still and self.still is False:
            stamp = to_time(msg.header.stamp)
            if self.still_hold > 0.0:
                self._still_since = stamp
            else:
                self.stop_stamp = stamp
        elif still and self._still_since is not None:
            held = (to_time(msg.header.stamp) - self._still_since).nanoseconds * 1e-9
            if held >= self.still_hold:
                # Dated to the EDGE, not to now: the base stopped then, and a
                # frame captured settle_time after it is what the caller wants.
                self.stop_stamp = self._still_since
                self._still_since = None
        elif not still:
            self._still_since = None    # a dip between steps, not a stop
        self.still = still
        self.latest = msg

    def expect_new_stop(self):
        """Forget the last stop, so the next one must be freshly observed."""
        self.stop_stamp = None
        self._still_since = None
