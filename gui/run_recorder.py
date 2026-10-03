"""What the Record button saves BESIDES the rosbag.

One run, one name, four artefacts:

    <bag_dir>/<name>/                        ros2 bag         (process_manager)
    records/logs/<name>.log                  the policy's decisions, one line per step
    records/videos/value_map/<name>.mp4      the value map, as the console showed it
    records/videos/policy/<name>.mp4         the camera frame annotated by the policy

`name` is the rosbag's directory name (`<stamp>_<target>`), so the four line up
by filename alone. Every log line carries wall-clock seconds, which is what the
policy server's own `logs/rpv_policy_*.log` ([state]/[event] lines) is stamped
with, so a step in one can be found in the other.

The frames come from the same GET /status snapshots that drive the console's
panels: `value_map_jpeg_bytes` and `annotated_rgb_jpeg_bytes`. The server only
attaches them every STATUS_IMAGE_EVERY steps and the console polls at
status_poll_hz, so each poll writes the newest frame it has and the video runs
at the poll rate -- wall-clock true, with a frame repeated until the next one
arrives. A snapshot with no image (server not ready, a poll between images)
writes nothing, so a video never contains a frame that was not there.

NO cv2 IN HERE, OR ANYWHERE UNDER gui/. The pip `opencv-python` wheel bundles
its own Qt and, on import, points QT_QPA_PLATFORM_PLUGIN_PATH at it; PyQt5 then
fails to load `xcb` and the console aborts before its window exists (the workstation,
2026-09-19, the first version of this file). The original design
asserts the import rule for the whole package.

Encoding, chosen once per recorder (`detect_encoder()`), best first:
  h264   PyAV (`import av`, ships its own ffmpeg libraries, no Qt, no numpy):
         each JPEG is decoded and H.264-encoded into an .mp4 that any player,
         browser or phone opens. `pip install av` if it is missing.
  mjpeg  fallback with no PyAV: the JPEGs are muxed as-is into a Motion-JPEG
         .avi (standard library only). VLC / ffmpeg open it; a phone will not.
         `ffmpeg -i x.avi x.mp4` converts.
The decision log's header names the encoder and the files.

`feed()` is called on the GUI thread; one small frame per poll is cheap.
"""
from __future__ import annotations

import os
import struct
import time
from fractions import Fraction
from typing import Dict, Optional, Tuple

VALUE_MAP_SUBDIR = os.path.join("videos", "value_map")
POLICY_SUBDIR = os.path.join("videos", "policy")
LOG_SUBDIR = "logs"

ENCODERS = {"h264": ".mp4", "mjpeg": ".avi"}


def detect_encoder() -> str:
    """'h264' when PyAV can encode libx264 here, else 'mjpeg'."""
    try:
        import av                                   # noqa: F401  (no Qt, no numpy)
        av.codec.Codec("libx264", "w")
        return "h264"
    except Exception:                               # noqa: BLE001 - any failure = fallback
        return "mjpeg"

# Snapshot key -> (subdirectory under records_dir, human label)
VIDEO_STREAMS = {
    "value_map_jpeg_bytes": (VALUE_MAP_SUBDIR, "value map"),
    "annotated_rgb_jpeg_bytes": (POLICY_SUBDIR, "policy camera"),
}

# target_confidence is SAM3's score for the target on that step's frame, or "-"
# on a step where SAM3 did not run (explore segments every few steps, navigate
# re-detects every few steps) or did not segment the target. target_detected is
# the object map's memory and stays 1 once the target has been seen at all, so
# the two columns differ on purpose.
LOG_FIELDS = ("mode", "objectgoal", "target_detected", "target_confidence",
              "called_stop", "done_initializing", "detections", "debug")


# --------------------------------------------------------------------------
# JPEG dimensions, without decoding
# --------------------------------------------------------------------------
_SOF_MARKERS = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}


def jpeg_size(data: bytes) -> Optional[Tuple[int, int]]:
    """(width, height) from the Start-Of-Frame marker, or None if not a JPEG."""
    if len(data) < 4 or data[0] != 0xFF or data[1] != 0xD8:
        return None
    i = 2
    n = len(data)
    while i + 4 <= n:
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        if marker == 0xFF:              # fill byte
            i += 1
            continue
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2                      # standalone markers carry no length
            continue
        if i + 4 > n:
            return None
        length = struct.unpack(">H", data[i + 2:i + 4])[0]
        if marker in _SOF_MARKERS:
            if i + 9 > n:
                return None
            height, width = struct.unpack(">HH", data[i + 5:i + 9])
            return (width, height) if width and height else None
        if marker == 0xDA:              # start of scan: no SOF before it
            return None
        i += 2 + length
    return None


# --------------------------------------------------------------------------
# Motion-JPEG AVI, written incrementally
# --------------------------------------------------------------------------
class MjpegAviWriter:
    """RIFF/AVI with one 'vids' stream of MJPG frames. The headers are written
    with placeholders and patched on close(), when the frame count and the
    movi/idx1 sizes are known. Standard layout, readable by ffmpeg / VLC /
    OpenCV."""

    def __init__(self, path: str, width: int, height: int, fps: float):
        self.path = path
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.frames = 0
        self._index = []                # (offset within movi, size) per frame
        self._f = open(path, "wb")
        self._write_headers()
        self._movi_start = self._f.tell()
        self._f.write(b"LIST" + struct.pack("<I", 0) + b"movi")   # size patched

    def _write_headers(self) -> None:
        f = self._f
        scale = 1000
        rate = max(1, int(round(self.fps * scale)))
        usec = int(round(1e6 / self.fps)) if self.fps > 0 else 0
        f.write(b"RIFF" + struct.pack("<I", 0) + b"AVI ")          # size patched
        # hdrl
        avih = b"avih" + struct.pack("<I", 56) + struct.pack(
            "<IIIIIIIIIIIII", usec, 0, 0, 0x10, 0, 0, 1, 0,
            self.width, self.height, 0, 0, 0) + struct.pack("<I", 0)
        strh = b"strh" + struct.pack("<I", 56) + b"vids" + b"MJPG" + struct.pack(
            "<IHHIIIIIIiI", 0, 0, 0, 0, scale, rate, 0, 0, 0, -1, 0) + struct.pack(
            "<HHHH", 0, 0, self.width, self.height)
        bmih = struct.pack("<IiiHH4sIiiII", 40, self.width, self.height, 1, 24,
                           b"MJPG", self.width * self.height * 3, 0, 0, 0, 0)
        strf = b"strf" + struct.pack("<I", 40) + bmih
        strl = b"LIST" + struct.pack("<I", 4 + len(strh) + len(strf)) + b"strl" + strh + strf
        hdrl = b"LIST" + struct.pack("<I", 4 + len(avih) + len(strl)) + b"hdrl" + avih + strl
        f.write(hdrl)
        self._avih_frames_at = 12 + 12 + 8 + 16          # dwTotalFrames inside avih
        self._strh_length_at = (12 + 12 + len(avih) + 12 + 8 + 32)  # dwLength inside strh

    def write(self, jpeg: bytes) -> None:
        f = self._f
        offset = f.tell() - self._movi_start - 8       # relative to the 'movi' fourcc
        size = len(jpeg)
        f.write(b"00dc" + struct.pack("<I", size) + jpeg)
        if size & 1:
            f.write(b"\0")
        self._index.append((offset, size))
        self.frames += 1

    def close(self) -> None:
        if self._f is None:
            return
        f = self._f
        movi_end = f.tell()
        movi_size = movi_end - self._movi_start - 8
        # idx1
        f.write(b"idx1" + struct.pack("<I", 16 * len(self._index)))
        for offset, size in self._index:
            f.write(b"00dc" + struct.pack("<III", 0x10, offset + 4, size))
        riff_size = f.tell() - 8
        f.seek(4)
        f.write(struct.pack("<I", riff_size))
        f.seek(self._avih_frames_at)
        f.write(struct.pack("<I", self.frames))
        f.seek(self._strh_length_at)
        f.write(struct.pack("<I", self.frames))
        f.seek(self._movi_start + 4)
        f.write(struct.pack("<I", movi_size))
        f.close()
        self._f = None


class H264Mp4Writer:
    """PyAV / libx264 into an .mp4. Every JPEG is decoded, reformatted to the
    first frame's (even) size in yuv420p and encoded, so a panel that changes
    size mid-run is rescaled rather than breaking the stream. Frames carry their
    own time base so the file plays at the poll rate (the stream's time base is
    the muxer's and would not)."""

    def __init__(self, path: str, width: int, height: int, fps: float):
        import av
        self._av = av
        self.path = path
        self.width = max(2, int(width) // 2 * 2)
        self.height = max(2, int(height) // 2 * 2)
        self.frames = 0
        rate = Fraction(fps).limit_denominator(1000)
        self._time_base = 1 / rate
        self._container = av.open(path, mode="w")
        self._stream = self._container.add_stream("libx264", rate=rate)
        self._stream.width = self.width
        self._stream.height = self.height
        self._stream.pix_fmt = "yuv420p"
        self._stream.options = {"crf": "23", "preset": "veryfast"}

    def write(self, jpeg: bytes) -> None:
        import io
        source = self._av.open(io.BytesIO(jpeg), format="mjpeg")
        try:
            frame = next(source.decode(video=0))
        finally:
            source.close()
        frame = frame.reformat(width=self.width, height=self.height, format="yuv420p")
        frame.pts = self.frames
        frame.time_base = self._time_base
        for packet in self._stream.encode(frame):
            self._container.mux(packet)
        self.frames += 1

    def close(self) -> None:
        if self._container is None:
            return
        for packet in self._stream.encode(None):
            self._container.mux(packet)
        self._container.close()
        self._container = None


def open_writer(encoder: str, path: str, width: int, height: int, fps: float):
    if encoder == "h264":
        return H264Mp4Writer(path, width, height, fps)
    return MjpegAviWriter(path, width, height, fps)


class _Video:
    """One video, sized from its first frame. A later frame of another size is
    rescaled (h264) or written as-is (mjpeg frames carry their own dimensions)
    and counted, so the log can say the panel changed size mid-run."""

    def __init__(self, path: str, fps: float, encoder: str):
        self.path = path
        self.fps = float(fps)
        self.encoder = encoder
        self._writer = None
        self._size = None
        self._latest = None
        self.frames = 0
        self.resized = 0

    def offer(self, jpeg: Optional[bytes]) -> None:
        """A new frame, if the snapshot carried one and it is a JPEG."""
        if not jpeg:
            return
        size = jpeg_size(jpeg)
        if size is None:
            return
        self._latest = (jpeg, size)

    def tick(self) -> None:
        """Write the newest frame for this poll. Nothing yet -> nothing written."""
        if self._latest is None:
            return
        jpeg, size = self._latest
        if self._writer is None:
            self._size = size
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            self._writer = open_writer(self.encoder, self.path, size[0], size[1], self.fps)
        elif size != self._size:
            self.resized += 1
        self._writer.write(jpeg)
        self.frames += 1

    def close(self) -> None:
        if self._writer is not None:
            self._writer.close()
            self._writer = None


class RunRecorder:

    def __init__(self, records_dir: str, fps: float, encoder: Optional[str] = None):
        self.records_dir = os.path.expanduser(str(records_dir))
        self.fps = float(fps)
        self.encoder = encoder or detect_encoder()
        self.video_ext = ENCODERS[self.encoder]
        self.name: Optional[str] = None
        self._videos: Dict[str, _Video] = {}
        self._log = None
        self._last_step = None
        self.started_at = None

    @property
    def active(self) -> bool:
        return self.name is not None

    # -- paths ----------------------------------------------------------------
    def log_path(self, name: str) -> str:
        return os.path.join(self.records_dir, LOG_SUBDIR, f"{name}.log")

    def video_path(self, key: str, name: str) -> str:
        return os.path.join(self.records_dir, VIDEO_STREAMS[key][0], f"{name}{self.video_ext}")

    # -- lifecycle ------------------------------------------------------------
    def start(self, name: str, bag_path: str = "", target: str = "") -> str:
        """Open the log and arm the videos. Returns the log path."""
        if self.active:
            self.stop()
        self.name = name
        self.started_at = time.time()
        self._last_step = None
        self._videos = {key: _Video(self.video_path(key, name), self.fps, self.encoder)
                        for key in VIDEO_STREAMS}
        path = self.log_path(name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._log = open(path, "w", encoding="utf-8")
        self._log.write(f"# run {name}\n")
        self._log.write(f"# started t={self.started_at:.3f} "
                        f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.started_at))}\n")
        self._log.write(f"# target {target!r}\n")
        self._log.write(f"# rosbag {bag_path}\n")
        for key, (subdir, label) in VIDEO_STREAMS.items():
            self._log.write(f"# {label} video {self.video_path(key, name)} "
                            f"({self.encoder}, {self.fps:g} fps = the status poll rate)\n")
        if self.encoder != "h264":
            self._log.write("# NOTE: PyAV not importable, so Motion-JPEG .avi instead of "
                            ".mp4. `pip install av` on this machine for mp4.\n")
        self._log.write("# policy server log: grep '^\\[state\\]' logs/rpv_policy_*.log "
                        "for t >= the started value above\n")
        self._log.write("# one line per policy step: t=<epoch> step=<n> "
                        + " ".join(f"{f}=" for f in LOG_FIELDS) + "\n")
        self._log.flush()
        return path

    def feed(self, snap: dict) -> None:
        """One GET /status snapshot. Videos advance every call; the log only
        when the policy's step counter has moved, so a poll that sees the same
        step twice does not write the decision twice."""
        if not self.active:
            return
        for key, video in self._videos.items():
            video.offer(snap.get(key))
            video.tick()
        step = snap.get("step")
        if step is None or step == self._last_step:
            return
        self._last_step = step
        parts = [f"t={time.time():.3f}", f"step={step}"]
        for field in LOG_FIELDS:
            parts.append(f"{field}={self._fmt(snap.get(field))}")
        self._log.write(" ".join(parts) + "\n")
        self._log.flush()

    def stop(self) -> Dict[str, int]:
        """Close everything. Returns frames written per video, by key."""
        counts = {}
        for key, video in self._videos.items():
            video.close()
            counts[key] = video.frames
            if video.resized and self._log is not None:
                self._log.write(f"# {VIDEO_STREAMS[key][1]} video: {video.resized} frames "
                                f"had a different size from the first\n")
        if self._log is not None:
            self._log.write(f"# stopped t={time.time():.3f} frames={counts}\n")
            self._log.close()
            self._log = None
        self._videos = {}
        self.name = None
        return counts

    @staticmethod
    def _fmt(value) -> str:
        if value is None:
            return "-"
        if isinstance(value, bool):
            return "1" if value else "0"
        if isinstance(value, float):
            return f"{value:.2f}"
        text = str(value).replace("\n", " | ").strip()
        return repr(text) if (" " in text or not text) else text
