"""Display widgets: a titled frame, a scaled image panel, the map, a status light.

Derived from the TurtleBot 4 console's widgets, with three changes for Spot:
  * TitledPanel takes a title colour, because the Policy panel's title has to be
    black -- grey-on-bone was unreadable.
  * MapPanel overlays the live /goal_pose and (optionally) the merged /scan on
    top of the occupancy grid, so the map view answers "where is it going and
    what can it see" without opening RViz.
  * The robot marker is drawn to Spot's footprint proportions rather than a
    circle, so its heading and its 1.1 x 0.5 m body are legible at map scale.
"""
from __future__ import annotations

import math
import time
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

from .qtcompat import QtCore, QtGui, QtWidgets

# Mode -> chip colour for the run readout. initialize = the 360 base scan
# (amber), explore = hunting frontiers (blue), navigate = driving at the target
# (green). Anything else falls back to grey.
MODE_COLORS = {
    "initialize": "#b58105",
    "explore": "#1f6feb",
    "navigate": "#1a7f37",
}
_OK_GREEN = "#1a7f37"
_IDLE_GREY = "#6e6e6e"


def format_elapsed(seconds: Optional[float]) -> str:
    """Seconds -> MM:SS, or --:-- when the clock is not running. Pure."""
    if seconds is None or seconds < 0:
        return "--:--"
    s = int(seconds)
    return f"{s // 60:02d}:{s % 60:02d}"


def run_readout_fields(snap: dict, elapsed_s: Optional[float] = None) -> dict:
    """Map a /status snapshot (+ elapsed seconds) to the readout's display
    strings and colours. Pure, so it is unit-tested without Qt.

    Precedence for the status chip: a called_stop wins over a live detection,
    which wins over the plain searching state -- the three the operator most
    needs to tell apart at a glance."""
    target = str(snap.get("objectgoal") or "—")
    raw_mode = str(snap.get("mode") or "—")
    step = snap.get("step", 0)
    mode_color = MODE_COLORS.get(raw_mode.lower(), _IDLE_GREY)

    if snap.get("called_stop"):
        status, status_color, mode_color = "TARGET REACHED", _OK_GREEN, _OK_GREEN
    elif snap.get("target_detected"):
        conf = snap.get("target_confidence")
        status = "DETECTED" if conf is None else f"DETECTED {float(conf):.2f}"
        status_color = _OK_GREEN
    else:
        status, status_color = "searching…", _IDLE_GREY

    return {
        "target": target,
        "mode": raw_mode.upper(),
        "mode_color": mode_color,
        "step": str(step),
        "elapsed": format_elapsed(elapsed_s),
        "status": status,
        "status_color": status_color,
    }


class RunReadout(QtWidgets.QFrame):
    """A one-line, at-a-glance header for the run in progress: current target,
    policy mode, step count, elapsed time, and the detect/stop status.

    The clock is the only state here. It (re)starts when a new run is detected --
    the target changed, the step count reset (a new policy process), or an
    explicit mark_run_start() from the Start/Set buttons -- and ticks once a
    second between the ~3 Hz status polls so the elapsed time stays live."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setStyleSheet("background:#ffffff;")
        self._run_start: Optional[float] = None
        self._last_key: Optional[str] = None
        self._last_step: int = -1

        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(10, 4, 10, 4)
        lay.setSpacing(18)
        self._values = {}
        # (key, caption, stretch) -- TARGET gets the room, the rest are compact.
        for key, caption, stretch in (
            ("target", "TARGET", 3),
            ("mode", "MODE", 1),
            ("step", "STEP", 1),
            ("elapsed", "TIME", 1),
            ("status", "STATUS", 2),
        ):
            lay.addWidget(self._chip(key, caption), stretch)

    def _chip(self, key: str, caption: str) -> QtWidgets.QWidget:
        box = QtWidgets.QWidget()
        col = QtWidgets.QVBoxLayout(box)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(0)
        cap = QtWidgets.QLabel(caption)
        cap.setStyleSheet("color:#6e6e6e; font-size:10px; font-weight:bold;")
        val = QtWidgets.QLabel("—")
        val.setStyleSheet("color:#000000; font-size:17px; font-weight:bold;")
        col.addWidget(cap)
        col.addWidget(val)
        self._values[key] = val
        return box

    def update_run(self, snap: dict) -> None:
        """Feed a /status snapshot. Manages the run clock, then repaints."""
        if snap.get("ready"):
            key = str(snap.get("objectgoal") or "")
            step = int(snap.get("step", 0) or 0)
            if self._run_start is None or key != self._last_key or step < self._last_step:
                self._run_start = time.monotonic()
            self._last_key, self._last_step = key, step
        self._render(snap)

    def mark_run_start(self) -> None:
        """Reset the clock now, for an operator-initiated run (Start / Set), so
        the timer starts the instant the bridge is (re)launched rather than when
        the first status arrives ~30 s later."""
        self._run_start = time.monotonic()

    def tick(self) -> None:
        """Advance just the elapsed-time chip between status polls."""
        if self._run_start is not None:
            self._values["elapsed"].setText(format_elapsed(time.monotonic() - self._run_start))

    def set_idle(self) -> None:
        self._run_start = None
        self._last_key, self._last_step = None, -1
        self._render({})

    def _elapsed(self) -> Optional[float]:
        return None if self._run_start is None else time.monotonic() - self._run_start

    def _render(self, snap: dict) -> None:
        fields = run_readout_fields(snap, self._elapsed())
        self._values["target"].setText(fields["target"])
        self._values["mode"].setText(fields["mode"])
        self._values["mode"].setStyleSheet(
            f"color:{fields['mode_color']}; font-size:17px; font-weight:bold;")
        self._values["step"].setText(fields["step"])
        self._values["elapsed"].setText(fields["elapsed"])
        self._values["status"].setText(fields["status"])
        self._values["status"].setStyleSheet(
            f"color:{fields['status_color']}; font-size:17px; font-weight:bold;")


def _event_pos(event) -> QtCore.QPoint:
    """Qt6 QMouseEvent.position() (QPointF) vs Qt5 .pos()."""
    if hasattr(event, "position"):
        return event.position().toPoint()
    return event.pos()


class StatusLight(QtWidgets.QLabel):
    """Tri-state indicator. green=up, yellow=coming up, red=down, grey=unknown.

    With `on_click` it doubles as a switch (the WS light starts and stops its
    session); the rest are read-only.
    """

    COLORS = {
        "green": "#1a7f37", "yellow": "#b58105", "red": "#c1121f", "grey": "#6e6e6e",
        True: "#1a7f37", False: "#c1121f", None: "#6e6e6e",
    }

    def __init__(self, name: str, on_click=None) -> None:
        super().__init__()
        self._name = name
        self.on_click = on_click
        self.setAlignment(QtCore.Qt.AlignCenter)
        if on_click is not None:
            self.setCursor(QtCore.Qt.PointingHandCursor)
        self.state = None
        self.set_state(None)

    def set_state(self, state) -> None:
        self.state = state
        color = self.COLORS.get(state, "#6e6e6e")
        self.setText(f"\u25cf\n{self._name}")
        self.setStyleSheet(f"color:{color}; font-weight:bold; font-size:20px; padding:2px 8px;")


class TitledPanel(QtWidgets.QWidget):
    """A titled frame wrapping a content widget."""

    def __init__(self, title: str, content: QtWidgets.QWidget,
                 title_color: str = "#111", parent=None) -> None:
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(2)
        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setStyleSheet(
            f"color:{title_color}; font-weight:bold; padding:2px;"
        )
        lay.addWidget(self.title_label)
        lay.addWidget(content, 1)


class ScaledImage(QtWidgets.QWidget):
    """Displays a QPixmap scaled to fit while keeping aspect ratio.

    ``rotate`` (degrees, applied to every incoming image) lets the policy's
    value / obstacle maps be shown at 180 so they line up with the north-up
    SLAM map next to them; the policy draws its maps with the opposite row
    order (2026-09-17, requested after comparing the two panels side by side).
    """

    def __init__(self, placeholder: str = "no signal", parent=None,
                 rotate: int = 0) -> None:
        super().__init__(parent)
        self._pix: Optional[QtGui.QPixmap] = None
        self._placeholder = placeholder
        self.rotate = rotate
        self.setMinimumSize(240, 200)
        self.setStyleSheet("background:#111;")

    def set_pixmap(self, pix: QtGui.QPixmap) -> None:
        if self.rotate and not pix.isNull():
            pix = pix.transformed(QtGui.QTransform().rotate(self.rotate))
        self._pix = pix
        self.update()

    def set_placeholder(self, text: str) -> None:
        self._placeholder = text
        self._pix = None
        self.update()

    def set_jpeg(self, data: bytes) -> None:
        img = QtGui.QImage()
        if img.loadFromData(data):
            self.set_pixmap(QtGui.QPixmap.fromImage(img))

    def set_rgb(self, buf: bytes, w: int, h: int) -> None:
        img = QtGui.QImage(buf, w, h, 3 * w, QtGui.QImage.Format_RGB888)
        self.set_pixmap(QtGui.QPixmap.fromImage(img.copy()))

    def _draw_rect(self) -> Optional[QtCore.QRect]:
        if self._pix is None or self._pix.isNull():
            return None
        ps, ws = self._pix.size(), self.size()
        scale = min(ws.width() / ps.width(), ws.height() / ps.height())
        w, h = int(ps.width() * scale), int(ps.height() * scale)
        x = (ws.width() - w) // 2
        y = (ws.height() - h) // 2
        return QtCore.QRect(x, y, w, h)

    def paintEvent(self, _event) -> None:
        p = QtGui.QPainter(self)
        # Without this hint Qt point-samples when rescaling, so a map wider than
        # the panel loses whole rows/columns -- thin features flicker in and out.
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        p.fillRect(self.rect(), QtGui.QColor("#111"))
        r = self._draw_rect()
        if r is None:
            p.setPen(QtGui.QColor("#777"))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, self._placeholder)
        else:
            p.drawPixmap(r, self._pix)
        p.end()


def occupancy_to_gray(data: np.ndarray) -> np.ndarray:
    """OccupancyGrid values -> an 8-bit image, north up.

    -1 unknown -> mid grey; 0..100 -> white..black. Row 0 of the grid is the
    SOUTH edge, so the result is flipped for display. Pure, and tested.
    """
    img = np.full(data.shape, 127, dtype=np.uint8)
    known = data >= 0
    img[known] = (255.0 * (1.0 - data[known].astype(np.float32) / 100.0)).astype(np.uint8)
    return np.ascontiguousarray(np.flipud(img))


class MapPanel(ScaledImage):
    """Occupancy grid + robot pose + goal + scan, with click-to-go."""

    # Spot is 1.1 x 0.5 m. Drawn to scale from the grid resolution so the
    # marker means something next to a doorway.
    BODY_LENGTH_M = 1.1
    BODY_WIDTH_M = 0.5

    def __init__(self, parent=None) -> None:
        super().__init__("waiting for /map …", parent)
        self._grid: Optional[dict] = None
        self._pose: Optional[Tuple[float, float, float]] = None
        self._goal: Optional[Tuple[float, float, float]] = None
        self._scan: Optional[Sequence[Tuple[float, float]]] = None
        self.on_click: Optional[Callable[[float, float], None]] = None
        self.click_enabled = False
        self.show_scan = True

    # --- data in --------------------------------------------------------- #
    def set_map(self, grid: dict) -> None:
        self._grid = grid
        img = occupancy_to_gray(grid["data"])
        h, w = img.shape
        qimg = QtGui.QImage(img.data, w, h, w, QtGui.QImage.Format_Grayscale8)
        self.set_pixmap(QtGui.QPixmap.fromImage(qimg.copy()))

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self._pose = (x, y, yaw)
        self.update()

    def set_goal(self, x: float, y: float, yaw: float) -> None:
        self._goal = (x, y, yaw)
        self.update()

    def set_scan(self, points) -> None:
        self._scan = points
        self.update()

    # --- world <-> widget pixel helpers ---------------------------------- #
    def _world_to_widget(self, wx: float, wy: float) -> Optional[QtCore.QPointF]:
        r = self._draw_rect()
        if r is None or self._grid is None:
            return None
        g = self._grid
        col = (wx - g["origin_x"]) / g["resolution"]
        row = (wy - g["origin_y"]) / g["resolution"]      # row from south
        row_top = g["height"] - row                        # flipped for display
        px = r.x() + (col / g["width"]) * r.width()
        py = r.y() + (row_top / g["height"]) * r.height()
        return QtCore.QPointF(px, py)

    def _widget_to_world(self, px: float, py: float):
        r = self._draw_rect()
        if r is None or self._grid is None or not r.contains(int(px), int(py)):
            return None
        g = self._grid
        col = (px - r.x()) / r.width() * g["width"]
        row_top = (py - r.y()) / r.height() * g["height"]
        row = g["height"] - row_top
        wx = g["origin_x"] + (col + 0.5) * g["resolution"]
        wy = g["origin_y"] + (row + 0.5) * g["resolution"]
        return wx, wy

    def _px_per_m(self) -> float:
        r = self._draw_rect()
        if r is None or self._grid is None:
            return 0.0
        return (r.width() / self._grid["width"]) / self._grid["resolution"]

    # --- interaction ----------------------------------------------------- #
    def mousePressEvent(self, event) -> None:
        if not self.click_enabled or event.button() != QtCore.Qt.LeftButton:
            return
        pos = _event_pos(event)
        world = self._widget_to_world(pos.x(), pos.y())
        if world is not None and self.on_click is not None:
            self.on_click(world[0], world[1])

    # --- painting -------------------------------------------------------- #
    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        if self._grid is None:
            return
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing
                        if hasattr(QtGui.QPainter, "RenderHint")
                        else QtGui.QPainter.Antialiasing)
        try:
            if self.show_scan:
                self._paint_scan(p)
            self._paint_goal(p)
            self._paint_robot(p)
        finally:
            p.end()

    def _paint_scan(self, p) -> None:
        if not self._scan:
            return
        p.setPen(QtGui.QPen(QtGui.QColor(0, 255, 128, 200), 2))
        for wx, wy in self._scan:
            pt = self._world_to_widget(wx, wy)
            if pt is not None:
                p.drawPoint(pt)

    def _paint_goal(self, p) -> None:
        if self._goal is None:
            return
        pt = self._world_to_widget(self._goal[0], self._goal[1])
        if pt is None:
            return
        p.setPen(QtGui.QPen(QtGui.QColor("#ff2a00"), 2))
        p.setBrush(QtCore.Qt.transparent)
        p.drawEllipse(pt, 9, 9)
        p.drawLine(QtCore.QPointF(pt.x() - 13, pt.y()), QtCore.QPointF(pt.x() + 13, pt.y()))
        p.drawLine(QtCore.QPointF(pt.x(), pt.y() - 13), QtCore.QPointF(pt.x(), pt.y() + 13))

    def _paint_robot(self, p) -> None:
        if self._pose is None:
            return
        pt = self._world_to_widget(self._pose[0], self._pose[1])
        if pt is None:
            return
        # Screen y grows downward, so a map-frame yaw becomes -yaw on screen.
        yaw = -self._pose[2]
        ppm = self._px_per_m()
        half_l = max(6.0, 0.5 * self.BODY_LENGTH_M * ppm)
        half_w = max(3.0, 0.5 * self.BODY_WIDTH_M * ppm)

        p.save()
        p.translate(pt)
        p.rotate(math.degrees(yaw))
        p.setPen(QtGui.QPen(QtGui.QColor("#00e0ff"), 2))
        p.setBrush(QtGui.QColor(0, 224, 255, 90))
        p.drawRect(QtCore.QRectF(-half_l, -half_w, 2 * half_l, 2 * half_w))
        # Nose marker: Spot's front, ~0.55 m ahead of the body origin, which is
        # also where stop_standoff_m is measured FROM.
        p.setBrush(QtGui.QColor("#00e0ff"))
        p.drawEllipse(QtCore.QPointF(half_l, 0.0), 3, 3)
        p.restore()
