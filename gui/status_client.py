"""Polls the policy server's GET /status and emits the parsed snapshot.

Its own QThread, so the HTTP poll never stalls the UI. stdlib only (urllib +
json), so it adds no dependency to the ROS environment. JPEG panels stay as raw
bytes in the emitted dict; the UI turns them into pixmaps with
QImage.loadFromData, so the GUI side needs no OpenCV.

The server binds LOOPBACK by default (RPV_POLICY_HOST=127.0.0.1) because the
console shares the workstation with it. Polling it from another machine means
setting that to 0.0.0.0 first.
"""
from __future__ import annotations

import base64
import json
import urllib.request

from .config import Config
from .qtcompat import QtCore, Signal


class StatusPoller(QtCore.QThread):
    status = Signal(dict)
    online = Signal(bool)

    def __init__(self, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        period_ms = int(max(0.1, 1.0 / max(0.5, self.cfg.status_poll_hz)) * 1000)
        while self._running:
            snap = self._poll_once()
            if snap is not None:
                self.online.emit(True)
                self.status.emit(snap)
            else:
                self.online.emit(False)
            slept = 0
            while self._running and slept < period_ms:    # responsive to stop()
                self.msleep(50)
                slept += 50

    def _poll_once(self):
        try:
            with urllib.request.urlopen(self.cfg.status_url, timeout=2) as resp:
                data = json.loads(resp.read().decode("utf-8"))
        except Exception:
            return None
        out = {}
        for k, v in data.items():
            if k.endswith("_jpeg") and isinstance(v, str):
                try:
                    out[k + "_bytes"] = base64.b64decode(v)
                except Exception:
                    pass
            else:
                out[k] = v
        return out
