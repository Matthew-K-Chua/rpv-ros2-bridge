"""Background poller for the indicator lights.

Its own QThread, like StatusPoller, because the SSH probe to the Pi takes real
time. Every call inside ProcessManager.probe() carries its own timeout, so a
wedged link shows up as a red light within one cycle rather than freezing the UI.

There is no log tail here: the RPV console has no terminal pane. What the
policy is doing comes from GET /status, which is the same information without a
tmux round trip per second.
"""
from __future__ import annotations

from .config import Config
from .process_manager import ProcessManager
from .qtcompat import QtCore, Signal


class SystemPoller(QtCore.QThread):
    state = Signal(dict)

    def __init__(self, cfg: Config, pm: ProcessManager, parent=None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self.pm = pm
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        period_ms = int(max(1.0, self.cfg.system_poll_s) * 1000)
        while self._running:
            try:
                self.state.emit(self.pm.probe())
            except Exception as e:
                print(f"[system-poller] {e}")
            slept = 0
            while self._running and slept < period_ms:
                self.msleep(100)
                slept += 100
