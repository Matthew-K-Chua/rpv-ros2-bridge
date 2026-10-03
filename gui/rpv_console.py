"""RPV console -- one window that runs the experiment.

    cd <rpv-ros2-bridge> && python3 -m gui.rpv_console     # with ROS 2 Humble sourced

Starts THIS machine's tmux session, shows the live camera, the policy's value
map and annotated camera, and lets you change the object being searched for.

THE PI IS STARTED BY HAND, from a Pi shell.

Layout, left to right:

    value / obstacle / SLAM tabs   camera (annotated by the policy)
    camera (live topic)            lights: PI WS AI BRIDGE SLAM CAM
    policy log                     target, controls, autonomy

The live camera took the SLAM map's slot on 2026-09-17: the raw topic streams
at full rate whether or not the policy is running, so it is the panel that
says "the robot is looking at X right now". The SLAM map (robot, /goal_pose,
/scan, and click-to-go in MANUAL) is the third tab of the AI-maps panel.

Everything that takes time -- SSH, tmux, HTTP -- runs on a worker thread and
reports back through a signal. The UI thread never blocks on the network.
"""
from __future__ import annotations

import sys
import threading
import time

from . import shortvis
from .config import Config
from .dialogs import SettingsDialog, ShortvisDialog
from .process_manager import ProcessManager, STAND_SEQUENCE
from .run_recorder import RunRecorder
from .qtcompat import QtCore, QtWidgets, Signal, exec_app, exec_dialog, qenum
from .ros_node import ROS_AVAILABLE, ROS_IMPORT_ERROR, RosBridge
from .status_client import StatusPoller
from .system_poller import SystemPoller
from .widgets import MapPanel, RunReadout, ScaledImage, StatusLight, TitledPanel

# The policy panel's palette. The TurtleBot 4 console put grey-on-black text on a
# bone-grey window and it was unreadable in a lit room; this is the fix.
# The PI light is EVIDENCE, not a session check: green means /scan is arriving
# over the zenoh bridge, which can only happen if the Pi is up, the camera is
# streaming, the scan chain resolved its TF and the tunnel is carrying traffic.
# It replaced an SSH `tmux has-session` that could never succeed on this
# topology, and it tells you strictly more.
ONBOARD_TOOLTIP = ("Green when /scan is arriving from the Pi.\n"
              "That means the Pi, the camera, the scan chain and the tunnel are\n"
              "all working. Start the Pi by hand -- see the command below.")
# How long /scan or a camera frame may be absent before its light goes red. The
# deployable depth profile is 6 fps, so one second is already several frames.
LINK_TIMEOUT_S = 2.0

LOG_BG = "#f7f6f3"
LOG_FG = "#000000"
TITLE_FG = "#000000"


class MainWindow(QtWidgets.QMainWindow):
    log_signal = Signal(str)

    def __init__(self, cfg: Config, pm: ProcessManager = None) -> None:
        super().__init__()
        self.cfg = cfg
        self.pm = pm or ProcessManager(cfg)
        self.setWindowTitle("RPV Console")
        self.resize(1360, 900)

        self._offboard_state = "red"
        # Monotonic deadline after which an unfinished yellow becomes red. 0 =
        # no deadline, which is what a light that was never armed by a start
        # click has, and what cfg.start_timeout_s = 0 gives you everywhere.
        self._offboard_deadline = 0.0
        self._last_cam_ts = 0.0
        self._last_scan_ts = 0.0
        self._manual = False
        self._recording = False
        # Decision log + the two videos, named after the bag. Fed from
        # _on_status; opened and closed by the Record button.
        self.recorder = RunRecorder(cfg.records_dir, cfg.status_poll_hz)

        self._build_ui()
        self.log_signal.connect(self._append_log)
        self._warn_about_config()

    def start_workers(self) -> None:
        """Start the pollers and ROS. Separate from __init__ so a test can build
        the window without opening sockets."""
        self.status_poller = StatusPoller(self.cfg)
        self.status_poller.status.connect(self._on_status)
        self.status_poller.start()

        self.system_poller = SystemPoller(self.cfg, self.pm)
        self.system_poller.state.connect(self._on_system_state)
        self.system_poller.start()

        self.ros = RosBridge(self.cfg)
        self.ros.map_received.connect(self.map_panel.set_map)
        self.ros.pose_received.connect(self.map_panel.set_pose)
        self.ros.goal_received.connect(self.map_panel.set_goal)
        self.ros.scan_received.connect(self.map_panel.set_scan)
        self.ros.scan_received.connect(self._on_scan)
        self.ros.camera_received.connect(self._on_camera)
        self.map_panel.on_click = self._on_map_click
        self.ros.start()
        if not ROS_AVAILABLE:
            self._append_log(f"[ros] unavailable ({ROS_IMPORT_ERROR}) -- map and click disabled; "
                             "the policy panels still work")

        self._link_timer = QtCore.QTimer(self)
        self._link_timer.timeout.connect(self._refresh_link_lights)
        self._link_timer.start(1000)

    # ------------------------------------------------------------------ #
    # UI construction
    # ------------------------------------------------------------------ #
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        # At-a-glance run readout, spanning the width above both columns.
        self.run_readout = RunReadout()
        root.addWidget(self.run_readout)

        columns = QtWidgets.QHBoxLayout()
        columns.setSpacing(4)
        columns.addWidget(self._build_left_column(), 3)
        columns.addWidget(self._build_right_column(), 2)
        root.addLayout(columns, 1)

        settings = self.menuBar().addMenu("&Console")
        act = settings.addAction("Settings…")
        act.triggered.connect(self._open_settings)
        act2 = settings.addAction("Edit SAM3 class list…")
        act2.triggered.connect(self._open_shortvis)

    SLAM_TAB_AUTO = "SLAM"
    SLAM_TAB_MANUAL = "SLAM · CLICK TO SEND A NAV2 GOAL"

    def _build_left_column(self) -> QtWidgets.QWidget:
        # Rotated 180 so they read the same way up as the SLAM map beside them.
        self.value_panel = ScaledImage("value map …", rotate=180)
        self.obstacle_panel = ScaledImage("obstacle map …", rotate=180)
        self.map_panel = MapPanel()
        self.map_panel.show_scan = self.cfg.show_scan
        self.ai_tabs = QtWidgets.QTabWidget()
        self.ai_tabs.addTab(self.value_panel, "Value")
        self.ai_tabs.addTab(self.obstacle_panel, "Obstacle")
        self.ai_tabs.addTab(self.map_panel, self.SLAM_TAB_AUTO)

        # The raw camera topic, every frame, regardless of the policy. The
        # annotated feed (right column) only updates when the policy steps.
        topic = self.cfg.live_camera_topic
        self.live_panel = ScaledImage(f"camera ({topic}) …")

        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        lay.addWidget(TitledPanel("AI maps (from the policy)  ·  SLAM", self.ai_tabs, TITLE_FG), 5)
        lay.addWidget(TitledPanel(f"Camera (live, {topic})", self.live_panel, TITLE_FG), 5)
        lay.addWidget(TitledPanel("Policy", self._build_policy_panel(), TITLE_FG), 3)
        return w

    def _build_policy_panel(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)

        self.thinking = QtWidgets.QLabel("Mode: —")
        self.thinking.setStyleSheet(
            f"color:{LOG_FG}; font-size:15px; font-weight:bold; padding:2px;")
        self.thinking.setWordWrap(True)
        lay.addWidget(self.thinking)

        # The policy log. Black on near-white: this panel sits on the window's
        # own bone-grey ground, where the old grey-on-dark was unreadable.
        self.policy_log = QtWidgets.QPlainTextEdit()
        self.policy_log.setReadOnly(True)
        self.policy_log.setMaximumBlockCount(1000)
        self.policy_log.setStyleSheet(
            f"background:{LOG_BG}; color:{LOG_FG}; font-family:monospace; font-size:12px;")
        lay.addWidget(self.policy_log, 1)
        return w

    def _build_right_column(self) -> QtWidgets.QWidget:
        self.cam_panel = ScaledImage("camera …")
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        lay.addWidget(TitledPanel("Camera (annotated by the policy)", self.cam_panel, TITLE_FG), 5)
        lay.addLayout(self._build_lights_row())
        lay.addLayout(self._build_object_row())
        lay.addLayout(self._build_stack_row())
        lay.addLayout(self._build_tools_row())
        lay.addLayout(self._build_onboard_command_row())
        lay.addLayout(self._build_mode_row())
        return w

    def _build_lights_row(self) -> QtWidgets.QHBoxLayout:
        self.lights = {
            # No on_click: nothing on this machine can start the Pi.
            "onboard": StatusLight("ONBOARD"),
            "offboard": StatusLight("OFFBOARD", on_click=self._toggle_offboard),
            "ai": StatusLight("AI"),
            "bridge": StatusLight("BRIDGE"),
            "slam": StatusLight("SLAM"),
            "cam": StatusLight("CAM"),
        }
        self.lights["onboard"].setToolTip(ONBOARD_TOOLTIP)
        self.lights["offboard"].setToolTip(
            "This machine's tmux session: zenoh, SLAM/Nav2, the VLM servers and\n"
            "the policy. Click to start or stop it.")
        self.lights["ai"].setToolTip("Green once the policy server answers on :13000")
        self.lights["bridge"].setToolTip("The ceai bridge window in rpv_offboard")
        self.lights["slam"].setToolTip("SLAM + Nav2 (the 'ros' window in rpv_offboard)")
        self.lights["cam"].setToolTip("Camera frames arriving in the last second")

        row = QtWidgets.QHBoxLayout()
        for key in ("onboard", "offboard", "ai", "bridge", "slam", "cam"):
            row.addWidget(self.lights[key])
        return row

    def _build_object_row(self) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()
        self.target_box = QtWidgets.QComboBox()
        self.target_box.setEditable(True)
        self._reload_targets()
        self.target_box.setCurrentText(self.cfg.default_target)
        row.addWidget(QtWidgets.QLabel("Desired object:"))
        row.addWidget(self.target_box, 1)

        set_btn = QtWidgets.QPushButton("Set")
        set_btn.setToolTip("Restart the bridge with this target. A fresh bridge sends\n"
                           "is_first=true, which resets the episode and re-pins the\n"
                           "episodic origin where the robot is standing NOW.")
        set_btn.clicked.connect(self._set_target)
        row.addWidget(set_btn)

        edit_btn = QtWidgets.QPushButton("Edit list…")
        edit_btn.setToolTip("Add or remove SAM3 prompt classes (data/shortvis.yaml)")
        edit_btn.clicked.connect(self._open_shortvis)
        row.addWidget(edit_btn)
        return row

    def _build_stack_row(self) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton("Start everything")
        self.start_btn.setStyleSheet("font-weight:bold; padding:8px;")
        self.start_btn.setToolTip("zenoh, SLAM/Nav2, the AI stack and the bridge on THIS\n"
                                  "machine. Start it BEFORE the Pi -- the Pi connects out\n"
                                  "and needs this side already listening.")
        self.start_btn.clicked.connect(self._start_all)
        row.addWidget(self.start_btn, 1)

        self.stop_btn = QtWidgets.QPushButton("Stop everything")
        self.stop_btn.setStyleSheet("padding:8px;")
        self.stop_btn.clicked.connect(self._stop_all)
        row.addWidget(self.stop_btn, 1)
        return row

    def _build_tools_row(self) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()
        self.record_btn = QtWidgets.QPushButton("● Record")
        self.record_btn.setToolTip("ros2 bag record into a new window of rpv_offboard.\n"
                                   "Stopping sends Ctrl-C first so the bag's metadata is written.")
        self.record_btn.clicked.connect(self._toggle_record)
        row.addWidget(self.record_btn, 1)

        self.rviz_btn = QtWidgets.QPushButton("RViz")
        self.rviz_btn.setToolTip("rviz2 with the robot profile's layout -- map, costmaps,\n"
                                 "/scan, plans, footprint and /goal_pose already set up.")
        self.rviz_btn.clicked.connect(self._start_rviz)
        row.addWidget(self.rviz_btn, 1)
        return row

    def _build_onboard_command_row(self) -> QtWidgets.QHBoxLayout:
        """The Pi's start command, ready to copy.

        There is no Start / Stand / Sit button here, and that is deliberate.
        This machine has no route to the Pi, and Spot's lifecycle services do
        not cross the zenoh bridge either, so all four have to be typed in a Pi
        shell. As buttons they could only ever fail: the PI light sat red, the
        log said "ssh: Network is unreachable", and the network got the blame
        for a topology that was working as designed.
        """
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Run on the Pi:"))
        self.onboard_cmd = QtWidgets.QLineEdit(self.pm.onboard_start_command())
        self.onboard_cmd.setReadOnly(True)
        self.onboard_cmd.setToolTip(
            "Type this in a Pi shell, AFTER this machine's stack is up.\n"
            "Then, in that session's `lifecycle` window:\n  "
            + "\n  ".join(STAND_SEQUENCE))
        row.addWidget(self.onboard_cmd, 1)

        copy_btn = QtWidgets.QPushButton("Copy")
        copy_btn.setToolTip("Copy the command to the clipboard.")
        copy_btn.clicked.connect(self._copy_onboard_command)
        row.addWidget(copy_btn)
        return row

    def _build_mode_row(self) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()
        self.mode_btn = QtWidgets.QPushButton("NAV: AUTO")
        self.mode_btn.setCheckable(True)
        self.mode_btn.setStyleSheet("font-weight:bold; padding:8px;")
        self.mode_btn.setToolTip("AUTO: the bridge drives.\n"
                                 "MANUAL: the bridge is stopped and a click on the SLAM map "
                                 "sends a Nav2 goal.")
        self.mode_btn.toggled.connect(self._toggle_mode)
        row.addWidget(self.mode_btn, 1)
        return row

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    def _arm(self, attr: str) -> None:
        """Start the clock on an optimistic yellow light.

        Yellow means "starting". Without an expiry it also came to mean "the
        launcher died in the first second and nobody told you", because the only
        way out of yellow was success. cfg.start_timeout_s = 0 restores that.
        """
        timeout = float(getattr(self.cfg, "start_timeout_s", 0.0) or 0.0)
        setattr(self, attr, time.monotonic() + timeout if timeout > 0 else 0.0)

    def _light_state(self, up, current: str, deadline_attr: str, label: str) -> str:
        """green when it is up, yellow while a start is still within its
        deadline, red otherwise."""
        if up:
            setattr(self, deadline_attr, 0.0)
            return "green"
        if current != "yellow":
            return "red"
        deadline = getattr(self, deadline_attr, 0.0)
        if deadline and time.monotonic() >= deadline:
            setattr(self, deadline_attr, 0.0)
            self._append_log(f"[{label}] still not up {self.cfg.start_timeout_s:.0f} s "
                             "after the start was requested -- calling it failed. "
                             "Scroll up for what the launcher said.")
            return "red"
        return "yellow"

    def _run_bg(self, label: str, fn, *args) -> None:
        def worker():
            self.log_signal.emit(f"» {label} …")
            try:
                res = fn(*args)
                if res is not None and hasattr(res, "ok"):
                    self.log_signal.emit(f"   {label}: {'ok' if res.ok else 'FAILED'} {res.out}".rstrip())
                else:
                    self.log_signal.emit(f"   {label}: done")
            except Exception as e:
                self.log_signal.emit(f"   {label}: error {type(e).__name__}: {e}")
        threading.Thread(target=worker, daemon=True).start()

    def _current_target(self) -> str:
        return self.target_box.currentText().strip()

    def _reload_targets(self) -> None:
        labels = shortvis.load_labels(self.cfg.shortvis_path)
        current = self.target_box.currentText()
        self.target_box.clear()
        self.target_box.addItems(labels or [self.cfg.default_target])
        if current:
            self.target_box.setCurrentText(current)

    def _warn_about_config(self) -> None:
        if self.cfg.d435_partial():
            self._append_log("[config] d435_* mount pose is PARTIALLY set. It is all six or "
                             "none -- a guessed pitch tilts everything both Nav2 and the "
                             "policy see. The Pi launcher refuses to start until this is "
                             "fixed; blank it to use the robot profile's recorded mount.")
        elif not self.cfg.d435_complete():
            self._append_log(f"[config] d435_* blank -- the Pi will use the mount pose "
                             f"recorded in ceai/robots/{self.cfg.robot}.py. Fill these in "
                             f"only to override it for a remounted camera.")
        if not self.cfg.shortvis_path.exists():
            self._append_log(f"[config] SAM3 class list not found at {self.cfg.shortvis_path} -- "
                             "check policy_repo in Settings.")

    # ------------------------------------------------------------------ #
    # button handlers
    # ------------------------------------------------------------------ #
    def _start_all(self) -> None:
        target = self._current_target()
        self.cfg.default_target = target
        self.cfg.save()
        self.run_readout.mark_run_start()
        self._offboard_state = "yellow"
        self._arm("_offboard_deadline")
        self.lights["offboard"].set_state("yellow")

        def worker():
            self.log_signal.emit(f"» starting this machine's stack (rpv_offboard) -> '{target}' …")
            self.pm.start_offboard(target, log=self.log_signal.emit)
            # The script creates `zenoh` first and only makes `ros` once the port
            # is listening, so `ros` appearing is the signal that the Pi can
            # safely connect out. Waiting on the window is better than sleeping:
            # a slow machine is not a failure.
            if self._wait_for_offboard_window("ros", 90):
                self.log_signal.emit("   zenoh is listening, SLAM starting. "
                                     "START THE PI NOW.")
            else:
                self.log_signal.emit("   zenoh did not come up within 90 s -- check "
                                     "`tmux attach -t rpv_offboard` before starting the Pi.")
            self.log_signal.emit(f"» on a Pi shell:  {self.pm.onboard_start_command()}")
            self.log_signal.emit("» the AI servers (:12182 :12183 :13000) are still loading. "
                                 "The AI light turns green when the policy answers; the PI "
                                 "light turns green when /scan arrives.")
        threading.Thread(target=worker, daemon=True).start()

    def _wait_for_offboard_window(self, name: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if name in self.pm.offboard_windows():
                return True
            time.sleep(2.0)
        return False

    def _stop_all(self) -> None:
        if not self._confirm(
                "Stop this machine's stack?",
                "This kills the rpv_offboard tmux session: SLAM, Nav2, the policy and "
                "the bridge.\n\nThe PI KEEPS RUNNING -- stop it from a Pi shell "
                "with `launch_onboard.sh stop`, which SIGINTs the camera first. "
                "Spot also stays standing; sit it first if you are finished."):
            return
        self._offboard_state = "red"
        self.lights["offboard"].set_state("red")
        self._run_bg("stop this machine's stack (rpv_offboard)", self.pm.stop_offboard)
        self._append_log(f"[pi] still running. To stop it, in a Pi shell: "
                         f"{self.cfg.onboard_ceai_dir}/scripts/launch_onboard.sh stop")

    def _toggle_offboard(self) -> None:
        if self._offboard_state in ("green", "yellow"):
            self._offboard_state = "red"
            self.lights["offboard"].set_state("red")
            self._run_bg("stop the workstation (rpv_offboard)", self.pm.stop_offboard)
        else:
            self._offboard_state = "yellow"
            self._arm("_offboard_deadline")
            self.lights["offboard"].set_state("yellow")
            target = self._current_target()
            self._run_bg(f"start the workstation -> '{target}'",
                         lambda: self.pm.start_offboard(target, log=self.log_signal.emit))

    def _set_target(self) -> None:
        target = self._current_target()
        if not target:
            return
        self.cfg.default_target = target
        self.cfg.save()
        self.run_readout.mark_run_start()
        self._run_bg(f"restart the bridge -> '{target}' (new episode)",
                     self.pm.restart_bridge, target)

    def _toggle_mode(self, manual: bool) -> None:
        self._manual = manual
        self.map_panel.click_enabled = manual
        self.mode_btn.setText("NAV: MANUAL" if manual else "NAV: AUTO")
        slam_idx = self.ai_tabs.indexOf(self.map_panel)
        self.ai_tabs.setTabText(slam_idx, self.SLAM_TAB_MANUAL if manual else self.SLAM_TAB_AUTO)
        if manual:
            self.ai_tabs.setCurrentIndex(slam_idx)   # the click target must be visible
        if manual:
            self._run_bg("MANUAL: stop the autonomous bridge", self.pm.stop_bridge)
        else:
            self.run_readout.mark_run_start()
            self._run_bg("AUTO: restart the bridge", self.pm.restart_bridge, self._current_target())

    def _toggle_record(self) -> None:
        if self._recording:
            self._stop_run_records()
            self._run_bg("stop recording (Ctrl-C, then close the window)", self.pm.stop_recording)
        else:
            target = self._current_target()
            name = self.pm.run_name(target)
            log_path = self.recorder.start(name, bag_path=self.pm.bag_path(name), target=target)
            self._append_log(f"[record] {name}: bag -> {self.pm.bag_path(name)}")
            self._append_log(f"[record] decisions -> {log_path}; videos -> "
                             f"{self.cfg.records_dir}/videos/{{value_map,policy}}/{name}{self.recorder.video_ext}"
                             f" ({self.recorder.encoder})")
            self._run_bg(f"start recording -> {self.cfg.bag_dir}",
                         self.pm.start_recording, target, name)

    def _stop_run_records(self) -> None:
        if not self.recorder.active:
            return
        name = self.recorder.name
        counts = self.recorder.stop()
        frames = ", ".join(f"{k.replace('_jpeg_bytes', '')}={v}" for k, v in counts.items())
        self._append_log(f"[record] {name}: closed decision log and videos ({frames} frames)")

    def _start_rviz(self) -> None:
        self._run_bg(f"open RViz with {self.cfg.rviz_path}", self.pm.start_rviz)

    def _copy_onboard_command(self) -> None:
        # A headless or minimal session can have no clipboard at all; selecting
        # the text is still useful and a traceback here is not.
        clipboard = QtWidgets.QApplication.clipboard()
        if clipboard is None:
            self.onboard_cmd.selectAll()
            self._append_log("[pi] no clipboard available -- the command is selected; "
                             "copy it by hand.")
            return
        clipboard.setText(self.onboard_cmd.text())
        self._append_log("[pi] start command copied to the clipboard.")

    def _confirm(self, title: str, text: str) -> bool:
        yes = qenum(QtWidgets.QMessageBox, "Yes")
        no = qenum(QtWidgets.QMessageBox, "No")
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(title)
        box.setText(text)
        box.setIcon(qenum(QtWidgets.QMessageBox, "Warning"))
        box.setStandardButtons(yes | no)
        box.setDefaultButton(no)
        return exec_dialog(box) == yes

    def _open_settings(self) -> None:
        exec_dialog(SettingsDialog(self.cfg, self))

    def _open_shortvis(self) -> None:
        dlg = ShortvisDialog(self.cfg, self)
        dlg.saved.connect(self._on_shortvis_saved)
        exec_dialog(dlg)

    def _on_shortvis_saved(self, backup: str) -> None:
        self._reload_targets()
        self._append_log(f"[sam3] class list written (backup: {backup})")
        if self._confirm("Restart the policy?",
                         "The policy reads the class list ONCE and caches it, so the change "
                         "does not take effect until it restarts.\n\n"
                         "Restart the policy server now? (~30 s; the models stay loaded in "
                         "the other windows.)"):
            self._run_bg("restart the policy server", self.pm.restart_policy)

    # ------------------------------------------------------------------ #
    # data slots
    # ------------------------------------------------------------------ #
    def _on_status(self, snap: dict) -> None:
        self.recorder.feed(snap)
        self.run_readout.update_run(snap)
        mode = str(snap.get("mode", "—")).upper()
        step = snap.get("step", 0)
        bits = [f"Mode: {mode}", f"step {step}", f"target: {snap.get('objectgoal', '—')}"]
        if snap.get("target_detected"):
            conf = snap.get("target_confidence")
            bits.append("TARGET DETECTED" if conf is None
                        else f"TARGET DETECTED (SAM3 {float(conf):.2f})")
        if snap.get("called_stop"):
            bits.append("STOPPED — target reached")
        self.thinking.setText(" · ".join(bits))

        line = f"step {step:>4}  mode={snap.get('mode', '?')}"
        det = str(snap.get("detections", "")).strip()
        if det:
            line += f"  {det}"
        dbg = str(snap.get("debug", "")).strip()
        if dbg:
            line += f"  {dbg}"
        if line != getattr(self, "_last_policy_line", None):
            self._append_log(line)
            self._last_policy_line = line

        if "annotated_rgb_jpeg_bytes" in snap:
            self.cam_panel.set_jpeg(snap["annotated_rgb_jpeg_bytes"])
            self._last_cam_ts = time.monotonic()
        if "value_map_jpeg_bytes" in snap:
            self.value_panel.set_jpeg(snap["value_map_jpeg_bytes"])
        if "obstacle_map_jpeg_bytes" in snap:
            self.obstacle_panel.set_jpeg(snap["obstacle_map_jpeg_bytes"])

    def _on_system_state(self, state: dict) -> None:
        # Hold the optimistic yellow set on click until the session appears, so a
        # slow launch does not flash red and look like a failure -- but only
        # until its deadline, because a yellow that never expires is how a dead
        # launcher passed for a slow one.
        self._offboard_state = self._light_state(state.get("offboard"), self._offboard_state,
                                           "_offboard_deadline", "rpv_offboard")
        self.lights["offboard"].set_state(self._offboard_state)

        if not state.get("offboard"):
            self.lights["ai"].set_state("red")
        elif state.get("policy"):
            self.lights["ai"].set_state("green")
        else:
            self.lights["ai"].set_state("yellow")

        self.lights["bridge"].set_state(bool(state.get("bridge")))
        self.lights["slam"].set_state(bool(state.get("slam")))

        was_recording = self._recording
        self._recording = bool(state.get("recording"))
        # The bag window went away without the button (Ctrl-C in tmux, the
        # session stopped): close the log and videos rather than record forever.
        if was_recording and not self._recording:
            self._stop_run_records()
        self.record_btn.setText("■ Stop recording" if self._recording else "● Record")
        self.record_btn.setStyleSheet("font-weight:bold; color:#c1121f;" if self._recording else "")

    def _refresh_link_lights(self) -> None:
        """PI and CAM are liveness, not session state: both mean "data is
        arriving right now"."""
        now = time.monotonic()
        self.lights["cam"].set_state("green" if (now - self._last_cam_ts) < 1.0 else "red")
        self.lights["onboard"].set_state(
            "green" if (now - self._last_scan_ts) < LINK_TIMEOUT_S else "red")
        # Advance the run clock between the ~3 Hz status polls.
        self.run_readout.tick()

    def _on_scan(self, _points) -> None:
        """/scan arrived. The points themselves go straight to the map panel."""
        self._last_scan_ts = time.monotonic()

    def _on_camera(self, payload: dict) -> None:
        # Every raw frame goes to the live panel. For the annotated panel the
        # /status feed wins whenever it is arriving; the raw topic is only a
        # fallback for when the policy is not running yet.
        live = (time.monotonic() - self._last_cam_ts) < 2.0
        targets = [self.live_panel] if live else [self.live_panel, self.cam_panel]
        for panel in targets:
            if "jpeg" in payload:
                panel.set_jpeg(payload["jpeg"])
            elif "rgb" in payload:
                panel.set_rgb(payload["rgb"], payload["w"], payload["h"])
        self._last_cam_ts = time.monotonic()

    def _on_map_click(self, wx: float, wy: float) -> None:
        if self.ros.publish_goal(wx, wy):
            self._append_log(f"[manual] goal sent: ({wx:.2f}, {wy:.2f}) on {self.cfg.goal_topic}")
        else:
            self._append_log("[manual] goal NOT sent (ROS unavailable)")

    def _append_log(self, text: str) -> None:
        self.policy_log.appendPlainText(f"{time.strftime('%H:%M:%S')}  {text}")

    # ------------------------------------------------------------------ #
    def closeEvent(self, event) -> None:
        self._stop_run_records()
        for attr in ("status_poller", "system_poller"):
            poller = getattr(self, attr, None)
            if poller is not None:
                poller.stop()
                poller.wait(1000)
        ros = getattr(self, "ros", None)
        if ros is not None:
            ros.shutdown()
        super().closeEvent(event)


def main() -> int:
    cfg = Config.load()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow(cfg)
    win.start_workers()
    win.show()
    return exec_app(app)


if __name__ == "__main__":
    sys.exit(main())
