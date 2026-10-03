"""The console's two modal dialogs: the SAM3 class list and Settings.

Both are plain QDialogs over `Config`. They live apart from the main window so
that rpv_console.py is only the window and its wiring.
"""
from __future__ import annotations

from . import shortvis
from .config import Config
from .qtcompat import QtWidgets, Signal, qenum

# Body text colour for the dialog blurbs; matches the console's policy panel.
TEXT_FG = "#000000"


class ShortvisDialog(QtWidgets.QDialog):
    """Add and remove SAM3 prompt classes, and write data/shortvis.yaml."""

    saved = Signal(str)

    def __init__(self, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self.setWindowTitle("SAM3 class list")
        self.resize(460, 460)

        lay = QtWidgets.QVBoxLayout(self)
        blurb = QtWidgets.QLabel(
            "These labels are prompted to SAM3 on every step, together with the "
            "target. What they detect is scored for co-occurrence with the target "
            "and becomes the value map -- so add what PREDICTS the target, not what "
            "is merely interesting. Every label costs time on every frame.\n\n"
            "The target itself is always prompted, whether or not it is listed."
        )
        blurb.setWordWrap(True)
        blurb.setStyleSheet(f"color:{TEXT_FG};")
        lay.addWidget(blurb)

        path_label = QtWidgets.QLabel(str(cfg.shortvis_path))
        path_label.setStyleSheet("color:#444; font-family:monospace; font-size:11px;")
        path_label.setWordWrap(True)
        lay.addWidget(path_label)

        self.list = QtWidgets.QListWidget()
        self.list.addItems(shortvis.load_labels(cfg.shortvis_path))
        lay.addWidget(self.list, 1)

        add_row = QtWidgets.QHBoxLayout()
        self.entry = QtWidgets.QLineEdit()
        self.entry.setPlaceholderText("new class, e.g. 'fire extinguisher'")
        self.entry.returnPressed.connect(self._add)
        add_row.addWidget(self.entry, 1)
        add_btn = QtWidgets.QPushButton("Add")
        add_btn.clicked.connect(self._add)
        add_row.addWidget(add_btn)
        rm_btn = QtWidgets.QPushButton("Remove selected")
        rm_btn.clicked.connect(self._remove)
        add_row.addWidget(rm_btn)
        lay.addLayout(add_row)

        self.error = QtWidgets.QLabel("")
        self.error.setStyleSheet("color:#c1121f;")
        self.error.setWordWrap(True)
        lay.addWidget(self.error)

        buttons = QtWidgets.QDialogButtonBox(
            qenum(QtWidgets.QDialogButtonBox, "Save")
            | qenum(QtWidgets.QDialogButtonBox, "Cancel"))
        buttons.accepted.connect(self._save)
        buttons.rejected.connect(self.reject)
        lay.addWidget(buttons)

    def labels(self):
        return [self.list.item(i).text() for i in range(self.list.count())]

    def _add(self) -> None:
        try:
            label = shortvis.validate(self.entry.text())
        except shortvis.ShortvisError as e:
            self.error.setText(str(e))
            return
        if label.lower() in {x.lower() for x in self.labels()}:
            self.error.setText(f"{label!r} is already in the list")
            return
        self.list.addItem(label)
        self.entry.clear()
        self.error.setText("")

    def _remove(self) -> None:
        for item in self.list.selectedItems():
            self.list.takeItem(self.list.row(item))
        self.error.setText("")

    def _save(self) -> None:
        try:
            backup = shortvis.save_labels(self.cfg.shortvis_path, self.labels())
        except Exception as e:
            self.error.setText(f"{type(e).__name__}: {e}")
            return
        self.saved.emit(str(backup))
        self.accept()


def _robot_choices():
    """Known robot names from the registry, so the dropdown stays in sync with
    ceai/robots/ -- add a profile there and it appears here, no GUI edit."""
    try:
        from ceai.robots import names
        return names()
    except Exception:
        return []


def _camera_choices():
    try:
        from ceai.cameras import names
        return names()
    except Exception:
        return []


class SettingsDialog(QtWidgets.QDialog):
    """One row per Config field in FIELDS; Save writes settings.json.

    Fields in CHOICES get a dropdown populated from the relevant registry (so a
    mistyped robot -- the failure ceai/robots/__init__.py is built to refuse --
    cannot be entered); everything else is a free-text line edit."""

    # field -> a callable returning the allowed values (resolved at open time).
    CHOICES = {
        "robot": _robot_choices,
        "camera_model": _camera_choices,
        "depth_source": lambda: ["realsense", "zoedepth", "lidar", "stub"],
    }

    FIELDS = [
        ("robot", "robot"),
        ("camera_model", "camera_model"),
        ("onboard_ssh_user", "Pi SSH user (for the displayed command only)"),
        ("onboard_ssh_host", "Pi SSH host (for the displayed command only)"),
        ("onboard_ceai_dir", "CEAI on the Pi"),
        ("offboard_ssh", "Tunnel target (user@workstation)"),
        ("policy_repo", "rpv-sim2real-policy checkout on this machine"),
        ("policy_port", "Policy port"),
        ("zoedepth_port", "ZoeDepth port (only used by depth_source=zoedepth)"),
        ("depth_source", "depth_source (realsense | zoedepth | lidar | stub)"),
        ("d435_x", "d435_x  (all six or none)"),
        ("d435_y", "d435_y"),
        ("d435_z", "d435_z"),
        ("d435_roll", "d435_roll"),
        ("d435_pitch", "d435_pitch (positive = lens down)"),
        ("d435_yaw", "d435_yaw"),
        ("camera_topic", "Live camera topic (blank = the profile's rgb_topic)"),
        ("bag_dir", "rosbag directory"),
        ("records_dir", "Run records directory (decision log + videos)"),
        ("rviz_config", "RViz config (blank = the robot profile's)"),
        ("onboard_launch_args", "Extra robot_onboard.launch.py args"),
        ("start_timeout_s", "Yellow-light timeout, seconds (0 = never expire)"),
    ]

    def __init__(self, cfg: Config, parent=None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self.setWindowTitle("Settings")
        form = QtWidgets.QFormLayout(self)
        self.edits = {}
        for key, label in self.FIELDS:
            cur = str(getattr(cfg, key))
            if key in self.CHOICES:
                e = QtWidgets.QComboBox()
                choices = list(self.CHOICES[key]())
                # Never silently drop a value the config already holds (an
                # unregistered robot/camera), even while flagging it is unknown.
                if cur and cur not in choices:
                    choices = [cur] + choices
                e.addItems(choices)
                e.setCurrentText(cur)
            else:
                e = QtWidgets.QLineEdit(cur)
            self.edits[key] = e
            form.addRow(label, e)
        buttons = QtWidgets.QDialogButtonBox(
            qenum(QtWidgets.QDialogButtonBox, "Save")
            | qenum(QtWidgets.QDialogButtonBox, "Cancel"))
        buttons.accepted.connect(self._save)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    @staticmethod
    def _widget_text(e) -> str:
        """The current value of a line edit OR a combo box."""
        getter = getattr(e, "currentText", None) or e.text
        return getter().strip()

    def _save(self) -> None:
        for key, e in self.edits.items():
            cur = getattr(self.cfg, key)
            val = self._widget_text(e)
            setattr(self.cfg, key, val if isinstance(cur, str) else type(cur)(val))
        self.cfg.save()
        QtWidgets.QMessageBox.information(
            self, "Settings", "Saved to gui/settings.json.\nRestart the console to apply.")
        self.accept()
