"""Qt binding shim.

The console must run inside the ROS 2 Humble Python env so it can also be an
``rclpy`` node. That env usually ships PyQt5 (rqt depends on it); a fresh
PyQt6/PySide6 may or may not be present. Import whichever is available and
normalise the handful of enums we use to *flat* access (``Qt.AlignCenter``,
``QImage.Format_RGB888``, ...) so the rest of the GUI code is binding-agnostic.

Preference order: PyQt5 (most likely in ROS Humble) -> PyQt6 -> PySide6.
"""
from __future__ import annotations

BINDING = None
QtCore = QtGui = QtWidgets = None
Signal = Slot = None

_errors = []
for _name in ("PyQt5", "PyQt6", "PySide6"):
    try:
        if _name == "PyQt5":
            from PyQt5 import QtCore, QtGui, QtWidgets  # type: ignore
            from PyQt5.QtCore import pyqtSignal as Signal, pyqtSlot as Slot  # type: ignore
        elif _name == "PyQt6":
            from PyQt6 import QtCore, QtGui, QtWidgets  # type: ignore
            from PyQt6.QtCore import pyqtSignal as Signal, pyqtSlot as Slot  # type: ignore
        else:
            from PySide6 import QtCore, QtGui, QtWidgets  # type: ignore
            from PySide6.QtCore import Signal, Slot  # type: ignore
        BINDING = _name
        break
    except Exception as e:  # pragma: no cover - depends on host
        _errors.append(f"{_name}: {e}")

if BINDING is None:
    raise ImportError(
        "No Qt binding found. Install one into the ROS env, e.g.\n"
        "  sudo apt install python3-pyqt5   (or)   pip install PyQt5\n"
        "Tried:\n  " + "\n  ".join(_errors)
    )


def _flatten_enums() -> None:
    """On Qt6, enum members live on nested enum classes (Qt.AlignmentFlag.*).
    Alias the ones we use back onto their parent so flat access works."""
    Qt = QtCore.Qt
    QImage = QtGui.QImage

    def alias(parent, names):
        for n in names:
            if hasattr(parent, n):
                continue
            for attr in dir(parent):
                member = getattr(parent, attr)
                if isinstance(member, type) and hasattr(member, n):
                    setattr(parent, n, getattr(member, n))
                    break

    alias(Qt, [
        "AlignCenter", "AlignLeft", "AlignTop", "KeepAspectRatio",
        "SmoothTransformation", "LeftButton", "black", "darkGray", "red",
        "green", "yellow", "white", "transparent",
    ])
    alias(QImage, ["Format_RGB888", "Format_Grayscale8", "Format_RGBA8888"])


if BINDING in ("PyQt6", "PySide6"):
    _flatten_enums()


def exec_app(app) -> int:
    """exec() (Qt6/PySide) vs exec_() (older PyQt) compatibility."""
    return app.exec() if hasattr(app, "exec") else app.exec_()


def exec_dialog(dlg) -> int:
    """exec() (Qt6/PySide) vs exec_() (older PyQt) for a modal dialog."""
    return dlg.exec() if hasattr(dlg, "exec") else dlg.exec_()


def qenum(owner, name):
    """Enum member, flat (Qt5) or nested (Qt6).

    `_flatten_enums` aliases the handful of Qt/QImage enums the widgets use;
    this looks up any other one on demand (dialog button roles, message-box
    icons) so the console works on PyQt5 and PyQt6/PySide6 alike.
    """
    if hasattr(owner, name):
        return getattr(owner, name)
    for attr in dir(owner):
        member = getattr(owner, attr)
        if isinstance(member, type) and hasattr(member, name):
            return getattr(member, name)
    raise AttributeError(f"{owner.__name__} has no enum member {name!r}")
