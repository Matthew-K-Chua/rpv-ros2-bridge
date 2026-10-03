"""Make THIS checkout's `ceai` the one a launch file imports, built or not.

Every launch file here reads its constants from the `ceai` package -- camera
profiles, robot profiles, the mount pose -- so that a number lives in exactly one
place. The trap is that there can be TWO copies of that place: the checkout the
launch file sits in, and a colcon-installed copy under `install/ceai/...` that a
sourced workspace puts on the path first.

Until 2026-09-16 this helper only added the checkout when `ceai` was not
importable at all. On a machine that has run `colcon build` once (the Pi does,
the Pi has) the installed copy was importable, so it won -- and then
`git pull` changed `ceai/robots/spot.py` in the checkout while the launch kept
publishing the mount pose from the stale install. The pitch "did nothing", with
no error anywhere. Now the directory beside the launch file is always first, and
`which_ceai()` says which copy was actually imported so the stack window shows
it at start-up.
"""
import importlib.util
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(REPO_ROOT, "config")
LAUNCH_DIR = os.path.join(REPO_ROOT, "launch")


def _is_under(path: str, root: str) -> bool:
    path = os.path.realpath(path)
    root = os.path.realpath(root)
    return path == root or path.startswith(root + os.sep)


def ensure_importable() -> str:
    """Guarantee `import ceai` resolves to the package beside this file.

    Puts the repo root at the FRONT of sys.path, ahead of any sourced install,
    and evicts a `ceai` that was already imported from somewhere else so the
    next import re-resolves here. When this file itself lives in an install
    (`ros2 launch ceai ...`), the root has no `ceai/` package and the installed
    one is imported as before. Returns the repo root.
    """
    if sys.path[:1] != [REPO_ROOT]:
        while REPO_ROOT in sys.path:
            sys.path.remove(REPO_ROOT)
        sys.path.insert(0, REPO_ROOT)

    if os.path.isdir(os.path.join(REPO_ROOT, "ceai")):
        loaded = sys.modules.get("ceai")
        loaded_from = getattr(loaded, "__file__", None) if loaded else None
        if loaded_from and not _is_under(loaded_from, REPO_ROOT):
            for name in [m for m in sys.modules
                         if m == "ceai" or m.startswith("ceai.")]:
                del sys.modules[name]
            importlib.invalidate_caches()
    return REPO_ROOT


def which_ceai() -> str:
    """The directory the imported `ceai` package came from, for the start-up
    print. A stale install and a fresh checkout look identical once the TF is
    up; this is the line that tells them apart."""
    module = sys.modules.get("ceai")
    if module is None:
        spec = importlib.util.find_spec("ceai")
        origin = spec.origin if spec else None
    else:
        origin = getattr(module, "__file__", None)
    return os.path.dirname(origin) if origin else "<ceai not importable>"
