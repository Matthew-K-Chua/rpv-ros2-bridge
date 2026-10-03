"""Locating the rpv-sim2real-policy checkout, for the depth wire codec.

The encoder lives next to the decoder the policy server uses so the two ends
cannot drift apart. It pulls in numpy and cv2 only -- nothing that would drag
torch or flask into this process. Both raster depth sources (`realsense` and
`zoedepth`) need it; `lidar` and `stub` do not.

The checkout is in a different place on every machine (SYSTEM_CONTEXT section 1),
so RPV_POLICY_REPO wins and the default location is tried after it. Note the
spelling: the variable is RPV, the repo is RVP.
"""
import os
import sys

CANDIDATES = [
    os.environ.get("RPV_POLICY_REPO"),
    os.path.join(os.path.expanduser("~"), "rpv-sim2real-policy"),
]

repo = None
tried = []
import_error = None
_encode = None

for _candidate in CANDIDATES:
    if not _candidate:
        continue
    tried.append(_candidate)
    if not os.path.isdir(os.path.join(_candidate, "vlfm")):
        continue
    if _candidate not in sys.path:
        sys.path.insert(0, _candidate)
    try:
        from vlfm.vlm.depth_codec import encode_png16_mm as _encode
        repo = _candidate
        import_error = None
        break
    except Exception as exc:        # depth_source='lidar' must not care
        _encode = None
        import_error = exc


def available() -> bool:
    return _encode is not None


def encode_png16_mm(mm):
    """uint16 millimetre raster -> base64 16-bit PNG."""
    if _encode is None:
        raise RuntimeError(unavailable_message())
    return _encode(mm)


def unavailable_message() -> str:
    detail = (f"  Last import error: {import_error}\n" if import_error
              else "  None of those contained a 'vlfm' directory.\n")
    return (
        "depth_source='realsense' and 'zoedepth' need the rpv-sim2real-policy checkout importable "
        "for the depth wire codec (vlfm.vlm.depth_codec).\n"
        "  Looked in: " + ", ".join(tried or ["<nothing>"]) + "\n" + detail +
        "  Fix: export RPV_POLICY_REPO=/path/to/rpv-sim2real-policy "
        "(the launcher scripts set "
        "it), or use depth_source='lidar' (the /scan curtain, no codec).")
