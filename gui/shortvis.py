"""Read and write the SAM3 prompt list (data/shortvis.yaml in rpv-sim2real-policy).

This list is not decoration. With SKIP_YOLOE=1 -- which is the default on
the workstation -- YOLO-E is not running, and these labels ARE the prompt set SAM3
is given on every frame, plus the target, which the policy appends itself. Every
object detected from them is scored by CLIP for co-occurrence with the target and
becomes part of the value map, so this file is what decides whether "there is a
monitor and a whiteboard over there" makes the robot believe that direction is
worth exploring.

Two consequences the editor surfaces in the UI:
  * Adding the target here changes nothing -- it is prompted regardless.
  * Every label costs SAM3 time on every step. This is why the list is eight
    items and data/lvis.yaml's 1200 are for the benchmark.

The written form is deliberately the plainest YAML that
vlfm/vlm/lvis_classes.py's loader accepts (a `names:` mapping of consecutive
integers), because that loader is also read by the no-PyYAML fallback parser
below and by a human in a hurry.
"""
from __future__ import annotations

import datetime
import shutil
from pathlib import Path
from typing import Iterable, List, Tuple

HEADER = """\
# SAM3 prompt list for the RPV experiment.
#
# Selected with SKIP_YOLOE_CLASSES (scripts/launch_offboard_ai.sh defaults to
# this file). With SKIP_YOLOE=1 these labels are prompted to SAM3 directly on
# every step, together with the target, and everything detected from them feeds
# the RPV value map.
#
# The policy reads this file ONCE, at the first step of a run, and caches it --
# restart the policy after editing.
#
# Managed by the RPV console's "Edit SAM3 list" dialog; hand edits are fine and
# are preserved as long as the `names:` mapping stays consecutive from 0.
"""


class ShortvisError(RuntimeError):
    """Raised for a label or a file this module refuses to write."""


def _parse_fallback(text: str) -> List[str]:
    """Parse `names:` without PyYAML, for a ROS env that lacks it.

    Deliberately tolerant in the same way the policy repository's loader is: it reads the
    VALUE after the first colon, so both `0: chair` and `0: "chair"` work.
    """
    labels: List[str] = []
    in_names = False
    for raw in text.splitlines():
        stripped = raw.strip()
        if not in_names:
            if stripped.startswith("names:"):
                in_names = True
            continue
        if raw and not raw[0].isspace():        # next top-level key ends the block
            break
        if stripped.startswith("#") or not stripped:
            continue
        if ":" in stripped:
            label = stripped.split(":", 1)[1].strip().strip("'\"")
            if label:
                labels.append(label)
    return labels


def load_labels(path: Path) -> List[str]:
    """Labels from `path`, in file order. Returns [] if it cannot be read."""
    try:
        text = Path(path).read_text()
    except Exception:
        return []

    try:
        import yaml  # type: ignore

        names = (yaml.safe_load(text) or {}).get("names")
        if isinstance(names, dict):
            # Sort by key so the file's own numbering wins over dict ordering.
            ordered = [names[k] for k in sorted(names, key=lambda k: int(k))]
            labels = [str(v).strip() for v in ordered if str(v).strip()]
            if labels:
                return labels
        if isinstance(names, list):
            return [str(v).strip() for v in names if str(v).strip()]
    except Exception:
        pass

    return _parse_fallback(text)


def validate(label: str) -> str:
    """Return the cleaned label, or raise ShortvisError saying why not.

    Spaces are fine and common ("dining table") -- a label is ONE prompt, never
    split on whitespace. A colon or a leading '#' would change the shape of the
    file rather than the content of the list, so those are refused rather than
    quoted around.
    """
    cleaned = " ".join(str(label).split())
    if not cleaned:
        raise ShortvisError("empty label")
    if ":" in cleaned:
        raise ShortvisError(f"{cleaned!r} contains ':' -- it would break the YAML mapping")
    if cleaned.startswith("#"):
        raise ShortvisError(f"{cleaned!r} starts with '#' -- YAML would read it as a comment")
    if len(cleaned) > 80:
        raise ShortvisError(f"{cleaned!r} is longer than 80 characters")
    return cleaned


def normalise(labels: Iterable[str]) -> Tuple[List[str], List[str]]:
    """(kept, dropped): clean every label, drop blanks and case-insensitive
    duplicates, preserve order. Duplicates are dropped rather than refused --
    prompting SAM3 with the same word twice is waste, not an error."""
    kept: List[str] = []
    dropped: List[str] = []
    seen = set()
    for raw in labels:
        try:
            cleaned = validate(raw)
        except ShortvisError:
            dropped.append(str(raw))
            continue
        if cleaned.lower() in seen:
            dropped.append(cleaned)
            continue
        seen.add(cleaned.lower())
        kept.append(cleaned)
    return kept, dropped


def render(labels: Iterable[str]) -> str:
    """The exact file text for `labels`. Pure, so the tests can check it."""
    kept, _ = normalise(labels)
    if not kept:
        raise ShortvisError("refusing to write an empty class list -- SAM3 would be prompted with the target alone")
    stamp = datetime.date.today().isoformat()
    out = [HEADER, f"# Last written by the RPV console on {stamp}.", "names:"]
    for i, label in enumerate(kept):
        out.append(f"  {i}: {label}")
    return "\n".join(out) + "\n"


def save_labels(path: Path, labels: Iterable[str]) -> Path:
    """Write the list, keeping one backup. Returns the backup path.

    The backup is worth the line: this file is in the OTHER repo, so a bad write
    shows up as a `git diff` in a checkout you were not thinking about.
    """
    path = Path(path)
    text = render(labels)                       # raises before anything is touched
    backup = path.with_suffix(path.suffix + ".bak")
    if path.exists():
        shutil.copy2(path, backup)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return backup
