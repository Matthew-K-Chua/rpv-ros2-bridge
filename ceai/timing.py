"""
Per-step stage timing for the ROS half of the control loop.

TimingLog replaces the bridge's ad-hoc stage dict (`self._t`) and its
`_log_timing()` method with one object that owns both the stage store and the
enable flag. Deliberately rclpy-free: it takes any logger with an `info()`
method, so it can be exercised offline (test_md/10) without a ROS graph.
"""
import os

# Per-step timing for the ROS half of the loop (docs §10.2). The policy process has
# its own RPV_TIMING line; without this one you cannot tell an A-11 win (less spin
# blocking) from an A-15 win (less camera lag), because both show up as "the step got
# faster" and neither is visible from the policy side.
TIMING_ENABLED = os.environ.get("RPV_TIMING", "") == "1"


class TimingLog:
    """Stage durations for the step being assembled, in ms.

    Stages are recorded dict-style (`timing['spin'] = ms`; `timing['post'] += ms`
    works too). Values are usually numbers (milliseconds) but may be short strings
    (the 'occ' stage stores 'full' / 'hash' / 'full(retry)'); log() formats both.
    """

    def __init__(self, logger):
        self._logger = logger
        self._t = {}

    def __setitem__(self, key, value):
        self._t[key] = value

    def __getitem__(self, key):
        return self._t[key]

    def log(self, note):
        """Emit one timing line for the step just finished, then reset the stages.

        Reset unconditionally, including on the disabled path: a stage recorded by
        _spin_join must not leak into the next step's line if this one bailed out.
        """
        stages, self._t = self._t, {}
        if not TIMING_ENABLED:
            return
        order = ['spin', 'settle', 'frame_wait', 'build_payload', 'post']
        keys = [k for k in order if k in stages] + [k for k in stages if k not in order]
        total = sum(v for v in stages.values() if isinstance(v, (int, float)))
        parts = [f"{k}={stages[k]:.0f}" if isinstance(stages[k], (int, float))
                 else f"{k}={stages[k]}" for k in keys]
        self._logger.info(
            f"[timing] ros_total={total:.0f}ms | " + " ".join(parts) + f" | {note}")
