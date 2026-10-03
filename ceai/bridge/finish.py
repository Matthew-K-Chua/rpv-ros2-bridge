"""What the bridge does AFTER the policy says the target is reached.

The policy's `called_stop` fires at pointnav_stop_radius (1.4 m on Spot) while
Nav2 is still walking the last stretch, so "finished" is not "stopped". The
sequence here turns one into the other and then sits the robot down:

    1. cancel every NavigateToPose goal, so Nav2 stops sending cmd_vel
       (a velocity command arriving after the sit would stand Spot back up);
    2. wait for /odometry to report the base still;
    3. announce "sit <run-id>" on /ceai/finish, repeatedly for a few seconds,
       for ceai/finish_listener.py on the Pi, which calls /sit where it is
       local (topics cross zenoh; the direct service call below sat nothing on
       2026-09-19);
    4. call the driver's `/sit` Trigger service directly as well.

OPT-IN. `sit_on_finish` defaults to False in ceai/bridge/config.py and the
control loop simply returns when it is off, exactly as it did before
2026-09-19. config/spot_bridge.yaml turns it on for Spot; disable with
`-p sit_on_finish:=false` or by deleting the line from the yaml.

The four actions are injected as plain callables so this module has no rclpy
import and the sequence is testable without a service server:

    cancel_nav()   -> bool
    wait_still()   -> bool
    announce(text) -> bool          (publishes one copy; the sequence repeats it)
    call_sit()     -> (ok: bool, message: str)

KNOWN RISK, UNVERIFIED on this rig: it has been measured (2026-09-09)
that Spot's services did not cross zenoh-bridge-ros2dds (measured 2026-09-09,
before the ROS_DISTRO cause was found). If `/sit` is still unreachable from
the workstation the call times out, the outcome is logged, and the robot is left
standing. Nothing else in the run is affected.
"""
import time

ANNOUNCE_HZ = 2.0


class FinishSequence:

    def __init__(self, enabled, cancel_nav, wait_still, call_sit, logger=None,
                 announce=None, announce_s=0.0, sleep=time.sleep, run_id=None):
        self.enabled = bool(enabled)
        self._cancel_nav = cancel_nav
        self._wait_still = wait_still
        self._call_sit = call_sit
        self._announce = announce
        self._announce_s = float(announce_s)
        self._sleep = sleep
        self._run_id = run_id
        self._logger = logger
        # What happened, for the log and for tests: None until run() is called.
        self.outcome = None

    def run(self):
        """Run the sequence once. Never raises: a failed step is logged and the
        result recorded in `outcome`, so the control loop always returns."""
        if not self.enabled:
            self.outcome = {"enabled": False}
            self._info("Finish: sit_on_finish is off; leaving the robot standing.")
            return self.outcome

        outcome = {"enabled": True, "cancelled": False, "still": False,
                   "announced": 0, "sat": False, "message": ""}
        self.outcome = outcome

        outcome["cancelled"] = self._attempt("cancel navigation", self._cancel_nav)
        if not outcome["cancelled"]:
            self._warn("Finish: could not cancel the Nav2 goal; waiting for the "
                       "base to stop anyway.")

        outcome["still"] = self._attempt("wait for the base to stop", self._wait_still)
        if not outcome["still"]:
            self._warn("Finish: base still moving at the timeout; sitting anyway.")

        outcome["announced"] = self._announce_finish()

        try:
            ok, message = self._call_sit()
        except Exception as exc:                        # noqa: BLE001 - logged, never raised
            ok, message = False, f"{type(exc).__name__}: {exc}"
        outcome["sat"] = bool(ok)
        outcome["message"] = str(message or "")
        if ok:
            self._info(f"Finish: Spot is sitting ({outcome['message'] or 'ok'}).")
        else:
            self._warn(f"Finish: /sit FAILED -- {outcome['message'] or 'no reply'}. "
                       f"The robot is still standing; sit it from a Pi shell.")
        return outcome

    def _announce_finish(self) -> int:
        """Publish 'sit <run-id>' once, then again every 1/ANNOUNCE_HZ s for
        announce_s seconds. Returns how many copies were published."""
        if self._announce is None:
            return 0
        run_id = self._run_id or f"{int(time.time())}"
        text = f"sit {run_id}"
        copies = 1 + int(self._announce_s * ANNOUNCE_HZ)
        sent = 0
        for i in range(copies):
            if self._attempt("announce the finish", lambda: self._announce(text)):
                sent += 1
            if i + 1 < copies:
                self._sleep(1.0 / ANNOUNCE_HZ)
        self._info(f"Finish: announced {text!r} x{sent} for the Pi's finish_listener.")
        return sent

    def _attempt(self, what, action) -> bool:
        try:
            return bool(action())
        except Exception as exc:                        # noqa: BLE001 - logged, never raised
            self._warn(f"Finish: {what} raised {type(exc).__name__}: {exc}")
            return False

    def _info(self, message):
        if self._logger:
            self._logger.info(message)

    def _warn(self, message):
        if self._logger:
            self._logger.warn(message)
