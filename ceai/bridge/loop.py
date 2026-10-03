"""One policy step, and the control loop that repeats it.

The loop runs on its own thread; rclpy.spin() services the callbacks that feed it.
"""
import math
import threading
import time

import rclpy


class StepRunner:
    """Build a payload, POST it, act on the answer."""

    def __init__(self, payload_builder, policy, goals, occupancy, timing, logger=None):
        self._payload = payload_builder
        self._policy = policy
        self._goals = goals
        self._occupancy = occupancy
        self._timing = timing
        self._logger = logger

    def step(self, is_first=False, finish_init=False, frame=None):
        started = time.monotonic()
        payload = self._payload.build(is_first=is_first, finish_init=finish_init,
                                      frame=frame)
        posted_at = time.monotonic()
        self._timing["build_payload"] = (posted_at - started) * 1e3
        if payload is None:
            self._timing.log("skipped: no payload")
            return None

        result = self._policy.post(payload)
        self._timing["post"] = (time.monotonic() - posted_at) * 1e3
        self._timing["occ"] = "full" if payload.get("occupancy") is not None else "hash"

        if result is not None and result.get("occ_cache_miss"):
            payload, result = self._resend_with_full_grid(is_first, finish_init, frame)

        if result is None:
            self._timing.log("failed: no result")
            return None

        # Only now is the server known to hold this grid: a failed POST must not
        # let the next step refer to one that never arrived.
        self._occupancy.mark_acked(payload.get("occ_hash"))
        self._publish(result)
        self._log(result)
        return result

    def _resend_with_full_grid(self, is_first, finish_init, frame):
        """The server does not hold the grid we referred to by hash -- it
        restarted mid-episode. It has done no work yet, so resending in full is a
        clean retry, not a duplicate step."""
        if self._logger:
            self._logger.warn("Policy server has no cached occupancy grid; "
                              "resending it in full.")
        self._occupancy.forget_ack()
        retried_at = time.monotonic()
        payload = self._payload.build(is_first=is_first, finish_init=finish_init,
                                      frame=frame, full_occ=True)
        result = self._policy.post(payload) if payload is not None else None
        # Charged to post, not hidden: a retry is real wall-clock the step paid.
        self._timing["post"] += (time.monotonic() - retried_at) * 1e3
        self._timing["occ"] = "full(retry)"
        return payload, result

    def _publish(self, result):
        goal = result.get("goal_map_xy")
        if goal is None:
            return
        goal_yaw = float(result.get("goal_yaw", 0.0))
        mode = result.get("mode")
        # In navigate mode the goal IS the object centroid; back it off so the
        # robot halts in front of the object, facing it.
        if mode == "navigate":
            goal = self._goals.apply_standoff(goal, goal_yaw)
        # After the standoff, so the point Nav2 is asked for is the one that
        # gets pulled inside the map.
        goal, _ = self._goals.clamp_to_map_bounds(goal)
        self._goals.publish(goal, goal_yaw, mode)

    def _log(self, result):
        if self._logger:
            self._logger.info(
                f"mode={result.get('mode')} goal={result.get('goal_map_xy')} "
                f"stop={result.get('called_stop')} "
                f"init_done={result.get('done_initializing')}")
        self._timing.log(f"mode={result.get('mode')}")


class ControlLoop:
    """Wait for sensors, run the 360 degree init scan, then explore until stop."""

    def __init__(self, config, payload_builder, steps, spin, frames, logger=None,
                 finish=None):
        self._config = config
        self._payload = payload_builder
        self._steps = steps
        self._spin = spin
        self._frames = frames
        self._logger = logger
        # ceai/bridge/finish.py, or None: what to do once the policy calls stop.
        self._finish = finish
        self._stop = threading.Event()
        # Policy steps the server answered this run (init scan + explore).
        # Compared against config.max_steps; 0 there means no limit.
        self.steps_taken = 0

    def request_stop(self):
        """Ask the loop to return at its next check. Used on shutdown and by
        tests; the loop otherwise runs until the policy calls stop."""
        self._stop.set()

    def _running(self) -> bool:
        return rclpy.ok() and not self._stop.is_set()

    def run(self):
        self._stop.clear()
        self.steps_taken = 0
        if not self._wait_for_sensors():
            return
        self._init_scan()
        self._explore()

    def _wait_for_sensors(self) -> bool:
        self._info("Control loop: waiting for sensors + TF...")
        while self._running() and not self._payload.sensors_ready():
            time.sleep(0.2)
        while self._running() and self._payload.build() is None:
            time.sleep(0.2)
        if not self._running():
            return False
        self._info("Control loop: sensors ready, starting 360 scan.")
        return True

    def _init_scan(self):
        """Rotate in init_steps increments, POSTing a frame at each so the server
        builds its object and value maps from many views.

        The rotation for pose i+1 is fired BEFORE the step for pose i is POSTed,
        so the base turns while the policy thinks. 

        The LAST step is deliberately not overlapped: it carries finish_init and
        can come back with a real goal, which must not be published into a base
        that is still rotating.
        """
        steps = self._config.init_steps
        if steps <= 0:
            return
        angle = 2.0 * math.pi / steps
        is_first = True
        spin_in_flight = False
        for index in range(steps):
            if not self._running():
                return
            if not spin_in_flight:
                self._spin.start(angle)
            self._spin.join()           # rotation done + a frame captured while still
            spin_in_flight = False

            frame = self._frames.latest     # pin it: the next spin starts now
            is_last = (index == steps - 1)
            if not is_last:
                self._spin.start(angle)
                spin_in_flight = True

            self._count(self._steps.step(is_first=is_first, finish_init=is_last,
                                         frame=frame))
            is_first = False

    def _count(self, result):
        if result is not None:
            self.steps_taken += 1
        return result

    def _step_limit_reached(self) -> bool:
        limit = int(getattr(self._config, "max_steps", 0) or 0)
        return limit > 0 and self.steps_taken >= limit

    def _explore(self):
        """Think -> goal every step_period, WHILE Nav2 drives, until stop.

        step_period is a DEADLINE, not a delay: the step already takes longer
        than the period, so sleeping the full period afterwards would add it to
        every step for nothing.
        """
        self._info("Control loop: scan done, exploring.")
        while self._running():
            started = time.monotonic()
            result = self._count(self._steps.step())
            if result is not None and result.get("called_stop"):
                self._info(f"Target reached after {self.steps_taken} steps; "
                           "stopping control loop.")
                if self._finish is not None:
                    self._finish.run()
                break
            if self._step_limit_reached():
                self._info(f"Step limit reached (max_steps={self._config.max_steps}) "
                           "without the policy calling stop; finishing the run.")
                if self._finish is not None:
                    self._finish.run()
                break
            time.sleep(max(0.0, self._config.step_period - (time.monotonic() - started)))

    def _info(self, message):
        if self._logger:
            self._logger.info(message)
