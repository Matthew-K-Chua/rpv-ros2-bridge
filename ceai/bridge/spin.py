"""In-place rotation via the Nav2 Spin behaviour, and the waits around it.

Rotation and capture are split (start / join) so the init scan can turn towards
pose i+1 while the policy is still thinking about pose i. The base moving does
not disturb that step: its frame was captured before the rotation began and its
pose is resolved at that frame's own stamp.
"""
import threading
import time

import rclpy
import rclpy.duration
from builtin_interfaces.msg import Duration    # the MESSAGE Duration, not rclpy's
from nav2_msgs.action import Spin

POLL_INTERVAL_S = 0.02
SPIN_TIME_ALLOWANCE_S = 15


class SpinController:

    def __init__(self, action_client, config, motion, frames, timing, to_time,
                 logger=None):
        self._client = action_client
        self._config = config
        self._motion = motion
        self._frames = frames
        self._timing = timing
        self._to_time = to_time
        self._logger = logger
        self._done = threading.Event()
        self._goal_handle = None
        # Which goal `_done` refers to. A result for any OTHER goal is ignored:
        # when join() gives up on a spin that never finished and the loop fires
        # the next one, Nav2 preempts the old goal and its ABORTED result comes
        # back -- and used to set `_done` for the NEW goal, which then "finished"
        # instantly, and the loop fired another, and another. From the outside:
        # one spin fails to complete, then the robot rotates continuously and
        # never observes the 30 deg increments again.
        self._generation = 0

    # -- rotation -----------------------------------------------------------
    def start(self, angle):
        """Fire a Spin goal and return immediately."""
        if self._generation and not self._done.is_set():
            self._warn("Previous Spin goal has not finished; Nav2 will preempt it. "
                       "If this repeats, the behavior server is not seeing yaw "
                       "progress -- check TF vision->body on this machine.")
        self._done.clear()
        self._motion.expect_new_stop()   # this spin's stop must be observed afresh
        self._send(angle)

    def join(self, timeout=15.0):
        """Block until the rotation finishes, then until a still frame lands."""
        started = time.monotonic()
        if not self._done.wait(timeout):
            self._warn("Spin goal timed out; continuing.")
        self._timing["spin"] = (time.monotonic() - started) * 1e3
        # Settle AFTER the motion, so every caller gets a frame captured with the
        # base already still.
        self.wait_for_still_frame()

    def spin_and_wait(self, angle, timeout=15.0):
        self.start(angle)
        self.join(timeout)

    def _send(self, angle):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self._error("Action server /spin not available (is nav2 behavior_server up?).")
            self._done.set()
            return
        goal = Spin.Goal()
        goal.target_yaw = float(angle)                          # relative, CCW positive
        goal.time_allowance = Duration(sec=SPIN_TIME_ALLOWANCE_S)
        self._generation += 1
        generation = self._generation
        self._info(f"Spinning {goal.target_yaw:.3f} rad...")
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(
            lambda done: self._on_goal_response(done, generation))

    def _on_feedback(self, feedback):
        pass        # feedback.feedback.angular_distance_traveled if ever needed

    def _current(self, generation) -> bool:
        return generation == self._generation

    def _on_goal_response(self, future, generation):
        handle = future.result()
        if not self._current(generation):
            return                      # superseded before Nav2 even answered
        if not handle.accepted:
            self._error("Spin goal rejected.")
            self._done.set()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda done: self._on_result(done, generation))

    def _on_result(self, future, generation):
        if not self._current(generation):
            # A preempted or late predecessor. Its outcome says nothing about
            # the goal the loop is now waiting on.
            self._warn("Ignoring the result of a superseded Spin goal.")
            return
        self._done.set()

    # -- waits --------------------------------------------------------------
    def wait_for_still_frame(self, timeout=None):
        """Wait for the base to stop, then for a frame CAPTURED after it stopped."""
        t0 = time.monotonic()
        _, stopped_at = self.wait_until_stationary(timeout)
        t1 = time.monotonic()
        ok = self.wait_for_frame_after(stopped_at)
        t2 = time.monotonic()
        # Split deliberately: settle is odom-side, frame_wait is camera-side.
        # Lumping them hides which one a change moved.
        self._timing["settle"] = (t1 - t0) * 1e3
        self._timing["frame_wait"] = (t2 - t1) * 1e3
        return ok

    def wait_until_stationary(self, timeout=None):
        """Block until /odom reports the base stopped -> (stopped, robot-clock stamp).

        On timeout, capture anyway rather than deadlock: the pose-at-stamp lookup
        keeps the bearing correct even while moving, so only blur remains.
        """
        if timeout is None:
            timeout = self._config.stationary_timeout
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            if self._motion.stop_stamp is not None:
                return True, self._motion.stop_stamp
            if self._motion.still and self._motion.latest is not None:
                # Already still with no edge to see: the base never moved for
                # this spin (rejected goal, zero angle). Date it to the newest.
                return True, self._to_time(self._motion.latest.header.stamp)
            if time.monotonic() > deadline:
                self._warn_rarely(f"Base still moving (or no /odom) after "
                                  f"{timeout:.1f}s; capturing while in motion.")
                return False, None
            time.sleep(POLL_INTERVAL_S)
        return False, None

    def wait_for_frame_after(self, stopped_at, timeout=None):
        """Block until a frame whose CAPTURE STAMP is after stopped_at + settle.

        Waiting on arrival instead would be wrong on this link: the next frame to
        arrive was captured one transport delay ago, i.e. possibly before the
        base even stopped -- still blurred, and instantly stale.

        Both stamps are robot-clock, so this never touches the PC clock.
        """
        settle = rclpy.duration.Duration(seconds=self._config.settle_time)
        if stopped_at is None:
            held = self._frames.latest
            if held is None:
                return False
            stopped_at = held[1]
        earliest = stopped_at + settle
        limit = timeout or self._config.frame_wait_timeout
        deadline = time.monotonic() + limit
        while rclpy.ok():
            frame = self._frames.latest
            if frame is not None and frame[1] >= earliest:
                return True
            if time.monotonic() > deadline:
                self._warn(f"No RGB frame captured after the base stopped within "
                           f"{limit:.1f}s; capturing anyway.")
                return False
            time.sleep(POLL_INTERVAL_S)
        return False

    # -- logging ------------------------------------------------------------
    def _info(self, message):
        if self._logger:
            self._logger.info(message)

    def _warn(self, message):
        if self._logger:
            self._logger.warn(message, throttle_duration_sec=2.0)

    def _warn_rarely(self, message):
        if self._logger:
            self._logger.warn(message, throttle_duration_sec=5.0)

    def _error(self, message):
        if self._logger:
            self._logger.error(message)
