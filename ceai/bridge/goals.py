"""Publishing goals to Nav2."""
import math

from ..n2np_helpers import make_goal, vlfm_frontier_to_world


def _clamp(value, low, high):
    """value into [low, high]; the midpoint when the interval is inverted."""
    if low > high:
        return 0.5 * (low + high)
    return min(max(value, low), high)


class GoalPublisher:
    """Publishes to /goal_pose, optionally suppressing unchanged goals.

    Every publish is a fresh NavigateToPose: bt_navigator cancels the running
    one, replans, and the controller restarts by rotating onto the new path. Send
    them faster than the robot can walk and it only ever turns -- measured
    2026-09-12, 41 of 44 goals preempted.

    The suppression is OFF by default (both epsilons 0.0), so behaviour is
    unchanged until you ask for it. Set goal_epsilon_m to about a quarter of a
    metre to enable it; set it back to 0.0 to disable.
    """

    def __init__(self, publisher, clock, map_frame, standoff_m,
                 epsilon_m=0.0, epsilon_yaw=0.0, logger=None,
                 map_bounds=None, clamp_to_map=False, map_margin_m=0.3):
        self._publisher = publisher
        self._clock = clock
        self.map_frame = map_frame
        self.standoff_m = float(standoff_m)
        self.epsilon_m = float(epsilon_m)
        self.epsilon_yaw = float(epsilon_yaw)
        self._logger = logger
        self._last = None           # (x, y, yaw, mode) of the last goal sent
        # OPT-IN (off by default). `map_bounds` is a callable returning the
        # current /map's (x_min, y_min, x_max, y_max), or None before a map.
        self._map_bounds = map_bounds
        self.clamp_to_map = bool(clamp_to_map)
        self.map_margin_m = float(map_margin_m)

    @property
    def dedupe_enabled(self) -> bool:
        return self.epsilon_m > 0.0 or self.epsilon_yaw > 0.0

    def publish(self, goal_map_xy, goal_yaw, mode=None) -> bool:
        """Publish the goal. Returns False if it was suppressed as unchanged."""
        if not self._is_new(goal_map_xy, goal_yaw, mode):
            if self._logger:
                self._logger.info(
                    f"Goal unchanged (< {self.epsilon_m:.2f} m); not republishing.",
                    throttle_duration_sec=5.0)
            return False
        self._publisher.publish(make_goal(
            goal_map_xy, goal_yaw,
            stamp=self._clock.now().to_msg(),
            frame_id=self.map_frame))
        self._last = (float(goal_map_xy[0]), float(goal_map_xy[1]),
                      float(goal_yaw), mode)
        if self._logger:
            self._logger.info(f"Sent goal {goal_map_xy} (yaw {goal_yaw:.2f})")
        return True

    def _is_new(self, goal_map_xy, goal_yaw, mode) -> bool:
        """A mode change always republishes: explore -> navigate is a different
        intent even when the point barely moved."""
        if not self.dedupe_enabled or self._last is None:
            return True
        last_x, last_y, last_yaw, last_mode = self._last
        if mode != last_mode:
            return True
        moved = math.hypot(float(goal_map_xy[0]) - last_x,
                           float(goal_map_xy[1]) - last_y)
        if moved > self.epsilon_m:
            return True
        turned = abs(math.atan2(math.sin(float(goal_yaw) - last_yaw),
                                math.cos(float(goal_yaw) - last_yaw)))
        return self.epsilon_yaw > 0.0 and turned > self.epsilon_yaw

    def apply_standoff(self, goal_map_xy, goal_yaw):
        """Move the goal back along the approach line, so Nav2 stops in FRONT of
        the object instead of driving into it. goal_yaw already points
        robot->object, so the standoff is collinear."""
        return [
            float(goal_map_xy[0]) - self.standoff_m * math.cos(goal_yaw),
            float(goal_map_xy[1]) - self.standoff_m * math.sin(goal_yaw),
        ]

    def clamp_to_map_bounds(self, goal_map_xy):
        """Pull the goal inside the current /map by map_margin_m -> (xy, moved).

        A frontier is, by definition, on the edge of the SLAM map, and the
        policy's goal lands a cell or two PAST it. Nav2's global costmap is
        resized to exactly that map by its static layer, so the planner answers
        "The goal sent to the planner is off the global costmap. Planning will
        always fail to this goal", the BT runs recoveries, and the robot never
        walks -- MEASURED 2026-09-19, nav_failed_again_no_bashrc.webm, every
        goal of the run (mx 159 against size_x 157 in the first video). A goal
        just inside the edge is in unknown space, which the planner is allowed
        into (allow_unknown), and its 0.5 m tolerance does the rest.

        A no-op when the clamp is off, before the first map, or when the goal
        is already inside the margin. A map narrower than two margins clamps
        to its centre line. Enable with clamp_goal_to_map; disable with
        -p clamp_goal_to_map:=false or by deleting the yaml line.
        """
        if not self.clamp_to_map or self._map_bounds is None:
            return goal_map_xy, False
        bounds = self._map_bounds()
        if bounds is None:
            return goal_map_xy, False
        x_min, y_min, x_max, y_max = (float(b) for b in bounds)
        m = self.map_margin_m
        x = _clamp(float(goal_map_xy[0]), x_min + m, x_max - m)
        y = _clamp(float(goal_map_xy[1]), y_min + m, y_max - m)
        moved = (x != float(goal_map_xy[0])) or (y != float(goal_map_xy[1]))
        if moved and self._logger:
            self._logger.info(
                f"Goal ({float(goal_map_xy[0]):.2f}, {float(goal_map_xy[1]):.2f}) "
                f"is off the map [{x_min:.2f}..{x_max:.2f}] x [{y_min:.2f}..{y_max:.2f}]; "
                f"clamped to ({x:.2f}, {y:.2f}).",
                throttle_duration_sec=5.0)
        return [x, y], moved

    def publish_frontier(self, frontier_epi, desired_heading, epi_to_map):
        """Publish an episodic frontier from obstacle_map.frontiers."""
        if epi_to_map is None:
            if self._logger:
                self._logger.warn("Transform not ready; cannot send goal.")
            return
        world_xy = vlfm_frontier_to_world(frontier_epi, epi_to_map)
        self._publisher.publish(make_goal(
            world_xy, desired_heading,
            stamp=self._clock.now().to_msg(),
            frame_id=self.map_frame))
        if self._logger:
            self._logger.info(f"Sent goal at {world_xy}")
