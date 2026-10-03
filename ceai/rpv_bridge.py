"""Nav2 <-> RPV policy bridge node. One node, any robot.

Forward path : nav_msgs/OccupancyGrid + RGB + depth  ->  the policy server
Reverse path : the policy's episodic goal            ->  geometry_msgs/PoseStamped

This module is the ROS adapter: parameters, subscriptions, publishers, the action
client, and the callbacks rclpy calls. Everything else lives in ceai/bridge/ --
see that package's docstring for the map.

Which robot it is talking to is the `robot` parameter, which selects a
ceai.robots.RobotProfile and with it every frame, topic and dimension that
differs between robots. The node itself contains no robot-specific value.

    ros2 run ceai rpv_bridge --ros-args --params-file config/spot_bridge.yaml
    python3 -m ceai.rpv_bridge  --ros-args -p robot:=spot

"""
import threading
import time

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import Spin
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ceai.bridge import config as bridge_config
from ceai.bridge import rpv
from ceai.bridge.curtain import CameraLidarTransform, CurtainProjector
from ceai.bridge.depth import make_depth_source
from ceai.bridge.finish import FinishSequence
from ceai.bridge.frames import EpisodicFrame, PoseLookup
from ceai.bridge.goals import GoalPublisher
from ceai.bridge.loop import ControlLoop, StepRunner
from ceai.bridge.occupancy import OccupancyCache
from ceai.bridge.payload import PayloadBuilder
from ceai.bridge.policy import PolicyClient
from ceai.bridge.sensors import FrameCache, Intrinsics, MotionTracker, StampedBuffer
from ceai.bridge.spin import SpinController
from ceai.timing import TimingLog

TF_CACHE_S = 30.0       # TF is looked up in the past; the default 10 s is too short
                        # to survive a slow policy round-trip.


class RpvBridge(Node):

    def __init__(self, **kwargs):
        super().__init__('nav2_vlfm_bridge', **kwargs)
        self.config = bridge_config.declare(self)
        # Both raster depth sources encode through the policy repository's codec, so
        # a missing checkout is fatal at construction rather than on the first
        # step -- 30 seconds into a run is a bad time to learn about a PYTHONPATH.
        if self.config.wants_depth_raster and not rpv.available():
            raise RuntimeError(rpv.unavailable_message())

        log = self.get_logger()
        log.info(f"Robot profile: {self.config.profile.name} "
                 f"(base {self.config.robot_frame}, odom {self.config.profile.odom_frame}, "
                 f"depth {self.config.depth_source})")
        self._timing = TimingLog(log)

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=TF_CACHE_S))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.intrinsics = Intrinsics()
        self.frames = FrameCache(log)
        self.scans = StampedBuffer(self.config.scan_buffer_len)
        self.depth_rasters = StampedBuffer(16)
        self.motion = MotionTracker(self.config.motion_lin_eps, self.config.motion_ang_eps,
                                    self.config.motion_still_hold)
        self.occupancy = OccupancyCache()
        self.latest_scan = None

        self.episodic = EpisodicFrame(
            self.tf_buffer, self.config.map_frame, self.config.robot_frame, log)
        self.pose_at = PoseLookup(
            self.tf_buffer, self.config.map_frame, self.config.robot_frame,
            rclpy.duration.Duration(seconds=self.config.tf_lookup_timeout), log)

        self.camera_lidar = CameraLidarTransform(
            self.tf_buffer, self.config.lidar_frame, self.config.camera_optical_frame, log)
        self.projector = CurtainProjector(
            self.config, self.intrinsics, self.camera_lidar, log)
        self.depth_source = make_depth_source(
            self.config, self.depth_rasters, self.projector, log)

        self._subscribe()

        self.goal_pub = self.create_publisher(
            PoseStamped, self.config.nav_topic('goal_topic'), 10)
        self.goals = GoalPublisher(
            self.goal_pub, self.get_clock(), self.config.map_frame,
            self.config.stop_standoff_m,
            epsilon_m=self.config.goal_epsilon_m,
            epsilon_yaw=self.config.goal_epsilon_yaw,
            logger=log,
            map_bounds=lambda: self.occupancy.bounds,
            clamp_to_map=self.config.clamp_goal_to_map,
            map_margin_m=self.config.goal_map_margin_m)

        self._client = ActionClient(self, Spin, self.config.nav_topic('spin_action'))
        self.spin_controller = SpinController(
            self._client, self.config, self.motion, self.frames, self._timing,
            self._stamp_to_time, log)

        self.payload_builder = PayloadBuilder(
            self.config, self.get_clock(), self.frames, self.scans, self.depth_source,
            self.occupancy, self.intrinsics, self.episodic, self.pose_at, log)
        self.policy = PolicyClient(
            self.config.policy_url, self.config.policy_timeout, log)
        self.steps = StepRunner(
            self.payload_builder, self.policy, self.goals, self.occupancy,
            self._timing, log)
        # After the policy calls stop: cancel Nav2, wait still, /sit. The
        # clients are created regardless of sit_on_finish (inert without a
        # server); the sequence itself checks the flag.
        self._sit_client = self.create_client(
            Trigger, self.config.robot_topic('sit_service'))
        self._nav_cancel_client = self.create_client(
            CancelGoal, self.config.nav_topic('nav_cancel_service'))
        # RELIABLE so the repeated copies are not dropped on this side either.
        self._finish_pub = self.create_publisher(
            String, self.config.nav_topic('finish_topic'),
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10))
        self.finish = FinishSequence(
            self.config.sit_on_finish,
            cancel_nav=self._cancel_navigation,
            wait_still=self._wait_until_stopped,
            call_sit=self._call_sit,
            logger=log,
            announce=self._announce_finish,
            announce_s=self.config.finish_announce_s)
        self.control_loop = ControlLoop(
            self.config, self.payload_builder, self.steps, self.spin_controller,
            self.frames, log, finish=self.finish)

        self._ctrl_thread = None
        log.info('Nav2<->VLFM bridge up')

    # -- wiring -------------------------------------------------------------
    def _subscribe(self):
        # /map is VOLATILE, not TRANSIENT_LOCAL. slam_toolbox latches it, but it
        # arrives here via zenoh-bridge-ros2dds, which republishes as VOLATILE. A
        # VOLATILE subscriber is compatible with both. Trade-off: no instantly
        # latched first map, we wait for the next periodic publish.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # BEST_EFFORT matches a RELIABLE publisher too, so these work whether the
        # topic arrives direct or republished by the zenoh bridge.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        config = self.config
        self.create_subscription(
            OccupancyGrid, config.nav_topic('map_topic'), self.on_map, map_qos)
        self.create_subscription(
            CompressedImage, config.robot_topic('rgb_topic'),
            self.image_callback, sensor_qos)
        self.create_subscription(
            CameraInfo, config.robot_topic('camera_info_topic'),
            self.camInfo_callback, sensor_qos)
        self.create_subscription(
            LaserScan, config.robot_topic('scan_topic'), self.scan_callback, sensor_qos)
        self.create_subscription(
            Odometry, config.robot_topic('odom_topic'), self.odom_callback, sensor_qos)
        # Rasters are ~300 kB each; only carry them when they are the depth
        # source. /scan stays subscribed either way -- it is the navigation
        # input, and the curtain's when depth_source is `lidar`.
        if config.wants_depth_raster:
            self.create_subscription(
                Image, config.resolved_depth_topic, self.depth_callback, sensor_qos)

    def _stamp_to_time(self, stamp):
        """builtin_interfaces/Time -> rclpy.time.Time on THIS node's clock type.

        from_msg() defaults to SYSTEM_TIME while the node clock is ROS_TIME, and
        rclpy refuses to subtract times of different clock types.
        """
        return rclpy.time.Time.from_msg(stamp, clock_type=self.get_clock().clock_type)

    # -- callbacks ----------------------------------------------------------
    def on_map(self, msg):
        # The episodic frame is pinned from the first TF: needed for goals, and
        # forwarded to the server as start_x/y/yaw.
        if not self.episodic.ensure():
            self.get_logger().warn('No map->robot TF yet; skipping this map.', once=True)
            return
        self.occupancy.update(msg)

    def image_callback(self, msg):
        self.frames.update(msg, self._stamp_to_time)

    def camInfo_callback(self, msg):
        self.intrinsics.update(msg)

    def odom_callback(self, msg):
        self.motion.update(msg, self._stamp_to_time)

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scans.append(self._stamp_to_time(msg.header.stamp), msg)

    def depth_callback(self, msg):
        """Cache a 16UC1 millimetre raster.

        No conversion: the bytes arrive as uint16 millimetres and leave inside a
        PNG as uint16 millimetres. Going through float metres would be lossless
        in principle and one more place for a unit error to hide.
        """
        if msg.encoding != "16UC1":
            self.get_logger().warn(
                f"{self.config.resolved_depth_topic} is {msg.encoding!r}, expected 16UC1 "
                f"(millimetres). Ignoring -- publishing 32FC1 metres here would be "
                f"silently read as millimetres.", throttle_duration_sec=5.0)
            return
        dtype = '>u2' if msg.is_bigendian else '<u2'
        mm = np.frombuffer(msg.data, dtype=dtype).reshape(
            msg.height, msg.width).astype(np.uint16)
        self.depth_rasters.append(self._stamp_to_time(msg.header.stamp), mm)

    # -- services -----------------------------------------------------------
    def _call_service(self, client, request):
        """Call from the CONTROL thread: rclpy.spin() on the main thread
        completes the future. Returns the response, or None on timeout / no
        server -- never raises, so the finish sequence always completes."""
        timeout = self.config.finish_service_timeout
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(
                f"{client.srv_name}: no server after {timeout:.1f}s. On this rig "
                f"Spot's services may not cross zenoh (see the README"
                f"section 2); call it from a Pi shell.")
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn(f"{client.srv_name}: no reply after {timeout:.1f}s.")
                return None
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _cancel_navigation(self) -> bool:
        """Cancel EVERY NavigateToPose goal: an all-zero CancelGoal request is
        the action protocol's cancel-all. The bridge publishes goals by topic,
        so it holds no goal handle of its own to cancel."""
        response = self._call_service(self._nav_cancel_client, CancelGoal.Request())
        if response is None:
            return False
        # ERROR_NONE (0) and ERROR_REJECTED (1) are both fine here: rejected
        # means no goal was active, i.e. Nav2 had already finished.
        ok = response.return_code in (CancelGoal.Response.ERROR_NONE,
                                      CancelGoal.Response.ERROR_REJECTED)
        self.get_logger().info(
            f"Nav2 cancel: return_code={response.return_code} "
            f"goals_cancelled={len(response.goals_canceling)}")
        return ok

    def _wait_until_stopped(self) -> bool:
        """Forget the last spin's stop first: MotionTracker keeps stop_stamp
        from the previous moving->still edge, and a stale one would make the
        wait return at once while Nav2 is still walking."""
        self.motion.expect_new_stop()
        return self.spin_controller.wait_until_stationary()[0]

    def _finish_pub_topic(self) -> str:
        return self.config.nav_topic('finish_topic')

    def _announce_finish(self, text: str) -> bool:
        self._finish_pub.publish(String(data=text))
        return True

    def _call_sit(self):
        response = self._call_service(self._sit_client, Trigger.Request())
        if response is None:
            return False, "no reply"
        return bool(response.success), str(response.message)

    # -- control ------------------------------------------------------------
    def start(self):
        """Launch the control loop in a daemon thread."""
        self._ctrl_thread = threading.Thread(target=self.run_control_loop, daemon=True)
        self._ctrl_thread.start()

    def run_control_loop(self):
        self.control_loop.run()

    def act(self):
        """Single manual policy step (debugging); the loop normally drives this."""
        return self.steps.step()


def main(args=None):
    rclpy.init(args=args)
    node = RpvBridge()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():      # Ctrl-C's signal handler may already have shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
