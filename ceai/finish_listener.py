"""Pi-side listener: sit the robot when the bridge says the run is finished.

    bridge (the workstation)  --/ceai/finish "sit <run-id>"-->  (zenoh)  -->  THIS NODE  -->  /sit
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:                                        # the driver's state message; Pi only
    from spot_msgs.msg import Feedback as SpotFeedback
except ImportError:                         # pragma: no cover - laptop without spot_msgs
    SpotFeedback = None

FINISH_TOPIC = "/ceai/finish"
FEEDBACK_TOPIC = "status/feedback"
ACTIONS = ("sit",)


class FinishListener:
    """The decision, without rclpy: which messages act, once per run, and
    whether to call /sit again.

    `tick()` is driven by a timer every `retry_s` seconds and returns what it
    did: "confirmed" (feedback says sitting), "retry" (called /sit again),
    "timeout" (gave up), or None (nothing pending). `on_feedback()` receives
    the driver's sitting/moving flags. `retry_s <= 0` keeps the one-shot
    behaviour: tick() never retries.
    """

    def __init__(self, call_sit, logger=None, retry_s=0.0, confirm_timeout_s=30.0,
                 clock=time.monotonic):
        self._call_sit = call_sit
        self._logger = logger
        self.retry_s = float(retry_s)
        self.confirm_timeout_s = float(confirm_timeout_s)
        self._clock = clock
        self.seen = set()
        self.outcomes = []      # (run_id, ok, message)
        self.sitting = None     # last feedback; None until one arrives
        self.moving = None
        self.pending = None     # {"run_id", "started", "attempts"} while a sit is unconfirmed

    def on_message(self, text: str):
        parts = str(text).split(maxsplit=1)
        if not parts or parts[0] not in ACTIONS:
            self._warn(f"ignoring finish message {text!r} (want 'sit <run-id>')")
            return None
        run_id = parts[1].strip() if len(parts) > 1 else ""
        key = (parts[0], run_id)
        if key in self.seen:
            return None                     # a repeat of an announcement already acted on
        self.seen.add(key)
        self._info(f"finish '{parts[0]}' for run {run_id or '(no id)'}: calling /sit")
        if self.retry_s > 0:
            self.pending = {"run_id": run_id, "started": self._clock(), "attempts": 0}
        return self._sit(run_id)

    def on_feedback(self, sitting, moving=None):
        self.sitting = bool(sitting)
        self.moving = None if moving is None else bool(moving)

    def tick(self):
        """One retry period has passed. Confirm, retry, or give up."""
        if self.pending is None:
            return None
        run_id = self.pending["run_id"]
        attempts = self.pending["attempts"]
        if self.sitting:
            self._info(f"run {run_id or '(no id)'}: Spot is sitting "
                       f"(confirmed by status/feedback after {attempts} /sit call(s)).")
            self.pending = None
            return "confirmed"
        elapsed = self._clock() - self.pending["started"]
        if elapsed >= self.confirm_timeout_s:
            seen = ("status/feedback never arrived" if self.sitting is None
                    else "status/feedback still says not sitting")
            self._warn(f"run {run_id or '(no id)'}: giving up after {attempts} /sit "
                       f"call(s) in {elapsed:.0f}s -- {seen}. Sit it from a Pi shell.")
            self.pending = None
            return "timeout"
        why = ("driver still reports moving (a cmd_vel is overriding the sit)"
               if self.moving else "not sitting yet")
        self._info(f"run {run_id or '(no id)'}: {why}; calling /sit again "
                   f"({attempts + 1}, {elapsed:.0f}s elapsed).")
        self._sit(run_id)
        return "retry"

    def _sit(self, run_id):
        if self.pending is not None:
            self.pending["attempts"] += 1
        try:
            ok, message = self._call_sit()
        except Exception as exc:            # noqa: BLE001 - logged, the node keeps running
            ok, message = False, f"{type(exc).__name__}: {exc}"
        self.outcomes.append((run_id, bool(ok), str(message)))
        if ok:
            self._info(f"/sit {message or 'ok'}")
        else:
            self._warn(f"/sit FAILED: {message or 'no reply'}")
        return bool(ok)

    def _info(self, message):
        if self._logger:
            self._logger.info(message)

    def _warn(self, message):
        if self._logger:
            self._logger.warn(message)


class FinishListenerNode(Node):

    def __init__(self, **kwargs):
        super().__init__("finish_listener", **kwargs)
        self.declare_parameter("namespace", "")
        self.declare_parameter("topic", FINISH_TOPIC)
        self.declare_parameter("sit_service", "/sit")
        self.declare_parameter("service_timeout", 10.0)
        # Re-call /sit every sit_retry_s until status/feedback says sitting,
        # for at most sit_confirm_timeout_s. 0.0 = one call, no confirmation.
        self.declare_parameter("sit_retry_s", 0.0)
        self.declare_parameter("sit_confirm_timeout_s", 30.0)
        self.declare_parameter("feedback_topic", FEEDBACK_TOPIC)
        ns = str(self.get_parameter("namespace").value).strip("/")
        prefix = f"/{ns}" if ns else ""
        self.sit_service = prefix + str(self.get_parameter("sit_service").value)
        self.topic = str(self.get_parameter("topic").value)
        self.timeout = float(self.get_parameter("service_timeout").value)
        self.retry_s = float(self.get_parameter("sit_retry_s").value)
        feedback = str(self.get_parameter("feedback_topic").value)
        self.feedback_topic = feedback if feedback.startswith("/") else f"{prefix}/{feedback}"

        self._sit = self.create_client(Trigger, self.sit_service)
        self.listener = FinishListener(
            self._call_sit, self.get_logger(), retry_s=self.retry_s,
            confirm_timeout_s=float(self.get_parameter("sit_confirm_timeout_s").value))
        self._retry_timer = None
        if self.retry_s > 0:
            if SpotFeedback is None:
                self.get_logger().warn(
                    "spot_msgs not importable: cannot read status/feedback, so /sit "
                    "will be re-called blind every retry period until the timeout.")
            else:
                self.create_subscription(SpotFeedback, self.feedback_topic,
                                         self._on_feedback, 1)
            self._retry_timer = self.create_timer(self.retry_s, self._on_retry_tick)
        # RELIABLE + KEEP_LAST 10: the announcement is repeated, and nothing
        # here must drop a copy that did arrive.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, self.topic, self._on_finish, qos)
        how = (f"re-calling every {self.retry_s:g}s until {self.feedback_topic} says sitting"
               if self.retry_s > 0 else "one call, unconfirmed (sit_retry_s=0)")
        self.get_logger().info(f"listening on {self.topic}; will call {self.sit_service}; {how}")

    def _on_finish(self, msg: String):
        self.listener.on_message(msg.data)

    def _on_feedback(self, msg):
        self.listener.on_feedback(msg.sitting, msg.moving)

    def _on_retry_tick(self):
        self.listener.tick()

    def _call_sit(self):
        """Called from the subscription callback, so it must NOT block: this
        node's single executor is the one that would deliver the reply, and
        spinning it from inside a callback never returns the reply (measured
        2026-09-19). The call is sent, a done-callback logs the outcome, and a
        one-shot timer reports a reply that never came."""
        if not self._sit.wait_for_service(timeout_sec=self.timeout):
            return False, f"{self.sit_service}: no server after {self.timeout:.0f}s"
        future = self._sit.call_async(Trigger.Request())
        future.add_done_callback(self._on_sit_reply)
        timer_holder = []

        def check():
            timer_holder[0].cancel()
            if not future.done():
                self.get_logger().warn(
                    f"{self.sit_service}: no reply after {self.timeout:.0f}s")
                self.listener.outcomes.append(("", False, "no reply"))
        timer_holder.append(self.create_timer(self.timeout, check))
        return True, "requested"

    def _on_sit_reply(self, future):
        try:
            response = future.result()
        except Exception as exc:            # noqa: BLE001 - logged, the node keeps running
            self.get_logger().warn(f"{self.sit_service} raised {type(exc).__name__}: {exc}")
            self.listener.outcomes.append(("", False, f"{type(exc).__name__}: {exc}"))
            return
        ok, message = bool(response.success), str(response.message)
        self.listener.outcomes.append(("", ok, message))
        if ok:
            self.get_logger().info(f"Spot is sitting ({message or 'ok'})")
        else:
            self.get_logger().warn(f"/sit FAILED: {message or 'no message'}")


def main(args=None):
    rclpy.init(args=args)
    node = FinishListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
