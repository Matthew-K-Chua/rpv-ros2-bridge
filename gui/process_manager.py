"""Start, stop and probe the workstation's tmux session.

    rpv_offboard   zenoh  ros  vlm-clip  vlm-sam3  policy  bridge
              [+ zoedepth, zoe-chain when depth_source is zoedepth]
              [+ bag, rviz on demand]

THE PI IS NOT STARTED FROM HERE, and there is deliberately no code that tries.
This machine has no route to the Pi -- the tailnet ACL lets the Pi connect OUT
and nothing in (MEASURED) -- so every SSH this module used to make failed. A
control that can only fail is worse than no control: the PI light sat red, the
log said "Network is unreachable", and the network got the blame for a topology
that was working as designed. `onboard_start_command()` returns the line to type on a
machine that does have a route, and the console shows it.

The session is created locally by scripts/launch_offboard.sh. This module never
reimplements what that script does -- it starts it, kills it, and asks tmux what
is alive.

TESTABILITY: every process call goes through `self.run` / `self.spawn`, which
default to real subprocess calls and are replaced wholesale in tests. The
command BUILDERS (`*_argv`, `*_command`) are pure and return exactly what would
be executed, which is what the tests assert against -- a command that is wrong in
a way only the robot would reveal is the failure mode worth catching at a desk.

Threading: the blocking helpers are called from worker threads, never the UI
thread. Every subprocess call carries a timeout, so a wedged tmux degrades to a
red light rather than a frozen window.
"""
from __future__ import annotations

import os
import shlex
import subprocess
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Dict, List, Optional

from .config import Config

# Windows the workstation script creates, always. The system poller maps lights
# onto these names, so they must match the script exactly --
# Keep them in step, because a rename shows up
# only as a light stuck on red.
OFFBOARD_WINDOWS = ("zenoh", "ros", "vlm-clip", "vlm-sam3", "policy", "bridge")
# Started ONLY when depth_source is zoedepth. With the D435 giving real metric
# depth these never exist, and a light watching for them would sit red forever.
ZOEDEPTH_WINDOWS = ("zoedepth", "zoe-chain")
# Windows the Pi's own script creates. Listed for the run-book and for the
# window-name contract test; nothing here can see them.
ONBOARD_WINDOWS = ("tunnel", "zenoh", "stack", "lifecycle")

BAG_WINDOW = "bag"
RVIZ_WINDOW = "rviz"

# Typed by a HUMAN in the Pi's `lifecycle` window. Services do NOT cross the
# zenoh bridge (measured 2026-09-09): a /stand issued on the workstation never
# resolves. Kept here so the console can show them.
STAND_SEQUENCE = (
    'ros2 service call /claim std_srvs/srv/Trigger "{}"',
    'ros2 service call /power_on std_srvs/srv/Trigger "{}"',
    'ros2 service call /stand std_srvs/srv/Trigger "{}"',
)
SIT_SEQUENCE = (
    'ros2 service call /sit std_srvs/srv/Trigger "{}"',
    'ros2 service call /power_off std_srvs/srv/Trigger "{}"',
    'ros2 service call /release std_srvs/srv/Trigger "{}"',
)


def offboard_windows_for(depth_source: str):
    """The windows rpv_offboard will really have, for this depth source."""
    if depth_source == "zoedepth":
        return OFFBOARD_WINDOWS + ZOEDEPTH_WINDOWS
    return OFFBOARD_WINDOWS


@dataclass
class CmdResult:
    ok: bool
    code: int
    out: str

    def __bool__(self) -> bool:            # so `if pm.stop_offboard():` reads naturally
        return self.ok


def _real_run(cmd: List[str], timeout: float) -> CmdResult:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return CmdResult(p.returncode == 0, p.returncode, (p.stdout + p.stderr).strip())
    except subprocess.TimeoutExpired:
        return CmdResult(False, -1, f"timeout after {timeout}s")
    except Exception as e:
        return CmdResult(False, -1, f"{type(e).__name__}: {e}")


def _real_spawn(cmd: List[str], cwd: Optional[str], env: Optional[dict]):
    """Spawn a launcher and KEEP ITS OUTPUT.

    Still detached (`start_new_session`), because the stacks must outlive the
    GUI: closing the console must not take the robot's driver with it. But
    detached is not the same as deaf. This used to send stdout and stderr to
    DEVNULL, which made every way the launcher can fail --

        launch_offboard.sh: No such file or directory        (wrong CEAI_DIR)
        ERROR: session 'rpv_offboard' exists                (a half-dead attempt)
        ERROR: DEPTH_SOURCE must be one of: ...        (a typo in settings)

    -- look exactly like success: a yellow light, no message, no reason. The
    pipe is drained by ProcessManager._drain, always, so a chatty launcher can
    never block on a full buffer.
    """
    return subprocess.Popen(
        cmd, cwd=cwd, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, start_new_session=True,
    )


class ProcessManager:
    def __init__(self, cfg: Config, run=None, spawn=None, url_open=None) -> None:
        self.cfg = cfg
        self.run = run or _real_run
        self.spawn = spawn or _real_spawn
        self.url_open = url_open or urllib.request.urlopen

    # ------------------------------------------------------------------ #
    # command builders -- pure, and the unit under test
    # ------------------------------------------------------------------ #
    def onboard_start_command(self) -> str:
        """The line to run IN A PI SHELL. Nothing here executes it.

        Displayed by the console so that the operator types the same thing every
        time, with this rig's settings already substituted -- including the six
        measured mount numbers, without which /scan never appears.
        """
        c = self.cfg
        env = [
            f"SESSION={c.onboard_session}",
            f"ROBOT={c.robot}",
            f"CAMERA_MODEL={c.camera_model}",
            f"CEAI_DIR={c.onboard_ceai_dir}",
            f"ROS_WS={c.onboard_ros_ws}",
            f"OFFBOARD_SSH={c.offboard_ssh}",
            f"TUNNEL_PORT={c.tunnel_port}",
        ]
        if c.d435_complete():
            env += [f"D435_X={c.d435_x}", f"D435_Y={c.d435_y}", f"D435_Z={c.d435_z}",
                    f"D435_ROLL={c.d435_roll}", f"D435_PITCH={c.d435_pitch}",
                    f"D435_YAW={c.d435_yaw}"]
        if c.onboard_launch_args.strip():
            env.append(f"LAUNCH_ARGS={shlex.quote(c.onboard_launch_args.strip())}")
        return " ".join(env) + f" {c.onboard_ceai_dir}/scripts/launch_onboard.sh"

    def offboard_start_argv(self) -> List[str]:
        return ["bash", os.path.join(self.cfg.ceai_dir, "scripts", "launch_offboard.sh")]

    def offboard_start_env(self, target: str, start_bridge: bool = True) -> Dict[str, str]:
        c = self.cfg
        env = dict(os.environ)
        env.update({
            "SESSION": c.offboard_session,
            "ROS_WS": c.ros_ws,
            "CEAI_DIR": c.ceai_dir,
            "POLICY_REPO": os.path.expanduser(c.policy_repo),
            "TARGET": target,
            "ROBOT": c.robot,
            "CAMERA_MODEL": c.camera_model,
            "DEPTH_SOURCE": c.depth_source,
            "POLICY_PORT": str(c.policy_port),
            "ZOEDEPTH_PORT": str(c.zoedepth_port),
            "TUNNEL_PORT": str(c.tunnel_port),
            "START_BRIDGE": "1" if start_bridge else "0",
        })
        # RPV_MAX_DEPTH_M is deliberately NOT set here. The script derives it
        # from the camera profile, so there is one source for the number that
        # silently discards detections when it is wrong.
        # All six or none -- the script refuses a partial set, and so does the
        # launch file. Passing them one at a time would let a blank through as a
        # zero, and a guessed pitch tilts everything the policy ranges.
        if c.d435_complete():
            env.update({
                "D435_X": c.d435_x, "D435_Y": c.d435_y, "D435_Z": c.d435_z,
                "D435_ROLL": c.d435_roll, "D435_PITCH": c.d435_pitch, "D435_YAW": c.d435_yaw,
            })
        return env

    def bridge_command(self, target: str) -> str:
        """The bridge, as a module from the checkout.

        The module form needs no colcon build, which keeps a `git pull` enough
        to run. `ros2 run ceai rpv_bridge` is equivalent once the workspace is
        built.

        Every `-p name:=value` fragment ends in a space. Without one the next
        fragment concatenates onto it and ROS parses the pair as one malformed
        parameter, which fails SILENTLY: the parameter keeps its declared
        default. That cost a second per step on the TurtleBot 4 rig before
        anyone noticed.
        """
        c = self.cfg
        ros_setup = (
            f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash && "
            f"source {c.ros_ws}/install/setup.bash && "
            f"export RPV_POLICY_REPO={os.path.expanduser(c.policy_repo)} && "
            f"export NO_PROXY=127.0.0.1,localhost,::1 && export no_proxy=$NO_PROXY"
        )
        return (
            f"{ros_setup} && cd {c.ceai_dir} && "
            f"python3 -m ceai.rpv_bridge --ros-args "
            f"--params-file {c.bridge_params_file} "
            f"-p robot:={c.robot} "
            f"-p target:={shlex.quote(target)} "
            f"-p depth_source:={c.depth_source} "
            f"-p policy_url:={c.policy_url}"
        )

    @staticmethod
    def run_name(target: str) -> str:
        """`<stamp>_<target>`: the rosbag's directory name, and the name the
        decision log and the two videos take (gui/run_recorder.py), so one run's
        four artefacts match by filename."""
        stamp = time.strftime("%Y%m%d_%H%M%S")
        safe = "".join(ch if ch.isalnum() else "_" for ch in target) or "run"
        return f"{stamp}_{safe}"

    def bag_path(self, name: str) -> str:
        return os.path.join(os.path.expanduser(self.cfg.bag_dir), name)

    def bag_command(self, target: str, name: str = None) -> str:
        c = self.cfg
        out = self.bag_path(name or self.run_name(target))
        ros_setup = (
            f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash && "
            f"source {c.ros_ws}/install/setup.bash"
        )
        topics = " ".join(c.bag_topics_resolved)
        return (f"{ros_setup} && mkdir -p {shlex.quote(os.path.expanduser(c.bag_dir))} && "
                f"ros2 bag record -o {shlex.quote(out)} {topics}")

    def rviz_command(self) -> str:
        c = self.cfg
        ros_setup = (
            f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash && "
            f"source {c.ros_ws}/install/setup.bash"
        )
        return f"{ros_setup} && rviz2 -d {shlex.quote(c.rviz_path)}"

    # ------------------------------------------------------------------ #
    # the workstation session
    # ------------------------------------------------------------------ #
    def _drain(self, proc, label: str, log=None) -> None:
        """Read a spawned launcher to EOF on a daemon thread.

        Runs whether or not anyone is listening: an undrained PIPE stops the
        launcher dead once the buffer fills. With a `log` sink every line the
        script printed reaches the console -- including the `ERROR: ...` it dies
        on, and its exit code, which is the whole point of the exercise.

        `proc` is None under the test doubles (their spawn records and returns
        nothing) and that is not an error.
        """
        if proc is None or getattr(proc, "stdout", None) is None:
            return

        def pump() -> None:
            try:
                for line in proc.stdout:
                    line = line.rstrip()
                    if line and log is not None:
                        log(f"[{label}] {line}")
            except Exception:
                pass
            finally:
                try:
                    proc.stdout.close()
                except Exception:
                    pass
            try:
                code = proc.wait()
            except Exception:
                return
            if code != 0 and log is not None:
                log(f"[{label}] LAUNCHER EXITED {code} -- nothing was started.")

        threading.Thread(target=pump, daemon=True).start()

    def start_offboard(self, target: str, start_bridge: bool = True, log=None) -> None:
        proc = self.spawn(self.offboard_start_argv(), self.cfg.ceai_dir,
                          self.offboard_start_env(target, start_bridge))
        self._drain(proc, "rpv_offboard", log)

    def stop_offboard(self) -> CmdResult:
        return self.run(["tmux", "kill-session", "-t", self.cfg.offboard_session], 10)

    def offboard_alive(self) -> bool:
        return self.run(["tmux", "has-session", "-t", self.cfg.offboard_session], 5).ok

    def offboard_windows(self) -> List[str]:
        r = self.run(["tmux", "list-windows", "-t", self.cfg.offboard_session,
                      "-F", "#{window_name}"], 5)
        return r.out.split() if r.ok else []

    # ------------------------------------------------------------------ #
    # servers
    # ------------------------------------------------------------------ #
    def _http_up(self, url: str, timeout: float = 3.0) -> bool:
        """Any HTTP reply means up -- a 404 on / is still a live server."""
        try:
            self.url_open(url, timeout=timeout)
            return True
        except urllib.error.HTTPError:
            return True
        except Exception:
            return False

    def policy_up(self) -> bool:
        return self._http_up(self.cfg.policy_base_url + "/")

    def zoedepth_up(self) -> bool:
        return self._http_up(f"http://{self.cfg.policy_host}:{self.cfg.zoedepth_port}/")

    # ------------------------------------------------------------------ #
    # windows within rpv_offboard
    # ------------------------------------------------------------------ #
    def _kill_window(self, name: str) -> CmdResult:
        return self.run(["tmux", "kill-window", "-t", f"{self.cfg.offboard_session}:{name}"], 5)

    def _new_window(self, name: str, command: str) -> CmdResult:
        return self.run(["tmux", "new-window", "-t", self.cfg.offboard_session,
                         "-n", name, command], 5)

    def stop_bridge(self) -> CmdResult:
        if not self.offboard_alive():
            return CmdResult(False, -1, f"session '{self.cfg.offboard_session}' not running")
        return self._kill_window("bridge")

    def restart_bridge(self, target: str) -> CmdResult:
        """Kill and recreate the bridge window.

        This IS the episode reset: a fresh bridge sends is_first=true on its
        first POST, which clears the policy's maps and re-pins the episodic
        origin wherever the robot is standing now. There is no separate reset
        call, and changing the target any other way makes the policy force a
        reset on its own mid-run.
        """
        if not self.offboard_alive():
            return CmdResult(False, -1,
                             f"session '{self.cfg.offboard_session}' not running -- start the workstation first")
        self._kill_window("bridge")
        return self._new_window("bridge", self.bridge_command(target))

    def restart_policy(self) -> CmdResult:
        """Re-run the policy server in its own window, after a SAM3 list edit.

        Ctrl-C, then Up + Enter: the window's shell has the full conda command in
        its history because launch_offboard_ai.sh typed it there, so this
        re-runs it verbatim rather than this module keeping a second copy of the
        AI environment (the scrub, NO_PROXY, every SPOT_* knob) that could drift
        from the script's.

        If it does not come back, kill the session and start it again -- that is
        always correct and only costs the model load.
        """
        if not self.offboard_alive():
            return CmdResult(False, -1, f"session '{self.cfg.offboard_session}' not running")
        target = f"{self.cfg.offboard_session}:policy"
        r = self.run(["tmux", "send-keys", "-t", target, "C-c"], 5)
        if not r.ok:
            return r
        time.sleep(2.0)                          # let Flask release :13000
        self.run(["tmux", "send-keys", "-t", target, "Up"], 5)
        return self.run(["tmux", "send-keys", "-t", target, "Enter"], 5)

    # --- rosbag ---------------------------------------------------------- #
    def recording(self) -> bool:
        return BAG_WINDOW in self.offboard_windows()

    def start_recording(self, target: str, name: str = None) -> CmdResult:
        if not self.offboard_alive():
            return CmdResult(False, -1, f"session '{self.cfg.offboard_session}' not running")
        if self.recording():
            return CmdResult(False, -1, "already recording")
        return self._new_window(BAG_WINDOW, self.bag_command(target, name))

    def stop_recording(self) -> CmdResult:
        """Ctrl-C first, THEN kill the window.

        ros2 bag finalises its metadata.yaml on SIGINT. Killing the tmux window
        outright leaves a bag directory whose metadata is missing or truncated,
        and that bag is the evidence for the run -- it is not worth two seconds.
        """
        if not self.offboard_alive():
            return CmdResult(False, -1, f"session '{self.cfg.offboard_session}' not running")
        if not self.recording():
            return CmdResult(False, -1, "not recording")
        self.run(["tmux", "send-keys", "-t", f"{self.cfg.offboard_session}:{BAG_WINDOW}", "C-c"], 5)
        time.sleep(2.0)
        return self._kill_window(BAG_WINDOW)

    # --- RViz ------------------------------------------------------------ #
    def rviz_alive(self) -> bool:
        return RVIZ_WINDOW in self.offboard_windows()

    def start_rviz(self) -> CmdResult:
        if not self.offboard_alive():
            return CmdResult(False, -1, f"session '{self.cfg.offboard_session}' not running")
        if not os.path.isfile(os.path.expanduser(self.cfg.rviz_path)):
            return CmdResult(False, -1, f"rviz config not found: {self.cfg.rviz_path}")
        if self.rviz_alive():
            self._kill_window(RVIZ_WINDOW)
        return self._new_window(RVIZ_WINDOW, self.rviz_command())

    def stop_rviz(self) -> CmdResult:
        return self._kill_window(RVIZ_WINDOW)

    # ------------------------------------------------------------------ #
    def probe(self) -> dict:
        """One cycle of state for the lights. Never raises.

        Nothing here reaches the Pi. The console's PI light is driven by /scan
        arriving over the zenoh bridge instead, which is EVIDENCE the Pi is up
        and its camera is streaming -- strictly more than `tmux has-session`
        told us, and it works on a topology where SSH does not.
        """
        dt = self.offboard_alive()
        windows = self.offboard_windows() if dt else []
        return {
            "offboard": dt,
            "offboard_windows": windows,
            "bridge": "bridge" in windows,
            "slam": "ros" in windows,
            "recording": BAG_WINDOW in windows,
            "rviz": RVIZ_WINDOW in windows,
            "policy": self.policy_up() if dt else False,
            "zoedepth": self.zoedepth_up() if dt else False,
        }
