#!/usr/bin/env bash
# =============================================================================
# The workstation's whole side of the experiment, in ONE tmux session.
#
#   zenoh      zenoh-bridge-ros2dds, LISTENING side -- started first, because
#              the Pi is the side that connects out
#   ros        nav_stack.launch.py: slam_toolbox + Nav2
#   vlm-clip   \
#   vlm-sam3    |  from rpv-sim2real-policy's own launch_offboard_ai.sh,
#   policy     /   re-used with REUSE_SESSION=1 so its windows land HERE.
#                  That script still owns the ROS-environment scrub, NO_PROXY,
#                  the port pre-flight and the port waits -- one source of truth.
#                  (Its resource-monitor window was removed 2026-09-19.)
#   bridge     python3 -m ceai.rpv_bridge -- the search itself
#
#   zoedepth   } ONLY when DEPTH_SOURCE=zoedepth. With the D435 supplying real
#   zoe-chain  } metric depth these two are not started at all.
#
#     TARGET=backpack ./scripts/launch_offboard.sh
#     tmux attach -t rpv_offboard          # Ctrl-B w to pick a window
#     tmux kill-session -t rpv_offboard
#
# RUN IT FROM A PLAIN SHELL OR A rosenv ONE -- either works. Every window sets
# its own environment (a tmux window inherits the tmux SERVER's environment, not
# this script's), and the AI windows scrub ROS out of themselves.
#
# Environment (all optional):
#   SESSION          tmux session name                [rpv_offboard]
#   ROBOT            which robot profile              [spot]
#   ROS_WS           [$HOME/ros2_ws]      CEAI_DIR    [this repository: the directory above this script]
#   POLICY_REPO      the rpv-sim2real-policy checkout [$RPV_POLICY_REPO, else $HOME/rpv-sim2real-policy]
#   TARGET           object goal for the bridge       [backpack]
#   DEPTH_SOURCE     realsense | zoedepth | lidar | stub   [realsense]
#   CAMERA_MODEL     which camera feeds /scan         [d435]
#   POLICY_PORT [13000]   ZOEDEPTH_PORT [12185]
#   RPV_MAX_DEPTH_M / RPV_MIN_DEPTH_M
#                    DERIVED from the camera profile unless set. See below --
#                    getting these wrong discards detections silently.
#   D435_PITCH D435_ROLL
#                    OVERRIDE the robot profile's recorded camera tilt. Normally
#                    unset: the profile's mount is what the Pi publishes as TF,
#                    and the policy is handed the same pitch so detections land
#                    at the right height.
#   START_ZENOH / START_ROS / START_AI / START_ZOE / START_CHAIN / START_BRIDGE
#                    1 or 0                           [all 1]
# =============================================================================
set -u

SESSION=${SESSION:-rpv_offboard}
ROBOT=${ROBOT:-spot}
ROS_WS=${ROS_WS:-$HOME/ros2_ws}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CEAI_DIR=${CEAI_DIR:-$(dirname "$SCRIPT_DIR")}
POLICY_REPO=${POLICY_REPO:-${RPV_POLICY_REPO:-$HOME/rpv-sim2real-policy}}
TARGET=${TARGET:-backpack}
DEPTH_SOURCE=${DEPTH_SOURCE:-realsense}
CAMERA_MODEL=${CAMERA_MODEL:-d435}
POLICY_PORT=${POLICY_PORT:-13000}
ZOEDEPTH_PORT=${ZOEDEPTH_PORT:-12185}
# config/<robot>_bridge.yaml (2026-09-26; was spot_bridge.yaml for every robot,
# which on the TurtleBot 4 meant sit_on_finish waiting 10 s for a /sit that
# does not exist).
BRIDGE_PARAMS=${BRIDGE_PARAMS:-$CEAI_DIR/config/${ROBOT}_bridge.yaml}
ROS_DISTRO_USE=${ROS_DISTRO_USE:-humble}
TUNNEL_PORT=${TUNNEL_PORT:-7447}

fail() { echo "ERROR: $*" >&2; exit 1; }
command -v tmux >/dev/null || fail "tmux not installed"
[ -d "$CEAI_DIR" ] || fail "CEAI not found at $CEAI_DIR (set CEAI_DIR=...)"
[ -d "$POLICY_REPO" ] || echo "WARN: rpv-sim2real-policy not found at $POLICY_REPO (set RPV_POLICY_REPO=...); the AI half will be skipped"
[ -f "$BRIDGE_PARAMS" ] || fail "bridge parameter file not found: $BRIDGE_PARAMS (config/<robot>_bridge.yaml; set BRIDGE_PARAMS=... to point elsewhere)"
[ -f "$CEAI_DIR/config/zenoh_offboard.json5" ] || fail "$CEAI_DIR/config/zenoh_offboard.json5 missing"
tmux has-session -t "$SESSION" 2>/dev/null && fail "session '$SESSION' exists (tmux kill-session -t $SESSION)"

# --- the policy's depth band and camera tilt, derived from ONE source -------
# RPV_MAX_DEPTH_M is BOTH the [0,1] normalisation divisor AND the value the
# policy reads as "out of range", so a wrong value silently DISCARDS detections
# rather than erroring. It used to be defaulted differently in two files (2.0 in
# the AI repo's script, 8.0 here) and had to be remembered by hand.
#
# It now comes from the same place as slam_toolbox's max_laser_range and
# Nav2's obstacle_max_range -- ceai/cameras.py obstacle_range_m, the distance
# inside which a return is trusted as an object -- when the depth is real, and
# from the ZoeDepth node's trust_max_m when it is monocular. NOT max_range_m:
# that is how far /scan is published (10 m), and a detection ranged out there
# is placed from depth the map itself no longer trusts.
#
# The camera tilt comes from the robot profile's recorded mount
# (ceai/robots/<robot>.py): the same six numbers the Pi publishes as
# base -> camera_link. The policy needs the SAME pitch, or it places every
# detection as though the lens sat at its own default, and nothing downstream
# can tell.
depth_band_from_camera() {
    # Prints: min_range max_range
    python3 - "$CEAI_DIR" "$CAMERA_MODEL" <<'PY' 2>/dev/null
import sys
sys.path.insert(0, sys.argv[1])
from ceai.cameras import load
camera = load(sys.argv[2])
print(f"{camera.min_range_m} {camera.obstacle_range_m}")
PY
}

camera_tilt_from_profile() {
    # Prints: pitch roll -- or nothing when the robot has no recorded mount.
    python3 - "$CEAI_DIR" "$ROBOT" <<'PY' 2>/dev/null
import sys
sys.path.insert(0, sys.argv[1])
from ceai.robots import load
mount = load(sys.argv[2]).camera_mount
if mount is not None:
    print(f"{mount.pitch!r} {mount.roll!r}")
PY
}

case "$DEPTH_SOURCE" in
  realsense|lidar|stub)
      # Real depth, or the /scan curtain built from it: the camera's own band.
      _band=$(depth_band_from_camera) || _band=""
      [ -n "$_band" ] || fail "could not read the $CAMERA_MODEL depth band from ceai/cameras.py. Set RPV_MIN_DEPTH_M and RPV_MAX_DEPTH_M by hand, and check that $CEAI_DIR is a CEAI checkout."
      RPV_MIN_DEPTH_M=${RPV_MIN_DEPTH_M:-${_band% *}}
      RPV_MAX_DEPTH_M=${RPV_MAX_DEPTH_M:-${_band#* }}
      ;;
  zoedepth)
      # ZoeDepth's own trust_max_m, not the camera's range.
      RPV_MIN_DEPTH_M=${RPV_MIN_DEPTH_M:-0.5}
      RPV_MAX_DEPTH_M=${RPV_MAX_DEPTH_M:-8.0}
      ;;
  *) fail "DEPTH_SOURCE must be one of: realsense zoedepth lidar stub (got '$DEPTH_SOURCE')" ;;
esac
echo "depth: $DEPTH_SOURCE, band ${RPV_MIN_DEPTH_M}-${RPV_MAX_DEPTH_M} m"

# Highest first: RPV_CAMERA_*_RAD (the policy's own names), D435_PITCH /
# D435_ROLL (a remounted camera -- the same override the Pi script takes), then
# the profile. Empty when none of them is set, and the AI half is then told
# nothing rather than an empty string.
_tilt=$(camera_tilt_from_profile) || _tilt=""
RPV_CAMERA_PITCH_RAD=${RPV_CAMERA_PITCH_RAD:-${D435_PITCH:-${_tilt% *}}}
RPV_CAMERA_ROLL_RAD=${RPV_CAMERA_ROLL_RAD:-${D435_ROLL:-${_tilt#* }}}

# ZoeDepth is only started when it is the depth source. With the D435 giving
# real metric depth, loading a monocular depth model would cost GPU memory the
# policy wants and publish a raster nothing reads.
if [ "$DEPTH_SOURCE" = "zoedepth" ]; then
    START_ZOE=${START_ZOE:-1}; START_CHAIN=${START_CHAIN:-1}
else
    START_ZOE=${START_ZOE:-0}; START_CHAIN=${START_CHAIN:-0}
fi
START_ZENOH=${START_ZENOH:-1}
START_ROS=${START_ROS:-1}
START_AI=${START_AI:-1}
START_BRIDGE=${START_BRIDGE:-1}

RGB_TOPIC=${RGB_TOPIC:-/camera/camera/color/image_raw/compressed}
CAMERA_INFO_TOPIC=${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}

ROS_SETUP="source /opt/ros/${ROS_DISTRO_USE}/setup.bash"
[ -f "$ROS_WS/install/setup.bash" ] && ROS_SETUP="$ROS_SETUP && source $ROS_WS/install/setup.bash"
ROS_SETUP="$ROS_SETUP && export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} && unset ROS_LOCALHOST_ONLY"
ROS_SETUP="$ROS_SETUP && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_DISTRO=${ROS_DISTRO_USE}"
# The bridge imports vlfm/vlm/depth_codec.py from the other repo for the depth
# wire codec. Forgetting this is a ModuleNotFoundError a long way from its cause.
ROS_SETUP="$ROS_SETUP && export RPV_POLICY_REPO=$POLICY_REPO"
# A configured http_proxy otherwise eats the loopback POST to the policy and
# reports a proxy error that names the proxy, not the policy.
ROS_SETUP="$ROS_SETUP && export NO_PROXY=127.0.0.1,localhost,::1 && export no_proxy=\$NO_PROXY"

new_win() { tmux new-window -t "$SESSION" -n "$1"; tmux send-keys -t "$SESSION:$1" "$2" C-m; }

wait_for_port() {
    local port=$1 label=$2 timeout=$3 waited=0
    echo -n "  waiting for $label (:$port) "
    while ! ss -ltn 2>/dev/null | grep -q ":$port "; do
        sleep 2; waited=$((waited+2)); echo -n "."
        [ "$waited" -ge "$timeout" ] && { echo " TIMEOUT (continuing)"; return 1; }
    done
    echo " up (${waited}s)"
}

echo "Starting '$SESSION' (robot: $ROBOT) ..."

# --- zenoh, FIRST -----------------------------------------------------------
ZENOH_CMD="ROS_DISTRO=${ROS_DISTRO_USE} zenoh-bridge-ros2dds -c $CEAI_DIR/config/zenoh_offboard.json5"
tmux new-session -d -s "$SESSION" -n zenoh
if [ "$START_ZENOH" = "1" ]; then
    tmux send-keys -t "$SESSION:zenoh" "$ZENOH_CMD" C-m
    wait_for_port "$TUNNEL_PORT" zenoh 30
else
    tmux send-keys -t "$SESSION:zenoh" "echo '[zenoh] START_ZENOH=0 -- not started'" C-m
fi

# --- SLAM + Nav2 -------------------------------------------------------------
# Starts before the Pi is necessarily up. That is fine and deliberate:
# slam_toolbox simply waits for /scan, and starting it late is the more common
# way to end up with a half-built map.
[ "$START_ROS" = "1" ] && new_win ros \
    "$ROS_SETUP && ros2 launch $CEAI_DIR/launch/nav_stack.launch.py robot:=$ROBOT camera_model:=$CAMERA_MODEL"

# --- the AI half, in THIS session -------------------------------------------
# `echo y |` answers launch_offboard_ai.sh's earlyoom prompt non-interactively
# (it refuses to start silently otherwise, and a GUI-spawned script has no tty).
if [ "$START_AI" = "1" ]; then
    if [ -x "$POLICY_REPO/scripts/launch_offboard_ai.sh" ]; then
        # The tilt is passed only when something supplied it: an empty value
        # would override the AI script's own default with nothing.
        AI_CAMERA_ENV=""
        if [ -n "$RPV_CAMERA_PITCH_RAD" ]; then
            AI_CAMERA_ENV="RPV_CAMERA_PITCH_RAD=$RPV_CAMERA_PITCH_RAD"
            echo "AI half: camera pitch $RPV_CAMERA_PITCH_RAD rad (lens down)"
        else
            echo "NOTE: robot '$ROBOT' has no recorded camera mount and no D435_PITCH was"
            echo "      given, so the policy keeps launch_offboard_ai.sh's default tilt."
        fi
        [ -n "$RPV_CAMERA_ROLL_RAD" ] && \
            AI_CAMERA_ENV="$AI_CAMERA_ENV RPV_CAMERA_ROLL_RAD=$RPV_CAMERA_ROLL_RAD"

        echo y | env REUSE_SESSION=1 SESSION="$SESSION" REPO="$POLICY_REPO" \
            RPV_ROBOT="$ROBOT" \
            RPV_POLICY_PORT="$POLICY_PORT" \
            RPV_MAX_DEPTH_M="$RPV_MAX_DEPTH_M" RPV_MIN_DEPTH_M="$RPV_MIN_DEPTH_M" \
            $AI_CAMERA_ENV \
            "$POLICY_REPO/scripts/launch_offboard_ai.sh"
    else
        echo "WARN: $POLICY_REPO/scripts/launch_offboard_ai.sh not found or not executable -- skipping the AI half."
    fi
fi

if [ "$START_ZOE" = "1" ]; then
    if [ -x "$POLICY_REPO/scripts/launch_zoedepth_server.sh" ]; then
        REUSE_SESSION=1 SESSION="$SESSION" REPO="$POLICY_REPO" \
            ZOEDEPTH_PORT="$ZOEDEPTH_PORT" \
            "$POLICY_REPO/scripts/launch_zoedepth_server.sh"
    else
        echo "WARN: $POLICY_REPO/scripts/launch_zoedepth_server.sh not found -- skipping ZoeDepth."
    fi
fi

# --- the ROS side of ZoeDepth ------------------------------------------------
# AFTER the model server: this node POSTs to loopback and dies with
# "Connection refused" if it starts first.
[ "$START_CHAIN" = "1" ] && new_win zoe-chain \
    "$ROS_SETUP && ros2 launch $POLICY_REPO/ros_nodes/zoedepth_chain.launch.py \
zoedepth_url:=http://127.0.0.1:$ZOEDEPTH_PORT/zoedepth \
rgb_topic:=$RGB_TOPIC camera_info_topic:=$CAMERA_INFO_TOPIC scale_align:=true"

# --- the bridge --------------------------------------------------------------
# Every `-p name:=value` fragment ends in a space. Without one the next fragment
# concatenates onto it and ROS parses the pair as a single malformed parameter,
# which fails SILENTLY: the parameter keeps its declared default.
# Run as a MODULE from the checkout, not `ros2 run ceai rpv_bridge`. Both work,
# but the module form needs no colcon build -- and nothing else in either
# launcher does either, so a fresh `git pull` on the Pi or here is enough to run.
BRIDGE_CMD="$ROS_SETUP && cd $CEAI_DIR && python3 -m ceai.rpv_bridge --ros-args"
BRIDGE_CMD="$BRIDGE_CMD --params-file $BRIDGE_PARAMS"
BRIDGE_CMD="$BRIDGE_CMD -p robot:=$ROBOT"
BRIDGE_CMD="$BRIDGE_CMD -p target:=\"$TARGET\""
BRIDGE_CMD="$BRIDGE_CMD -p depth_source:=$DEPTH_SOURCE"
BRIDGE_CMD="$BRIDGE_CMD -p policy_url:=http://127.0.0.1:$POLICY_PORT/rpv_policy"
[ "$START_BRIDGE" = "1" ] && new_win bridge "$BRIDGE_CMD"

cat <<EOF

Workstation session '$SESSION' is up.
  robot     : $ROBOT      target: $TARGET      depth: $DEPTH_SOURCE
  policy    : http://127.0.0.1:$POLICY_PORT/rpv_policy   (GET /status for the console)
  depth band: ${RPV_MIN_DEPTH_M} - ${RPV_MAX_DEPTH_M} m  (RPV_MIN/MAX_DEPTH_M)
  cam tilt  : pitch ${RPV_CAMERA_PITCH_RAD:-<policy default>} rad, roll ${RPV_CAMERA_ROLL_RAD:-<policy default>} rad
  attach    : tmux attach -t $SESSION       stop: tmux kill-session -t $SESSION

RESTARTING THE BRIDGE IS THE EPISODE RESET. A fresh bridge sends is_first=true,
which clears the policy's maps and re-pins the episodic origin where the robot
is standing now. There is no reset service.

The Pi is the OTHER half. If it is not up yet:
  <rpv-ros2-bridge>/scripts/launch_onboard.sh      # in a Pi shell
EOF
