#!/usr/bin/env bash
# =============================================================================
# The Pi's whole side of the experiment, in ONE tmux session.
#
#   tunnel     autossh -L 7447 -> the workstation  (zenoh rides inside it)
#   zenoh      zenoh-bridge-ros2dds, CONNECTING side
#   stack      robot_onboard.launch.py: the robot wrapper + the D435 (RGBD) +
#              the depth -> cloud -> /scan chain
#   lifecycle  an idle shell. /claim, /power_on, /stand are typed HERE and
#              nowhere else -- services do not cross the zenoh bridge
#              (measured 2026-09-09).
#
#     ./scripts/launch_onboard.sh
#     tmux attach -t rpv_onboard          # Ctrl-B w to pick a window, Ctrl-B d to leave
#     ./scripts/launch_onboard.sh stop     # SIGINT the stack first, THEN kill the session
#
# NEVER `kill -9` THE CAMERA. On this rig SIGKILL on a streaming RealSense node
# skips the UVC close and leaves the USB3 link unable to re-enumerate UNTIL THE
# CAMERA IS PHYSICALLY REPLUGGED -- it cost two replugs during the 2026-09-15
# investigation. `stop` below sends SIGINT and waits for "Close Sensor - Done.".
# `tmux kill-session` on its own does not do that; use `stop`.
#
# ORDER MATTERS AND THIS SCRIPT CANNOT ENFORCE ALL OF IT. The workstation's
# zenoh must already be LISTENING before this runs: the Pi is the side that
# connects out. The console sequences that; by hand, start the workstation first.
#
# Environment (all optional):
#   SESSION      tmux session name                     [rpv_onboard]
#   ROBOT        which robot profile                   [spot]
#   CEAI_DIR     this repository                      [the directory above this script]
#   ROS_WS       the Pi's workspace -- ros_ws, NOT ros2_ws  [$HOME/ros_ws]
#   OFFBOARD_SSH user@host for the tunnel              [REQUIRED -- the workstation]
#   TUNNEL_PORT  forwarded port                        [7447]
#   SPOT_LOGIN   credentials sourced before the driver [$HOME/.spot_login]
#   TUNNEL_KEY   ssh key for the autossh tunnel        [$HOME/.ssh/id_tunnel]
#   D435_X D435_Y D435_Z D435_ROLL D435_PITCH D435_YAW
#                OVERRIDE the robot profile's recorded mount pose. All six or
#                none. Normally leave them unset -- ceai/robots/<robot>.py has
#                the measured values with their provenance.
#   CAMERA_MODEL which camera                           [d435]
#   COLOR_PROFILE / DEPTH_PROFILE  WxHxFPS each        [424x240x6]
#   LAUNCH_ARGS  extra robot_onboard.launch.py arguments    [empty]
#   START_TUNNEL / START_ZENOH / START_STACK   1 or 0  [1]
# =============================================================================
set -u

SESSION=${SESSION:-rpv_onboard}
ROBOT=${ROBOT:-spot}
ROS_WS=${ROS_WS:-$HOME/ros_ws}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CEAI_DIR=${CEAI_DIR:-$(dirname "$SCRIPT_DIR")}
OFFBOARD_SSH=${OFFBOARD_SSH:-}
TUNNEL_PORT=${TUNNEL_PORT:-7447}
SPOT_LOGIN=${SPOT_LOGIN:-$HOME/.spot_login}
TUNNEL_KEY=${TUNNEL_KEY:-$HOME/.ssh/id_tunnel}
CAMERA_MODEL=${CAMERA_MODEL:-d435}
COLOR_PROFILE=${COLOR_PROFILE:-424x240x6}
DEPTH_PROFILE=${DEPTH_PROFILE:-424x240x6}
LAUNCH_ARGS=${LAUNCH_ARGS:-}
START_TUNNEL=${START_TUNNEL:-1}
START_ZENOH=${START_ZENOH:-1}
START_STACK=${START_STACK:-1}
ROS_DISTRO_USE=${ROS_DISTRO_USE:-humble}
LAUNCH_FILE=$CEAI_DIR/launch/robot_onboard.launch.py

fail() { echo "ERROR: $*" >&2; exit 1; }

# --- stop: SIGINT the camera, wait, THEN kill ---------------------------------
# A streaming RealSense node must close its UVC handles. Killing the session
# outright skips that and can cost a physical replug.
if [ "${1:-}" = "stop" ]; then
    tmux has-session -t "$SESSION" 2>/dev/null || { echo "no session '$SESSION'"; exit 0; }
    echo "Stopping '$SESSION': SIGINT to the stack first (the camera must close cleanly)..."
    tmux send-keys -t "$SESSION:stack" C-c 2>/dev/null
    for _ in $(seq 1 15); do
        sleep 1
        tmux capture-pane -p -t "$SESSION:stack" 2>/dev/null | grep -q 'Close Sensor - Done' && \
            { echo "  camera closed cleanly."; break; }
    done
    tmux kill-session -t "$SESSION"
    echo "Stopped."
    exit 0
fi

command -v tmux >/dev/null || fail "tmux not installed"
[ -d "$CEAI_DIR" ] || fail "CEAI not found at $CEAI_DIR (set CEAI_DIR=...)"
[ -f "$LAUNCH_FILE" ] || fail "$LAUNCH_FILE missing"
[ -f "$CEAI_DIR/config/zenoh_onboard.json5" ] || fail "$CEAI_DIR/config/zenoh_onboard.json5 missing"
tmux has-session -t "$SESSION" 2>/dev/null && fail "session '$SESSION' exists ($0 stop)"

# --- the six mount numbers: all or none, and OPTIONAL here --------------------
# The robot profile carries the measured mount (ceai/robots/<robot>.py), so the
# normal case is to pass nothing. These override it for a camera that has been
# remounted and not yet written down.
#
# A PARTIAL set is still refused, here rather than 40 seconds later inside the
# launch: a guessed pitch tilts everything both Nav2 and the policy see. The
# launch refuses on its own if neither this nor the profile has a mount, because
# then /scan never appears at all.
MOUNT_ARGS=""
_given=0
for _v in "${D435_X:-}" "${D435_Y:-}" "${D435_Z:-}" "${D435_ROLL:-}" "${D435_PITCH:-}" "${D435_YAW:-}"; do
    [ -n "$_v" ] && _given=$((_given+1))
done
case "$_given" in
  6) MOUNT_ARGS="d435_x:=$D435_X d435_y:=$D435_Y d435_z:=$D435_Z d435_roll:=$D435_ROLL d435_pitch:=$D435_PITCH d435_yaw:=$D435_YAW"
     echo "mount pose: OVERRIDDEN from the environment -- $MOUNT_ARGS" ;;
  0) echo "mount pose: from the $ROBOT profile (ceai/robots/$ROBOT.py)" ;;
  *) fail "the D435 mount pose is all-or-nothing: got $_given of 6. Supply all six MEASURED numbers, or none to use the $ROBOT profile's recorded mount." ;;
esac

# Every window sets its own environment. A tmux window inherits the tmux
# SERVER's environment, not this script's, so exporting here would not reach
# them -- a class of bug that shows up as "it works in my terminal".
#
# ROS_DISTRO is LOAD-BEARING for the zenoh bridge: unset, it assumes 'iron'
# (16-byte GIDs), cannot parse Humble's ros_discovery_info (24-byte), and every
# service call across the link dies with "sequence size exceeds remaining
# buffer" -- while topics keep working, which is what makes it confusing.
ROS_SETUP="source /opt/ros/${ROS_DISTRO_USE}/setup.bash"
[ -f "$ROS_WS/install/setup.bash" ] && ROS_SETUP="$ROS_SETUP && source $ROS_WS/install/setup.bash"
ROS_SETUP="$ROS_SETUP && export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} && unset ROS_LOCALHOST_ONLY"
ROS_SETUP="$ROS_SETUP && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_DISTRO=${ROS_DISTRO_USE}"

new_win() { tmux new-window -t "$SESSION" -n "$1"; tmux send-keys -t "$SESSION:$1" "$2" C-m; }

# --- the tunnel --------------------------------------------------------------
# FOREGROUND autossh, not the bashrc alias's `-f`: a window that owns the tunnel
# can be watched, and killing the session kills the tunnel with it. If a
# backgrounded tunnel is already holding the port we say so and idle rather than
# dying on "Address already in use" -- that is a working tunnel, not an error.
TUNNEL_CMD="if ss -ltn 2>/dev/null | grep -q '127.0.0.1:${TUNNEL_PORT} '; then"
TUNNEL_CMD="$TUNNEL_CMD echo '[tunnel] port ${TUNNEL_PORT} already forwarded (started outside this session) -- reusing it';"
TUNNEL_CMD="$TUNNEL_CMD echo '[tunnel] to hand it to this window: pkill -f \"${TUNNEL_PORT}:localhost:${TUNNEL_PORT}\" and restart the session';"
TUNNEL_CMD="$TUNNEL_CMD exec sleep infinity; fi;"
# -i NAMES THE KEY, and that is load-bearing. id_tunnel is not a filename ssh
# tries by default, so without -i the tunnel falls through to the default keys
# and dies on "Permission denied (publickey,password)" -- measured 2026-09-11.
# IdentitiesOnly stops ssh offering the other keys first and tripping
# MaxAuthTries.
TUNNEL_ID=""
[ -f "$TUNNEL_KEY" ] && TUNNEL_ID="-i $TUNNEL_KEY -o IdentitiesOnly=yes"
[ -n "$TUNNEL_ID" ] || echo "NOTE: $TUNNEL_KEY not found -- the tunnel will use ssh's default keys."
TUNNEL_CMD="$TUNNEL_CMD exec autossh $TUNNEL_ID -M 0 -N -o ServerAliveInterval=10 -o ServerAliveCountMax=3"
TUNNEL_CMD="$TUNNEL_CMD -o ExitOnForwardFailure=yes -o BatchMode=yes"
TUNNEL_CMD="$TUNNEL_CMD -L ${TUNNEL_PORT}:localhost:${TUNNEL_PORT} ${OFFBOARD_SSH}"

ZENOH_CMD="ROS_DISTRO=${ROS_DISTRO_USE} zenoh-bridge-ros2dds -c $CEAI_DIR/config/zenoh_onboard.json5"

STACK_CMD="$ROS_SETUP"
[ -f "$SPOT_LOGIN" ] && STACK_CMD="$STACK_CMD && source $SPOT_LOGIN"
STACK_CMD="$STACK_CMD && ros2 launch $LAUNCH_FILE robot:=$ROBOT"
STACK_CMD="$STACK_CMD camera_model:=$CAMERA_MODEL"
STACK_CMD="$STACK_CMD color_profile:=$COLOR_PROFILE depth_profile:=$DEPTH_PROFILE"
[ -n "$MOUNT_ARGS" ] && STACK_CMD="$STACK_CMD $MOUNT_ARGS"
[ -n "$LAUNCH_ARGS" ] && STACK_CMD="$STACK_CMD $LAUNCH_ARGS"

echo "Starting '$SESSION' on $(hostname) for robot '$ROBOT' ..."
tmux new-session -d -s "$SESSION" -n tunnel
if [ "$START_TUNNEL" = "1" ]; then
    tmux send-keys -t "$SESSION:tunnel" "$TUNNEL_CMD" C-m
    sleep 2
else
    tmux send-keys -t "$SESSION:tunnel" "echo '[tunnel] START_TUNNEL=0 -- not started'" C-m
fi

[ "$START_ZENOH" = "1" ] && new_win zenoh "$ZENOH_CMD"
[ "$START_STACK" = "1" ] && new_win stack "$STACK_CMD"

# The lifecycle window is a plain shell on purpose: it is where a HUMAN types the
# service calls. They cannot be typed on the workstation -- services do not cross
# the zenoh bridge.
tmux new-window -t "$SESSION" -n lifecycle
tmux send-keys -t "$SESSION:lifecycle" "$ROS_SETUP" C-m
tmux send-keys -t "$SESSION:lifecycle" "clear; echo 'Lifecycle terminal. Services live HERE -- they do not cross zenoh.'; echo '  ros2 service call /claim std_srvs/srv/Trigger \"{}\"'; echo '  ros2 service call /power_on std_srvs/srv/Trigger \"{}\"'; echo '  ros2 service call /stand std_srvs/srv/Trigger \"{}\"'" C-m

cat <<EOF

Pi session '$SESSION' is up.
  windows : tunnel  zenoh  stack  lifecycle
  attach  : tmux attach -t $SESSION      (Ctrl-B w = pick, Ctrl-B d = leave)
  stop    : $0 stop        <- NOT tmux kill-session; the camera must close cleanly

Checks worth running before the robot stands:
  $CEAI_DIR/scripts/check_onboard.sh            # proves the PATCHED realsense is loaded
  timeout 20 ros2 topic hz /scan                                        # ~= depth fps
  timeout 20 ros2 topic hz /camera/camera/color/image_raw
  timeout 20 ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
  ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw --field encoding --once
                                                                        # MUST be 16UC1
EOF
