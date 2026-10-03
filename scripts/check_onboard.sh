#!/usr/bin/env bash
# Verify the Pi's one-time setup. Read-only.
#   OFFBOARD_SSH  user@host of the workstation the tunnel points at (optional)
ok(){ printf '  \033[32mPASS\033[0m %s\n' "$1"; }
no(){ printf '  \033[31mFAIL\033[0m %s\n' "$1"; }
na(){ printf '  \033[33m--  \033[0m %s\n' "$1"; }
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CEAI=${CEAI_DIR:-$(dirname "$SCRIPT_DIR")}
OFFBOARD_SSH=${OFFBOARD_SSH:-}

echo "== S1 bashrc =="
grep -q 'ros_ws/install/setup.bash' ~/.bashrc && ok "ros_ws sourced" || no "ros_ws line missing from ~/.bashrc"
[ "${ROS_DOMAIN_ID:-unset}" != "unset" ] && ok "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || no "ROS_DOMAIN_ID unset (must match the workstation)"
[ -z "${ROS_LOCALHOST_ONLY:-}" ] && ok "ROS_LOCALHOST_ONLY not set" || no "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY -- kills off-machine DDS"
[ -n "${SPOT_IP:-}" ] && ok "SPOT_IP=$SPOT_IP" || no "SPOT_IP unset"
ip -4 addr show eth0 2>/dev/null | grep -q inet && \
  na "eth0 subnet: $(ip -4 -o addr show eth0 | awk '{print $4}') -- does SPOT_IP sit in it?" || \
  na "eth0 has no IPv4 (Spot unplugged? fine for desk tests)"
[ -f ~/.spot_login ] && { ok "~/.spot_login exists"; \
  [ "$(stat -c %a ~/.spot_login)" = 600 ] && ok "  mode 600" || no "  mode $(stat -c %a ~/.spot_login), should be 600"; } \
  || no "~/.spot_login missing"
# aliases/functions do NOT reach a non-interactive child shell -- grep the file.
grep -q 'alias tunnel-up=' ~/.bashrc && ok "tunnel-up alias in ~/.bashrc" || no "tunnel-up alias missing from ~/.bashrc"
grep -q 'alias tunnel-up=.*-i .*id_tunnel' ~/.bashrc && ok "  names -i ~/.ssh/id_tunnel" || no "  alias has no -i <key>: autossh will not use id_tunnel"
grep -q 'alias zenoh-pi=' ~/.bashrc && ok "zenoh-pi alias in ~/.bashrc" || no "zenoh-pi alias missing"

echo "== S1 swap =="
sw=$(free -m | awk '/^Swap:/{print $2}')
if [ "${sw:-0}" -ge 3500 ]; then ok "swap ${sw} MB"
elif [ "${sw:-0}" -gt 0 ]; then na "swap only ${sw} MB (T1 asks for 4G; fine unless you rebuild)"
else no "NO SWAP -- colcon will OOM"; fi
grep -q '^/swapfile' /etc/fstab && ok "swap in /etc/fstab (survives reboot)" || no "swap NOT in /etc/fstab -- lost on reboot"

echo "== S2 one-time installs =="
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f "$HOME/ros_ws/install/setup.bash" ] && source "$HOME/ros_ws/install/setup.bash" 2>/dev/null
for p in spot_driver spot_msgs ceai depth_image_proc pointcloud_to_laserscan; do
  ros2 pkg list 2>/dev/null | grep -qx "$p" && ok "pkg $p" || no "pkg $p MISSING"
done
python3 -c "import bosdyn.client" 2>/dev/null && ok "import bosdyn.client" || no "bosdyn.client MISSING"
v=$(zenoh-bridge-ros2dds --version 2>&1 | head -1)
case "$v" in *1.9.0*) ok "zenoh $v";; *GLIBC*) no "zenoh: $v  <- you installed 1.10.x, pin ZV=1.9.0";; *) no "zenoh: ${v:-not installed}";; esac
command -v autossh >/dev/null && ok "autossh installed" || no "autossh missing"
command -v chronyc >/dev/null && ok "chrony installed" || no "chrony missing"
dpkg -l 2>/dev/null | grep -q ros-humble-image-transport-plugins && ok "image-transport-plugins" || no "image-transport-plugins missing (no /compressed)"
ls "$CEAI/config/"zenoh_onboard.json5 >/dev/null 2>&1 && ok "zenoh_onboard.json5 present" || na "zenoh_onboard.json5 missing from $CEAI/config"

echo "== S2 realsense dual-stream patch =="
# The camera only streams depth+colour because realsense2_camera here is a LOCAL
# build of patched 4.58.4 (patches/rs-pi4-dualstream.patch). A reflash, an apt
# install of ros-humble-realsense2-camera, or a fresh clone loses it SILENTLY:
# the node still starts, then fails to open ~28% of the time and hardware-resets
# itself mid-stream. Rebuild with scripts/setup_pi_realsense.sh.
# Resolve the .so the node loader would pick -- first AMENT_PREFIX_PATH hit wins.
rs_so=""
IFS=: read -ra _pp <<< "${AMENT_PREFIX_PATH:-}"
for _p in "${_pp[@]}"; do
  [ -f "$_p/lib/librealsense2_camera.so" ] && { rs_so="$_p/lib/librealsense2_camera.so"; break; }
done
if [ -z "$rs_so" ]; then
  no "librealsense2_camera.so not on AMENT_PREFIX_PATH -- realsense2_camera not built"
else
  na "node would load: $rs_so"
  # "Ignoring hardware notification" is the ROS_WARN_STREAM the patch adds; it is
  # absent from vanilla 4.58.4. -a is required, the .so is a binary and plain
  # grep reports no match on it rather than searching.
  if grep -aqF 'Ignoring hardware notification' "$rs_so"; then
    ok "patched build (dual-stream fix compiled in)"
  else
    no "UNPATCHED realsense2_camera -- depth+colour WILL fail; run scripts/setup_pi_realsense.sh"
  fi
  case "$rs_so" in
    "$HOME/ros_ws/install/"*) ok "  from ~/ros_ws/install (local build wins)" ;;
    *) no "  from $rs_so, not ~/ros_ws/install -- a system package is shadowing the patched build" ;;
  esac
fi
dpkg -l ros-humble-realsense2-camera 2>/dev/null | grep -q '^ii' \
  && na "apt ros-humble-realsense2-camera installed -- keep ~/ros_ws/install ahead of /opt/ros/humble" \
  || ok "no apt realsense2-camera to shadow the local build"

echo "== S2 key to the workstation =="
[ -f ~/.ssh/id_tunnel ] && ok "~/.ssh/id_tunnel exists" || na "no id_tunnel (using another key?)"
if [ -f ~/.ssh/id_tunnel ]; then
  ssh-keygen -y -P "" -f ~/.ssh/id_tunnel >/dev/null 2>&1 && ok "  passphrase-less (autossh can use it)" \
    || no "  HAS A PASSPHRASE -- autossh -f cannot unlock it"
fi
# MUST name the key: id_tunnel is not a filename ssh tries by default.
if [ -z "$OFFBOARD_SSH" ]; then
  na "OFFBOARD_SSH unset -- skipping the tunnel login check (OFFBOARD_SSH=user@host $0)"
elif timeout 12 ssh -i ~/.ssh/id_tunnel -o BatchMode=yes -o IdentitiesOnly=yes -o ConnectTimeout=8 "$OFFBOARD_SSH" true 2>/dev/null; then
  ok "passwordless ssh $OFFBOARD_SSH with id_tunnel"
else
  no "id_tunnel NOT authorised on $OFFBOARD_SSH -- run: ssh-copy-id -i ~/.ssh/id_tunnel.pub $OFFBOARD_SSH"
fi

echo "== clocks =="
chronyc tracking 2>/dev/null | grep -i 'System time' || na "chronyc unavailable"
