#!/usr/bin/env bash
# Rebuild the patched realsense2_camera on the Pi: clone 4.58.4, apply
# patches/rs-pi4-dualstream.patch, build, verify the marker is in the .so.
#
# Needed after a reflash, a fresh Pi, or anything that replaced the local build
# (see the README). Safe to re-run: it refuses to touch an existing
# checkout unless you pass --force.
#
#   ./scripts/setup_pi_realsense.sh              # clone, patch, build, verify
#   ./scripts/setup_pi_realsense.sh --force      # re-clone over an existing tree
#   ./scripts/setup_pi_realsense.sh --verify-only
#
# Env: ROS_WS (default ~/ros_ws), ROS_DISTRO (default humble)
set -euo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PATCH="$REPO_ROOT/patches/rs-pi4-dualstream.patch"

WS="${ROS_WS:-$HOME/ros_ws}"
SRC="$WS/src/realsense-ros"
DISTRO="${ROS_DISTRO:-humble}"

UPSTREAM="https://github.com/IntelRealSense/realsense-ros.git"
# Tag "4.58.4", NOT "v4.58.4" -- both exist and they are different commits.
TAG="4.58.4"
COMMIT="9a11121700cb4780e273e34141f6402fe184321d"
# ROS_WARN_STREAM text added by the ros_sensor.cpp @@ -49 hunk. Absent from
# vanilla 4.58.4, so its presence in the .so proves the patch is compiled in.
MARKER="Ignoring hardware notification"

ok(){   printf '  \033[32mPASS\033[0m %s\n' "$1"; }
no(){   printf '  \033[31mFAIL\033[0m %s\n' "$1"; }
na(){   printf '  \033[33m--  \033[0m %s\n' "$1"; }
step(){ printf '\n\033[1m== %s ==\033[0m\n' "$1"; }
die(){  no "$1"; exit 1; }

# ROS setup.bash reads unset variables, which is fatal under `set -u` -- and
# `|| true` does NOT catch it, the shell just exits 127. Drop -u/-e across it.
ros_source(){
  [ -f "$1" ] || return 0
  set +eu
  # shellcheck disable=SC1090
  . "$1"
  set -eu
}

FORCE=0
VERIFY_ONLY=0
for a in "$@"; do
  case "$a" in
    --force)       FORCE=1 ;;
    --verify-only) VERIFY_ONLY=1 ;;
    -h|--help)     sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)             die "unknown argument: $a" ;;
  esac
done

# Resolve the library the node loader would actually pick: first hit along
# AMENT_PREFIX_PATH wins, so an apt package ahead of the workspace shadows it.
resolve_so(){
  local p
  IFS=: read -ra _pp <<< "${AMENT_PREFIX_PATH:-}"
  for p in "${_pp[@]}"; do
    if [ -f "$p/lib/librealsense2_camera.so" ]; then
      printf '%s\n' "$p/lib/librealsense2_camera.so"
      return 0
    fi
  done
  return 1
}

verify(){
  step "verify"
  local rc=0 so
  ros_source "/opt/ros/$DISTRO/setup.bash"
  ros_source "$WS/install/setup.bash"

  if ! so=$(resolve_so); then
    no "librealsense2_camera.so not found on AMENT_PREFIX_PATH -- build did not install"
    return 1
  fi
  na "node would load: $so"

  # -a is required: the .so is binary, and plain grep reports no match on it.
  if grep -aqF "$MARKER" "$so"; then
    ok "patched build -- marker \"$MARKER\" present"
  else
    no "UNPATCHED build -- marker absent; this is vanilla or apt realsense2_camera"
    rc=1
  fi

  case "$so" in
    "$WS/install/"*) ok "loading from $WS/install (local build wins)" ;;
    *) no "loading from $so, not $WS/install -- an apt/system package is shadowing it"; rc=1 ;;
  esac

  if dpkg -l ros-humble-realsense2-camera 2>/dev/null | grep -q '^ii'; then
    na "apt ros-humble-realsense2-camera is installed; keep $WS/install ahead of /opt/ros/$DISTRO"
  fi

  [ $rc -eq 0 ] && ok "realsense dual-stream patch is live" || no "run without --verify-only to rebuild"
  return $rc
}

if [ "$VERIFY_ONLY" -eq 1 ]; then
  verify
  exit $?
fi

step "preflight"
[ -f "$PATCH" ] || die "patch not found: $PATCH"
ok "patch: $PATCH"
command -v git     >/dev/null || die "git not installed"
command -v colcon  >/dev/null || die "colcon not installed (apt install python3-colcon-common-extensions)"
[ -d "/opt/ros/$DISTRO" ] || die "/opt/ros/$DISTRO not found"
ok "ROS 2 $DISTRO"

# realsense2_camera links against apt librealsense2; without it cmake fails late.
if dpkg -l "ros-$DISTRO-librealsense2" 2>/dev/null | grep -q '^ii'; then
  ok "ros-$DISTRO-librealsense2 $(dpkg-query -W -f='${Version}' "ros-$DISTRO-librealsense2" 2>/dev/null)"
else
  die "ros-$DISTRO-librealsense2 missing -- apt install ros-$DISTRO-librealsense2"
fi

# 4 GB Pi: colcon OOMs partway through realsense2_camera without swap.
sw=$(free -m | awk '/^Swap:/{print $2}')
if [ "${sw:-0}" -ge 3500 ]; then ok "swap ${sw} MB"
elif [ "${sw:-0}" -gt 0 ]; then na "swap only ${sw} MB -- 4 GB recommended, this build may OOM"
else die "NO SWAP -- this build will OOM on a 4 GB Pi. Add a 4 G swapfile first."; fi

step "checkout $TAG"
mkdir -p "$WS/src"
if [ -d "$SRC" ]; then
  if [ "$FORCE" -eq 1 ]; then
    na "removing existing $SRC (--force)"
    rm -rf "$SRC"
  else
    die "$SRC already exists. Re-run with --force to re-clone, or --verify-only to just check."
  fi
fi
git clone --branch "$TAG" --depth 1 "$UPSTREAM" "$SRC"
have=$(git -C "$SRC" rev-parse HEAD)
[ "$have" = "$COMMIT" ] || die "expected $COMMIT at tag $TAG, got $have -- upstream tag moved, re-check the pinned commit in this script"
ok "at $COMMIT (tag $TAG)"

step "apply patch"
git -C "$SRC" apply --check "$PATCH" || die "patch does not apply to this tree"
ok "applies cleanly"
git -C "$SRC" apply "$PATCH"
ok "applied: $(git -C "$SRC" diff --name-only | tr '\n' ' ')"

step "build"
ros_source "/opt/ros/$DISTRO/setup.bash"
( cd "$WS" && colcon build --symlink-install --packages-up-to realsense2_camera ) \
  || die "colcon build failed -- see $WS/log/latest_build"
ok "built"

verify
