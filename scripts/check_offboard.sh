#!/usr/bin/env bash
# Verify the workstation's one-time setup. Read-only.
ok(){ printf '  \033[32mPASS\033[0m %s\n' "$1"; }
no(){ printf '  \033[31mFAIL\033[0m %s\n' "$1"; }
na(){ printf '  \033[33m--  \033[0m %s\n' "$1"; }

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CEAI=${CEAI_DIR:-$(dirname "$SCRIPT_DIR")}

echo "== S1 bashrc =="
[ -z "${ROS_DISTRO:-}" ] && ok "no ROS in a fresh shell (conda terminals are clean)" \
  || no "ROS_DISTRO=$ROS_DISTRO in a fresh shell -- conda will inherit it and break"
# functions/aliases do NOT reach a non-interactive child shell -- grep the file.
grep -q 'rosenv()' ~/.bashrc && ok "rosenv() in ~/.bashrc" || no "rosenv() missing from ~/.bashrc"
[ -n "${NO_PROXY:-}" ] && ok "NO_PROXY=$NO_PROXY" || no "NO_PROXY unset -- a proxy can eat the :13000 POST"
[ -n "${CONDA_ENV:-}" ] && ok "CONDA_ENV=$CONDA_ENV" || no "CONDA_ENV unset"
# NOT a failure since 2026-09-16: launch_offboard.sh derives it from ceai/cameras.py.
# A value in the environment now OVERRIDES that, which is checked against the
# camera further down.
[ -n "${RPV_MAX_DEPTH_M:-}" ] \
  && na "RPV_MAX_DEPTH_M=$RPV_MAX_DEPTH_M in the environment -- this OVERRIDES the derived value" \
  || ok "RPV_MAX_DEPTH_M unset (launch_offboard.sh derives it -- the right default)"
for a in ai-up ai-down zoe-up zenoh-dt; do
  grep -q "alias $a=" ~/.bashrc && ok "alias $a in ~/.bashrc" || no "alias $a missing from ~/.bashrc"; done

echo "== conda =="
if command -v conda >/dev/null || [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
  . "$HOME/miniconda3/etc/profile.d/conda.sh" 2>/dev/null
  conda env list 2>/dev/null | grep -v '^#' | awk '$1!=""{print "       env: "$1}'
  conda env list 2>/dev/null | grep -qE "^${CONDA_ENV:-__none__}\s" && ok "CONDA_ENV '$CONDA_ENV' exists" \
    || no "CONDA_ENV '$CONDA_ENV' NOT in the list above -- fix it in ~/.bashrc"
else no "conda not found at ~/miniconda3"; fi

echo "== S2 one-time =="
v=$(zenoh-bridge-ros2dds --version 2>&1 | head -1)
case "$v" in *1.9.0*) ok "zenoh $v";; *GLIBC*) no "zenoh: $v  <- 1.10.x on Ubuntu 22.04, pin ZV=1.9.0";; *) no "zenoh: ${v:-not installed}";; esac
[ -f "$CEAI/config/zenoh_offboard.json5" ] && ok "zenoh_offboard.json5 present" \
  || na "zenoh_offboard.json5 missing from $CEAI/config"
command -v chronyc >/dev/null && chronyc tracking 2>/dev/null | grep -i 'System time' || na "chrony not answering"
grep -qE '^allow' /etc/chrony/chrony.conf 2>/dev/null && ok "chrony 'allow' present (Pi can sync from here)" \
  || no "no 'allow' in chrony.conf -- the Pi cannot use this as a time server"
zoe_weights_cached() {
python3 - <<'PY' 2>/dev/null || echo "  --   could not check HF cache"
import os,glob
h=os.path.expanduser("~/.cache/huggingface")
hit=glob.glob(h+"/**/*zoedepth*",recursive=True)
print("  PASS zoedepth weights cached" if hit else "  FAIL zoedepth weights NOT cached (T6 download)")
PY
}

# The depth band is the single most dangerous number in the tree: it is BOTH
# the [0,1] normalisation divisor AND the "out of range" sentinel, so a wrong
# value silently DISCARDS detections instead of erroring. It used to be
# defaulted 2.0 in one script and 8.0 in another; ceai/cameras.py is the one
# source now and launch_offboard.sh derives it. This checks nothing overrode it.
ds=$(grep -oP 'depth_source:\s*"?\K[a-z]+' "$CEAI/config/spot_bridge.yaml" 2>/dev/null)
case "$ds" in
  realsense)
    ok "spot_bridge.yaml depth_source: realsense (the D435's aligned depth)"
    band=$(python3 -c "import sys; sys.path.insert(0,'$CEAI'); from ceai.cameras import load; c=load('${CAMERA_MODEL:-d435}'); print(c.min_range_m, c.obstacle_range_m)" 2>/dev/null)
    if [ -z "$band" ]; then
      no "  cannot read the camera band from $CEAI/ceai/cameras.py"
    else
      ok "  policy band: $band m (ceai/cameras.py min_range_m / obstacle_range_m)"
      want=${band#* }
      if [ -z "${RPV_MAX_DEPTH_M:-}" ]; then
        na "  RPV_MAX_DEPTH_M unset -- launch_offboard.sh will derive $want"
      elif [ "$RPV_MAX_DEPTH_M" = "$want" ]; then
        ok "  RPV_MAX_DEPTH_M=$RPV_MAX_DEPTH_M matches the camera"
      else
        no "  MISMATCH: RPV_MAX_DEPTH_M=$RPV_MAX_DEPTH_M but the camera band ends at $want -- detections past it are SILENTLY discarded"
      fi
    fi ;;
  zoedepth)
    ok "spot_bridge.yaml depth_source: zoedepth (the monocular fallback)"
    if [ "${RPV_MAX_DEPTH_M:-8.0}" = 8.0 ]; then
      ok "  RPV_MAX_DEPTH_M=${RPV_MAX_DEPTH_M:-8.0} matches ZoeDepth's trust_max_m"
    else
      no "  MISMATCH: ZoeDepth's trust_max_m is 8.0, you have $RPV_MAX_DEPTH_M"
    fi
    zoe_weights_cached ;;
  lidar|stub)
    na "spot_bridge.yaml depth_source: $ds (a diagnostic path, not a run configuration)" ;;
  *)
    no "spot_bridge.yaml has no readable depth_source" ;;
esac

echo "== git (yesterday's fix) =="
git config --global --get user.email >/dev/null && ok "git identity: $(git config --global --get user.name) <$(git config --global --get user.email)>" \
  || no "no global git identity -- commits will be refused"
git config --global --get-regexp 'url\..*insteadOf' >/dev/null 2>&1 && ok "insteadOf rewrite present" \
  || no "no HTTPS->SSH rewrite -- pushes will ask for a password"
ssh -o BatchMode=yes -o ConnectTimeout=8 -T git@github.com 2>&1 | grep -q 'successfully authenticated' \
  && ok "github ssh auth works" || no "github ssh auth failing (ssh-add ~/.ssh/id_ed25519?)"
