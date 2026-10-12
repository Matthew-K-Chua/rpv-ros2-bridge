# rpv-ros2-bridge

A robot-agnostic ROS 2 bridge for deploying Room-Mediated Co-occurrence (RPV),
a zero-shot Object-Goal Navigation (ObjectNav) policy, onto physical ROS 2-compatible robots. We
use this package to deploy RPV onto a Boston Dynamics Spot and a Clearpath
Robotics TurtleBot 4, and it is the deployment framework reported in *Sim-to-Real
Room-Mediated Object-Goal Navigation with ROS 2 and Nav2*.

Rather than programming against a proprietary SDK, we rely on ROS 2 and standard
navigation interfaces (laser scan, SLAM Toolbox, Nav2, velocity commands), which
confines the robot-specific software to a driver and a short profile of unique
frames, topics, and footprints. The end goal is to enable physical evaluation of
ObjectNav policies on a wider range of affordable and attainable hardware.

The policy itself is a separate repository,
[rpv-sim2real-policy](https://github.com/Matthew-K-Chua/rpv-sim2real-policy),
which hosts the perception stack and frontier scoring behind an HTTP interface.
This package is everything on the ROS side including: the bridge node that feeds the
policy and hands its chosen frontier to Nav2, the launch files for the on-board
module and the off-board host, the launcher scripts, and an operator console.

## Contents

- [System overview](#system-overview)
- [Repository layout](#repository-layout)
- [Hardware](#hardware)
- [Installation](#installation)
- [Running an episode](#running-an-episode)
- [Configuration](#configuration)
- [Reproducing the reported configuration](#reproducing-the-reported-configuration)
- [The policy interface](#the-policy-interface)
- [Safety](#safety)
- [Troubleshooting](#troubleshooting)
- [Citation and licence](#citation-and-licence)

## System overview

We split compute between an off-board GPU host and a low-cost on-board sensing
module, which can be moved between robotic platforms and so enables repeatable
testing on an identical sensing setup. In exchange for this democratisation, the
system depends on an additional network layer to relay communication between the
two.

At runtime the on-board module runs the robot's ROS 2 driver and the camera
pipeline, projecting a raw 2.5D depth stream into a 2D laser scan for mapping
and forwarding the depth-aligned RGB-D frames the policy requires. On the host,
SLAM and Nav2 build an occupancy map and plan motion, and the bridge node sends
a snapshot of the occupancy map, the robot's pose, and the RGB-D observation to
the policy over HTTP. The policy's selected frontier is returned to Nav2 as the
next navigation goal, and the resulting velocity commands are relayed back to
the robot through the on-board module.

### The episode

On start the bridge waits for the RGB frame, the aligned depth, `/map`, and the
`map -> base` transform, then turns the robot in place through twelve 30 degree
steps so that the policy's maps are initialised from a full sweep. It then
loops in "explore" mode, gathering the observation, POSTing it, publishing the returned frontier as
`/goal_pose`, and waiting for Nav2. An episode ends when one of the following
occurs:

1. **Target arrival.** The policy reports the target detected and within the
   stopping radius, and the bridge halts the control loop. On Spot, the Pi-side
   `finish_listener` then sits the robot.
2. **Timeout.** The step budget `max_steps` is reached without the target being
   detected, after which the bridge ends the run and sits the robot.
3. **Frontier exhaustion.** The policy returns no further frontier.
4. **Manual intervention.** An operator stops the bridge or issues an emergency
   stop from the tablet.

**Restarting the bridge is the episode reset.** The first request of a run
carries `is_first=true`, which clears the policy's maps and re-pins the episodic
origin at the robot's current pose.

### Robot-agnostic deployment

Every value that differs between platforms is confined to one module per robot
under `ceai/robots/`: the base and odometry frame names, the topic names, the
footprint polygon, the agent radius, the stopping standoff, the camera mount
pose, and the scan height band. The bridge, both launch files, and the console
read the profile selected by `robot:=`, so adding a platform is one module
there, one line in `robots/__init__.py`, and a pair of parameter files.

The bridge and both launch files will refuse to start without an
explicit `robot:=`.

## Repository layout

```
ceai/
  rpv_bridge.py              the ROS node: parameters, subscriptions, publishers, callbacks
  bridge/                    the work, one module per responsibility (config, sensors, frames,
                             occupancy, curtain, depth, payload, policy, goals, spin, loop, finish)
  robots/                    the only per-robot code: base.py (the dataclass), spot.py, turtlebot4.py
  cameras.py                 what each camera offers, and the depth band shared by /scan, SLAM, Nav2 and the policy
  mount.py                   the camera mount pose
  finish_listener.py         module-side node that sits Spot when the bridge announces the finish
launch/
  robot_onboard.launch.py    the module: camera, scan chain, robot driver
  rgbd_camera.launch.py      the D435 as RGB-D (included by the above)
  nav_stack.launch.py        the host: SLAM Toolbox and Nav2
  includes/                  spot_driver.include.py (safety arguments pinned), turtlebot4.include.py
config/
  <ROBOT>_bridge.yaml        bridge parameters per robot (spot/turtlebot4)
  nav2_<ROBOT>.yaml          Nav2 parameters per robot (spot/turtlebot4)
  spot_driver.yaml           the Spot driver, with the E-stop and the auto-* settings pinned off
  zenoh_onboard.json5        zenoh bridge, module side (connects out)
  zenoh_offboard.json5       zenoh bridge, host side (listens)
  fastdds_pi_no_shm.xml      Fast DDS profile for the module (UDPv4 only, interface whitelist)
  spot.rviz                  an RViz layout
gui/                         the operator console: python3 -m gui.rpv_console
scripts/
  launch_offboard.sh         the whole host side in one tmux session; calls the policy repository's launcher
  launch_onboard.sh          the whole module side in one tmux session; `stop` closes the camera cleanly
  check_offboard.sh          one-time setup verification, host
  check_onboard.sh           one-time setup verification, module, including the patched camera driver
  setup_pi_realsense.sh      clone, patch, and build realsense-ros on the module
patches/rs-pi4-dualstream.patch   our realsense-ros patch (see Installation)
config.yaml.example          template for Spot credentials; the real file is git-ignored
package.xml, setup.py        colcon packaging, which is optional
```

## Hardware

| Component | What we use | Role |
|---|---|---|
| Off-board GPU host | Dell Precision 5820, RTX 3090, 24 GB VRAM | SLAM, Nav2, the bridge, the console, and the policy and perception servers |
| On-board sensing module | Raspberry Pi 4 Model B, RealSense D435, portable battery bank | The robot driver, the camera, and the depth-to-scan chain |
| Robot | Boston Dynamics Spot, or Clearpath Robotics TurtleBot 4 | Locomotion |

The module mounts on the platform and connects to it by Ethernet. The module
communicates with the host over WiFi, and the host never addresses the robot
directly.

The module connects *out* to the host: an `autossh` tunnel forwards port 7447
with `zenoh-bridge-ros2dds` inside it. The host's side therefore must be started
first. Two consequences of this transport look like faults but are not:

- **Services do not cross the bridge.** Spot's `/claim`, `/power_on`, `/stand`,
  `/sit`, `/power_off`, and `/release` must be called from a module shell.
- **`ROS_DISTRO` is required.** Left unset, the zenoh bridge assumes a newer
  distribution, cannot parse Humble's discovery messages, and every service call
  across it fails with `sequence size exceeds remaining buffer`, while topics
  continue to work. The topics working is what conceals the cause.

## Installation

### Off-board GPU host

ROS 2 Humble on Ubuntu 22.04. The package runs from the checkout; `colcon build`
is optional and only adds the `ros2 launch ceai ...` and `ros2 run ceai ...`
spellings.

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-rviz2 tmux chrony

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Matthew-K-Chua/rpv-ros2-bridge.git
cd ~/ros2_ws && colcon build --packages-select ceai && source install/setup.bash   # optional
```

Install the policy half by following the README of
[rpv-sim2real-policy](https://github.com/Matthew-K-Chua/rpv-sim2real-policy). The
launcher and the console expect that checkout at `~/rpv-sim2real-policy`; export
`RPV_POLICY_REPO=/path/to/it` or set it in the console's Settings otherwise.

The console requires a Qt binding, for which PyQt5 ships with ROS Humble, plus
`numpy` and `tmux`. `PyAV` (`pip install av`) is optional and produces H.264 run
videos rather than Motion JPEG. We do not install `opencv-python` into the ROS
environment, as its wheel hijacks PyQt5's plugin path such that the console
aborts before its window exists; the ROS environment also requires `numpy<2`, or
`cv_bridge` breaks.

Install `zenoh-bridge-ros2dds` **v1.9.0** from the
[zenoh-plugin-ros2dds releases](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases).
Later releases are built against glibc 2.38 and will not start on Ubuntu 22.04,
which ships 2.35.

We keep ROS opt-in on this machine, as a conda Python that inherits a sourced
ROS environment imports ROS's `numpy` and `cv2` and fails in a manner
indistinguishable from a broken installation:

```bash
rosenv() {
    source /opt/ros/humble/setup.bash
    [ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
    export ROS_DOMAIN_ID=0
    unset ROS_LOCALHOST_ONLY
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
}
export NO_PROXY=127.0.0.1,localhost,::1     # a configured http_proxy otherwise consumes the loopback POST
export no_proxy=$NO_PROXY
```

`scripts/check_offboard.sh` verifies the above.

### On-board sensing module

ROS 2 Humble on Ubuntu 22.04 (64-bit) flashed onto a Raspberry Pi 4 ModuleB, in the workspace `~/ros_ws`.

```bash
sudo apt install ros-humble-depth-image-proc ros-humble-pointcloud-to-laserscan \
                 ros-humble-image-transport-plugins ros-humble-librealsense2 \
                 autossh chrony tmux
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/Matthew-K-Chua/rpv-ros2-bridge.git
```

**The patched camera driver is mandatory for our setup.** Unpatched, `realsense2_camera`
fails to open the D435's second stream on approximately 28 % of cold starts on
this Pi, and hardware-resets itself mid-stream. Both failures present as faulty hardware or a faulty cable. We
vendor the patch at `patches/rs-pi4-dualstream.patch`, which applies to
realsense-ros tag `4.58.4`. The setup script clones that tree, verifies the
commit, applies the patch, builds, and confirms that the patched log string is
present in the resulting library:

```bash
~/ros_ws/src/rpv-ros2-bridge/scripts/setup_pi_realsense.sh
~/ros_ws/src/rpv-ros2-bridge/scripts/check_onboard.sh      # "patched realsense2_camera" must PASS
```

The patched build exists only in `~/ros_ws/install` and is lost by a
reflash, or by `apt install ros-humble-realsense2-camera` landing ahead of it on
`AMENT_PREFIX_PATH`. We re-run the check after either.

For Spot, install the [spot_ros2](https://github.com/bdaiinstitute/spot_ros2)
driver into the same workspace (`install_spot_ros2.sh --arm64`) and keep the
credentials outside the repository:

```bash
# ~/.spot_login, mode 600; sourced by launch_onboard.sh before the driver starts
export SPOT_IP=<Spot's address on the module's Ethernet segment>
export BOSDYN_CLIENT_USERNAME=<user>
export BOSDYN_CLIENT_PASSWORD=<password>
# alternatively, a YAML in the form of config.yaml.example, located by SPOT_LOGIN_YAML
```

The module's `~/.bashrc` should source ROS and `~/ros_ws/install/setup.bash`, set
`ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and
`FASTRTPS_DEFAULT_PROFILES_FILE=~/ros_ws/src/rpv-ros2-bridge/config/fastdds_pi_no_shm.xml`,
and leave `ROS_LOCALHOST_ONLY` unset. Create a passphrase-less key for the
tunnel (`ssh-copy-id -i ~/.ssh/id_tunnel.pub <user>@<host>`), and point `chrony`
at the host such that the two clocks agree to within 50 ms.

## Running an episode

### From the console

On the host, in a shell with ROS sourced:

```bash
cd ~/ros2_ws/src/rpv-ros2-bridge
python3 -m gui.rpv_console
```

We set the module's host and user, the host's `user@host` as seen from the
module, the module's checkout path, and the policy checkout once in **Settings**;
they are written to `gui/settings.json`, which is git-ignored.

| Control | What it does |
|---|---|
| **Start everything** | Runs `scripts/launch_offboard.sh`, producing one tmux session, `rpv_offboard`, containing zenoh (listening), SLAM and Nav2, CLIP, SAM 3, the policy server, and the bridge. It waits for zenoh's port, then prints the exact command to run in a module shell, with a copy button. |
| **Stop everything** | Kills both sessions, after confirmation. Spot remains standing; we sit it before finishing. |
| **PI light** | `/scan` liveness, which is evidence rather than a session check: green requires the module to be up, the camera streaming, the scan chain's TF resolved, and the tunnel carrying traffic. |
| **WS, AI, BRIDGE, SLAM, CAM lights** | The host session exists; the policy answers on its port; the `bridge` and `ros` windows exist; RGB frames arrived within the last second. The window checks are necessary rather than sufficient, as a command that dies leaves its window behind. |
| **Desired object, then Set** | Recreates the `bridge` window with the new target, which is the episode reset. |
| **Edit list** | Edits the policy repository's `data/shortvis.yaml`, retains a `.bak`, and offers to restart the policy server, which caches the list on its first step. This list is only read when `SKIP_YOLOE=1`, so it has no effect on a run configured as the reported trials were. |
| **Record** | Records a rosbag of the map, goal, TF, odometry, scan, and camera topics, together with the policy's per-step decision log and the value-map and annotated-camera panels as video, under `records/`. Stopping sends Ctrl-C first, as `ros2 bag` writes its metadata on SIGINT and a bag killed outright is unreadable. |
| **RViz** | Opens `rviz2` with the selected profile's layout. |
| **NAV: AUTO / MANUAL** | MANUAL stops the bridge and allows a click on the SLAM map to publish a `/goal_pose`, which we use to reposition between trials. |

The window presents the live camera, the policy's value and obstacle maps and
its annotated camera (served by the policy's `GET /status`), the SLAM map with
the robot's footprint drawn to scale, `/goal_pose`, and `/scan`, and the policy
log. Everything the console starts is an ordinary tmux session, which
`tmux attach -t rpv_offboard` will show.

### By hand

The host's zenoh listens and the module connects out,
and the bridge terminates with a connection error if the policy server is not
yet answering. The host script waits on both ports itself.

```bash
# 1. Host, any shell
TARGET=backpack ~/ros2_ws/src/rpv-ros2-bridge/scripts/launch_offboard.sh
#    ROBOT=turtlebot4 for the TurtleBot 4; DEPTH_SOURCE=zoedepth for a colour-only camera

# 2. Module shell, once the host's `ros` window exists
OFFBOARD_SSH=<user>@<host> ~/ros_ws/src/rpv-ros2-bridge/scripts/launch_onboard.sh
#    windows: tunnel, zenoh, stack (driver, camera, scan chain), lifecycle

```

Both scripts are configured entirely from the environment, and the header of
each documents every variable and its default. The bridge alone, for reference:

```bash
cd ~/ros2_ws/src/rpv-ros2-bridge
python3 -m ceai.rpv_bridge --ros-args \
  --params-file config/spot_bridge.yaml \
  -p robot:=spot -p target:=backpack -p depth_source:=realsense \
  -p policy_url:=http://127.0.0.1:13000/rpv_policy
```

The bridge's log should report, in order:

```
Control loop: waiting for sensors + TF...
Control loop: sensors ready, starting 360 scan.
Spinning 0.524 rad...                     # twelve times
Control loop: scan done, exploring.
mode=explore goal=[...]                   # one line per policy step
mode=navigate ...                         # the target has been detected
Target reached; stopping control loop.
```

### Verification before the robot stands

```bash
ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw --field encoding --once
#   Must report 16UC1. The bridge rejects anything else deliberately, as 32FC1 metres
#   interpreted as uint16 millimetres is a factor-of-1000 error with no symptom.
timeout 20 ros2 topic hz /scan                        # approximately the depth frame rate
ros2 topic echo /scan --once --field header.frame_id  # the robot's base frame
ros2 topic list | grep -E "^/depth/|/camera/camera/depth/"   # empty: the raw raster stayed on the module
chronyc tracking                                      # both machines, offset below 0.05 s
```

With the robot standing still, we also inspect `/scan` in RViz for a ring of
near returns at the leg bearings. The D435 looks outward from the front of the
body and cannot observe Spot's thighs, so such a ring indicates that the mount
pose or the scan height band is wrong.

### The target and the prompt list

The target is free text and open-vocabulary, so `"office chair"` is acceptable.
Synonyms may be supplied `|`-separated, in which case all are prompted to SAM 3
but only the first is matched against a returned mask's label. Changing the
target requires restarting the bridge; the policy and perception servers remain
up.

| `SKIP_YOLOE` | Vocabulary | Behaviour |
|---|---|---|
| `0`, the reported trials | `YOLOE_CLASSES`, which we set to `data/lvis.yaml` (1203 classes) | The full dual perception funnel: YOLOE produces coarse proposals from the LVIS vocabulary, which SAM 3 then refines into instance masks and from which poor detections are removed. Requires `checkpoints/yoloe-26x-seg.pt` |
| `1`, the committed default | `SKIP_YOLOE_CLASSES`, which defaults to `data/shortvis.yaml` (14 labels) | YOLOE is not started at all and SAM 3 is prompted directly with the target and those labels, which frees the detector's VRAM at the cost of recall over the wider vocabulary |

In either case the target itself is always prompted, whether or not it appears
in the list, so adding a new target to a vocabulary file changes nothing about
whether it is detected. What the vocabulary determines is the *other* objects
that are found, and those detections are what the value map is constructed from,
as each is scored by its room co-occurrence with the target. A label therefore
earns its place only if observing it should change where the robot goes, and
every label costs perception time on every step. The policy reads whichever file
applies once, on the first step of a run, so we restart it after editing.

The room lexicon is separate and lives in `data/room_types_lab.yaml` and should be changed as per the physical layout
of an experiment at the researcher's discretion.

### Stopping

```bash
~/ros_ws/src/rpv-ros2-bridge/scripts/launch_onboard.sh stop   # SIGINT, then waits for the camera to close
tmux kill-session -t rpv_offboard                              # host
```

We sit, power off, and release Spot before stopping the module's stack. The
module's `stop` is important as we find sending `kill -9` on a streaming RealSense
node skips the UVC close and leaves the USB 3 link unable to re-enumerate until
the camera is physically replugged. Any `pkill` is
scoped with `pgrep -f` first, as a bare pattern can match the SSH session in use.

## Configuration

### Robot profiles

`ceai/robots/<name>.py` is the only per-robot code.

| | Spot | TurtleBot 4 |
|---|---|---|
| base / odometry frame | `body` / `vision` | `base_link` / `odom` |
| odometry topic | `/odometry` | `/odom` |
| agent radius (inscribed) | 0.25 m | 0.18 m |
| stopping standoff | 1.0 m | 0.4 m |
| scan height band, base-frame z | −0.30 to 0.50 m | 0.08 to 1.00 m |
| camera mount | measured, in the profile | measured, in the profile |
| driver | `spot_driver`, via `launch/includes/spot_driver.include.py` | none; the module addresses the Create 3 over the cable |

On Spot, `spot_name` and `frame_prefix` must both be `""`. Frames appearing as
`Spot/body` indicate that `spot_name` has been set, and no TF lookup in the stack
will resolve.

The six mount numbers (`d435_x` through `d435_yaw`) are required for
navigation, as `/scan` is produced by transforming the camera's cloud into the
base frame.

### Bridge parameters

`config/<robot>_bridge.yaml`, loaded by node name (`nav2_vlfm_bridge`). 

| Parameter | Default | Meaning |
|---|---|---|
| `robot` | none; required | Selects the profile |
| `target` | `backpack` | The object goal, which reaches the policy only on an episode reset |
| `depth_source` | `realsense` | `realsense` (aligned D435 depth), `zoedepth` (monocular), `lidar` (a curtain tiled from `/scan`), or `stub` |
| `policy_url` | `http://127.0.0.1:13000/rpv_policy` | Where the observation is POSTed |
| `init_steps` | 12 | The 30 degree turns in the initial sweep, which must equal the policy's `INIT_SCAN_STEPS` |
| `max_steps` | 500 | Ends the run and sits the robot after this many policy steps, the initial sweep included; `0` runs until the policy calls stop |
| `sit_on_finish` | `true` on Spot | Announces the finish on `/ceai/finish` for the module's `finish_listener` |

### What the host launcher derives

`scripts/launch_offboard.sh` derives the policy's depth band
(`RPV_MIN_DEPTH_M` and `RPV_MAX_DEPTH_M`) from `ceai/cameras.py`, and the camera
tilt (`RPV_CAMERA_PITCH_RAD` and `RPV_CAMERA_ROLL_RAD`) from the selected robot
profile, before passing them with `RPV_ROBOT` and the policy port to the policy
repository's `scripts/launch_offboard_ai.sh`. 

`RPV_MAX_DEPTH_M` is both the [0, 1] normalisation divisor and the sentinel the policy reads as "out of
range". 

### Constants that be changed in more than one file

| Constant | Source of truth | Partners |
|---|---|---|
| `max_range_m`, 10.0 m: how far `/scan` is published | `ceai/cameras.py` | The flattener's `range_min` and `range_max`, and the no-return fill |
| `obstacle_range_m`, 3.0 m: how far a return counts as an object | `ceai/cameras.py` | SLAM Toolbox's `max_laser_range`; Nav2's `obstacle_max_range` and `raytrace_max_range` in both costmaps; the bridge's `max_depth`; `RPV_MAX_DEPTH_M` |
| footprint polygon | `ceai/robots/<robot>.py` | Both Nav2 costmaps |
| base and odometry frames | `ceai/robots/<robot>.py` | SLAM Toolbox, both costmaps, the behaviour server, the bridge |
| `agent_radius_m` | `ceai/robots/<robot>.py` | The policy's per-robot `agent_radius`, which is the obstacle-map dilation and therefore the narrowest gap it will plan through |
| `init_steps`, 12 | The bridge parameters | The policy's `INIT_SCAN_STEPS` |
| camera mount pose | `ceai/robots/<robot>.py` | The module's `base -> camera_link` static TF, and the policy's camera pitch and roll, both derived from the profile by the launchers |

## Reproducing the reported configuration

We ran the 66 reported trials from the console, with the perception stack in its
full two-stage configuration:

```bash
SKIP_YOLOE=0 YOLOE_CLASSES=data/lvis.yaml python3 -m gui.rpv_console
```

The defaults committed here are our current operating settings, which differ in
three respects from that configuration:

| Setting | Committed default | Reported trials |
|---|---|---|
| Perception stage 2 | `SKIP_YOLOE=1`, which does not start YOLOE and prompts SAM 3 directly with the 14 labels in `data/shortvis.yaml` | `SKIP_YOLOE=0` with `YOLOE_CLASSES=data/lvis.yaml`, which runs YOLOE over the 1203-class LVIS vocabulary and passes its proposals to SAM 3. Requires `checkpoints/yoloe-26x-seg.pt` |
| Step budget | `max_steps: 500` | 250; pass `-p max_steps:=250` |
| Stopping criterion | `stop_standoff_m: 1.0`, measured from the body centre | A success distance of 1.4 m from the target |

The settings that are already aligned, and that should not be altered when
reproducing, are Mask2Former disabled (`SKIP_MASK2FORMER=1`), YOLOE at a
confidence of 0.3 and SAM 3 at 0.35, which are the code defaults and need no
override, perception on every step (`STEPS_BETWEEN_DETECTIONS=1`), a propagation
cap of 6.0 m (`MAX_PROPAGATION_M`), a depth band of 0.3 to 3.0 m, and the
nine-term room lexicon in `data/room_types_lab.yaml`. That lexicon is specific
to our university environment and replaces the policy's published household
rooms, so results produced with it are not directly comparable to the simulated
benchmark.

## The policy interface

The bridge POSTs the following to `/rpv_policy` as JSON.

| Field | Meaning |
|---|---|
| `rgb`, `rgb_order` | Base64 JPEG, forwarded as the camera's own bytes, and its channel order; `"bgr"` from the live bridge |
| `depth_png16` | Base64 16-bit PNG of millimetres, where 0 denotes invalid, carrying its own shape. The encoder is imported from the policy repository's `vlfm/vlm/depth_codec.py` such that the encoder and decoder cannot drift |
| `depth_row`, `depth_shape` | One row of the `/scan` curtain, tiled to the image by the policy (`depth_source=lidar`) |
| `occ_hash`; `occupancy`, `occ_shape`, `origin_x/y`, `resolution` | A hash of the Nav2 grid, and the grid itself only when that hash is new, so an unchanged map travels as 32 characters |
| `robot_map_x/y/yaw`, `start_x/y/yaw` | The live pose and the pinned episodic origin, in the map frame |
| `fx`, `fy`, `img_w`, `camera_height`, `camera_pitch`, `camera_roll` | Intrinsics and extrinsics, where positive pitch places the lens down |
| `min_depth`, `max_depth` | Advisory only; the policy's own band is authoritative and a disagreement is logged once |
| `objectgoal`, `is_first`, `finish_init` | The target; the episode reset; and whether the initial sweep has completed |

The response is `{mode, goal_map_xy, goal_yaw, called_stop, done_initializing}`,
or `{occ_cache_miss: true}` when the policy requires the full grid rather than
the hash.

## Safety

- **Never set `start_estop: True`** in `config/spot_driver.yaml`. A driver that
  takes the E-stop evicts the tablet, and the physical stop button then silently
  ceases to function. `auto_claim`, `auto_power_on`, and `auto_stand` remain
  `False`, and `cmd_duration` remains `0.25`, which is the interval after which
  Spot's watchdog halts the robot once `cmd_vel` ceases. We take "Acquire Cut
  Motor Power Authority" on the tablet before starting the driver; the driver
  reporting `Waiting for estop to be released...` and then `Found estop!` is the
  intended behaviour, as it means someone else holds the stop button.
- **Stop the module with `launch_onboard.sh stop`**, and never `kill -9` the
  camera.
- `config.yaml` is git-ignored and holds real Spot credentials.

## Troubleshooting

| Symptom | Cause | Resolution |
|---|---|---|
| `No module named 'rclpy._rclpy_pybind11'` | The shell is in the conda environment | A new shell with ROS sourced |
| Service calls fail with `sequence size exceeds remaining buffer` | `ROS_DISTRO` was unset when the zenoh bridge started | Restart it with `ROS_DISTRO=humble`; topics work regardless, which is what conceals this |
| `/claim` and the other lifecycle services are absent on the host | Services do not cross the zenoh bridge | Call them from a module shell |
| Topics present on the module but not on the host | The tunnel or zenoh | Confirm `127.0.0.1:7447` is listening on the module, and that the host's side started first |
| Frames appear as `Spot/body` | `spot_name` is set | Both it and `frame_prefix` must be `""` |
| The bridge remains at `waiting for sensors + TF` | A required input is absent | Check the RGB topic, `/map`, and `camera_info` with `ros2 topic hz`, then `ros2 run tf2_ros tf2_echo map body` |
| The bridge raises `ModuleNotFoundError` naming `vlfm` | The policy checkout was not found | `export RPV_POLICY_REPO=/path/to/rpv-sim2real-policy` |
| Repeated `Stale RGB frame` warnings | Clock skew rather than latency | `chronyc tracking` on both machines |
| `/scan` never appears and nothing errors | No `base -> camera_link` transform | Supply the six mount numbers, in the profile or as all six arguments |
| The D435 fails to start in roughly one start in four, or degrades mid-stream | The unpatched camera driver | `check_onboard.sh`, then `setup_pi_realsense.sh` |
| Depth arrives but every step is skipped | The encoding or the topic | The encoding must be `16UC1`, and the topic must be `aligned_depth_to_color` rather than `image_rect_raw` |
| A flat wall across the front of `/scan` that moves with stand height | The floor, lifted into the scan band by a mount pitch that is too shallow | Re-measure the mount pitch |
| A detected object is placed incorrectly on the map | The mount numbers, or monocular depth scale | Re-measure the mount; with ZoeDepth, inspect `/zoedepth/diagnostics` |
| Spot sits mid-run and `cmd_vel` falls silent | The lease was lost, as someone touched the tablet | Re-claim and re-stand |
| The run finished and `/sit` reported success, yet Spot is standing | `/sit` is non-blocking: the driver returns once Spot accepts the command, and a later `cmd_vel` overrides it | `finish_listener` re-calls `/sit` until the driver reports sitting |
| Nav2 refuses to plan in a narrow passage | The footprint against the inflation radius | We observed this on the TurtleBot 4 in 180 degree turns with as little as 20 mm of clearance; it is a property of the space rather than a fault |

## Citation and licence

The policy this package deploys is Room-Mediated Co-occurrence (Scicluna et al.,
2026), which extends Vision-Language Frontier Maps (Yokoyama et al., 2024).

```bibtex
@misc{scicluna2026roommediatedcooccurrencezeroshotobjectcentric,
      title={Room-Mediated Co-occurrence for Zero-Shot Object-Centric Semantic Navigation via Frontier Scoring}, 
      author={Adam Scicluna and Gavin Paul and Alen Alempijevic},
      year={2026},
      eprint={2607.25448},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2607.25448}, 
}
```

```bibtex
@inproceedings{yokoyama2024vlfm,
  title={VLFM: Vision-Language Frontier Maps for Zero-Shot Semantic Navigation},
  author={Naoki Yokoyama and Sehoon Ha and Dhruv Batra and Jiuguang Wang and Bernadette Bucher},
  booktitle={International Conference on Robotics and Automation (ICRA)},
  year={2024},
}
```

Released under the MIT licence; see `LICENSE`. The vendored patch applies to
[realsense-ros](https://github.com/IntelRealSense/realsense-ros) and is
distributed under that project's terms.
