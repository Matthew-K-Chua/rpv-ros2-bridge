"""
Nav2 <-> NumPy helper functions
VLFM uses NumPy, so translator functions help communicate info across the two systems
"""
import base64
import math
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, Quaternion

#TODO - remove all unused functions
def occupancy_to_bool(msg):
    """OccupancyGrid -> (H, W) bool. occupied(100) -> True; free(0)/unknown(-1) -> False."""

    # Convert the ros2 msg into an array, then reshape it to have (height) rows, and (width) columns
    grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

    # Then mask the array so that any cell with 100 is marked as 1, else 0
    return grid == 100

def resample_to_vlfm(occ_bool, src_resolution, pixels_per_meter=20):
    """Rescale a bool grid from src_resolution m/cell to VLFM's 1/ppm m/cell (NEAREST only)."""

    vlfm_cell = 1.0 / pixels_per_meter            # 0.05 m
    if abs(src_resolution - vlfm_cell) < 1e-9:
        return occ_bool                           # already 5 cm, nothing to do
    scale = src_resolution / vlfm_cell                    # 0.03 / 0.05 = 0.6
    h, w = occ_bool.shape
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    resized = cv2.resize(
        occ_bool.astype(np.uint8), (new_w, new_h),   # cv2 wants (width, height)
        interpolation=cv2.INTER_NEAREST,             # NEVER interpolate occupancy
    )
    return resized.astype(bool)

def occupied_world_xy(occ_bool, origin_x, origin_y, cell_size):
    """Occupied cell centres -> (N, 2) map-frame xy. cell_size is 1/ppm after resample."""

    # reframes the coordinates so that they're in the middle of each cell, and also transforms them around the correct origin in vlfm which is 500,500
    rows, cols = np.nonzero(occ_bool)
    world_x = origin_x + (cols + 0.5) * cell_size
    world_y = origin_y + (rows + 0.5) * cell_size
    return np.column_stack([world_x, world_y])

def write_to_canvas(world_xy, obstacle_map, T_map_to_epi, logger=None):
    """map-frame xy -> episodic -> VLFM pixels -> crop to canvas -> write _map True."""
    if len(world_xy) == 0:
        return
    epi_xy = T_map_to_epi(world_xy)                 # map frame  -> episodic frame
    px = obstacle_map._xy_to_px(epi_xy)            # episodic xy -> VLFM pixels

    size = obstacle_map.size

    #TODO How does this work?
    keep = (px[:, 0] >= 0) & (px[:, 0] < size) & \
           (px[:, 1] >= 0) & (px[:, 1] < size)
    dropped = int((~keep).sum())

    if dropped and logger is not None:
        logger.warning(f"{dropped} obstacle cells fell outside the 50 m canvas; cropped.")

    px = px[keep]
    obstacle_map._map[px[:, 1], px[:, 0]] = True    # matches VLFM obstacle_map.py:101

# ===
# map <-> episodic rigid transform
# ===
def make_T_map_to_epi(start_x, start_y, start_yaw):
    """Returns (forward, inverse) callables, each mapping (N,2) -> (N,2).

    Tested handedness 05/06/2026 on bridge_validation.py and confirmed
    """
    c, s = np.cos(start_yaw), np.sin(start_yaw)
    R = np.array([[c, -s], [s, c]])     # rotation by +yaw
    t = np.array([start_x, start_y])

    def map_to_epi(pts):
        pts = np.atleast_2d(pts)
        return (pts - t) @ R.T          # apply R(yaw) to each row vector
    def epi_to_map(pts):
        pts = np.atleast_2d(pts)
        return pts @ R + t              # apply R(yaw)^T and add t

    return map_to_epi, epi_to_map

# ===
# NumPy -> Nav2 
# ===
def yaw_from_quat(q):
    """geometry_msgs/Quaternion -> planar yaw (radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quat(yaw):
    """Planar yaw (radians) -> geometry_msgs/Quaternion (pure z rotation)."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def quat_to_rotmat(q):
    """geometry_msgs/Quaternion -> (3, 3) rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)

def transform_to_matrix(transform):
    """geometry_msgs/Transform -> (4, 4) homogeneous matrix (maps source->target)."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rotmat(transform.rotation)
    T[0, 3] = transform.translation.x
    T[1, 3] = transform.translation.y
    T[2, 3] = transform.translation.z
    return T

def vlfm_frontier_to_world(frontier_xy, epi_to_map):
    """frontier_xy: (2,) already-episodic xy from obstacle_map.frontiers -> (2,) world xy."""
    return epi_to_map(np.asarray(frontier_xy, dtype=float).reshape(1, 2))[0]

def make_goal(world_xy, desired_heading, stamp, frame_id='map'):
    """Build a geometry_msgs/PoseStamped goal in the map frame."""
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = stamp                       # node.get_clock().now().to_msg()
    goal.pose.position.x = float(world_xy[0])
    goal.pose.position.y = float(world_xy[1])
    goal.pose.orientation = yaw_to_quat(desired_heading)
    return goal

# ===
# Payload encoding (node -> policy server)
# ===
def array_to_b64(arr):
    """Raw (lossless) base64 of a contiguous numpy array's bytes.

    Used for depth (float32) and occupancy (int8): JPEG would corrupt them, so
    we send the raw bytes and let the server reshape with the *_shape field.
    """
    return base64.b64encode(np.ascontiguousarray(arr).tobytes()).decode('utf-8')

