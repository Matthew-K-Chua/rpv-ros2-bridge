"""The Nav2 occupancy grid, and the hash that keeps it off the wire.

slam_toolbox republishes an unchanged map far more often than it changes one, and
the grid is most of a ~1 MB payload. So it is hashed on arrival and, while the
hash matches what the server has acknowledged, only the hash travels.

The rule that matters: the bridge may refer to a grid by hash ONLY once the
server has answered a step containing it. A failed POST must not let the next
step point at a grid that never arrived.
"""
import base64
import hashlib

import numpy as np


class OccupancyCache:

    def __init__(self):
        self._grid = None
        self._acked_hash = None

    def update(self, msg):
        """Cache a raw Nav2 grid. The server does its own rasterisation, so the
        trinary data is forwarded verbatim with the metadata to place it."""
        occ_bytes = np.ascontiguousarray(np.asarray(msg.data, dtype=np.int8)).tobytes()
        self._grid = {
            "occ_bytes": occ_bytes,                                     # row-major, len H*W
            "occ_b64": None,                                            # built lazily
            "occ_hash": hashlib.blake2b(occ_bytes, digest_size=16).hexdigest(),
            "occ_shape": [msg.info.height, msg.info.width],
            "origin_x": msg.info.origin.position.x,
            "origin_y": msg.info.origin.position.y,
            "resolution": msg.info.resolution,
        }

    @property
    def ready(self) -> bool:
        return self._grid is not None

    @property
    def grid(self):
        return self._grid

    @property
    def hash(self):
        return self._grid["occ_hash"] if self._grid else None

    @property
    def bounds(self):
        """(x_min, y_min, x_max, y_max) of the cached grid in the map frame,
        or None before the first map. The grid's origin is its lower-left
        corner; the far edge is origin + cells * resolution, EXCLUSIVE, which
        is what Nav2's worldToMap enforces."""
        if self._grid is None:
            return None
        rows, cols = self._grid["occ_shape"]
        res = float(self._grid["resolution"])
        x0, y0 = float(self._grid["origin_x"]), float(self._grid["origin_y"])
        return (x0, y0, x0 + cols * res, y0 + rows * res)

    @property
    def acked_hash(self):
        return self._acked_hash

    def forget_ack(self):
        self._acked_hash = None

    def mark_acked(self, occ_hash):
        self._acked_hash = occ_hash

    def fields(self, force=False):
        """Payload fields for the current grid, or None if there is no grid.

        `occupancy` carries the base64 grid when the server needs it and None
        when the hash alone will do.
        """
        grid = self._grid
        if grid is None:
            return None
        send = force or grid["occ_hash"] != self._acked_hash
        if send and grid["occ_b64"] is None:
            # Encoded here rather than on arrival: /map is published whether or
            # not the grid changed, and an unchanged one never needs its base64.
            grid["occ_b64"] = base64.b64encode(grid["occ_bytes"]).decode("utf-8")
        return {
            "occ_hash": grid["occ_hash"],
            "occupancy": grid["occ_b64"] if send else None,
            "occ_shape": grid["occ_shape"],
            "origin_x": grid["origin_x"],
            "origin_y": grid["origin_y"],
            "resolution": grid["resolution"],
        }
