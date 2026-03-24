# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""TSDFMap: dynamic global map using voxel-hashed TSDF.

Each voxel stores (signed_distance, weight).  New observations update via
weighted averaging with a max weight cap.  Moved obstacles fade because
rays that now pass through formerly-occupied voxels assign large positive
SDF values (free space), shifting the weighted average away from zero.

See DESIGN.md for algorithmic background.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TSDFMapConfig(ModuleConfig):
    """Configuration for the TSDF global mapper."""

    voxel_size: float = 0.15
    """Voxel edge length in metres."""

    sdf_trunc: float = 0.3
    """Truncation distance. Controls clearing range. Typically 2-3× voxel_size."""

    max_range: float = 15.0
    """Ignore points beyond this distance from sensor."""

    map_publish_rate: float = 0.5
    """Global map publication rate (Hz)."""

    max_weight: float = 50.0
    """Per-voxel weight cap. Lower = faster to forget stale data."""

    key_trans: float = 0.5
    """Integrate only when robot moved this many metres."""


class TSDFMap(Module["TSDFMapConfig"]):
    """Voxel-hash TSDF dynamic global mapping module.

    Streams are compatible with IncrementalMap / DynamicMap for blueprint
    interchangeability.
    """

    default_config = TSDFMapConfig

    registered_scan: In[PointCloud2]
    raw_odom: In[PoseStamped]
    global_map: Out[PointCloud2]
    odom: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._voxels: dict[tuple[int, int, int], tuple[float, float]] = {}
        self._lock = threading.Lock()
        self._latest_pose: PoseStamped | None = None
        self._last_integrate_pos: np.ndarray | None = None
        self._running = threading.Event()
        self._publish_thread: threading.Thread | None = None

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Exclude unpicklable threading primitives for worker deployment."""
        state = super().__getstate__()
        state.pop("_lock", None)
        state.pop("_running", None)
        state.pop("_publish_thread", None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        """Restore from pickled state and reinitialise threading primitives."""
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._publish_thread = None

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.raw_odom.subscribe(self._on_odom))
        self._disposables.add(self.registered_scan.subscribe(self._on_scan))
        self._running.set()
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()
        logger.info("[TSDFMap] Started")

    @rpc
    def stop(self) -> None:
        self._running.clear()
        if self._publish_thread is not None:
            self._publish_thread.join(timeout=2)
            self._publish_thread = None
        super().stop()

    # ── Callbacks ──

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg
        self.odom.publish(msg)

    def _on_scan(self, cloud: PointCloud2) -> None:
        with self._lock:
            pose = self._latest_pose
        if pose is None:
            return

        raw = cloud.as_numpy()
        pts = raw[0] if isinstance(raw, tuple) else raw
        if pts is None or len(pts) == 0:
            return

        origin = np.array([pose.position.x, pose.position.y, pose.position.z])

        if not self._should_integrate(origin):
            return

        # Range filter
        dists = np.linalg.norm(pts[:, :3] - origin, axis=1)
        pts = pts[dists < self.config.max_range]
        if len(pts) == 0:
            return

        with self._lock:
            self._integrate_points(pts[:, :3].astype(np.float64), origin.astype(np.float64))

    # ── TSDF Integration ──

    def _integrate_points(self, points: np.ndarray, origin: np.ndarray) -> None:
        """Integrate points into the voxel-hash TSDF.

        For each point, march along the ray from origin within the truncation
        band and update voxels with the signed distance via weighted average.
        """
        vs = self.config.voxel_size
        trunc = self.config.sdf_trunc
        step = vs * 0.5
        max_w = self.config.max_weight

        for pt in points:
            ray = pt - origin
            ray_len = np.linalg.norm(ray)
            if ray_len < 1e-6:
                continue
            ray_dir = ray / ray_len

            t = max(0.0, ray_len - trunc)
            end_t = ray_len + trunc

            while t <= end_t:
                p = origin + ray_dir * t
                sdf = ray_len - t  # positive = free, negative = behind surface

                vk = (int(np.floor(p[0] / vs)), int(np.floor(p[1] / vs)), int(np.floor(p[2] / vs)))

                if vk in self._voxels:
                    old_sdf, old_w = self._voxels[vk]
                    new_w = min(old_w + 1.0, max_w)
                    new_sdf = (old_sdf * old_w + sdf) / new_w
                    self._voxels[vk] = (new_sdf, new_w)
                else:
                    self._voxels[vk] = (sdf, 1.0)

                t += step

    def _should_integrate(self, pos: np.ndarray) -> bool:
        if self._last_integrate_pos is None:
            self._last_integrate_pos = pos.copy()
            return True
        if np.linalg.norm(pos - self._last_integrate_pos) >= self.config.key_trans:
            self._last_integrate_pos = pos.copy()
            return True
        return False

    # ── Map Publishing ──

    def _publish_loop(self) -> None:
        while self._running.is_set():
            time.sleep(1.0 / max(self.config.map_publish_rate, 0.01))
            self._publish_map()

    def _publish_map(self) -> None:
        with self._lock:
            if not self._voxels:
                return
            threshold = self.config.voxel_size
            vs = self.config.voxel_size
            occupied = []
            for vk, (sdf, w) in self._voxels.items():
                if abs(sdf) < threshold and w >= 2.0:
                    occupied.append([(vk[0] + 0.5) * vs, (vk[1] + 0.5) * vs, (vk[2] + 0.5) * vs])

        if not occupied:
            return

        pts = np.array(occupied, dtype=np.float32)
        self.global_map.publish(PointCloud2.from_numpy(pts, "map", time.time()))


tsdf_map = TSDFMap.blueprint
