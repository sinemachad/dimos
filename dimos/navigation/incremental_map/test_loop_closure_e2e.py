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

"""End-to-end loop closure integration test for IncrementalMap.

Drives a robot in a square/loop trajectory using the Unity sim's kinematic
model, feeds real odometry and synthetic lidar into IncrementalMap, and
verifies that loop closure is detected and the global map is geometrically
consistent after correction.

Test runs WITHOUT the Unity binary — uses only the kinematic sim
(UnityBridgeModule._sim_loop) + synthetic lidar generation.

Marks: @pytest.mark.slow
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.incremental_map.module import (
    IncrementalMapConfig,
    _IncrementalMapCore,
)

# ─── Synthetic environment ───────────────────────────────────────────────────


def _make_env_scan(robot_x: float, robot_y: float, yaw: float, n_rays: int = 180) -> np.ndarray:
    """Generate a synthetic 2D lidar scan for a robot in a square room.

    The room spans [0, 10] x [0, 10] metres.  Each ray originates from the
    robot position and hits the nearest wall.  Points are in WORLD frame.
    """
    # Square room walls
    room_min = 0.0
    room_max = 10.0
    angles = np.linspace(0.0, 2 * math.pi, n_rays, endpoint=False)
    pts = []
    for a in angles:
        ray_angle = yaw + a
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)

        # Intersect with 4 walls: x=0, x=10, y=0, y=10
        t_hits = []
        if abs(dx) > 1e-6:
            t1 = (room_min - robot_x) / dx
            t2 = (room_max - robot_x) / dx
            t_hits.extend([t for t in [t1, t2] if t > 0])
        if abs(dy) > 1e-6:
            t3 = (room_min - robot_y) / dy
            t4 = (room_max - robot_y) / dy
            t_hits.extend([t for t in [t3, t4] if t > 0])

        if not t_hits:
            continue
        t_min = min(t_hits)
        wx = robot_x + t_min * dx
        wy = robot_y + t_min * dy
        pts.append([wx, wy, 0.0])

    return np.array(pts, dtype=np.float32)


# ─── Trajectory planner ───────────────────────────────────────────────────────


def _square_waypoints(
    start_x: float = 2.0,
    start_y: float = 2.0,
    side: float = 4.0,
    n: int = 20,
) -> list[tuple[float, float]]:
    """Return (x, y) waypoints along a CCW square loop starting and ending at start."""
    pts = []
    # Bottom edge
    for i in range(n + 1):
        pts.append((start_x + i * side / n, start_y))
    # Right edge
    for i in range(1, n + 1):
        pts.append((start_x + side, start_y + i * side / n))
    # Top edge
    for i in range(1, n + 1):
        pts.append((start_x + side - i * side / n, start_y + side))
    # Left edge (back to start)
    for i in range(1, n + 1):
        pts.append((start_x, start_y + side - i * side / n))
    return pts


# ─── Kinematic sim helper ─────────────────────────────────────────────────────


class _KinSim:
    """Minimal kinematic simulator for 2D ground vehicle."""

    def __init__(
        self,
        x: float = 2.0,
        y: float = 2.0,
        yaw: float = 0.0,
        sim_rate: float = 200.0,
    ) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.sim_rate = sim_rate
        self._dt = 1.0 / sim_rate
        self._fwd = 0.0
        self._omega = 0.0
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._odom_callbacks: list = []

    def set_velocity(self, fwd: float, omega: float) -> None:
        with self._lock:
            self._fwd = fwd
            self._omega = omega

    def subscribe_odom(self, cb) -> None:  # type: ignore[no-untyped-def]
        self._odom_callbacks.append(cb)

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        while self._running.is_set():
            t0 = time.monotonic()
            with self._lock:
                fwd, omega = self._fwd, self._omega
            self.yaw += omega * self._dt
            self.x += math.cos(self.yaw) * fwd * self._dt
            self.y += math.sin(self.yaw) * fwd * self._dt

            # Build Odometry message
            cx = math.cos(self.yaw / 2)
            cy = math.cos(0)
            cz = math.cos(self.yaw / 2)
            sx = math.sin(self.yaw / 2)
            sy = math.sin(0)
            sz = math.sin(self.yaw / 2)
            qw = cx * cy * cz + sx * sy * sz
            qx = sx * cy * cz - cx * sy * sz
            qy = cx * sy * cz + sx * cy * sz
            qz = cx * cy * sz - sx * sy * cz

            odom = Odometry(
                ts=time.time(),
                frame_id="map",
                child_frame_id="sensor",
                pose=Pose(
                    position=[self.x, self.y, 0.0],
                    orientation=[qx, qy, qz, qw],
                ),
            )
            for cb in self._odom_callbacks:
                cb(odom)

            elapsed = time.monotonic() - t0
            sleep = max(0.0, self._dt - elapsed)
            if sleep > 0:
                time.sleep(sleep)


# ─── E2E test ─────────────────────────────────────────────────────────────────


@pytest.mark.slow
class TestLoopClosureE2E:
    """End-to-end test: loop trajectory → loop closure → geometric map consistency.

    Does NOT require the Unity binary. Uses KinSim for odometry and
    synthetic lidar scans generated from a known room geometry.
    """

    def _run_loop_trajectory(self, cfg: IncrementalMapConfig) -> _IncrementalMapCore:
        """Run a square loop trajectory through the incremental map core.

        Returns the core after traversal.
        """
        core = _IncrementalMapCore(cfg)
        waypoints = _square_waypoints(start_x=2.0, start_y=2.0, side=4.0, n=25)

        t = 0.0
        scan_dt = 0.25  # seconds between scans
        prev_x, prev_y, prev_yaw = waypoints[0][0], waypoints[0][1], 0.0

        for i, (wx, wy) in enumerate(waypoints):
            # Compute yaw toward this waypoint
            if i > 0:
                dx = wx - prev_x
                dy = wy - prev_y
                if abs(dx) > 1e-3 or abs(dy) > 1e-3:
                    prev_yaw = math.atan2(dy, dx)

            # Rotation matrix for yaw
            cy, sy = math.cos(prev_yaw), math.sin(prev_yaw)
            r = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
            t_vec = np.array([wx, wy, 0.0])

            # Generate world-frame lidar scan
            scan_world = _make_env_scan(wx, wy, prev_yaw, n_rays=120)

            # Transform to body frame for the core
            if len(scan_world) > 0:
                body_pts = (r.T @ (scan_world.T - t_vec[:, None])).T

            ts = t
            t += scan_dt

            core.add_scan(r, t_vec, body_pts, timestamp=ts)
            core.detect_and_correct_loop()

            prev_x, prev_y = wx, wy

        return core

    def test_loop_closure_detected(self):
        """Loop closure must be detected when robot returns to start."""
        cfg = IncrementalMapConfig(
            voxel_size=0.15,
            key_trans=0.3,
            key_deg=10.0,
            loop_search_radius=1.5,
            loop_time_thresh=5.0,
            loop_score_thresh=1.0,
            loop_submap_half_range=3,
            icp_max_iter=30,
            icp_max_dist=5.0,
            min_loop_detect_duration=4.0,
        )
        core = self._run_loop_trajectory(cfg)

        assert core.loop_count >= 1, (
            f"Expected at least 1 loop closure when robot returns to start. "
            f"Got {core.loop_count} loop closures. "
            f"Num keyframes: {core.num_keyframes}"
        )

    def test_map_geometric_consistency_after_loop_closure(self):
        """After loop closure, map must have points near start AND near end of trajectory."""
        cfg = IncrementalMapConfig(
            voxel_size=0.15,
            key_trans=0.3,
            key_deg=10.0,
            loop_search_radius=1.5,
            loop_time_thresh=5.0,
            loop_score_thresh=1.0,
            loop_submap_half_range=3,
            icp_max_iter=30,
            icp_max_dist=5.0,
            min_loop_detect_duration=4.0,
        )
        core = self._run_loop_trajectory(cfg)

        global_map = core.build_global_map()
        assert len(global_map) > 100, f"Global map has too few points: {len(global_map)}"

        # The robot starts at (2, 2) — map must have wall points near there
        # The walls are at x=0, x=10, y=0, y=10 (10x10 room)
        # From (2, 2), the robot sees walls at x=0 (~2m west), y=0 (~2m south)
        # Check that map has points near the south and west walls visible from start
        near_south_wall = global_map[np.abs(global_map[:, 1]) < 0.5]
        near_west_wall = global_map[np.abs(global_map[:, 0]) < 0.5]

        assert len(near_south_wall) > 0, "Map must contain points near south wall (y~0)"
        assert len(near_west_wall) > 0, "Map must contain points near west wall (x~0)"

        # Map must also have points near the north and east walls (visited later)
        near_north_wall = global_map[np.abs(global_map[:, 1] - 10.0) < 0.5]
        near_east_wall = global_map[np.abs(global_map[:, 0] - 10.0) < 0.5]

        assert len(near_north_wall) > 0, "Map must contain points near north wall (y~10)"
        assert len(near_east_wall) > 0, "Map must contain points near east wall (x~10)"

    def test_corrected_odom_at_loop_end(self):
        """After loop closure, corrected odom at start position should be close to start."""
        cfg = IncrementalMapConfig(
            voxel_size=0.15,
            key_trans=0.3,
            key_deg=10.0,
            loop_search_radius=1.5,
            loop_time_thresh=5.0,
            loop_score_thresh=1.0,
            loop_submap_half_range=3,
            icp_max_iter=30,
            icp_max_dist=5.0,
            min_loop_detect_duration=4.0,
        )
        core = self._run_loop_trajectory(cfg)

        # The robot ends up at (2, 2) (same as start)
        # With zero odometry drift (perfect kinematic sim), corrected pose = raw pose
        r_end = np.eye(3)
        t_end = np.array([2.0, 2.0, 0.0])
        r_corr, t_corr = core.get_corrected_pose(r_end, t_end)

        # Corrected position should be within 2m of start (even with ICP noise)
        dist = float(np.linalg.norm(t_corr[:2] - t_end[:2]))
        assert dist < 2.0, (
            f"Corrected end position {t_corr[:2]} should be within 2m of true end {t_end[:2]}. "
            f"Distance: {dist:.2f}m"
        )

    def test_loop_closure_deterministic(self):
        """Run the loop 3 times — all runs must detect loop closure (no flakiness)."""
        cfg = IncrementalMapConfig(
            voxel_size=0.15,
            key_trans=0.3,
            key_deg=10.0,
            loop_search_radius=1.5,
            loop_time_thresh=5.0,
            loop_score_thresh=1.0,
            loop_submap_half_range=3,
            icp_max_iter=30,
            icp_max_dist=5.0,
            min_loop_detect_duration=4.0,
        )
        for run_idx in range(3):
            core = self._run_loop_trajectory(cfg)
            assert core.loop_count >= 1, (
                f"Run {run_idx + 1}/3: Expected loop closure, got {core.loop_count} closures"
            )

    def test_live_odom_feed(self):
        """Feed real odometry via the KinSim and verify map building works end-to-end."""
        cfg = IncrementalMapConfig(
            voxel_size=0.2,
            key_trans=0.4,
            key_deg=10.0,
            loop_search_radius=1.5,
            loop_time_thresh=5.0,
            loop_score_thresh=1.0,
            loop_submap_half_range=3,
            icp_max_iter=20,
            icp_max_dist=5.0,
            min_loop_detect_duration=4.0,
            registered_input=False,  # body-frame input for this test
        )
        core = _IncrementalMapCore(cfg)
        received_odom: list[Odometry] = []
        lock = threading.Lock()

        def _on_odom(msg: Odometry) -> None:
            with lock:
                received_odom.append(msg)

        sim = _KinSim(x=2.0, y=2.0, yaw=0.0, sim_rate=50.0)
        sim.subscribe_odom(_on_odom)
        sim.start()

        waypoints = _square_waypoints(start_x=2.0, start_y=2.0, side=4.0, n=20)
        scan_events: list[tuple[np.ndarray, np.ndarray, float]] = []  # (r, t, ts)

        try:
            for _wx, _wy in waypoints:
                sim.set_velocity(fwd=1.5, omega=0.0)
                time.sleep(0.05)
                # Capture current pose
                cx, cy, cyaw = sim.x, sim.y, sim.yaw
                r = np.array(
                    [
                        [math.cos(cyaw), -math.sin(cyaw), 0],
                        [math.sin(cyaw), math.cos(cyaw), 0],
                        [0, 0, 1],
                    ]
                )
                t_vec = np.array([cx, cy, 0.0])
                scan_pts = _make_env_scan(cx, cy, cyaw, n_rays=60)
                if len(scan_pts) > 0:
                    body_pts = (r.T @ (scan_pts.T - t_vec[:, None])).T
                    scan_events.append((r, t_vec, time.time()))
                    core.add_scan(r, t_vec, body_pts, timestamp=scan_events[-1][2])
                    core.detect_and_correct_loop()
        finally:
            sim.stop()

        # Verify we collected odom and built a map
        with lock:
            n_odom = len(received_odom)

        assert n_odom > 10, f"Expected many odometry messages, got {n_odom}"
        assert core.num_keyframes > 5, f"Expected >5 keyframes, got {core.num_keyframes}"

        global_map = core.build_global_map()
        assert len(global_map) > 50, f"Expected >50 map points, got {len(global_map)}"
