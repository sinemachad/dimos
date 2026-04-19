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

"""E2E integration test: cross-wall planning through Unity sim.

Verifies that the FAR planner routes through doorways instead of through walls.
Uses the full navigation stack (same blueprint as unitree_g1_nav_sim) and
tracks the robot position via odometry to verify goal-reaching.

Test sequence:
  p0  (-0.3,  2.5) — open corridor speed test
  p1  (11.2, -1.8) — navigate with furniture
  p2  ( 3.3, -4.9) — intermediate waypoint near doorway (explore lower area)
  p3  ( 7.0, -5.0) — through the doorway into the right room
  p4  (11.3, -5.6) — explore right room
  p4→p1 (11.2, -1.8) — CRITICAL: must route through doorway, NOT wall
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import threading
import time

import lcm as lcmlib
import pytest

os.environ.setdefault("DISPLAY", ":1")

ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
FAR_PATH_TOPIC = "/goal_path#nav_msgs.Path"
FAR_WAYPOINT_TOPIC = "/way_point#geometry_msgs.PointStamped"
LOCAL_PATH_TOPIC = "/path#nav_msgs.Path"

# Waypoint definitions: (name, x, y, z, timeout_sec, reach_threshold_m)
WAYPOINTS = [
    ("p0", -0.3, 2.5, 0.0, 30, 1.5),
    ("p1", 11.2, -1.8, 0.0, 120, 2.0),
    ("p2", 3.3, -4.9, 0.0, 120, 2.0),
    ("p3", 7.0, -5.0, 0.0, 120, 2.0),  # Through doorway into right room
    ("p4", 11.3, -5.6, 0.0, 120, 2.0),  # Deep in right room
    ("p4→p1", 11.2, -1.8, 0.0, 180, 2.0),  # CRITICAL: cross-wall back
]

WARMUP_SEC = 15.0  # seconds to let nav stack build terrain + visibility graph


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


pytestmark = [pytest.mark.slow]


class TestCrossWallPlanning:
    """E2E integration test: cross-wall routing through Unity sim."""

    def test_cross_wall_sequence(self) -> None:
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.core.coordination.module_coordinator import ModuleCoordinator
        from dimos.core.global_config import global_config
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.nav_msgs.Odometry import Odometry
        from dimos.navigation.smart_nav.main import smart_nav, smart_nav_rerun_config
        from dimos.robot.unitree.g1.g1_rerun import (
            g1_static_robot,
        )
        from dimos.simulation.unity.module import UnityBridgeModule
        from dimos.visualization.vis_module import vis_module

        # -- Clear stale nav paths from previous runs -------------------------
        paths_dir = Path(__file__).resolve().parents[3] / "data" / "smart_nav_paths"
        if paths_dir.exists():
            for f in paths_dir.iterdir():
                f.unlink(missing_ok=True)

        # -- Build blueprint (same composition as unitree_g1_nav_sim) ----------
        blueprint = (
            autoconnect(
                UnityBridgeModule.blueprint(
                    unity_binary="",
                    unity_scene="home_building_1",
                    vehicle_height=1.24,
                ),
                smart_nav(
                    body_frame="sensor",
                    terrain_analysis={
                        "obstacle_height_threshold": 0.1,
                        "ground_height_threshold": 0.05,
                        "max_relative_z": 0.3,
                        "min_relative_z": -1.5,
                    },
                    local_planner={
                        "max_speed": 2.0,
                        "autonomy_speed": 2.0,
                        "obstacle_height_threshold": 0.1,
                        "max_relative_z": 0.3,
                        "min_relative_z": -1.5,
                        "freeze_ang": 180.0,
                        "two_way_drive": False,
                    },
                    path_follower={
                        "max_speed": 2.0,
                        "autonomy_speed": 2.0,
                        "max_acceleration": 4.0,
                        "slow_down_distance_threshold": 0.5,
                        "omni_dir_goal_threshold": 0.5,
                        "two_way_drive": False,
                    },
                ),
                vis_module(
                    viewer_backend=global_config.viewer,
                    rerun_config=smart_nav_rerun_config(
                        {
                            "blueprint": UnityBridgeModule.rerun_blueprint,
                            "visual_override": {
                                "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
                            },
                            "static": {
                                "world/color_image": UnityBridgeModule.rerun_static_pinhole,
                                "world/tf/robot": g1_static_robot,
                            },
                        }
                    ),
                ),
            )
            .remappings(
                [
                    (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
                ]
            )
            .global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
        )

        coordinator = ModuleCoordinator.build(blueprint)

        # -- Odom tracking via LCM -------------------------------------------
        lock = threading.Lock()
        odom_count = 0
        robot_x = 0.0
        robot_y = 0.0

        lcm_url = os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")
        lc = lcmlib.LCM(lcm_url)

        def _odom_handler(channel: str, data: bytes) -> None:
            nonlocal odom_count, robot_x, robot_y
            msg = Odometry.lcm_decode(data)
            with lock:
                odom_count += 1
                robot_x = msg.x
                robot_y = msg.y

        lc.subscribe(ODOM_TOPIC, _odom_handler)

        # -- Path churn tracking -----------------------------------------------
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped as PS
        from dimos.msgs.nav_msgs.Path import Path as NavPath

        far_path_count = 0
        far_path_lengths: list[int] = []  # number of poses per path
        far_path_endpoints: list[tuple[float, float]] = []  # endpoint of each path
        far_waypoints: list[tuple[float, float, float]] = []  # (x, y, timestamp)
        local_path_count = 0
        local_path_lengths: list[int] = []

        def _far_path_handler(channel: str, data: bytes) -> None:
            nonlocal far_path_count
            msg = NavPath.lcm_decode(data)
            n = len(msg.poses) if hasattr(msg, "poses") else 0
            with lock:
                far_path_count += 1
                far_path_lengths.append(n)
                if n > 0:
                    last = msg.poses[-1]
                    far_path_endpoints.append((last.position.x, last.position.y))

        def _far_wp_handler(channel: str, data: bytes) -> None:
            msg = PS.lcm_decode(data)
            with lock:
                far_waypoints.append((msg.x, msg.y, time.monotonic()))

        def _local_path_handler(channel: str, data: bytes) -> None:
            nonlocal local_path_count
            msg = NavPath.lcm_decode(data)
            n = len(msg.poses) if hasattr(msg, "poses") else 0
            with lock:
                local_path_count += 1
                local_path_lengths.append(n)

        lc.subscribe(FAR_PATH_TOPIC, _far_path_handler)
        lc.subscribe(FAR_WAYPOINT_TOPIC, _far_wp_handler)
        lc.subscribe(LOCAL_PATH_TOPIC, _local_path_handler)

        # LCM receive thread
        lcm_running = True

        def _lcm_loop() -> None:
            while lcm_running:
                try:
                    lc.handle_timeout(100)
                except Exception:
                    pass

        lcm_thread = threading.Thread(target=_lcm_loop, daemon=True)
        lcm_thread.start()

        try:
            print("[test] Blueprint started, waiting for odom…")

            # Wait for first odom (sim is up)
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                with lock:
                    if odom_count > 0:
                        break
                time.sleep(0.5)

            with lock:
                assert odom_count > 0, "No odometry received after 60s — sim not running?"

            print(f"[test] Odom online. Robot at ({robot_x:.2f}, {robot_y:.2f})")

            # Let the nav stack warm up (terrain analysis, PGO, FAR visibility graph)
            print(f"[test] Warming up for {WARMUP_SEC}s…")
            time.sleep(WARMUP_SEC)
            with lock:
                print(
                    f"[test] Warmup complete. odom_count={odom_count}, "
                    f"pos=({robot_x:.2f}, {robot_y:.2f})"
                )

            # -- Navigate waypoint sequence -----------------------------------
            for name, gx, gy, gz, timeout_sec, threshold in WAYPOINTS:
                with lock:
                    sx, sy = robot_x, robot_y

                print(
                    f"\n[test] === {name}: goal ({gx}, {gy}) | "
                    f"robot ({sx:.2f}, {sy:.2f}) | "
                    f"dist={_distance(sx, sy, gx, gy):.2f}m | "
                    f"budget={timeout_sec}s ==="
                )

                # Reset per-segment churn counters
                with lock:
                    seg_far_start = far_path_count
                    seg_local_start = local_path_count
                    seg_wp_start = len(far_waypoints)

                # Publish goal
                goal = PointStamped(x=gx, y=gy, z=gz, ts=time.time(), frame_id="map")
                lc.publish(GOAL_TOPIC, goal.lcm_encode())
                print(f"[test] Goal published for {name}")

                # Wait for robot to reach goal or timeout
                t0 = time.monotonic()
                reached = False
                last_print = t0
                cx, cy = sx, sy
                dist = _distance(cx, cy, gx, gy)
                while True:
                    with lock:
                        cx, cy = robot_x, robot_y

                    dist = _distance(cx, cy, gx, gy)
                    now = time.monotonic()
                    elapsed = now - t0

                    # Progress log every 5 seconds
                    if now - last_print >= 5.0:
                        with lock:
                            fp = far_path_count - seg_far_start
                            lp = local_path_count - seg_local_start
                            wps = len(far_waypoints) - seg_wp_start
                            last_wp = far_waypoints[-1] if far_waypoints else None
                        wp_str = f"wp=({last_wp[0]:.1f},{last_wp[1]:.1f})" if last_wp else "wp=none"
                        print(
                            f"[test]   {name}: {elapsed:.0f}s/{timeout_sec}s | "
                            f"pos ({cx:.2f}, {cy:.2f}) | dist={dist:.2f}m | "
                            f"far_paths={fp} local_paths={lp} wps={wps} {wp_str}"
                        )
                        last_print = now

                    if dist <= threshold:
                        reached = True
                        print(
                            f"[test] ✓ {name}: reached in {elapsed:.1f}s "
                            f"(dist={dist:.2f}m ≤ {threshold}m)"
                        )
                        break

                    if elapsed >= timeout_sec:
                        print(
                            f"[test] ✗ {name}: NOT reached after {elapsed:.1f}s "
                            f"(dist={dist:.2f}m > {threshold}m)"
                        )
                        break

                    time.sleep(0.1)

                # Per-segment churn summary
                with lock:
                    seg_far = far_path_count - seg_far_start
                    seg_local = local_path_count - seg_local_start
                    seg_wps = len(far_waypoints) - seg_wp_start
                    # Waypoint churn: how much the FAR planner's waypoint jumped
                    seg_wp_list = far_waypoints[seg_wp_start:]
                wp_churn = 0.0
                if len(seg_wp_list) > 1:
                    for i in range(1, len(seg_wp_list)):
                        wp_churn += _distance(
                            seg_wp_list[i - 1][0],
                            seg_wp_list[i - 1][1],
                            seg_wp_list[i][0],
                            seg_wp_list[i][1],
                        )
                far_rate = seg_far / max(elapsed, 1.0)
                print(
                    f"[churn] {name}: far_paths={seg_far} ({far_rate:.1f}/s) "
                    f"local_paths={seg_local} wps={seg_wps} "
                    f"wp_churn={wp_churn:.1f}m (total wp displacement)"
                )

                assert reached, (
                    f"{name}: robot did not reach ({gx}, {gy}) within {timeout_sec}s. "
                    f"Final pos=({cx:.2f}, {cy:.2f}), dist={dist:.2f}m"
                )

        finally:
            print("\n[test] Stopping blueprint…")
            lcm_running = False
            lcm_thread.join(timeout=3)
            coordinator.stop()
            print("[test] Done.")
