# Copyright 2025 Dimensional Inc.
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

import math
import random

import numpy as np

from dimos.manipulation.grasp_planner import (
    OBB,
    GraspCandidate,
    GraspPlanner,
    KinpyIKSolver,
    WorldCollisionChecker,
)


def make_topdown_transform(x, y, z, yaw_deg=0.0):
    yaw = math.radians(yaw_deg)
    # Top-down: gripper +Z aligns with +Z world, rotate around Z by yaw
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = Rz
    T[:3, 3] = np.array([x, y, z], dtype=float)
    return T


def demo():
    # Create some synthetic candidates above a table at z=0.75m
    table_height = 0.75
    candidates = []
    for _i in range(20):
        x = random.uniform(0.2, 0.5)
        y = random.uniform(-0.2, 0.2)
        z = random.uniform(table_height + 0.03, table_height + 0.15)
        yaw = random.uniform(-90, 90)
        T = make_topdown_transform(x, y, z, yaw)
        score = random.uniform(0.3, 0.95)
        candidates.append(GraspCandidate(T, score, width=0.04))

    current_q = np.zeros(6, dtype=float)
    ql = np.array([-2.8, -2.1, -2.8, -2.8, -2.8, -2.8], dtype=float)
    qh = np.array([2.8, 2.1, 2.8, 2.8, 2.8, 2.8], dtype=float)

    # IK: use kinpy-based DLS to generate an actual q solution
    ik = KinpyIKSolver(max_iters=150)

    # Build a few random obstacle OBBs (pretend other products) above table
    obstacles = []
    for _i in range(5):
        center = np.array(
            [
                random.uniform(0.25, 0.55),
                random.uniform(-0.18, 0.18),
                table_height + random.uniform(0.02, 0.08),
            ],
            dtype=float,
        )
        yaw = random.uniform(-math.pi, math.pi)
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        half = np.array(
            [random.uniform(0.03, 0.06), random.uniform(0.02, 0.05), random.uniform(0.02, 0.06)],
            dtype=float,
        )
        obstacles.append(OBB(center=center, rotation=Rz, half_extents=half))

    cc = WorldCollisionChecker(obstacles=obstacles, table_margin=0.015, ee_clearance=0.005)
    planner = GraspPlanner(ik, cc, topdown_max_angle_deg=25.0, table_margin_m=0.02)

    result = planner.plan(
        candidates,
        current_q=current_q,
        joint_limits_low=ql,
        joint_limits_high=qh,
        table_height=table_height,
        ee_start_pose_world=None,
    )

    if result is None:
        print("No feasible grasp found.")
    else:
        cand, q = result
        print(f"Selected grasp with score={cand.score:.3f}, pos={cand.pose_world[:3, 3].tolist()}")
        print(f"q solution: {q.tolist()}")


if __name__ == "__main__":
    demo()
