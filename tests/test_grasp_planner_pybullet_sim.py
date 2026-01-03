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

import argparse
import math
import os
import random
import sys
import time

import numpy as np

# Ensure local package root is imported first
_THIS_DIR = os.path.dirname(__file__)
_PKG_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
sys.path.insert(0, _PKG_ROOT)

from dimos.manipulation.grasp_planner import (
    OBB,
    GraspCandidate,
    GraspPlanner,
    KinpyIKSolver,
    PyBulletCollisionChecker,
    TableSpec,
)


def make_topdown_transform(x, y, z, yaw_deg: float = 0.0):
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = Rz
    T[:3, 3] = np.array([x, y, z], dtype=float)
    return T


def draw_grasp_debug(p_client, grasp: GraspCandidate, color=(1, 0, 0), length=0.08, thickness=3):
    # Draw approach direction (+Z of gripper frame) only
    T = grasp.pose_world
    z_axis = T[:3, 2]
    t = T[:3, 3]
    a = t
    b = t + z_axis * length
    return p_client.addUserDebugLine(a, b, lineColorRGB=color, lineWidth=thickness)


def demo(use_gui: bool = False, urdf_path: str | None = None, end_link: str = "gripper_base"):
    # Table spec close to robot
    table_height = 0.75
    table_half = np.array([0.45, 0.35, 0.03], dtype=float)
    table_center = np.array([0.35, 0.0, table_height - table_half[2]], dtype=float)

    # Robot state/limits
    current_q = np.zeros(6, dtype=float)
    ql = np.array([-2.8, -2.1, -2.8, -2.8, -2.8, -2.8], dtype=float)
    qh = np.array([2.8, 2.1, 2.8, 2.8, 2.8, 2.8], dtype=float)

    # IK: kinpy with URDF
    urdf_path = "/home/jalaj/DimosGraspPlanner/dimos/dimos/dimos/hardware/xarm6_description.urdf"
    end_link = "link_tcp"
    print(f"Using URDF: {urdf_path}\nEnd link: {end_link}")
    ik = KinpyIKSolver(urdf_path=urdf_path, end_link=end_link)

    # Compute a base pose that sits on the table near the objects region
    base_z = float(table_height + 0.02)  # base box half-height = 0.02
    base_x = float(table_center[0] - 0.18)
    base_y = 0.0

    # PyBullet collision checker with robot/table/obstacles
    checker = PyBulletCollisionChecker(
        urdf_path=urdf_path,
        base_position=(base_x, base_y, base_z),  # robot on top of table near objects
        base_orientation_euler=(0.0, 0.0, 0.0),
        use_gui=use_gui,
        self_collision=False,  # relax self-collision for demo feasibility
        gravity=0.0,
        table_spec=TableSpec(size_xyz=table_half * 2, center_world=table_center),
    )
    print(f"Robot base pose set to: x={base_x:.3f}, y={base_y:.3f}, z={base_z:.3f}")

    # Create a few obstacles centered on the table top near the robot (reduced to 2 for clarity)
    rng = np.random.RandomState(7)
    obstacles = []
    for _i in range(10):
        center = np.array(
            [
                float(rng.uniform(base_x + 0.12, base_x + 0.20)),
                float(rng.uniform(-0.06, 0.06)),
                float(table_height + rng.uniform(0.02, 0.05)),
            ],
            dtype=float,
        )
        yaw = float(rng.uniform(-np.pi, np.pi))
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        half = np.array([0.04, 0.03, 0.03], dtype=float)
        obstacles.append(OBB(center=center, rotation=Rz, half_extents=half))
    checker.set_obstacles(obstacles)

    # Generate candidate grasps above obstacle centers (guaranteed over objects)
    candidates = []
    for ob in obstacles:
        x, y, _ = ob.center
        z = table_height + 0.08
        for yaw_deg in (0, 45, 90, 135):
            T = make_topdown_transform(x, y, z, yaw_deg)
            candidates.append(GraspCandidate(T, score=0.9, width=0.04))

    import pybullet as p

    # Draw candidates (red approach vectors)
    for gc in candidates:
        draw_grasp_debug(p, gc, color=(1, 0, 0), length=0.08, thickness=2)

    # Choose a single target EE pose near object band and move there via IK
    target_x = float(base_x + 0.18)
    target_y = 0.0
    target_z = float(table_height + 0.12)
    target_yaw_deg = 0.0
    T_goal = make_topdown_transform(target_x, target_y, target_z, target_yaw_deg)
    q_goal = ik.solve(T_goal, current_q, ql, qh)
    if q_goal is not None:
        checker._set_joint_state(q_goal)  # type: ignore[attr-defined]
        current_q = q_goal
        print(
            f"Moved to target pose x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}, yaw={target_yaw_deg:.1f} deg"
        )
    else:
        print("Warning: IK could not reach the target pose; keeping initial configuration.")

    # Rank feasible grasps and draw top results (green)
    planner = GraspPlanner(ik, checker, topdown_max_angle_deg=45.0, table_margin_m=0.01)
    ranked = planner.plan_ranked(
        candidates,
        current_q=current_q,
        joint_limits_low=ql,
        joint_limits_high=qh,
        table_height=table_height,
    )

    if ranked:
        print("Feasible grasps (top 5):")
        for i, plan in enumerate(ranked[:5]):
            t = plan.candidate.pose_world[:3, 3].tolist()
            print(f"  {i + 1}. score={plan.combined_score:.3f} pos={t}")
            draw_grasp_debug(p, plan.candidate, color=(0, 1, 0), length=0.10, thickness=4)
        best = ranked[0]
        print(
            f"Selected score={best.combined_score:.3f}, pos={best.candidate.pose_world[:3, 3].tolist()}"
        )
        print(f"q: {best.q_solution.tolist()}")
    else:
        print("No feasible grasps found for the current target pose.")

    # Keep GUI open until Ctrl+C
    if use_gui:
        print("PyBullet GUI running. Press Ctrl+C to exit.")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    checker.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    demo(use_gui=True)
