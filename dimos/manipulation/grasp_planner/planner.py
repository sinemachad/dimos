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

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import numpy as np

from .types import GraspCandidate, PlannedGrasp

if TYPE_CHECKING:
    from collections.abc import Iterable

    from .collision_base import CollisionChecker
    from .ik_base import IKSolver


def _is_top_down(
    R_world_gripper: np.ndarray, table_normal_world: np.ndarray, max_angle_deg: float
) -> bool:
    """
    Check if gripper approach axis (Z of gripper) aligns with table normal within tolerance.
    Convention here: gripper +Z is approach direction.
    """
    z_axis = R_world_gripper[:, 2]
    z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-9)
    n = table_normal_world / (np.linalg.norm(table_normal_world) + 1e-9)
    cosang = np.clip(float(np.dot(z_axis, n)), -1.0, 1.0)
    ang = np.degrees(np.arccos(cosang))
    return ang <= max_angle_deg


def _limit_penalty(q: np.ndarray, ql: np.ndarray, qh: np.ndarray) -> float:
    """
    Higher penalty near limits. 0 at center, grows towards limits.
    """
    center = 0.5 * (ql + qh)
    halfspan = 0.5 * (qh - ql) + 1e-9
    rel = np.abs(q - center) / halfspan
    return float(np.clip(np.max(rel), 0.0, 1.0))


class GraspPlanner:
    """
    Filter, feasibility-check, and score grasp candidates to select a good grasp.
    """

    def __init__(
        self,
        ik_solver: IKSolver,
        collision_checker: CollisionChecker,
        table_normal_world: np.ndarray = np.array([0.0, 0.0, 1.0], dtype=float),
        topdown_max_angle_deg: float = 20.0,
        table_margin_m: float = 0.02,
        w_grasp_score: float = 1.0,
        w_joint_distance: float = 0.3,
        w_limit_penalty: float = 0.3,
    ) -> None:
        self.ik = ik_solver
        self.cc = collision_checker
        self.table_normal_world = table_normal_world.astype(float)
        self.topdown_max_angle_deg = float(topdown_max_angle_deg)
        self.table_margin_m = float(table_margin_m)
        self.w_grasp = float(w_grasp_score)
        self.w_dist = float(w_joint_distance)
        self.w_limit = float(w_limit_penalty)

    def plan(
        self,
        grasp_candidates: Iterable[GraspCandidate],
        current_q: np.ndarray,
        joint_limits_low: np.ndarray,
        joint_limits_high: np.ndarray,
        table_height: float,
        ee_start_pose_world: np.ndarray | None = None,
    ) -> tuple[GraspCandidate, np.ndarray] | None:
        """
        Return the best feasible (grasp, q) or None.
        """
        for _ in []:
            pass  # keep formatting stable

        for plan in self.plan_ranked(
            grasp_candidates,
            current_q,
            joint_limits_low,
            joint_limits_high,
            table_height,
            ee_start_pose_world,
        ):
            return plan.candidate, plan.q_solution
        return None

    def plan_ranked(
        self,
        grasp_candidates: Iterable[GraspCandidate],
        current_q: np.ndarray,
        joint_limits_low: np.ndarray,
        joint_limits_high: np.ndarray,
        table_height: float,
        ee_start_pose_world: np.ndarray | None = None,
        top_k: int | None = None,
    ) -> list[PlannedGrasp]:
        """
        Return a ranked list of feasible plans (highest score first).
        """
        feasible: list[PlannedGrasp] = []

        for cand in grasp_candidates:
            T = cand.pose_world
            if T.shape != (4, 4) or not np.isfinite(T).all():
                continue

            # Geometric pre-filters
            R = T[:3, :3]
            p = T[:3, 3]

            # Top-down check
            if not _is_top_down(R, self.table_normal_world, self.topdown_max_angle_deg):
                continue

            # Height above table
            if float(p[2]) <= float(table_height + self.table_margin_m):
                continue

            # IK
            q_sol = self.ik.solve(T, current_q, joint_limits_low, joint_limits_high)
            if q_sol is None:
                continue

            # Collision (state)
            if not self.cc.is_state_collision_free(q_sol, table_height):
                continue

            # Optional coarse approach/path check
            if not self.cc.is_approach_collision_free(
                current_q, q_sol, T, ee_start_pose_world, table_height
            ):
                continue

            # Score: combine AnyGrasp score, -joint distance, -limit penalty
            any_score = float(cand.score)
            joint_dist = float(np.linalg.norm(q_sol - current_q))
            limit_pen = _limit_penalty(q_sol, joint_limits_low, joint_limits_high)

            combined = (
                self.w_grasp * any_score - self.w_dist * joint_dist - self.w_limit * limit_pen
            )

            feasible.append(PlannedGrasp(candidate=cand, q_solution=q_sol, combined_score=combined))

        feasible.sort(key=lambda p: p.combined_score, reverse=True)
        if top_k is not None and top_k > 0:
            return feasible[:top_k]
        return feasible
