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

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from .collision_base import CollisionChecker

if TYPE_CHECKING:
    from collections.abc import Sequence


@dataclass
class OBB:
    center: np.ndarray  # (3,)
    rotation: np.ndarray  # (3,3) world_R_box
    half_extents: np.ndarray  # (3,)
    source_id: str | None = None

    def inverse_transform(self) -> np.ndarray:
        """Return 4x4 T_inv that maps world points to box local coordinates."""
        R = self.rotation
        c = self.center
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ c
        return T_inv


def _segment_hits_aabb(p0: np.ndarray, p1: np.ndarray, half_extents: np.ndarray) -> bool:
    """
    Segment-vs-AABB test in box local space using the slab method.
    AABB extents are [-hx,hx],[-hy,hy],[-hz,hz].
    """
    d = p1 - p0
    tmin, tmax = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if p0[i] < -half_extents[i] or p0[i] > half_extents[i]:
                return False
        else:
            inv_d = 1.0 / d[i]
            t1 = (-half_extents[i] - p0[i]) * inv_d
            t2 = (half_extents[i] - p0[i]) * inv_d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return False
    return True


class WorldCollisionChecker(CollisionChecker):
    """
    World collision checker using OBBs for obstacles and coarse path tests.
    - Ensures EE target path does not intersect with any OBB except the grasped object.
    - Ensures Z stays above table plane by a margin along approach.
    """

    def __init__(
        self,
        obstacles: Sequence[OBB] | None = None,
        table_margin: float = 0.01,
        ignore_source_id: str | None = None,
        ee_clearance: float = 0.0,
        approach_distance: float = 0.10,
    ) -> None:
        self.obstacles: list[OBB] = list(obstacles) if obstacles else []
        self.table_margin = float(table_margin)
        self.ignore_source_id = ignore_source_id
        self.ee_clearance = float(ee_clearance)
        self.approach_distance = float(approach_distance)

    def set_obstacles(self, obstacles: Sequence[OBB]) -> None:
        self.obstacles = list(obstacles)

    def is_state_collision_free(self, q: np.ndarray, table_height: float) -> bool:
        # Without full robot geometry we cannot check link collisions.
        # This checker focuses on path and table constraints elsewhere.
        return np.isfinite(q).all()

    def is_approach_collision_free(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        target_pose_world: np.ndarray,
        ee_start_pose_world: np.ndarray | None,
        table_height: float,
        num_checks: int = 5,
    ) -> bool:
        # Determine start and end points of EE center path
        if ee_start_pose_world is not None and ee_start_pose_world.shape == (4, 4):
            p_start = ee_start_pose_world[:3, 3]
        else:
            # If EE start not known, use pre-grasp point offset along -Z of target pose
            R = target_pose_world[:3, :3]
            p_end = target_pose_world[:3, 3]
            p_start = p_end - R[:, 2] * self.approach_distance
        p_end = target_pose_world[:3, 3]

        # Table clearance along segment
        if min(p_start[2], p_end[2]) <= (table_height + self.table_margin):
            return False

        # Check segment intersection against all OBBs (excluding target if ID provided)
        for obb in self.obstacles:
            if self.ignore_source_id and obb.source_id == self.ignore_source_id:
                continue
            T_inv = obb.inverse_transform()
            # Homogeneous transform to local
            p0_h = np.hstack([p_start, 1.0])
            p1_h = np.hstack([p_end, 1.0])
            p0_local = (T_inv @ p0_h)[:3]
            p1_local = (T_inv @ p1_h)[:3]
            if _segment_hits_aabb(p0_local, p1_local, obb.half_extents + self.ee_clearance):
                return False

        return True
