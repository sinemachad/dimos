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

try:
    import pybullet as p  # type: ignore[import-not-found]
    import pybullet_data  # type: ignore[import-not-found]
except Exception:  # pragma: no cover - optional dependency
    p = None  # type: ignore[assignment]
    pybullet_data = None  # type: ignore[assignment]

from .collision_base import CollisionChecker

if TYPE_CHECKING:
    from collections.abc import Sequence

    from .world_collision import OBB


@dataclass
class TableSpec:
    size_xyz: np.ndarray  # (x, y, z)
    center_world: np.ndarray  # (x, y, z)


class PyBulletCollisionChecker(CollisionChecker):
    """
    PyBullet-backed collision checker.
    - Loads a URDF robot and table box.
    - Adds OBB obstacles as collision boxes.
    - Checks link/world collisions for discrete states and along joint-space interpolations.
    """

    def __init__(
        self,
        urdf_path: str,
        base_position: Sequence[float] = (0.0, 0.0, 0.0),
        base_orientation_euler: Sequence[float] = (0.0, 0.0, 0.0),
        use_gui: bool = False,
        self_collision: bool = True,
        gravity: float = 0.0,
        table_spec: TableSpec | None = None,
    ) -> None:
        if p is None:
            raise RuntimeError("pybullet is not installed. Install with: pip install pybullet")

        self.client = p.connect(p.GUI if use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # type: ignore[union-attr]
        p.setGravity(0, 0, gravity, physicsClientId=self.client)

        self.self_collision = self_collision
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=tuple(base_position),
            baseOrientation=p.getQuaternionFromEuler(tuple(base_orientation_euler)),
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MERGE_FIXED_LINKS,
            physicsClientId=self.client,
        )

        # Collect controllable joint indices (revolute or prismatic)
        self.joint_indices: list[int] = []
        n_j = p.getNumJoints(self.robot_id, physicsClientId=self.client)
        for ji in range(n_j):
            jinfo = p.getJointInfo(self.robot_id, ji, physicsClientId=self.client)
            jtype = jinfo[2]
            if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                self.joint_indices.append(ji)

        # Table body
        if table_spec is None:
            # Default table: 1.2 x 0.8 x 0.06 at height 0.75
            half = np.array([0.6, 0.4, 0.03], dtype=float)
            center = np.array([0.5, 0.0, 0.75 - half[2]], dtype=float)
            table_spec = TableSpec(size_xyz=half * 2, center_world=center)
        self.table_id = self._create_box(table_spec.size_xyz, table_spec.center_world)

        # Obstacles registry
        self.obstacle_ids: list[int] = []

    def _create_box(self, size_xyz: np.ndarray, center_world: np.ndarray) -> int:
        half = 0.5 * np.asarray(size_xyz, dtype=float)
        col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=half.tolist(), physicsClientId=self.client
        )
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half.tolist(),
            rgbaColor=[0.5, 0.5, 0.5, 1.0],
            physicsClientId=self.client,
        )
        body = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=center_world.tolist(),
            baseOrientation=[0, 0, 0, 1],
            physicsClientId=self.client,
        )
        return body

    def set_obstacles(self, obstacles: Sequence[OBB]) -> None:
        # Clear existing
        for oid in self.obstacle_ids:
            p.removeBody(oid, physicsClientId=self.client)
        self.obstacle_ids = []

        for obb in obstacles:
            half = np.asarray(obb.half_extents, dtype=float)
            col = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=half.tolist(), physicsClientId=self.client
            )
            vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=half.tolist(),
                rgbaColor=[0.2, 0.7, 0.2, 0.8],
                physicsClientId=self.client,
            )
            quat = p.getQuaternionFromEuler(_matrix_to_euler_zyx(obb.rotation))
            body = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=obb.center.tolist(),
                baseOrientation=quat,
                physicsClientId=self.client,
            )
            self.obstacle_ids.append(body)

    def _set_joint_state(self, q: np.ndarray) -> None:
        # Map q to available joints (truncate if needed)
        n = min(len(q), len(self.joint_indices))
        for k in range(n):
            p.resetJointState(
                self.robot_id, self.joint_indices[k], float(q[k]), physicsClientId=self.client
            )

    def _has_collisions(self) -> bool:
        # Check contact points between robot and any other body (table or obstacles or self)
        # Robot with table/obstacles:
        for other in [self.table_id, *self.obstacle_ids]:
            cps = p.getContactPoints(self.robot_id, other, physicsClientId=self.client)
            if len(cps) > 0:
                return True
        # Robot self-collision
        if self.self_collision:
            cps = p.getContactPoints(self.robot_id, self.robot_id, physicsClientId=self.client)
            if len(cps) > 0:
                return True
        return False

    def is_state_collision_free(self, q: np.ndarray, table_height: float) -> bool:
        self._set_joint_state(q)
        return not self._has_collisions()

    def is_approach_collision_free(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        target_pose_world: np.ndarray,
        ee_start_pose_world: np.ndarray | None,
        table_height: float,
        num_checks: int = 10,
    ) -> bool:
        # Interpolate in joint space and check collisions
        for i in range(1, num_checks + 1):
            alpha = i / (num_checks + 1)
            q = (1 - alpha) * q_start + alpha * q_goal
            self._set_joint_state(q)
            if self._has_collisions():
                return False
        return True

    def close(self) -> None:
        if p is not None:
            try:
                p.disconnect(self.client)
            except Exception:
                pass


def _matrix_to_euler_zyx(R: np.ndarray) -> Sequence[float]:
    """
    Convert 3x3 rotation matrix to ZYX euler angles.
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return [roll, pitch, yaw]
