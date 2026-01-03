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

from typing import Optional

import kinpy as kp
import numpy as np

from .ik_base import IKSolver


class KinpyIKSolver(IKSolver):
    """
    Damped least-squares IK using kinpy serial chain and Jacobian pseudo-inverse.
    Produces actual joint configurations for a 6-DoF arm described by a URDF.
    """

    def __init__(
        self,
        urdf_path: str = "dimos/hardware/piper_description.urdf",
        end_link: str = "gripper_base",
        max_iters: int = 200,
        pos_tol: float = 1e-3,
        rot_tol: float = 2e-2,  # radians
        step_size: float = 0.5,
        damping: float = 1e-3,
    ) -> None:
        # kinpy expects a URDF XML string
        with open(urdf_path) as f:
            urdf_text = f.read()
        self.chain = kp.build_serial_chain_from_urdf(urdf_text, end_link)
        self.end_link = end_link
        self.max_iters = int(max_iters)
        self.pos_tol = float(pos_tol)
        self.rot_tol = float(rot_tol)
        self.step = float(step_size)
        self.damping = float(damping)

    def solve(
        self,
        target_pose_world: np.ndarray,
        seed_q: np.ndarray,
        joint_limits_low: np.ndarray,
        joint_limits_high: np.ndarray,
    ) -> np.ndarray | None:
        if target_pose_world.shape != (4, 4):
            return None
        q = np.clip(seed_q.astype(float).copy(), joint_limits_low, joint_limits_high)

        target_pos = target_pose_world[:3, 3]
        target_rot = target_pose_world[:3, :3]

        for _ in range(self.max_iters):
            # Forward kinematics - kinpy may return a Transform or a dict
            fk = self.chain.forward_kinematics(q)
            if hasattr(fk, "pos") and hasattr(fk, "rot"):
                ee = fk
            elif isinstance(fk, dict):
                ee = fk.get(self.end_link, next(reversed(fk.values())))
            else:
                return None

            # Extract pose robustly across kinpy versions
            if hasattr(ee, "matrix"):
                M_attr = ee.matrix
                M = M_attr() if callable(M_attr) else M_attr
                M = np.asarray(M)
                cur_pos = M[:3, 3]
                cur_rot = M[:3, :3]
            elif hasattr(ee, "to_matrix"):
                M = np.asarray(ee.to_matrix())
                cur_pos = M[:3, 3]
                cur_rot = M[:3, :3]
            else:
                cur_pos = ee.pos
                cur_rot = ee.rot
                if isinstance(cur_rot, np.ndarray) and cur_rot.shape == (4, 4):
                    cur_rot = cur_rot[:3, :3]

            # Position error
            e_pos = target_pos - cur_pos
            if np.linalg.norm(e_pos) < self.pos_tol:
                # Orientation error using rotation vector
                R_err = target_rot @ cur_rot.T
                rot_vec = _rotation_vector_from_matrix(R_err)
                if np.linalg.norm(rot_vec) < self.rot_tol:
                    return np.clip(q, joint_limits_low, joint_limits_high)

            # Jacobian
            J = self.chain.jacobian(q)
            # Build task-space error (position + orientation vector)
            R_err = target_rot @ cur_rot.T
            rot_vec = _rotation_vector_from_matrix(R_err)
            err6 = np.hstack([e_pos, rot_vec])

            # Damped least squares
            JT = J.T
            H = J @ JT + self.damping * np.eye(6)
            dq = JT @ np.linalg.solve(H, err6) * self.step
            q = q + dq
            q = np.clip(q, joint_limits_low, joint_limits_high)

        return None


def _rotation_vector_from_matrix(R: np.ndarray) -> np.ndarray:
    """
    Convert rotation matrix to axis-angle vector (so(3) log map).
    """
    cos_theta = (np.trace(R) - 1.0) * 0.5
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = np.arccos(cos_theta)
    if theta < 1e-8:
        return np.zeros(3)
    wx = R[2, 1] - R[1, 2]
    wy = R[0, 2] - R[2, 0]
    wz = R[1, 0] - R[0, 1]
    w = np.array([wx, wy, wz]) / (2.0 * np.sin(theta))
    return w * theta
