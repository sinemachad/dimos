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

import numpy as np

from .ik_base import IKSolver


class SimpleIKSolver(IKSolver):
    """
    A very lightweight placeholder IK "solver".
    This does NOT compute true IK. It models a spherical workspace and returns
    the seed configuration if the target is within a nominal reach and orientation
    is not too extreme.
    """

    def __init__(
        self,
        max_reach_meters: float = 0.8,
        max_yaw_pitch_roll_rad: float = np.deg2rad(170.0),
    ) -> None:
        self.max_reach = float(max_reach_meters)
        self.max_angle = float(max_yaw_pitch_roll_rad)

    def solve(
        self,
        target_pose_world: np.ndarray,
        seed_q: np.ndarray,
        joint_limits_low: np.ndarray,
        joint_limits_high: np.ndarray,
    ) -> np.ndarray | None:
        # Quick sanity checks
        if target_pose_world.shape != (4, 4):
            return None
        if seed_q is None or joint_limits_low is None or joint_limits_high is None:
            return None
        if seed_q.shape != joint_limits_low.shape or seed_q.shape != joint_limits_high.shape:
            return None

        # Position reach check
        t = target_pose_world[:3, 3]
        if np.linalg.norm(t) > self.max_reach + 1e-6:
            return None

        # Orientation sanity: ensure rotation matrix is valid-ish
        R = target_pose_world[:3, :3]
        if not np.isfinite(R).all():
            return None
        # bound the implied euler values (very coarse)
        # Using atan2 form: roll from R32,R33 etc. Not exact—coarse validation.
        try:
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
        except Exception:
            return None

        if any(abs(a) > self.max_angle for a in (roll, pitch, yaw)):
            return None

        # Respect joint limits by clamping the seed (still a placeholder)
        q = np.clip(seed_q, joint_limits_low, joint_limits_high)
        return q
