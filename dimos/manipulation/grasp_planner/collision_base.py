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

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import numpy as np


class CollisionChecker(ABC):
    """
    Abstract interface for collision checks used during grasp planning.
    """

    @abstractmethod
    def is_state_collision_free(
        self,
        q: np.ndarray,
        table_height: float,
    ) -> bool:
        """
        Return True if the robot in configuration q is collision-free.
        Minimum baseline: ensure no link/EE is below the table plane.
        """
        raise NotImplementedError

    def is_approach_collision_free(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        target_pose_world: np.ndarray,
        ee_start_pose_world: np.ndarray | None,
        table_height: float,
        num_checks: int = 5,
    ) -> bool:
        """
        Optional coarse path check (default: assume free).
        ee_start_pose_world: 4x4 pose of current EE if available (for better path checking)
        """
        return True
