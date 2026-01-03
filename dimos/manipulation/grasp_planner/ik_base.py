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


class IKSolver(ABC):
    """
    Abstract interface for inverse kinematics solvers.
    Implementations may wrap MoveIt, TRAC-IK, IKPy, MuJoCo, etc.
    """

    @abstractmethod
    def solve(
        self,
        target_pose_world: np.ndarray,  # 4x4 homogeneous pose
        seed_q: np.ndarray,
        joint_limits_low: np.ndarray,
        joint_limits_high: np.ndarray,
    ) -> np.ndarray | None:
        """
        Return a feasible joint vector for the given target pose, or None if not solvable.
        """
        raise NotImplementedError
