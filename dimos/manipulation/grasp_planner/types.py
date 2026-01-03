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
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import numpy as np


@dataclass
class GraspCandidate:
    """
    A single 6-DoF grasp proposal in world frame.
    """

    pose_world: np.ndarray  # 4x4 homogeneous transform (world_T_gripper)
    score: float  # generator score (e.g., AnyGrasp)
    width: float | None = None  # gripper jaw width (m)
    source_id: str | None = None


@dataclass
class PlannedGrasp:
    """
    A feasible grasp with a joint solution.
    """

    candidate: GraspCandidate
    q_solution: np.ndarray
    combined_score: float
