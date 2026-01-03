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

import numpy as np

from .collision_base import CollisionChecker


class TableCollisionChecker(CollisionChecker):
    """
    Minimal collision checker:
    - Ensures the end-effector target pose is above the table plane with clearance
    - Optionally enforces that configurations do not exceed joint limits (caller clamps)
    This is a placeholder for a real checker (MoveIt/MuJoCo/etc.).
    """

    def __init__(self, ee_clearance: float = 0.02) -> None:
        self.ee_clearance = float(ee_clearance)

    def is_state_collision_free(self, q: np.ndarray, table_height: float) -> bool:
        # Without a full model, we can't compute link heights here.
        # The planner will ensure the EE target pose has adequate z.
        # Here we only validate q is finite.
        return np.isfinite(q).all()
