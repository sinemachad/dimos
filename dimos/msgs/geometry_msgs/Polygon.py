# Copyright 2025-2026 Dimensional Inc.
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

"""Polygon message type."""

from __future__ import annotations

from dimos_lcm.geometry_msgs import Polygon as LCMPolygon

from dimos.msgs.geometry_msgs.Point32 import Point32


class Polygon(LCMPolygon):  # type: ignore[misc]
    """geometry_msgs.Polygon — ordered list of Point32 vertices."""

    msg_name = "geometry_msgs.Polygon"

    def __init__(self, points: list[Point32] | None = None) -> None:
        self.points = points or []
        self.points_length = len(self.points)

    def __repr__(self) -> str:
        return f"Polygon(points={self.points})"
