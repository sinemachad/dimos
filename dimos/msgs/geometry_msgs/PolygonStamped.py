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

"""PolygonStamped message type."""

from __future__ import annotations

import time
from typing import BinaryIO

from dimos_lcm.geometry_msgs import PolygonStamped as LCMPolygonStamped

from dimos.msgs.geometry_msgs.Point32 import Point32
from dimos.msgs.geometry_msgs.Polygon import Polygon
from dimos.types.timestamped import Timestamped


class PolygonStamped(Timestamped):
    """geometry_msgs.PolygonStamped — polygon with header."""

    msg_name = "geometry_msgs.PolygonStamped"
    ts: float
    frame_id: str
    polygon: Polygon

    def __init__(
        self,
        polygon: Polygon | None = None,
        ts: float = 0.0,
        frame_id: str = "",
    ) -> None:
        self.polygon = polygon or Polygon()
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()

    @property
    def points(self) -> list[Point32]:
        """Shortcut to polygon.points."""
        return self.polygon.points

    def lcm_encode(self) -> bytes:
        """Encode to LCM binary format."""
        lcm_msg = LCMPolygonStamped()
        lcm_msg.polygon = self.polygon
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = self.ros_timestamp()
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> PolygonStamped:
        """Decode from LCM binary format."""
        lcm_msg = LCMPolygonStamped.lcm_decode(data)
        points = [Point32(x=p.x, y=p.y, z=p.z) for p in lcm_msg.polygon.points]
        return cls(
            polygon=Polygon(points=points),
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
        )

    def __repr__(self) -> str:
        return f"PolygonStamped(polygon={self.polygon}, frame_id={self.frame_id!r})"
