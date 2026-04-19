# Copyright 2026 Dimensional Inc.
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

"""LineSegments3D: collection of 3D line segments for graph edge visualization.

**Decode-only visualization helper for FAR planner.**

This type exists to decode debug visualization data published by FAR planner's
C++ binary (``far_planner_node``).  On the wire it uses ``nav_msgs/Path`` where
consecutive pose pairs form line segments.  The ``orientation.w`` quaternion
component is **repurposed as a data channel** to encode traversability (it is
NOT a valid quaternion):

    1.0 = fully traversable (reachable from robot)
    0.5 = partially traversable
    0.0 = non-traversable / unreachable

Rerun visualization renders as ``rr.LineStrips3D`` color-coded by
traversability (green / yellow / red).

Why ``msgs/nav_msgs/``?
    The transport layer discovers message types by their ``msg_name`` attribute
    (here ``"nav_msgs.LineSegments3D"``).  Stream auto-connection and LCM topic
    resolution depend on this module living under ``msgs/nav_msgs/`` so that
    the ``Out[LineSegments3D]`` streams declared in ``far_planner.py`` are
    wired correctly.

See also:
    - ``GraphNodes3D`` — graph node positions (same ``nav_msgs/Path`` pattern)
    - ``ContourPolygons3D`` — contour polygons (``sensor_msgs/PointCloud2``)
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.nav_msgs import Path as LCMPath

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype


class LineSegments3D(Timestamped):
    """Line segments for FAR planner graph edge visualization.

    **Decode-only** — the C++ ``far_planner_node`` produces these messages;
    the Python side only decodes them for Rerun rendering.  ``lcm_encode``
    intentionally raises ``NotImplementedError``.

    Wire format
    -----------
    ``nav_msgs/Path`` where consecutive pose pairs form line segments.
    ``orientation.w`` is **hijacked** to carry traversability as a float
    (1.0 = traversable, 0.5 = partial, 0.0 = unreachable).  The other
    quaternion components (x, y, z) are unused.
    """

    msg_name = "nav_msgs.LineSegments3D"
    ts: float
    frame_id: str
    _segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]]
    _traversability: list[float]

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "map",
        segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]] | None = None,
        traversability: list[float] | None = None,
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        self._segments = segments or []
        self._traversability = traversability or [1.0] * len(self._segments)

    def lcm_encode(self) -> bytes:
        # This type is strictly decode-only.  The C++ far_planner_node is the
        # sole producer of these messages; there is no Python code path that
        # needs to encode them.  Raising here makes that contract explicit and
        # prevents silent misuse.
        raise NotImplementedError("Encoded on C++ side")

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> LineSegments3D:
        lcm_msg = LCMPath.lcm_decode(data)
        header_ts = lcm_msg.header.stamp.sec + lcm_msg.header.stamp.nsec / 1e9
        frame_id = lcm_msg.header.frame_id

        segments = []
        traversability = []
        poses = lcm_msg.poses
        for i in range(0, len(poses) - 1, 2):
            p1, p2 = poses[i], poses[i + 1]
            segments.append(
                (
                    (p1.pose.position.x, p1.pose.position.y, p1.pose.position.z),
                    (p2.pose.position.x, p2.pose.position.y, p2.pose.position.z),
                )
            )
            # orientation.w carries traversability as a float, not a
            # quaternion component — see module docstring.
            traversability.append(p1.pose.orientation.w)
        return cls(
            ts=header_ts, frame_id=frame_id, segments=segments, traversability=traversability
        )

    def to_rerun(
        self,
        z_offset: float = 1.7,
        color: tuple[int, int, int, int] = (0, 255, 150, 255),
        radii: float = 0.04,
    ) -> Archetype:
        """Render as ``rr.LineStrips3D`` — color-coded by traversability.

        Green = traversable (reachable from robot), red = non-traversable.
        """
        import rerun as rr

        if not self._segments:
            return rr.LineStrips3D([])

        strips = []
        colors = []
        for idx, (p1, p2) in enumerate(self._segments):
            strips.append(
                [
                    [p1[0], p1[1], p1[2] + z_offset],
                    [p2[0], p2[1], p2[2] + z_offset],
                ]
            )
            trav = self._traversability[idx] if idx < len(self._traversability) else 1.0
            if trav >= 0.9:
                colors.append((0, 220, 100, 200))  # green = fully traversable
            elif trav >= 0.4:
                colors.append((255, 180, 0, 200))  # yellow = partially traversable
            else:
                colors.append((255, 50, 50, 150))  # red = non-traversable

        return rr.LineStrips3D(
            strips,
            colors=colors,
            radii=[radii] * len(strips),
        )

    def __len__(self) -> int:
        return len(self._segments)

    def __str__(self) -> str:
        return f"LineSegments3D(frame_id='{self.frame_id}', segments={len(self._segments)})"
