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

"""MovementManager: click-to-goal + teleop/nav velocity mux in one module.

Combines the responsibilities of ClickToGoal and CmdVelMux:
- Validates and forwards clicked_point → goal (+ way_point)
- Multiplexes nav_cmd_vel and tele_cmd_vel → cmd_vel
- When teleop starts: cancels the active nav goal and publishes stop_movement
- When teleop ends: nav resumes but stays idle until a new click

This avoids the round-trip where CmdVelMux had to publish stop_movement
over a stream to ClickToGoal, which then had to publish a NaN goal to the
planner. Now goal cancellation is immediate and internal.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any
import weakref

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MovementManagerConfig(ModuleConfig):
    """Config for MovementManager."""

    # Seconds after the last teleop message before nav_cmd_vel is re-enabled.
    tele_cooldown_sec: float = 1.0


class MovementManager(Module):
    """Click-to-goal relay + teleop/nav velocity mux.

    Ports:
        clicked_point (In[PointStamped]): Click from viewer → publishes goal.
        odometry (In[Odometry]): Robot pose (used for stop waypoint on cancel).
        nav_cmd_vel (In[Twist]): Velocity from the autonomous planner.
        tele_cmd_vel (In[Twist]): Velocity from keyboard/joystick teleop.
        goal (Out[PointStamped]): Navigation goal for the global planner.
        way_point (Out[PointStamped]): Immediate waypoint (disconnected in smart_nav).
        cmd_vel (Out[Twist]): Merged velocity — teleop wins when active.
        stop_movement (Out[Bool]): Fired once when teleop takes over, for
            modules that listen directly (e.g. FarPlanner C++ binary).
    """

    config: MovementManagerConfig

    clicked_point: In[PointStamped]
    odometry: In[Odometry]
    nav_cmd_vel: In[Twist]
    tele_cmd_vel: In[Twist]

    goal: Out[PointStamped]
    way_point: Out[PointStamped]
    cmd_vel: Out[Twist]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._teleop_active = False
        self._timer: threading.Timer | None = None
        self._timer_gen = 0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0

    def __getstate__(self) -> dict[str, Any]:
        state: dict[str, Any] = super().__getstate__()  # type: ignore[no-untyped-call]
        for k in ("_lock", "_timer"):
            state.pop(k, None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._timer = None
        self._timer_gen = 0

    def __del__(self) -> None:
        timer = getattr(self, "_timer", None)
        if timer is not None:
            timer.cancel()
            timer.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

    @rpc
    def start(self) -> None:
        super().start()
        self.odometry.subscribe(self._on_odom)
        self.clicked_point.subscribe(self._on_click)
        self.nav_cmd_vel.subscribe(self._on_nav)
        self.tele_cmd_vel.subscribe(self._on_teleop)

    @rpc
    def stop(self) -> None:
        with self._lock:
            self._timer_gen += 1
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
        super().stop()

    # ── Odometry ──────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._robot_x = msg.pose.position.x
            self._robot_y = msg.pose.position.y
            self._robot_z = msg.pose.position.z

    # ── Click-to-goal ─────────────────────────────────────────────────────

    def _on_click(self, msg: PointStamped) -> None:
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            logger.warning("Ignored invalid click", x=msg.x, y=msg.y, z=msg.z)
            return
        if abs(msg.x) > 500 or abs(msg.y) > 500 or abs(msg.z) > 50:
            logger.warning("Ignored out-of-range click", x=msg.x, y=msg.y, z=msg.z)
            return

        logger.info("Goal", x=round(msg.x, 1), y=round(msg.y, 1), z=round(msg.z, 1))
        self.way_point.publish(msg)
        self.goal.publish(msg)

    def _cancel_goal(self) -> None:
        """Publish NaN goal so planners clear their active goal."""
        cancel = PointStamped(
            ts=time.time(), frame_id="map", x=float("nan"), y=float("nan"), z=float("nan")
        )
        self.way_point.publish(cancel)
        self.goal.publish(cancel)
        logger.info("Navigation cancelled — waiting for new goal")

    # ── Velocity mux ─────────────────────────────────────────────────────

    def _on_nav(self, msg: Twist) -> None:
        with self._lock:
            if self._teleop_active:
                return
        self.cmd_vel.publish(msg)

    def _on_teleop(self, msg: Twist) -> None:
        was_active: bool
        old_timer: threading.Timer | None = None
        with self._lock:
            was_active = self._teleop_active
            self._teleop_active = True
            if self._timer is not None:
                self._timer.cancel()
                old_timer = self._timer
            self._timer_gen += 1
            my_gen = self._timer_gen
            self_ref = weakref.ref(self)

            def _end() -> None:
                obj = self_ref()
                if obj is not None:
                    obj._end_teleop(my_gen)

            self._timer = threading.Timer(self.config.tele_cooldown_sec, _end)
            self._timer.daemon = True
            self._timer.start()

        if old_timer is not None:
            old_timer.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        if not was_active:
            # Cancel the nav goal directly and notify external listeners.
            self._cancel_goal()
            self.stop_movement.publish(Bool(data=True))
            logger.info("Teleop active")

        self.cmd_vel.publish(msg)

    def _end_teleop(self, expected_gen: int) -> None:
        with self._lock:
            if expected_gen != self._timer_gen:
                return
            self._teleop_active = False
            self._timer = None
