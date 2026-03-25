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

"""Galaxea R1 Pro arm adapter — implements ManipulatorAdapter via ROS 2.

The R1 Pro is a bimanual humanoid with 7-DOF arms.  Each arm is
controlled by publishing ``sensor_msgs/JointState`` commands and
subscribing to joint feedback over ROS 2.  One adapter instance is
created per arm (``side="left"`` or ``side="right"``).

SDK Units: radians (no conversion needed — matches DimOS SI convention).

ROS topics (parameterized by *side*):
  - Feedback : ``/hdas/feedback_arm_{side}``   (JointState)
  - Command  : ``/motion_target/target_joint_state_arm_{side}`` (JointState)
  - Gripper  : ``/motion_target/target_position_gripper_{side}`` (JointState)
  - Brake    : ``/motion_target/brake_mode``  (Bool)

All topics use BEST_EFFORT + VOLATILE QoS to match the robot's
``chassis_control_node`` and HDAS drivers.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from functools import partial
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorAdapter,
    ManipulatorInfo,
)

log = logging.getLogger(__name__)

# Default tracking speed (rad/s) used when the coordinator sends
# velocity=1.0 (the "go as fast as reasonable" default).
_DEFAULT_TRACKING_SPEED = 0.5  # rad/s — conservative, tested on hardware

# DDS discovery takes 3-5 s across the Humble↔Jazzy ethernet link.
_DISCOVERY_TIMEOUT_S = 5.0


def _make_qos() -> Any:
    """Create BEST_EFFORT + VOLATILE QoS profile required by R1 Pro topics."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class R1ProArmAdapter:
    """Galaxea R1 Pro arm adapter.

    Implements the ``ManipulatorAdapter`` protocol via duck typing.
    Uses ``RawROS`` internally for all ROS 2 communication.

    Args:
        address: Unused (kept for registry compatibility). R1 Pro
            communicates via ROS topics, not a TCP/IP address.
        dof: Degrees of freedom (always 7 for R1 Pro arms).
        side: ``"left"`` or ``"right"``.
        hardware_id: Coordinator hardware ID (used for node naming).
        tracking_speed: Default tracking speed in rad/s when
            ``velocity=1.0`` is passed to ``write_joint_positions``.
    """

    def __init__(
        self,
        address: str | None = None,
        dof: int = 7,
        side: str = "left",
        hardware_id: str = "arm",
        tracking_speed: float = _DEFAULT_TRACKING_SPEED,
        **_: object,
    ) -> None:
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        if dof != 7:
            log.warning("R1 Pro arms have 7 DOF; got dof=%d — overriding to 7", dof)
            dof = 7

        self._side = side
        self._dof = dof
        self._hardware_id = hardware_id
        self._tracking_speed = tracking_speed

        # ROS handles (populated on connect)
        self._ros: Any | None = None  # RawROS instance

        # Topic descriptors (populated on connect)
        self._feedback_topic: Any | None = None
        self._command_topic: Any | None = None
        self._gripper_topic: Any | None = None
        self._brake_topic: Any | None = None

        # Cached feedback (protected by _lock)
        self._lock = threading.Lock()
        self._positions: list[float] = [0.0] * self._dof
        self._velocities: list[float] = [0.0] * self._dof
        self._efforts: list[float] = [0.0] * self._dof
        self._feedback_received = False

        # State
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.SERVO_POSITION
        self._unsubscribe_feedback: Any | None = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Connect to the R1 Pro arm via ROS 2."""
        from dimos.hardware.r1pro_ros_env import ensure_r1pro_ros_env
        from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic

        ensure_r1pro_ros_env()

        # Lazy import ROS message types
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        qos = _make_qos()

        # Build topic descriptors
        side = self._side
        self._feedback_topic = RawROSTopic(
            f"/hdas/feedback_arm_{side}", JointState, qos=qos
        )
        self._command_topic = RawROSTopic(
            f"/motion_target/target_joint_state_arm_{side}", JointState, qos=qos
        )
        self._gripper_topic = RawROSTopic(
            f"/motion_target/target_position_gripper_{side}", JointState, qos=qos
        )
        self._brake_topic = RawROSTopic(
            "/motion_target/brake_mode", Bool, qos=qos
        )

        # Create and start ROS node
        node_name = f"r1pro_arm_{side}_{self._hardware_id}"
        self._ros = RawROS(node_name=node_name)

        try:
            self._ros.start()
        except Exception:
            log.exception("Failed to start RawROS node for R1 Pro %s arm", side)
            self._ros = None
            return False

        # Subscribe to feedback
        self._unsubscribe_feedback = self._ros.subscribe(
            self._feedback_topic, self._on_feedback
        )

        # Wait for first feedback message (DDS discovery delay)
        log.info(
            "Waiting up to %.0fs for R1 Pro %s arm feedback...",
            _DISCOVERY_TIMEOUT_S,
            side,
        )
        deadline = time.monotonic() + _DISCOVERY_TIMEOUT_S
        while not self._feedback_received and time.monotonic() < deadline:
            time.sleep(0.05)

        if not self._feedback_received:
            log.warning(
                "No feedback from /hdas/feedback_arm_%s within %.0fs — "
                "adapter connected but positions may be stale.",
                side,
                _DISCOVERY_TIMEOUT_S,
            )

        self._connected = True
        log.info("R1 Pro %s arm adapter connected (feedback=%s)", side, self._feedback_received)
        return True

    def disconnect(self) -> None:
        """Disconnect from the R1 Pro arm."""
        if self._unsubscribe_feedback:
            self._unsubscribe_feedback()
            self._unsubscribe_feedback = None

        if self._ros:
            self._ros.stop()
            self._ros = None

        self._connected = False
        self._enabled = False
        self._feedback_received = False
        log.info("R1 Pro %s arm adapter disconnected", self._side)

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="Galaxea",
            model=f"R1 Pro ({self._side} arm)",
            dof=self._dof,
        )

    def get_dof(self) -> int:
        return self._dof

    # Measured effective joint limits (radians) from test_09_joint_limits.py.
    _LIMITS = {
        "left": {
            "lower": [-4.4485, -0.1717, -2.3547, -2.0889, -2.3553, -1.0462, -0.4713],
            "upper": [1.3087, 3.1409, 2.3557, 0.3474, 2.3555, 1.0470, 0.5862],
        },
        "right": {
            # Not yet measured — using URDF limits
            "lower": [-4.4506, -3.1416, -2.3562, -2.0944, -2.3562, -1.0472, -1.5708],
            "upper": [1.3090, 0.1745, 2.3562, 0.3491, 2.3562, 1.0472, 1.5708],
        },
    }

    def get_limits(self) -> JointLimits:
        limits = self._LIMITS[self._side]
        return JointLimits(
            position_lower=list(limits["lower"]),
            position_upper=list(limits["upper"]),
            velocity_max=[math.pi] * self._dof,
        )

    # ------------------------------------------------------------------
    # Control mode
    # ------------------------------------------------------------------

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            self._control_mode = mode
            return True
        log.warning("R1 Pro arms only support POSITION/SERVO_POSITION, got %s", mode)
        return False

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read_joint_positions(self) -> list[float]:
        with self._lock:
            return list(self._positions)

    def read_joint_velocities(self) -> list[float]:
        with self._lock:
            return list(self._velocities)

    def read_joint_efforts(self) -> list[float]:
        with self._lock:
            return list(self._efforts)

    def read_state(self) -> dict[str, int]:
        return {
            "state": 0 if self._enabled else 1,
            "mode": 1,  # always servo position
        }

    def read_error(self) -> tuple[int, str]:
        if not self._connected:
            return 1, "not connected"
        if not self._feedback_received:
            return 2, "no feedback received"
        return 0, ""

    def read_enabled(self) -> bool:
        return self._enabled

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Command joint positions.

        Args:
            positions: 7 target positions in radians.
            velocity: Speed fraction (0-1). Scaled by tracking_speed to
                produce the per-joint tracking velocity sent to the robot.
        """
        if not self._ros or not self._connected:
            return False

        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        tracking_vel = velocity * self._tracking_speed

        cmd = JointState()
        cmd.header.stamp = self._ros._node.get_clock().now().to_msg()
        cmd.name = [""]
        cmd.position = list(positions)
        cmd.velocity = [tracking_vel] * self._dof
        cmd.effort = [0.0]

        self._ros.publish(self._command_topic, cmd)

        # Continuously release brakes alongside commands
        self._ros.publish(self._brake_topic, Bool(data=False))

        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        # R1 Pro does not support native velocity control via ROS topics.
        log.warning("R1 Pro arms do not support velocity control mode")
        return False

    def write_stop(self) -> bool:
        """Stop by holding the current position with zero tracking velocity."""
        if not self._ros or not self._connected:
            return False

        from sensor_msgs.msg import JointState

        cmd = JointState()
        cmd.header.stamp = self._ros._node.get_clock().now().to_msg()
        cmd.name = [""]
        with self._lock:
            cmd.position = list(self._positions)
        cmd.velocity = [0.0] * self._dof
        cmd.effort = [0.0]

        self._ros.publish(self._command_topic, cmd)
        return True

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable the arm (releases or engages brakes)."""
        if not self._ros:
            return False

        from std_msgs.msg import Bool

        self._ros.publish(self._brake_topic, Bool(data=not enable))
        self._enabled = enable
        return True

    def write_clear_errors(self) -> bool:
        # No error-clearing mechanism exposed via R1 Pro ROS topics.
        return True

    # ------------------------------------------------------------------
    # Optional features
    # ------------------------------------------------------------------

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        return False

    def read_gripper_position(self) -> float | None:
        # TODO: subscribe to gripper feedback topic once format is confirmed
        return None

    def write_gripper_position(self, position: float) -> bool:
        """Command gripper position.

        Args:
            position: Target opening in meters (0 = closed, max TBD).
        """
        if not self._ros or not self._connected:
            return False

        from sensor_msgs.msg import JointState

        cmd = JointState()
        cmd.header.stamp = self._ros._node.get_clock().now().to_msg()
        cmd.position = [position]
        self._ros.publish(self._gripper_topic, cmd)
        return True

    def read_force_torque(self) -> list[float] | None:
        return None

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _on_feedback(self, msg: Any, _topic: Any) -> None:
        """Callback for ``/hdas/feedback_arm_{side}``."""
        with self._lock:
            n = min(len(msg.position), self._dof)
            self._positions[:n] = msg.position[:n]
            if msg.velocity:
                nv = min(len(msg.velocity), self._dof)
                self._velocities[:nv] = msg.velocity[:nv]
            if msg.effort:
                ne = min(len(msg.effort), self._dof)
                self._efforts[:ne] = msg.effort[:ne]
            self._feedback_received = True


# ------------------------------------------------------------------
# Registry
# ------------------------------------------------------------------


def register(registry: AdapterRegistry) -> None:
    """Register R1 Pro arm adapters with the manipulator registry."""
    registry.register("r1pro_arm_left", partial(R1ProArmAdapter, side="left"))
    registry.register("r1pro_arm_right", partial(R1ProArmAdapter, side="right"))
    registry.register("r1pro_arm", R1ProArmAdapter)


__all__ = ["R1ProArmAdapter"]
