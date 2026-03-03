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

"""Unitree G1 adapter — wraps Unitree SDK2 LocoClient for humanoid base control.

The G1 is a humanoid robot with 3 DOF velocity control: [vx, vy, wz].
This adapter uses the Unitree SDK2 Python bindings to communicate via DDS.

G1 FSM states (discovered via testing):
  FSM 0 = ZeroTorque
  FSM 1 = Damp (robot collapses — NEVER call from adapter)
  FSM 2 = Squat/Crouch
  FSM 3 = Sit
  FSM 4 = Lock Stand (rigid standing, no locomotion)
  FSM 200 = Start (locomotion active, accepts Move commands)
  FSM 702 = Lie2StandUp
  FSM 706 = Squat2StandUp

Initialization sequence:
  1. ChannelFactoryInitialize(0, interface) - Initialize DDS
  2. SetFsmId(4) - Lock stand (FSM 4)
  3. Start() - Activate locomotion (FSM 200)
  4. Move(vx, vy, vyaw) - Send velocity commands

Shutdown: StopMove() only (robot stays standing).

Note: Damp() and ZeroTorque() are NEVER called by the adapter — they
cause the robot to collapse and should only be invoked by the user.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry

logger = setup_logger()


@dataclass
class _Session:
    """Active connection state for a G1."""

    client: LocoClient
    lock: threading.Lock
    state_sub: ChannelSubscriber | None = None
    latest_state: SportModeState_ | None = None
    enabled: bool = False
    locomotion_ready: bool = False


class UnitreeG1TwistAdapter:
    """TwistBaseAdapter implementation for Unitree G1 humanoid.

    Communicates with G1 via Unitree SDK2 Python over DDS.
    Expects 3 DOF: [vx, vy, wz] where:
      - vx: forward/backward velocity (m/s)
      - vy: left/right lateral velocity (m/s)
      - wz: yaw rotation velocity (rad/s)

    Args:
        dof: Number of velocity DOFs (must be 3 for G1)
        network_interface: DDS network interface ID or name (default: 0)
    """

    def __init__(
        self,
        dof: int = 3,
        network_interface: int | str | None = None,
        address: str | None = None,
        **_: object,
    ) -> None:
        if dof != 3:
            raise ValueError(f"G1 only supports 3 DOF (vx, vy, wz), got {dof}")

        # Accept either network_interface= or address= (coordinator passes address=)
        self._network_interface = network_interface or address or "eth0"
        self._session: _Session | None = None

    def _get_session(self) -> _Session:
        """Return active session or raise if not connected."""
        if self._session is None:
            raise RuntimeError("G1 not connected")
        return self._session

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to G1 via DDS and initialize the LocoClient."""
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            # Initialize DDS
            logger.info(f"Initializing DDS with network interface {self._network_interface}...")
            ChannelFactoryInitialize(0, self._network_interface)

            # Create loco client
            logger.info("Connecting to G1 LocoClient...")
            client = LocoClient()
            client.SetTimeout(10.0)
            client.Init()

            # Create session — callback closes over it for state updates
            session = _Session(client=client, lock=threading.Lock())

            def state_callback(msg: SportModeState_) -> None:
                with session.lock:
                    session.latest_state = msg

            state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            state_sub.Init(state_callback, 10)
            session.state_sub = state_sub

            self._session = session
            logger.info("Connected to G1")

            # Stand up and activate locomotion
            if not self._initialize_locomotion():
                logger.error("Failed to initialize locomotion mode")
                self.disconnect()
                return False

            return True

        except Exception as e:
            logger.error(f"Failed to connect to G1: {e}")
            self._session = None
            return False

    def disconnect(self) -> None:
        """Disconnect and safely shut down the robot.

        Stops motion but keeps the robot standing (in locomotion mode).
        Does NOT call Damp/ZeroTorque/Squat — the user should manage
        those transitions manually.
        """
        session = self._session
        if session is not None:
            try:
                logger.info("Stopping G1 motion...")
                session.client.StopMove()
                time.sleep(0.5)
            except Exception as e:
                logger.error(f"Error during disconnect: {e}")

            if session.state_sub is not None:
                try:
                    session.state_sub.Close()
                except Exception as e:
                    logger.error(f"Error closing state subscriber: {e}")

        self._session = None

    def is_connected(self) -> bool:
        """Check if connected to G1."""
        return self._session is not None

    # =========================================================================
    # Info
    # =========================================================================

    def get_dof(self) -> int:
        """G1 base is always 3 DOF (vx, vy, wz)."""
        return 3

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_velocities(self) -> list[float]:
        """Read actual velocities from SportModeState as [vx, vy, wz]."""
        session = self._get_session()
        with session.lock:
            if session.latest_state is None:
                return [0.0, 0.0, 0.0]
            try:
                state = session.latest_state
                return [
                    float(state.velocity[0]),  # vx
                    float(state.velocity[1]),  # vy
                    float(state.imu_state.gyroscope[2]),  # wz (yaw rate)
                ]
            except Exception as e:
                logger.warning(f"Error reading G1 velocities: {e}")
                return [0.0, 0.0, 0.0]

    def read_odometry(self) -> list[float] | None:
        """Read odometry from G1 as [x, y, theta].

        Returns position from SportModeState which provides:
          - position[0]: x (meters)
          - position[1]: y (meters)
          - imu_state.rpy[2]: yaw (radians)
        """
        session = self._get_session()
        with session.lock:
            if session.latest_state is None:
                return None

            try:
                state = session.latest_state
                return [
                    float(state.position[0]),
                    float(state.position[1]),
                    float(state.imu_state.rpy[2]),  # yaw
                ]
            except Exception as e:
                logger.error(f"Error reading G1 odometry: {e}")
                return None

    # =========================================================================
    # Control
    # =========================================================================

    def write_velocities(self, velocities: list[float]) -> bool:
        """Send velocity command to G1.

        Args:
            velocities: [vx, vy, wz] in standard frame (m/s, m/s, rad/s)
        """
        if len(velocities) != 3:
            return False

        session = self._get_session()

        if not session.enabled:
            logger.warning("G1 not enabled, ignoring velocity command")
            return False

        if not session.locomotion_ready:
            logger.warning("G1 locomotion not ready, ignoring velocity command")
            return False

        vx, vy, wz = velocities
        return self._send_velocity(vx, vy, wz)

    def write_stop(self) -> bool:
        """Stop all motion."""
        session = self._get_session()
        try:
            session.client.StopMove()
            return True
        except Exception as e:
            logger.error(f"Error stopping G1: {e}")
            return False

    # =========================================================================
    # Enable/Disable
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable the platform.

        When enabling, ensures the robot is stood up and locomotion is ready.
        When disabling, stops motion but keeps standing.
        """
        session = self._get_session()

        if enable:
            if not session.locomotion_ready:
                logger.info("Locomotion not ready, initializing...")
                if not self._initialize_locomotion():
                    logger.error("Failed to initialize locomotion")
                    return False

            session.enabled = True
            logger.info("G1 enabled")
            return True
        else:
            self.write_stop()
            session.enabled = False
            logger.info("G1 disabled")
            return True

    def read_enabled(self) -> bool:
        """Check if platform is enabled."""
        return self._session is not None and self._session.enabled

    # =========================================================================
    # Internal
    # =========================================================================

    def _initialize_locomotion(self) -> bool:
        """Initialize locomotion mode.

        G1-specific sequence (discovered via hardware testing):
        1. SetFsmId(4) - Enter lock stand (FSM 4, rigid standing)
        2. Start() - Activate locomotion (FSM 200)
        3. Now Move() commands will work

        Note: Damp() is intentionally NOT called here — it causes the
        robot to collapse and should only be invoked by the user.

        Returns:
            True if successful, False otherwise
        """
        session = self._get_session()

        try:
            logger.info("Entering lock stand (FSM 4) on G1...")
            session.client.SetFsmId(4)
            time.sleep(2)  # Wait for lock stand to settle

            logger.info("Starting G1 locomotion (FSM 200)...")
            session.client.Start()
            time.sleep(2)  # Wait for locomotion to activate

            session.locomotion_ready = True
            logger.info("G1 locomotion ready")
            return True

        except Exception as e:
            logger.error(f"Error initializing locomotion: {e}")
            return False

    def _send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """Send raw velocity to G1 via LocoClient.Move().

        Uses default duration (1 second) since the coordinator tick loop
        calls at 100Hz, providing continuous updates.

        Args:
            vx: forward/backward velocity (m/s)
            vy: left/right lateral velocity (m/s)
            wz: yaw rotation velocity (rad/s)
        """
        session = self._get_session()
        try:
            with session.lock:
                session.client.Move(vx, vy, wz)

            return True

        except Exception as e:
            logger.error(f"Error sending G1 velocity: {e}")
            return False


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("unitree_g1", UnitreeG1TwistAdapter)


__all__ = ["UnitreeG1TwistAdapter"]
