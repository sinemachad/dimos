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

"""Galaxea R1 Pro keyboard teleop blueprints.

WASD/QE keyboard control for the R1 Pro chassis (and optionally arms).
Uses the same ``KeyboardTeleop`` module as the Unitree Go2 teleop —
keys map to twist commands routed through the ControlCoordinator.

Robot-side prerequisites (must be running before starting):
  1. Robot stack + CAN: ``bash ~/can.sh && ./robot_startup.sh ...``
  2. Launch file remap: ``('/controller', '/controller_unused')``
  3. Gate 2 publisher: ``ros2 topic pub /controller_unused ...``

Usage:
    dimos run r1pro-keyboard-teleop           # chassis only
    dimos run r1pro-keyboard-teleop-full      # chassis + arms
"""

from __future__ import annotations

from dimos.control.blueprints._hardware import r1pro_arm_left, r1pro_arm_right, r1pro_chassis
from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_base_joints = make_twist_base_joints("base")
_left_joints = [f"left_arm_joint{i + 1}" for i in range(7)]
_right_joints = [f"right_arm_joint{i + 1}" for i in range(7)]

# --- Chassis-only coordinator (no arms) ---
_r1pro_chassis_coordinator = ControlCoordinator.blueprint(
    hardware=[r1pro_chassis()],
    tasks=[
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)

# --- Full coordinator (arms + chassis) ---
_r1pro_full_coordinator = ControlCoordinator.blueprint(
    hardware=[r1pro_arm_left(), r1pro_arm_right(), r1pro_chassis()],
    tasks=[
        TaskConfig(
            name="traj_left",
            type="trajectory",
            joint_names=_left_joints,
            priority=10,
        ),
        TaskConfig(
            name="traj_right",
            type="trajectory",
            joint_names=_right_joints,
            priority=10,
        ),
        TaskConfig(
            name="vel_base",
            type="velocity",
            joint_names=_base_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)

# Keyboard teleop — chassis only (WASD/QE drives the base)
r1pro_keyboard_teleop = autoconnect(
    _r1pro_chassis_coordinator,
    KeyboardTeleop.blueprint(),
)

# Keyboard teleop — full robot (chassis teleop + arms connected for trajectory control)
r1pro_keyboard_teleop_full = autoconnect(
    _r1pro_full_coordinator,
    KeyboardTeleop.blueprint(),
)

__all__ = [
    "r1pro_keyboard_teleop",
    "r1pro_keyboard_teleop_full",
]
