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

"""Galaxea R1 Pro coordinator blueprints.

The R1 Pro is a bimanual humanoid with 7-DOF arms and a 3-wheel swerve
chassis.  All communication is via ROS 2 (BEST_EFFORT + VOLATILE QoS).

Robot-side prerequisites (must be running before starting the coordinator):
  1. CAN bus:  ``bash ~/can.sh``
  2. Stack:    ``./robot_startup.sh boot ... R1PROBody.d/``
  3. Remap:    ``remappings=[('/controller', '/controller_unused')]``
               in ``r1_pro_chassis_control_launch.py``
  4. Gate 2:   ``ros2 topic pub /controller_unused
               hdas_msg/msg/ControllerSignalStamped ...``
               (mode=5 publisher — see test_03_chassis_on_robot.py)

Usage:
    dimos run coordinator-r1pro         # 2 arms + chassis
    dimos run coordinator-r1pro-arms    # 2 arms only (bench testing)
"""

from __future__ import annotations

from dimos.control.blueprints._hardware import (
    r1pro_arm_left,
    r1pro_arm_right,
    r1pro_chassis,
)
from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState

_left_joints = [f"left_arm_joint{i + 1}" for i in range(7)]
_right_joints = [f"right_arm_joint{i + 1}" for i in range(7)]
_base_joints = make_twist_base_joints("base")

# R1 Pro — 2 arms + holonomic chassis
coordinator_r1pro = ControlCoordinator.blueprint(
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
        ("joint_state", JointState): LCMTransport(
            "/coordinator/joint_state", JointState
        ),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
    }
)

# R1 Pro — 2 arms only (no chassis, for bench testing)
coordinator_r1pro_arms = ControlCoordinator.blueprint(
    hardware=[r1pro_arm_left(), r1pro_arm_right()],
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
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport(
            "/coordinator/joint_state", JointState
        ),
    }
)


__all__ = [
    "coordinator_r1pro",
    "coordinator_r1pro_arms",
]
