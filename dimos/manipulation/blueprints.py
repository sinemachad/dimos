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

"""
Manipulation blueprints.

Quick start:
    # 1. Verify manipulation deps load correctly (standalone, no hardware):
    dimos run xarm6-planner-only

    # 2. Keyboard teleop with mock arm:
    dimos run keyboard-teleop-xarm7

    # 3. Interactive RPC client (plan, preview, execute from Python):
    dimos run xarm7-planner-coordinator
    python -i -m dimos.manipulation.planning.examples.manipulation_client
"""

import math

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.coordinator import ControlCoordinator
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.catalog.ufactory import xarm6 as _catalog_xarm6, xarm7 as _catalog_xarm7
from dimos.robot.foxglove_bridge import FoxgloveBridge  # TODO: migrate to rerun

# Single XArm6 planner (standalone, no coordinator)
_xarm6_planner_cfg = _catalog_xarm6(name="arm")

xarm6_planner_only = ManipulationModule.blueprint(
    robots=[_xarm6_planner_cfg.to_robot_model_config()],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
    }
)


# Dual XArm6 planner with coordinator integration
# Usage: Start with coordinator_dual_mock, then plan/execute via RPC
_left_arm_cfg = _catalog_xarm6(name="left_arm", y_offset=0.5)
_right_arm_cfg = _catalog_xarm6(name="right_arm", y_offset=-0.5)

dual_xarm6_planner = ManipulationModule.blueprint(
    robots=[
        _left_arm_cfg.to_robot_model_config(),
        _right_arm_cfg.to_robot_model_config(),
    ],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# Single XArm7 planner + mock coordinator (standalone, no external coordinator needed)
# Usage: dimos run xarm7-planner-coordinator
_xarm7_cfg = _catalog_xarm7(name="arm")

xarm7_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_xarm7_cfg.to_robot_model_config()],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_cfg.to_hardware_component()],
        tasks=[_xarm7_cfg.to_task_config()],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# XArm7 planner + LLM agent for testing base ManipulationModule skills
# No perception — uses the base module's planning + gripper skills only.
# Usage: dimos run coordinator-mock, then dimos run xarm7-planner-coordinator-agent
_BASE_MANIPULATION_AGENT_SYSTEM_PROMPT = """\
You are a robotic manipulation assistant controlling an xArm7 robot arm.

Available skills:
- get_robot_state: Get current joint positions, end-effector pose, and gripper state.
- move_to_pose: Move end-effector to ABSOLUTE x, y, z (meters) with optional roll, pitch, yaw (radians).
- move_to_joints: Move to a joint configuration (comma-separated radians).
- open_gripper / close_gripper / set_gripper: Control the gripper.
- go_home: Move to the home/observe position.
- go_init: Return to the startup position.
- reset: Clear a FAULT state and return to IDLE. Use this when a motion fails.

COORDINATE SYSTEM (world frame, meters):
- X axis = forward (away from the robot base)
- Y axis = left
- Z axis = up
- Z=0 is the robot base level; typical working height is Z = 0.2-0.5

CRITICAL WORKFLOW for relative movement requests (e.g. "move 20cm forward"):
1. Call get_robot_state to get the current EE pose.
2. Add the requested offset to the CURRENT position. Example: if EE is at \
(0.3, 0.0, 0.4) and user says "move 20cm forward", target is (0.5, 0.0, 0.4).
3. Call move_to_pose with the computed ABSOLUTE target.
NEVER pass only the offset as coordinates — that would send the robot to near-origin.

ERROR RECOVERY: If a motion fails or the state becomes FAULT, call reset before retrying.
"""

xarm7_planner_coordinator_agent = autoconnect(
    xarm7_planner_coordinator,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_BASE_MANIPULATION_AGENT_SYSTEM_PROMPT),
)


# XArm7 with eye-in-hand RealSense camera for perception-based manipulation
# TF chain: world → link7 (ManipulationModule) → camera_link (RealSense)
# Usage: dimos run coordinator-mock, then dimos run xarm-perception
_XARM_PERCEPTION_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.06693724, y=-0.0309563, z=0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),  # xyzw
)

_xarm7_perception_cfg = _catalog_xarm7(
    name="arm",
    pitch=math.radians(45),
    add_gripper=True,
    tf_extra_links=["link7"],
)

xarm_perception = (
    autoconnect(
        PickAndPlaceModule.blueprint(
            robots=[_xarm7_perception_cfg.to_robot_model_config()],
            planning_timeout=10.0,
            enable_viz=True,
            floor_z=-0.02,
        ),
        RealSenseCamera.blueprint(
            base_frame_id="link7",
            base_transform=_XARM_PERCEPTION_CAMERA_TRANSFORM,
        ),
        ObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            distance_threshold=0.08,
            min_detections_for_permanent=3,
            max_distance=1.0,
            use_aabb=True,
            max_obstacle_width=0.06,
        ),
        FoxgloveBridge.blueprint(),  # TODO: migrate to rerun
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(viewer="foxglove", n_workers=4)
)


# XArm7 perception + LLM agent for agentic manipulation.
# Skills (pick, place, move_to_pose, etc.) auto-register with the agent's SkillCoordinator.
# Usage: XARM7_IP=<ip> dimos run coordinator-xarm7 xarm-perception-agent
_MANIPULATION_AGENT_SYSTEM_PROMPT = """\
You are a robotic manipulation assistant controlling an xArm7 robot arm with an \
eye-in-hand RealSense camera and a gripper.

# Skills

## Perception
- **look**: Quick snapshot of objects visible from the current camera pose. Does NOT \
move the arm. Example: "what do you see?", "what's on the table?"
- **scan_objects**: Full scan — moves the arm to the init position for a clear view, \
then refreshes detections. Use before pick/place, after a failed grasp, or when the \
user explicitly asks to scan. Example: "scan the table", "what objects are there?"

## Pick & Place
- **pick <object_name>**: Pick up a detected object by name. Use the EXACT name from \
look/scan_objects output. When duplicates exist, pass the object_id shown in brackets \
(e.g. [id=abc12345]). Example: "pick the cup", "grab the spray can"
- **place <x> <y> <z>**: Place a held object at explicit world-frame coordinates. \
Example: "place it at 0.4, 0.3, 0.1"
- **drop_on <object_name>**: Drop a held object onto another detected object. \
Automatically compensates for camera occlusion. Example: "drop it in the bowl", \
"put it on the box"
- **place_back**: Return a held object to its original pick position.
- **pick_and_place <object_name> <x> <y> <z>**: Pick then place in one command.

## Motion
- **move_to_pose <x> <y> <z> [roll pitch yaw]**: Move end-effector to an absolute \
world-frame pose (meters / radians).
- **move_to_joints <j1, j2, ..., j7>**: Move to a joint configuration (radians).
- **go_home**: Move to the home/observe position.
- **go_init**: Return to the startup position. Use after pick/place as a safe resting pose.

## Gripper
- **open_gripper / close_gripper / set_gripper**: Direct gripper control.

## Status & Recovery
- **get_robot_state**: Current joint positions, end-effector pose, and gripper state.
- **get_scene_info**: Full robot state, detected objects, and scene overview.
- **reset**: Clear a FAULT state and return to IDLE. Available as both a skill and RPC.
- **clear_perception_obstacles**: Remove detected obstacles from the planning world. \
Use when planning fails with COLLISION_AT_START.

# Choosing look vs scan_objects
- "what can you see?" / "what's there?" → **look** (instant, no movement)
- "scan the scene" / before pick-and-place → **scan_objects** (thorough, moves arm)
- If objects were ALREADY detected by a previous look, do NOT scan again — just proceed.

# Rules
- Use the EXACT object name from detection output. Do NOT substitute similar names \
(e.g. if detection says "spray can", do not use "grinder").
- "drop it in/on [object]" → use **drop_on**. "place it at [coords]" → use **place**.
- "bring it back" → pick, then **go_init**. Do NOT place randomly.
- "bring it to me" / "hand it over" → pick, then move toward user (≈ X=0, Y=0.5).
- NEVER open the gripper while holding an object unless the user asks or you are \
executing place/drop_on. The gripper stays closed during movement.
- After pick or place, return to init with **go_init** unless another action follows.

# Coordinate System
World frame (meters): X = forward, Y = left, Z = up. Z = 0 is robot base.
Typical working area: X 0.3-0.7, Y -0.5 to 0.5, Z 0.05-0.5.

# Error Recovery
If planning fails with COLLISION_AT_START: call **clear_perception_obstacles**, then \
**reset**, then retry.
"""

xarm_perception_agent = autoconnect(
    xarm_perception,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=_MANIPULATION_AGENT_SYSTEM_PROMPT),
)


# ---------------------------------------------------------------------------
# Galaxea R1 Pro
# ---------------------------------------------------------------------------

# R1 Pro gripper collision exclusions
# The gripper fingers and camera mounts can legitimately overlap with the gripper body
R1PRO_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    # Left gripper
    ("left_gripper_link", "left_gripper_finger_link1"),
    ("left_gripper_link", "left_gripper_finger_link2"),
    ("left_gripper_finger_link1", "left_gripper_finger_link2"),
    ("left_arm_link7", "left_gripper_link"),
    ("left_gripper_link", "left_D405_link"),
    # Right gripper
    ("right_gripper_link", "right_gripper_finger_link1"),
    ("right_gripper_link", "right_gripper_finger_link2"),
    ("right_gripper_finger_link1", "right_gripper_finger_link2"),
    ("right_arm_link7", "right_gripper_link"),
    ("right_gripper_link", "right_D405_link"),
]


def _get_r1pro_urdf_path() -> Path:
    """Get path to R1 Pro URDF."""
    return get_data("r1_pro_description") / "urdf" / "r1_pro.urdf"


def _get_r1pro_package_paths() -> dict[str, Path]:
    """Get package paths for R1 Pro URDF resolution."""
    return {"r1_pro_description": get_data("r1_pro_description")}


# Measured effective joint limits (radians) from test_09_joint_limits.py.
# Joint 7 is significantly tighter than the URDF declares.
_R1PRO_LEFT_ARM_LIMITS_LOWER = [-4.4485, -0.1717, -2.3547, -2.0889, -2.3553, -1.0462, -0.4713]
_R1PRO_LEFT_ARM_LIMITS_UPPER = [1.3087, 3.1409, 2.3557, 0.3474, 2.3555, 1.0470, 0.5862]

# Right arm — not yet measured, use URDF limits (mirrored joint2 sign)
_R1PRO_RIGHT_ARM_LIMITS_LOWER = [-4.4506, -3.1416, -2.3562, -2.0944, -2.3562, -1.0472, -1.5708]
_R1PRO_RIGHT_ARM_LIMITS_UPPER = [1.3090, 0.1745, 2.3562, 0.3491, 2.3562, 1.0472, 1.5708]


def _make_r1pro_arm_config(
    side: str = "left",
    coordinator_task: str | None = None,
) -> RobotModelConfig:
    """Create R1 Pro arm config for one side.

    Loads the full-body URDF but only controls one arm's joints.
    Drake's SetAutoRenaming handles the duplicate model names when
    both arms are added.

    Args:
        side: "left" or "right"
        coordinator_task: Task name for coordinator RPC execution
    """
    joint_names = [f"{side}_arm_joint{i + 1}" for i in range(7)]

    if side == "left":
        limits_lower = _R1PRO_LEFT_ARM_LIMITS_LOWER
        limits_upper = _R1PRO_LEFT_ARM_LIMITS_UPPER
    else:
        limits_lower = _R1PRO_RIGHT_ARM_LIMITS_LOWER
        limits_upper = _R1PRO_RIGHT_ARM_LIMITS_UPPER

    return RobotModelConfig(
        name=f"{side}_arm",
        urdf_path=_get_r1pro_urdf_path(),
        base_pose=_make_base_pose(),
        joint_names=joint_names,
        joint_limits_lower=limits_lower,
        joint_limits_upper=limits_upper,
        end_effector_link=f"{side}_arm_link7",
        base_link="base_link",
        package_paths=_get_r1pro_package_paths(),
        collision_exclusion_pairs=R1PRO_GRIPPER_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        coordinator_task_name=coordinator_task,
        home_joints=[0.0] * 7,
    )


_r1pro_left_joints = [f"left_arm_joint{i + 1}" for i in range(7)]
_r1pro_right_joints = [f"right_arm_joint{i + 1}" for i in range(7)]

# R1 Pro dual-arm planner + real hardware coordinator
# Usage:
#   dimos run r1pro-planner-coordinator
#   python -i -m dimos.manipulation.planning.examples.manipulation_client
#
# Then in the client REPL:
#   joints(robot_name="left_arm")
#   plan([0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0], robot_name="left_arm")
#   preview()
#   execute()
r1pro_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[
            _make_r1pro_arm_config("left", coordinator_task="traj_left"),
        ],
        planning_timeout=10.0,
        enable_viz=True,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[
            HardwareComponent(
                hardware_id="left_arm",
                hardware_type=HardwareType.MANIPULATOR,
                joints=make_joints("left_arm", 7),
                adapter_type="r1pro_arm",
                auto_enable=True,
                adapter_kwargs={"side": "left"},
            ),
            HardwareComponent(
                hardware_id="right_arm",
                hardware_type=HardwareType.MANIPULATOR,
                joints=make_joints("right_arm", 7),
                adapter_type="r1pro_arm",
                auto_enable=True,
                adapter_kwargs={"side": "right"},
            ),
        ],
        tasks=[
            TaskConfig(
                name="traj_left",
                type="trajectory",
                joint_names=_r1pro_left_joints,
                priority=10,
            ),
            TaskConfig(
                name="traj_right",
                type="trajectory",
                joint_names=_r1pro_right_joints,
                priority=10,
            ),
        ],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


__all__ = [
    "dual_xarm6_planner",
    "r1pro_planner_coordinator",
    "xarm6_planner_only",
    "xarm7_planner_coordinator",
    "xarm7_planner_coordinator_agent",
    "xarm_perception",
    "xarm_perception_agent",
]
